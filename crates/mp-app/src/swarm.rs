//! MissionPlanner/Swarm: the "Swarm" tool, FormationControl with its Formation controller, the
//! offset Grid and the per-vehicle Status panel. Every vehicle on every connection
//! (MainV2.Comports / MAVlist) takes part; the leader's position plus each follower's offset,
//! rotated by the leader's heading, is sent at 10 Hz while running.
use crate::{app::Toasts, theme};
use egui::{Color32, Pos2, Rect, Sense, Stroke, Vec2};
use mp_core::{
    comms::ConnectionSpec,
    firmware::Firmware,
    link::{Link, LinkCommand},
    mavlink::dialects::ardupilotmega::*,
};
use std::collections::{BTreeMap, HashMap};
use std::time::{Duration, Instant};

/// (connection index in MainV2.Comports order, sysid, compid)
pub type Key = (usize, u8, u8);

/// A vehicle as the swarm tools see its MAVState.
#[derive(Clone, Debug, Default)]
pub struct Mav {
    pub key: Key,
    pub port: String,
    pub firmware: Firmware,
    pub mode: String,
    pub armed: bool,
    pub lat: f64,
    pub lng: f64,
    /// relative altitude (m)
    pub alt: f64,
    pub yaw: f64,
    pub roll: f64,
    pub groundspeed: f64,
    pub vx: f64,
    pub vy: f64,
    pub vz: f64,
    pub gpsstatus: u8,
}

impl Mav {
    /// CurrentState.radius: the turn radius from groundspeed and bank.
    fn radius(&self) -> f64 {
        if self.groundspeed <= 1.0 {
            return 0.0;
        }
        self.groundspeed * self.groundspeed / (9.80665 * self.roll.to_radians().tan())
    }
}

/// BaseStream.PortName
fn port_name(spec: Option<&ConnectionSpec>) -> String {
    match spec {
        Some(ConnectionSpec::Serial { port, .. }) => port.clone(),
        Some(ConnectionSpec::Tcp { .. }) => "TCP".into(),
        Some(ConnectionSpec::UdpListen { .. }) => "UDP".into(),
        Some(ConnectionSpec::UdpClient { .. }) => "UDPCl".into(),
        _ => "AUTO".into(),
    }
}

/// Every MAVState on every connection, from each vehicle's latest packets.
pub fn vehicles(links: &[&Link]) -> Vec<Mav> {
    let mut out = Vec::new();
    for (i, l) in links.iter().enumerate() {
        let s = l.shared.read();
        let port = port_name(s.spec.as_ref());
        for ((sys, comp), m) in &s.mavs {
            let get = |id: u32| s.inspector.get(&(*sys, *comp, id)).map(|m| &m.msg);
            let mut v = Mav {
                key: (i, *sys, *comp),
                port: port.clone(),
                firmware: m.firmware,
                mode: m.firmware.modes().into_iter().find(|(n, _)| *n == m.custom_mode).map(|(_, s)| s).unwrap_or_else(|| m.custom_mode.to_string()),
                armed: m.armed,
                lat: m.lat,
                lng: m.lng,
                alt: m.alt as f64,
                yaw: m.heading as f64,
                ..Default::default()
            };
            if let Some(MavMessage::GLOBAL_POSITION_INT(p)) = get(33) {
                v.lat = p.lat as f64 / 1e7;
                v.lng = p.lon as f64 / 1e7;
                v.alt = p.relative_alt as f64 / 1000.0;
                v.vx = p.vx as f64 / 100.0;
                v.vy = p.vy as f64 / 100.0;
                v.vz = p.vz as f64 / 100.0;
            }
            if let Some(MavMessage::ATTITUDE(a)) = get(30) {
                v.roll = (a.roll as f64).to_degrees();
                v.yaw = (a.yaw as f64).to_degrees().rem_euclid(360.0);
            }
            if let Some(MavMessage::VFR_HUD(h)) = get(74) {
                v.groundspeed = h.groundspeed as f64;
            }
            if let Some(MavMessage::GPS_RAW_INT(g)) = get(24) {
                v.gpsstatus = g.fix_type as u8;
            }
            out.push(v);
        }
    }
    out
}

fn command(l: &Link, sys: u8, comp: u8, command: MavCmd, p: [f32; 7]) {
    l.send(LinkCommand::Send(Box::new(MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        param1: p[0],
        param2: p[1],
        param3: p[2],
        param4: p[3],
        param5: p[4],
        param6: p[5],
        param7: p[6],
        command,
        target_system: sys,
        target_component: comp,
        confirmation: 0,
    }))));
}

/// setMode(sysid, compid, name)
fn set_mode(l: &Link, m: &Mav, name: &str) {
    if let Some((number, _)) = m.firmware.modes().into_iter().find(|(_, n)| n.eq_ignore_ascii_case(name)) {
        l.send(LinkCommand::Send(Box::new(MavMessage::SET_MODE(SET_MODE_DATA {
            custom_mode: number,
            target_system: m.key.1,
            base_mode: MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        }))));
    }
}

/// UTM in the leader's zone, as FormationControl builds it with ProjNet.
fn to_utm(zone: i32, lat: f64, lng: f64) -> (f64, f64) {
    let (_, e, n) = mp_core::coords::to_utm_in_zone(zone, lat, lng);
    (e, n)
}

fn leader_zone(lat: f64, lng: f64) -> i32 {
    let zone = ((lng + 186.0) / 6.0) as i32;
    if lat < 0.0 { -zone } else { zone }
}

/// getOffsetFromLeader: the follower's position in the leader's frame (x forward, y right).
pub fn offset_from_leader(leader: &Mav, mav: &Mav) -> (f64, f64, f64) {
    let zone = leader_zone(leader.lat, leader.lng);
    let (me, mn) = to_utm(zone, leader.lat, leader.lng);
    let (ve, vn) = to_utm(zone, mav.lat, mav.lng);
    let heading = (-leader.yaw).to_radians();
    let (nx, ny) = (-(mn - vn), -(me - ve));
    (nx * heading.cos() - ny * heading.sin(), nx * heading.sin() + ny * heading.cos(), 0.0)
}

/// Formation.SendCommand's target: the leader position plus the rotated offset.
pub fn target_for(leader: &Mav, offset: (f64, f64, f64)) -> (f64, f64, f64) {
    let zone = leader_zone(leader.lat, leader.lng);
    let (mut e, mut n) = to_utm(zone, leader.lat, leader.lng);
    let heading = (-leader.yaw).to_radians();
    let (x, y) = (offset.0, offset.1);
    e += x * heading.cos() - y * heading.sin();
    n += x * heading.sin() + y * heading.cos();
    let (lat, lng) = mp_core::coords::utm_signed_to_geo(zone, e, n);
    (lat, lng, leader.alt + offset.2)
}

fn wrap_180(v: f64) -> f64 {
    if v > 180.0 {
        v - 360.0
    } else if v < -180.0 {
        v + 360.0
    } else {
        v
    }
}

fn bearing(a: (f64, f64), b: (f64, f64)) -> f64 {
    mp_core::geo::PointLatLngAlt::new(a.0, a.1, 0.0).bearing(&mp_core::geo::PointLatLngAlt::new(b.0, b.1, 0.0))
}

fn distance(a: (f64, f64), b: (f64, f64)) -> f64 {
    mp_core::geo::PointLatLngAlt::new(a.0, a.1, 0.0).distance(&mp_core::geo::PointLatLngAlt::new(b.0, b.1, 0.0))
}

fn newpos(p: (f64, f64), bearing: f64, dist: f64) -> (f64, f64) {
    let q = mp_core::geo::PointLatLngAlt::new(p.0, p.1, 0.0).newpos(bearing, dist);
    (q.lat, q.lng)
}

/// Formation.cs PID (AC_PID with an input filter).
#[derive(Clone, Debug)]
pub struct Pid {
    dt: f32,
    input: f32,
    derivative: f32,
    kp: f32,
    ki: f32,
    kd: f32,
    imax: f32,
    integrator: f32,
    filt_hz: f32,
    reset_filter: bool,
}

impl Pid {
    pub fn new(p: f32, i: f32, d: f32, imax: f32, filt_hz: f32, dt: f32) -> Self {
        Self { dt, input: 0.0, derivative: 0.0, kp: p, ki: i, kd: d, imax: imax.abs(), integrator: 0.0, filt_hz: filt_hz.max(0.01), reset_filter: true }
    }

    fn alpha(&self) -> f32 {
        if self.filt_hz == 0.0 {
            return 1.0;
        }
        let rc = 1.0 / (std::f32::consts::TAU * self.filt_hz);
        self.dt / (self.dt + rc)
    }

    pub fn set_input_filter_all(&mut self, input: f32) {
        if input.is_infinite() {
            return;
        }
        if self.reset_filter {
            self.reset_filter = false;
            self.input = input;
            self.derivative = 0.0;
        }
        let change = self.alpha() * (input - self.input);
        self.input += change;
        if self.dt > 0.0 {
            self.derivative = change / self.dt;
        }
    }

    pub fn get_pid(&mut self) -> f32 {
        let p = self.input * self.kp;
        let mut i = 0.0;
        if self.ki != 0.0 && self.dt != 0.0 {
            self.integrator = (self.integrator + self.input * self.ki * self.dt).clamp(-self.imax, self.imax);
            i = self.integrator;
        }
        p + i + self.kd * self.derivative
    }

    pub fn reset_i(&mut self) {
        self.integrator = 0.0;
    }
}

/// Quaternion.from_euler312 (Matrix3.from_euler312 then from_rotation_matrix): [q1..q4].
pub fn quaternion_euler312(roll: f64, pitch: f64, yaw: f64) -> [f64; 4] {
    let (cp, sp, sr, cr, sy, cy) = (pitch.cos(), pitch.sin(), roll.sin(), roll.cos(), yaw.sin(), yaw.cos());
    let (m00, m11, m22) = (cy * cp - sy * sr * sp, cy * cr, cp * cr);
    let (m01, m02) = (-cr * sy, sp * cy + cp * sr * sy);
    let (m10, m12) = (cp * sy + sp * sr * cy, sy * sp - sr * cy * cp);
    let (m20, m21) = (-sp * cr, sr);
    let tr = m00 + m11 + m22;
    if tr > 0.0 {
        let s = (tr + 1.0).sqrt() * 2.0;
        [0.25 * s, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s]
    } else if m00 > m11 && m00 > m22 {
        let s = (1.0 + m00 - m11 - m22).sqrt() * 2.0;
        [(m21 - m12) / s, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s]
    } else if m11 > m22 {
        let s = (1.0 + m11 - m00 - m22).sqrt() * 2.0;
        [(m02 - m20) / s, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s]
    } else {
        let s = (1.0 + m22 - m00 - m11).sqrt() * 2.0;
        [(m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s]
    }
}

/// One follower command from Formation.SendCommand.
#[derive(Clone, Debug, PartialEq)]
pub enum FollowerCommand {
    /// setPositionTargetGlobalInt(pos, vel) and CONDITION_YAW when the heading differs
    Position { lat: f64, lng: f64, alt: f64, vel: (f64, f64, f64), yaw: Option<f64> },
    /// ArduPlane: SET_ATTITUDE_TARGET from the PIDs
    Attitude { q: [f64; 4], thrust: f64, type_mask: u8 },
}

/// Grid.icon
#[derive(Clone, Debug)]
struct Icon {
    x: f32,
    y: f32,
    z: f32,
    movable: bool,
    color: Color32,
    name: String,
}

enum Drag {
    Icon(Key),
    Pan(Pos2),
}

/// The offsets grid (Swarm/Grid.cs).
struct Grid {
    xdist: i32,
    ydist: i32,
    centerx: f32,
    centery: f32,
    vertical: bool,
    icons: BTreeMap<Key, Icon>,
    hover: Option<Key>,
    drag: Option<Drag>,
    bg: Option<egui::TextureHandle>,
    /// BGImagex, BGImagey, BGImagew, BGImageh (grid cells) and BGImageStepSize
    bg_rect: [f32; 4],
    bg_step: f32,
}

impl Default for Grid {
    fn default() -> Self {
        Self {
            xdist: 40,
            ydist: 40,
            centerx: 0.0,
            centery: 0.0,
            vertical: false,
            icons: BTreeMap::new(),
            hover: None,
            drag: None,
            bg: None,
            bg_rect: [0.0, 0.0, 1.0, 1.0],
            bg_step: 1.0,
        }
    }
}

impl Grid {
    /// setScale
    fn set_scale(&mut self, dist: i32, size: Vec2) {
        if dist > 6 {
            self.ydist = (dist as f64 * (size.y as f64 / size.x as f64)) as i32;
            self.xdist = dist;
            if self.ydist % 2 == 1 {
                self.ydist += 1;
            }
        }
    }

    fn clear(&mut self) {
        self.icons.clear();
    }

    /// UpdateIcon
    fn update_icon(&mut self, key: Key, name: String, x: f32, y: f32, z: f32, movable: bool) {
        let icon = self.icons.entry(key).or_insert(Icon { x, y, z, movable, color: Color32::RED, name });
        icon.movable = movable;
        if movable {
            (icon.x, icon.y, icon.z, icon.color) = (x, y, z, Color32::RED);
        } else {
            (icon.x, icon.y, icon.z, icon.color) = (0.0, 0.0, 0.0, Color32::BLUE);
        }
    }

    fn bounds(&self, icon: &Icon, rect: Rect) -> Rect {
        let (w, h) = (rect.width() as i32, rect.height() as i32);
        // width / xdist and height / ydist are integer divisions in the original
        let (sx, sy) = ((w / self.xdist.max(1)) as f32, (h / self.ydist.max(1)) as f32);
        let v = if self.vertical { icon.z } else { icon.y };
        let x = (w / 2) as f32 + sx * (icon.x - self.centerx) - 10.0;
        let y = (h / 2) as f32 - sy * (v - self.centery) - 10.0;
        Rect::from_min_size(rect.min + Vec2::new(x, y), Vec2::splat(20.0))
    }

    /// OnPaint plus the mouse handling; returns the icon moved (and its x, y, z) and a
    /// right-clicked icon for "Change Alt".
    fn ui(&mut self, ui: &mut egui::Ui, rect: Rect) -> (Option<(Key, f32, f32, f32)>, Option<Key>) {
        self.set_scale(self.xdist, rect.size());
        let resp = ui.interact(rect, ui.id().with("swarm_grid"), Sense::click_and_drag());
        let p = ui.painter_at(rect);
        let font = crate::fonts::ui_id(crate::fonts::UI_SIZE);
        let xline = (rect.width() - 1.0) / self.xdist as f32;
        let yline = (rect.height() - 1.0) / self.ydist as f32;
        let silver = Color32::from_rgb(0xc0, 0xc0, 0xc0);
        let dash = |p: &egui::Painter, a: Pos2, b: Pos2| p.extend(egui::Shape::dashed_line(&[a, b], Stroke::new(1.0, silver), 4.0, 2.0));
        if let Some(tex) = &self.bg {
            let [bx, by, bw, bh] = self.bg_rect;
            let r = Rect::from_min_size(rect.min + Vec2::new(bx * xline, by * yline), Vec2::new(bw * xline, bh * yline));
            p.image(tex.id(), r, Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)), Color32::WHITE);
        }
        for x in 0..=self.xdist {
            let a = rect.min + Vec2::new(x as f32 * xline, 0.0);
            let b = Pos2::new(a.x, rect.max.y);
            if x == self.xdist / 2 {
            } else if x % 2 == 0 {
                p.line_segment([a, b], Stroke::new(1.0, silver));
            } else {
                dash(&p, a, b);
            }
        }
        for y in 0..=self.ydist {
            let a = rect.min + Vec2::new(0.0, y as f32 * yline);
            let b = Pos2::new(rect.max.x, a.y);
            if y as f32 == self.ydist as f32 / 2.0 {
            } else if y % 2 == 0 {
                p.line_segment([a, b], Stroke::new(1.0, silver));
            } else {
                dash(&p, a, b);
            }
        }
        let green = Stroke::new(1.0, Color32::from_rgb(0, 128, 0));
        let cx = rect.min.x + (self.xdist / 2) as f32 * xline;
        let cy = rect.min.y + (self.ydist / 2) as f32 * yline;
        p.line_segment([Pos2::new(cx, rect.min.y), Pos2::new(cx, rect.max.y)], green);
        p.line_segment([Pos2::new(rect.min.x, cy), Pos2::new(rect.max.x, cy)], green);
        for x in (1..=self.xdist).filter(|x| x % 2 == 0) {
            let v = (self.xdist / -2) as f32 + x as f32 + self.centerx;
            p.text(rect.min + Vec2::new(x as f32 * xline, 0.0), egui::Align2::LEFT_TOP, format!("{v:.1}"), font.clone(), Color32::RED);
        }
        for y in (0..=self.ydist).filter(|y| y % 2 == 0) {
            let v = ((self.ydist / -2) as f32 + y as f32 - self.centery) * -1.0;
            p.text(rect.min + Vec2::new(0.0, y as f32 * yline), egui::Align2::LEFT_TOP, format!("{v:.1}"), font.clone(), Color32::RED);
        }
        for icon in self.icons.values() {
            let b = self.bounds(icon, rect);
            // DrawPie(0, 359): the circle with its closing radius
            p.circle_stroke(b.center(), 10.0, Stroke::new(1.0, icon.color));
            p.line_segment([b.center(), Pos2::new(b.right(), b.center().y)], Stroke::new(1.0, icon.color));
            p.text(Pos2::new(b.right(), b.top()), egui::Align2::LEFT_TOP, &icon.name, font.clone(), Color32::RED);
            if !self.vertical {
                p.text(Pos2::new(b.right(), b.bottom()), egui::Align2::LEFT_TOP, icon.z.to_string(), font.clone(), Color32::RED);
            }
        }
        let pointer = resp.interact_pointer_pos().or(resp.hover_pos());
        let hit = |g: &Self, pos: Pos2| g.icons.iter().filter(|(_, i)| g.bounds(i, rect).contains(pos)).map(|(k, _)| *k).last();
        let mut moved = None;
        if resp.drag_started_by(egui::PointerButton::Primary) {
            if let Some(pos) = resp.interact_pointer_pos() {
                self.drag = Some(match hit(self, pos) {
                    Some(k) => Drag::Icon(k),
                    None => Drag::Pan(pos),
                });
            }
        }
        if resp.dragged_by(egui::PointerButton::Primary) {
            if let Some(pos) = pointer {
                let (ex, ey) = (pos.x - rect.min.x, pos.y - rect.min.y);
                match &mut self.drag {
                    Some(Drag::Icon(k)) => {
                        let (xd, yd, vertical, (cx, cy)) = (self.xdist, self.ydist, self.vertical, (self.centerx, self.centery));
                        if let Some(icon) = self.icons.get_mut(k).filter(|i| i.movable) {
                            if vertical {
                                icon.z = ((yd / -2) as f32 + ey / yline) * -1.0 + cy;
                            } else {
                                icon.x = (xd / -2) as f32 + ex / xline + cx;
                                icon.y = ((yd / -2) as f32 + ey / yline) * -1.0 + cy;
                            }
                            moved = Some((*k, icon.x, icon.y, icon.z));
                        }
                    }
                    Some(Drag::Pan(last)) => {
                        let d = pos - *last;
                        self.centerx -= d.x / xline;
                        self.centery += d.y / yline;
                        *last = pos;
                    }
                    None => {}
                }
            }
        }
        if resp.drag_stopped() {
            self.drag = None;
        }
        if resp.secondary_clicked() {
            if let Some(pos) = resp.interact_pointer_pos() {
                self.hover = hit(self, pos);
            }
        } else if self.drag.is_none() && ui.rect_contains_pointer(rect) {
            self.hover = pointer.and_then(|pos| hit(self, pos));
        }
        let mut alt = None;
        resp.context_menu(|ui| {
            if ui.button(crate::i18n::t("Change Alt")).clicked() {
                alt = self.hover;
                ui.close();
            }
        });
        let r = Rect::from_min_size(rect.max - Vec2::new(64.0, 20.0), Vec2::new(61.0, 17.0));
        ui.put(r, egui::Checkbox::new(&mut self.vertical, crate::i18n::t("Vertical")));
        (moved, alt)
    }
}

/// FormationControl (Swarm)
pub struct FormationControl {
    pub open: bool,
    offsets: HashMap<Key, (f64, f64, f64)>,
    leader: Option<Key>,
    selected: Option<Key>,
    running: bool,
    last_send: Instant,
    pids: HashMap<Key, [Pid; 4]>,
    grid: Grid,
    /// MAVState.GuidedMode: the last target sent to each follower
    guided: HashMap<Key, (f64, f64, f64)>,
    alt_prompt: Option<(Key, String)>,
    start_enabled: bool,
    shown: bool,
}

impl Default for FormationControl {
    fn default() -> Self {
        Self {
            open: false,
            offsets: HashMap::new(),
            leader: None,
            selected: None,
            running: false,
            last_send: Instant::now(),
            pids: HashMap::new(),
            grid: Grid::default(),
            guided: HashMap::new(),
            alt_prompt: None,
            start_enabled: false,
            shown: false,
        }
    }
}

impl FormationControl {
    pub fn show(&mut self) {
        *self = Self { open: true, ..Default::default() };
    }

    fn set_offsets(&mut self, key: Key, x: f64, y: f64, z: f64) {
        self.offsets.insert(key, (x, y, z));
    }

    /// getOffsets: an unknown vehicle reports (count, 0, 0) without storing it.
    fn get_offsets(&self, key: Key) -> (f64, f64, f64) {
        self.offsets.get(&key).copied().unwrap_or((self.offsets.len() as f64, 0.0, 0.0))
    }

    /// updateicons
    fn update_icons(&mut self, mavs: &[Mav]) {
        for m in mavs {
            if Some(m.key) == self.leader {
                self.set_offsets(m.key, 0.0, 0.0, 0.0);
                self.grid.update_icon(m.key, m.key.1.to_string(), 0.0, 0.0, 0.0, false);
            } else {
                let v = self.get_offsets(m.key);
                self.grid.update_icon(m.key, m.key.1.to_string(), v.0 as f32, v.1 as f32, v.2 as f32, true);
            }
        }
    }

    /// BUT_leader_Click with the selected vehicle.
    fn set_leader(&mut self, key: Key, mavs: &[Mav]) {
        let lead = self.get_offsets(key);
        for m in mavs {
            let v = self.get_offsets(m.key);
            self.set_offsets(m.key, v.0 - lead.0, v.1 - lead.1, v.2 - lead.2);
        }
        self.leader = Some(key);
        self.update_icons(mavs);
        self.start_enabled = true;
    }

    /// BUT_Updatepos_Click
    fn update_positions(&mut self, mavs: &[Mav]) {
        let Some(leader) = self.leader.and_then(|k| mavs.iter().find(|m| m.key == k)).cloned() else { return };
        for m in mavs.iter().filter(|m| m.key != leader.key) {
            let o = offset_from_leader(&leader, m);
            if o.0.abs() < 200.0 && o.1.abs() < 200.0 {
                self.grid.update_icon(m.key, m.key.1.to_string(), o.1 as f32, o.0 as f32, o.2 as f32, true);
                self.set_offsets(m.key, o.1, o.0, o.2);
            }
        }
    }

    /// Formation.SendCommand for one follower.
    pub fn follower_command(&mut self, leader: &Mav, mav: &Mav) -> Option<FollowerCommand> {
        let offset = *self.offsets.get(&mav.key)?;
        let (lat, lng, alt) = target_for(leader, offset);
        self.guided.insert(mav.key, (lat, lng, alt));
        if mav.firmware != Firmware::ArduPlane {
            let yaw = ((mav.yaw - leader.yaw).abs() > 3.0).then_some(leader.yaw);
            return Some(FollowerCommand::Position { lat, lng, alt, vel: (leader.vx, leader.vy, leader.vz), yaw });
        }
        let here = (mav.lat, mav.lng);
        let target = (lat, lng);
        let mut dist = distance(target, here);
        let trailer = newpos(target, leader.yaw, dist.abs() * -0.25);
        let lead_point = newpos(target, leader.yaw, 10.0 + dist);
        let yawerror = if dist < 100.0 {
            let targyaw = bearing(here, lead_point);
            if wrap_180(bearing(here, target) - targyaw).abs() > 45.0 {
                dist *= -1.0;
            }
            wrap_180(targyaw - mav.yaw)
        } else {
            wrap_180(bearing(here, trailer) - mav.yaw)
        };
        let pid = self.pids.entry(mav.key).or_insert_with(|| {
            [
                Pid::new(1.0, 0.03, 0.02, 10.0, 20.0, 0.1),
                Pid::new(1.0, 0.03, 0.02, 10.0, 20.0, 0.1),
                Pid::new(1.0, 0.0, 0.0, 15.0, 20.0, 0.1),
                Pid::new(0.01, 0.001, 0.0, 0.5, 20.0, 0.1),
            ]
        });
        let mut type_mask: u8 = 0xff;
        type_mask -= 0b0000_0010;
        pid[1].set_input_filter_all((alt - mav.alt) as f32);
        let newpitch = pid[1].get_pid() as f64;
        let mavturnradius = leader.radius() - offset.0;
        let dist_to_target = distance(here, target);
        let mut bearing_to_target = bearing(here, target);
        if dist_to_target < 30.0 {
            bearing_to_target = bearing(here, lead_point);
        }
        if dist_to_target > 100.0 {
            bearing_to_target = bearing(here, trailer);
        }
        let bearing_delta = wrap_180(bearing_to_target - mav.yaw);
        let tangent90 = if bearing_delta > 0.0 { 90.0 } else { -90.0 };
        let mut newroll = 0.0;
        if bearing_delta.abs() < 85.0 {
            let inside = (tangent90 - bearing_delta).abs();
            let center = 180.0 - inside * 2.0;
            let sine1 = dist_to_target.max(40.0) / center.to_radians().sin();
            let radius = ((sine1 * inside.to_radians().sin()).abs() + mavturnradius.abs()) / 2.0;
            let bank = (mav.groundspeed * mav.groundspeed / radius / 9.8).to_degrees();
            newroll = if bearing_delta > 0.0 { bank.abs() } else { -bank.abs() };
        }
        newroll += bearing_delta.clamp(-20.0, 20.0);
        type_mask -= 0b0100_0000;
        pid[3].set_input_filter_all(dist as f32);
        if dist > 40.0 {
            pid[3].reset_i();
        }
        let thrust = (pid[3].get_pid() as f64).clamp(0.1, 1.0);
        let q = quaternion_euler312(newroll.to_radians(), newpitch.to_radians(), yawerror.to_radians());
        type_mask -= 0b1000_0101;
        Some(FollowerCommand::Attitude { q, thrust, type_mask })
    }

    fn send(&mut self, links: &[&Link], mavs: &[Mav]) {
        let Some(leader) = self.leader.and_then(|k| mavs.iter().find(|m| m.key == k)).cloned() else { return };
        if leader.lat == 0.0 || leader.lng == 0.0 {
            return;
        }
        for m in mavs.iter().filter(|m| m.key != leader.key) {
            let Some(l) = links.get(m.key.0) else { continue };
            match self.follower_command(&leader, m) {
                Some(FollowerCommand::Position { lat, lng, alt, vel, yaw }) => {
                    // FORCE and position/velocity enabled
                    l.send(LinkCommand::Send(Box::new(MavMessage::SET_POSITION_TARGET_GLOBAL_INT(SET_POSITION_TARGET_GLOBAL_INT_DATA {
                        lat_int: (lat * 1e7) as i32,
                        lon_int: (lng * 1e7) as i32,
                        alt: alt as f32,
                        vx: vel.0 as f32,
                        vy: vel.1 as f32,
                        vz: vel.2 as f32,
                        target_system: m.key.1,
                        target_component: m.key.2,
                        coordinate_frame: MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                        type_mask: PositionTargetTypemask::from_bits_truncate(u16::MAX - (1 << 9) - 0b111 - 0b111_000),
                        ..Default::default()
                    }))));
                    if let Some(y) = yaw {
                        command(l, m.key.1, m.key.2, MavCmd::MAV_CMD_CONDITION_YAW, [y as f32, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
                    }
                }
                Some(FollowerCommand::Attitude { q, thrust, type_mask }) => {
                    l.send(LinkCommand::Send(Box::new(MavMessage::SET_ATTITUDE_TARGET(SET_ATTITUDE_TARGET_DATA {
                        q: q.map(|v| v as f32),
                        thrust: thrust as f32,
                        target_system: m.key.1,
                        target_component: m.key.2,
                        type_mask: AttitudeTargetTypemask::from_bits_truncate(type_mask),
                        ..Default::default()
                    }))));
                }
                None => {}
            }
        }
    }

    pub fn ui(&mut self, ctx: &egui::Context, links: &[&Link], toasts: &mut Toasts) {
        if !self.open {
            self.running = false;
            return;
        }
        let mavs = vehicles(links);
        if !self.shown {
            self.shown = true;
            if !mavs.is_empty() {
                self.selected = mavs.first().map(|m| m.key);
                self.update_icons(&mavs);
                toasts.error("this is beta, use at own risk");
            }
        }
        // mainloop: Update + SendCommand every 100 ms while running
        if self.running && self.last_send.elapsed() >= Duration::from_millis(100) {
            self.last_send = Instant::now();
            self.send(links, &mavs);
        }
        if self.running {
            ctx.request_repaint_after(Duration::from_millis(100));
        }
        let mut open = true;
        let by_key = |k: Key| mavs.iter().find(|m| m.key == k);
        egui::Window::new(crate::i18n::t("Control")).id(egui::Id::new("swarm_formation")).open(&mut open).collapsible(false).resizable(false).order(egui::Order::Foreground).fixed_size([931.0, 471.0]).show(ctx, |ui| {
            let (area, _) = ui.allocate_exact_size(Vec2::new(931.0, 471.0), Sense::hover());
            let o = area.min;
            let r = |x: f32, y: f32, w: f32, h: f32| Rect::from_min_size(o + Vec2::new(x, y), Vec2::new(w, h));
            let button = |ui: &mut egui::Ui, rect: Rect, text: &str, enabled: bool| -> bool {
                ui.put(rect, |ui: &mut egui::Ui| ui.add_enabled_ui(enabled, |ui| theme::button_sized(ui, text, rect.size())).inner).clicked()
            };
            let followers = |leader: Option<Key>| mavs.iter().filter(move |m| Some(m.key) != leader);
            if button(ui, r(12.0, 12.0, 75.0, 23.0), "Arm (exl leader)", true) {
                for m in followers(self.leader) {
                    command(links[m.key.0], m.key.1, m.key.2, MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
                }
            }
            if button(ui, r(93.0, 12.0, 75.0, 23.0), "Disarm (exl leader)", true) {
                for m in followers(self.leader) {
                    command(links[m.key.0], m.key.1, m.key.2, MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, [0.0; 7]);
                }
            }
            if button(ui, r(174.0, 12.0, 75.0, 23.0), "Takeoff", true) {
                for m in followers(self.leader) {
                    set_mode(links[m.key.0], m, "GUIDED");
                    command(links[m.key.0], m.key.1, m.key.2, MavCmd::MAV_CMD_NAV_TAKEOFF, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0]);
                }
            }
            if button(ui, r(255.0, 12.0, 75.0, 23.0), "Land (all)", true) {
                for m in &mavs {
                    set_mode(links[m.key.0], m, "Land");
                }
            }
            // CMB_mavs: "PortName sysid compid"
            let label = |m: &Mav| format!("{} {} {}", m.port, m.key.1, m.key.2);
            let cur = self.selected.and_then(by_key).map(label).unwrap_or_default();
            let cr = r(336.0, 12.0, 121.0, 21.0);
            ui.put(cr, |ui: &mut egui::Ui| {
                egui::ComboBox::from_id_salt("CMB_mavs").width(cr.width() - 8.0).selected_text(cur).show_ui(ui, |ui| {
                    for m in &mavs {
                        if ui.selectable_label(self.selected == Some(m.key), label(m)).clicked() {
                            self.selected = Some(m.key);
                            // MainV2.comPort / sysidcurrent follow the selection
                            links[m.key.0].send(LinkCommand::SetTarget { sysid: m.key.1, compid: m.key.2 });
                        }
                    }
                }).response
            });
            if button(ui, r(463.0, 12.0, 75.0, 23.0), "Set Leader", true) {
                if let Some(k) = self.selected {
                    self.set_leader(k, &mavs);
                }
            }
            if button(ui, r(544.0, 12.0, 75.0, 23.0), "Guided Mode (exl leader)", true) {
                for m in followers(self.leader) {
                    set_mode(links[m.key.0], m, "GUIDED");
                }
            }
            if button(ui, r(625.0, 12.0, 75.0, 23.0), "Auto Mode  (exl leader)", true) {
                for m in followers(self.leader) {
                    set_mode(links[m.key.0], m, "AUTO");
                }
            }
            if button(ui, r(706.0, 12.0, 75.0, 23.0), "Update Pos", self.start_enabled) {
                self.update_positions(&mavs);
            }
            if button(ui, r(787.0, 12.0, 75.0, 23.0), if self.running { "Stop" } else { "Start" }, self.start_enabled) {
                self.running = !self.running;
                if self.running {
                    // the leader's position stream at 10 Hz
                    if let Some(lead) = self.leader {
                        if let Some(l) = links.get(lead.0) {
                            l.send(LinkCommand::Send(Box::new(MavMessage::REQUEST_DATA_STREAM(REQUEST_DATA_STREAM_DATA {
                                req_message_rate: 10,
                                target_system: lead.1,
                                target_component: lead.2,
                                req_stream_id: MavDataStream::MAV_DATA_STREAM_POSITION,
                                start_stop: 1,
                            }))));
                        }
                    }
                    self.last_send = Instant::now() - Duration::from_millis(100);
                }
            }
            // tabControl1 / "Stage 1" / grid1
            let tab = r(12.0, 39.0, 769.0, 420.0);
            ui.painter().rect_stroke(tab.with_min_y(tab.min.y + 22.0), 0.0, Stroke::new(1.0, Color32::from_gray(0x82)), egui::StrokeKind::Inside);
            ui.painter().text(tab.min + Vec2::new(8.0, 4.0), egui::Align2::LEFT_TOP, crate::i18n::t("Stage 1"), crate::fonts::ui_id(crate::fonts::UI_SIZE), Color32::WHITE);
            let grid_rect = r(19.0, 64.0, 755.0, 388.0);
            if ui.rect_contains_pointer(grid_rect) {
                let wheel: f32 = ui.input(|i| i.events.iter().filter_map(|e| if let egui::Event::MouseWheel { delta, .. } = e { Some(delta.y) } else { None }).sum());
                if wheel != 0.0 {
                    let scale = self.grid.xdist + if wheel < 0.0 { 4 } else { -4 };
                    self.grid.set_scale(scale, grid_rect.size());
                }
            }
            let (moved, alt) = self.grid.ui(ui, grid_rect);
            if let Some((k, x, y, z)) = moved {
                if Some(k) == self.leader {
                    toasts.error("Can not move Leader");
                } else {
                    self.set_offsets(k, x as f64, y as f64, z as f64);
                }
            }
            if let Some(k) = alt {
                let z = self.grid.icons.get(&k).map(|i| i.z).unwrap_or(0.0);
                self.alt_prompt = Some((k, z.to_string()));
            }
            // PNL_status: (783, 61, 147, 398) FlowLayoutPanel with AutoScroll
            let pnl_rect = r(783.0, 61.0, 147.0, 398.0);
            let mut pnl_ui = ui.new_child(egui::UiBuilder::new().max_rect(pnl_rect));
            egui::ScrollArea::vertical().id_salt("swarm_formation_pnl_status").show(&mut pnl_ui, |ui| {
                for m in &mavs {
                    let (card_rect, _) = ui.allocate_exact_size(Vec2::new(113.0, 128.0), Sense::hover());
                    let color = if Some(m.key) == self.leader { Color32::RED } else { Color32::WHITE };
                    let f = crate::fonts::ui_id(crate::fonts::UI_SIZE);
                    let at = |x: f32, yy: f32| card_rect.min + Vec2::new(x, yy);
                    let p = ui.painter();
                    let guided = self.guided.get(&m.key).copied().unwrap_or_default();
                    for (x, yy, t) in [
                        (3.0, 0.0, m.key.1.to_string()),
                        (3.0, 13.0, "Armed".into()),
                        (46.0, 13.0, if m.armed { "True" } else { "False" }.into()),
                        (3.0, 26.0, "Mode".into()),
                        (46.0, 26.0, m.mode.clone()),
                        (3.0, 39.0, "GPS".into()),
                        (46.0, 39.0, if m.gpsstatus >= 3 { "OK" } else { "Bad" }.into()),
                        (3.0, 52.0, "Guided".into()),
                        (3.0, 82.0, "Location".into()),
                        (3.0, 112.0, "Speed".into()),
                        (46.0, 112.0, format!("{:.2}", m.groundspeed)),
                    ] {
                        p.text(at(x, yy), egui::Align2::LEFT_TOP, t, f.clone(), color);
                    }
                    for (yy, t) in [
                        (52.0, format!("{:.5},{:.5},{:.2}", guided.0, guided.1, guided.2)),
                        (82.0, format!("{:.5},{:.5},{:.2}", m.lat, m.lng, m.alt)),
                    ] {
                        let mut job = egui::text::LayoutJob::simple(t, f.clone(), color, 67.0);
                        job.wrap.break_anywhere = true;
                        let galley = ui.painter().layout_job(job);
                        ui.painter().with_clip_rect(Rect::from_min_size(at(46.0, yy), Vec2::new(67.0, 30.0))).galley(at(46.0, yy), galley, color);
                    }
                    ui.add_space(4.0);
                }
            });
        });
        if !open {
            // Control_FormClosing
            self.open = false;
            self.running = false;
        }
        if let Some((k, mut text)) = self.alt_prompt.take() {
            let mut done = None;
            egui::Modal::new(egui::Id::new("swarm_alt")).show(ctx, |ui| {
                ui.heading(crate::i18n::t("Alt"));
                theme::label(ui, "Enter New Alt");
                ui.text_edit_singleline(&mut text);
                ui.horizontal(|ui| {
                    if theme::button(ui, "OK").clicked() {
                        done = Some(true);
                    }
                    if theme::button(ui, "Cancel").clicked() {
                        done = Some(false);
                    }
                });
            });
            match done {
                Some(true) => {
                    if let (Ok(z), Some(icon)) = (text.trim().parse::<f32>(), self.grid.icons.get_mut(&k)) {
                        icon.z = z;
                        let (x, y) = (icon.x, icon.y);
                        if Some(k) == self.leader {
                            toasts.error("Can not move Leader");
                            icon.z = 0.0;
                        } else {
                            self.set_offsets(k, x as f64, y as f64, z as f64);
                        }
                    }
                }
                Some(false) => {}
                None => self.alt_prompt = Some((k, text)),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn mav(key: Key, lat: f64, lng: f64, yaw: f64) -> Mav {
        Mav { key, lat, lng, alt: 20.0, yaw, firmware: Firmware::ArduCopter2, gpsstatus: 3, ..Default::default() }
    }

    #[test]
    fn offsets_round_trip_through_the_leader_frame() {
        for yaw in [0.0, 37.0, 90.0, 251.0] {
            let leader = mav((0, 1, 1), -35.363261, 149.165230, yaw);
            let follower = mav((0, 2, 1), -35.36310, 149.16540, 0.0);
            let o = offset_from_leader(&leader, &follower);
            // Update Pos stores (y, x) as the formation offset
            let (lat, lng, alt) = target_for(&leader, (o.1, o.0, 5.0));
            assert!((lat - follower.lat).abs() < 1e-7 && (lng - follower.lng).abs() < 1e-7, "yaw {yaw}: {lat} {lng}");
            assert_eq!(alt, 25.0);
        }
        // with north up, an offset of x metres moves the target x metres east
        let leader = mav((0, 1, 1), -35.0, 149.0, 0.0);
        let (lat, lng, _) = target_for(&leader, (10.0, 0.0, 0.0));
        assert!((distance((lat, lng), (-35.0, 149.0)) - 10.0).abs() < 0.05);
        // grid east: UTM convergence 2 degrees from the zone meridian at 35S is about 1.15 degrees
        assert!((bearing((-35.0, 149.0), (lat, lng)) - 90.0).abs() < 1.5);
    }

    #[test]
    fn set_leader_rebases_offsets_and_followers_get_position_targets() {
        let mut f = FormationControl::default();
        let mavs = [mav((0, 1, 1), -35.0, 149.0, 90.0), mav((0, 2, 1), -35.0001, 149.0, 0.0), mav((1, 3, 1), -35.0002, 149.0, 92.0)];
        f.set_offsets((0, 2, 1), 5.0, -3.0, 2.0);
        f.set_leader((0, 1, 1), &mavs);
        // the leader had no offset: getOffsets gave (count, 0, 0) = (1, 0, 0)
        assert_eq!(f.offsets[&(0, 1, 1)], (0.0, 0.0, 0.0));
        assert_eq!(f.offsets[&(0, 2, 1)], (4.0, -3.0, 2.0));
        assert!(!f.grid.icons[&(0, 1, 1)].movable && f.grid.icons[&(0, 2, 1)].movable);
        let leader = mavs[0].clone();
        match f.follower_command(&leader, &mavs[1]).unwrap() {
            FollowerCommand::Position { alt, yaw, .. } => {
                assert_eq!(alt, 22.0);
                assert_eq!(yaw, Some(90.0), "a heading 90 degrees off gets CONDITION_YAW");
            }
            other => panic!("{other:?}"),
        }
        match f.follower_command(&leader, &mavs[2]).unwrap() {
            FollowerCommand::Position { yaw, .. } => assert_eq!(yaw, None, "within 3 degrees"),
            other => panic!("{other:?}"),
        }
        assert!(f.guided.contains_key(&(0, 2, 1)));
        let mut plane = mavs[1].clone();
        plane.firmware = Firmware::ArduPlane;
        plane.groundspeed = 20.0;
        match f.follower_command(&leader, &plane).unwrap() {
            FollowerCommand::Attitude { q, thrust, type_mask } => {
                assert_eq!(type_mask, 0b0011_1000);
                assert!((0.1..=1.0).contains(&thrust));
                assert!(((q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]) - 1.0).abs() < 1e-9);
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn grid_scale_and_icon_rules_match_the_original() {
        let mut g = Grid::default();
        g.set_scale(40, Vec2::new(755.0, 388.0));
        assert_eq!((g.xdist, g.ydist), (40, 20));
        g.set_scale(6, Vec2::new(755.0, 388.0));
        assert_eq!(g.xdist, 40, "setScale ignores 6 and below");
        g.update_icon((0, 1, 1), "1".into(), 3.0, 4.0, 5.0, false);
        let i = &g.icons[&(0, 1, 1)];
        assert_eq!((i.x, i.y, i.z, i.color), (0.0, 0.0, 0.0, Color32::BLUE));
        let q = quaternion_euler312(0.0, 0.0, 0.0);
        assert_eq!(q, [1.0, 0.0, 0.0, 0.0]);
    }
}

#[cfg(test)]
mod sim_tests {
    use super::*;
    use mp_core::sim;

    /// Three simulated Copters on one connection: GUIDED, arm and take off the followers, set
    /// the leader, then run Formation.SendCommand until each follower holds its offset.
    #[test]
    fn followers_fly_to_their_formation_offsets() {
        let port = std::net::UdpSocket::bind("127.0.0.1:0").unwrap().local_addr().unwrap().port();
        let link = Link::connect_with_params(ConnectionSpec::UdpListen { port }, None, false);
        let _sims: Vec<_> = (1..=3u8)
            .map(|i| sim::spawn_with(sim::SimOptions { gcs_port: port, sysid: i, kind: sim::SimKind::Copter, offset: (0.0, i as f64 * 10.0) }).unwrap())
            .collect();
        let links = [&link];
        let wait = |what: &str, secs: u64, f: &dyn Fn(&[Mav]) -> bool, retry: &dyn Fn(&[Mav])| {
            let end = Instant::now() + Duration::from_secs(secs);
            let mut next = Instant::now();
            loop {
                let mavs = vehicles(&links);
                if f(&mavs) {
                    return mavs;
                }
                assert!(Instant::now() < end, "timed out: {what}: {mavs:?}");
                if Instant::now() >= next {
                    retry(&mavs);
                    next = Instant::now() + Duration::from_secs(1);
                }
                std::thread::sleep(Duration::from_millis(50));
            }
        };
        let positioned = |m: &[Mav]| m.len() == 3 && m.iter().all(|v| v.lat != 0.0 && v.gpsstatus >= 3);
        wait("three vehicles with positions", 15, &positioned, &|_| {});
        // Takeoff: GUIDED + arm + TAKEOFF for every vehicle (the leader too, so it is airborne)
        wait("everyone flying", 30, &|m| m.iter().all(|v| v.alt > 4.0), &|m| {
            for v in m {
                if v.mode != "Guided" {
                    set_mode(&link, v, "GUIDED");
                } else if !v.armed {
                    command(&link, v.key.1, v.key.2, MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
                } else {
                    command(&link, v.key.1, v.key.2, MavCmd::MAV_CMD_NAV_TAKEOFF, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0]);
                }
            }
        });
        let mut f = FormationControl::default();
        let leader = (0, 1, 1);
        f.set_offsets((0, 2, 1), 15.0, 0.0, 0.0);
        f.set_offsets((0, 3, 1), -15.0, -10.0, 5.0);
        let mavs = vehicles(&links);
        f.set_leader(leader, &mavs);
        let end = Instant::now() + Duration::from_secs(30);
        loop {
            let mavs = vehicles(&links);
            f.send(&links, &mavs);
            let lead = mavs.iter().find(|m| m.key == leader).unwrap().clone();
            let errors: Vec<f64> = mavs
                .iter()
                .filter(|m| m.key != leader)
                .map(|m| {
                    let (lat, lng, alt) = target_for(&lead, f.offsets[&m.key]);
                    distance((lat, lng), (m.lat, m.lng)).hypot(alt - m.alt)
                })
                .collect();
            if errors.iter().all(|e| *e < 3.0) {
                break;
            }
            assert!(Instant::now() < end, "followers still off their slots: {errors:?}");
            std::thread::sleep(Duration::from_millis(100));
        }
    }
}

#[cfg(test)]
mod ui_tests {
    use super::*;

    #[test]
    fn window_frames_without_input_leave_the_grid_centred() {
        let ctx = egui::Context::default();
        let mut f = FormationControl::default();
        f.show();
        let mut toasts = Toasts::default();
        for _ in 0..5 {
            let input = egui::RawInput { screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(1280.0, 800.0))), ..Default::default() };
            let mut out = ctx.run_ui(input, |ui| f.ui(ui.ctx(), &[], &mut toasts));
            out.textures_delta.clear();
        }
        assert_eq!((f.grid.centerx, f.grid.centery, f.grid.xdist), (0.0, 0.0, 40));
    }

    #[test]
    fn hovering_icons_does_not_pan() {
        let ctx = egui::Context::default();
        let mut g = Grid::default();
        g.update_icon((0, 1, 1), "1".into(), 0.0, 0.0, 0.0, true);
        g.update_icon((0, 2, 1), "2".into(), 0.0, 0.0, 0.0, true);
        for i in 0..20 {
            let pos = Pos2::new(300.0 + i as f32 * 7.0, 200.0 + i as f32 * 3.0);
            let input = egui::RawInput { screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(1280.0, 800.0))), events: vec![egui::Event::PointerMoved(pos)], ..Default::default() };
            let mut out = ctx.run_ui(input, |ui| {
                g.ui(ui, Rect::from_min_size(Pos2::new(19.0, 64.0), Vec2::new(755.0, 388.0)));
            });
            out.textures_delta.clear();
        }
        assert_eq!((g.centerx, g.centery), (0.0, 0.0));
    }
}


/// Swarm/Sequence/Layout.cs: a named set of per-sysid offsets (east, north, alt).
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Layout {
    #[serde(rename = "Id")]
    pub id: String,
    #[serde(rename = "DelayStart", default)]
    pub delay_start: i32,
    #[serde(rename = "DelayEnd", default)]
    pub delay_end: i32,
    #[serde(rename = "Offset", default)]
    pub offset: BTreeMap<u32, Xyz>,
}

/// Utilities.Vector3 as Newtonsoft writes it.
#[derive(Clone, Copy, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Xyz {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// Swarm/Sequence: the layouts and the ordered steps that name them.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Sequence {
    #[serde(rename = "Layouts", default)]
    pub layouts: Vec<Layout>,
    #[serde(rename = "Steps", default)]
    pub steps: Vec<String>,
}

impl Sequence {
    pub fn load(path: &std::path::Path) -> Option<Self> {
        serde_json::from_str(&std::fs::read_to_string(path).ok()?).ok()
    }

    /// Formatting.Indented
    pub fn save(&self, path: &std::path::Path) -> std::io::Result<()> {
        std::fs::write(path, serde_json::to_string_pretty(self).unwrap_or_default())
    }
}

/// num_drones_ValueChanged: shrinking removes the sysid equal to the old count, growing adds
/// max + 1 at (sysid, 0, 0) to every layout.
pub fn set_drone_count(seq: &mut Sequence, value: u32, known: &mut std::collections::BTreeSet<u32>) {
    let count = seq.layouts.first().map(|l| l.offset.len() as u32).unwrap_or(0);
    if value == count {
        return;
    }
    if value < count {
        for l in &mut seq.layouts {
            l.offset.remove(&count);
        }
    } else {
        let sysid = seq.layouts.first().and_then(|l| l.offset.keys().max().copied()).map(|m| m + 1).unwrap_or(1);
        for l in &mut seq.layouts {
            l.offset.insert(sysid, Xyz { x: sysid as f64, y: 0.0, z: 0.0 });
        }
        known.insert(sysid);
    }
}

/// Run Step: each drone in the layout goes to startpos.gps_offset(x east, y north) at z.
pub fn step_targets(layout: &Layout, start: (f64, f64)) -> Vec<(u32, f64, f64, f64)> {
    layout
        .offset
        .iter()
        .map(|(sysid, v)| {
            let bearing = v.x.atan2(v.y).to_degrees();
            let dist = v.x.hypot(v.y);
            let (lat, lng) = newpos(start, bearing, dist);
            (*sysid, lat, lng, v.z)
        })
        .collect()
}

/// Sequence/LayoutEditor ("Sequence Swarm")
enum SequenceFile { Load, Image, Save }

#[derive(Default)]
pub struct LayoutEditor {
    pub open: bool,
    seq: Sequence,
    working: Option<usize>,
    /// the sysids that have icons (mavs)
    known: std::collections::BTreeSet<u32>,
    num_drones: u32,
    step: usize,
    label: String,
    startpos: Option<(f64, f64)>,
    selected_step: Option<usize>,
    dragging_step: Option<usize>,
    grid: Grid,
    name_prompt: Option<String>,
    /// "Move Image" button list
    image_buttons: bool,
    /// DroneGroup.CurrentMode: position streams requested once per Controller.Start
    streams_requested: bool,
    file_picker: Option<(SequenceFile, crate::platform::FilePicker)>,
}

impl LayoutEditor {
    pub fn show(&mut self) {
        *self = Self { open: true, num_drones: 1, label: "label1".into(), ..Default::default() };
    }

    /// UpdateDisplay
    fn update_display(&mut self) {
        let Some(l) = self.working.and_then(|i| self.seq.layouts.get(i)) else { return };
        self.grid.clear();
        for (sysid, v) in &l.offset {
            let key = (0, *sysid as u8, 0);
            self.grid.update_icon(key, sysid.to_string(), v.x as f32, v.y as f32, v.z as f32, true);
        }
    }

    /// Controller.Start: every vehicle on every connection, position streams at 10 Hz.
    fn start(&mut self, links: &[&Link]) -> Vec<Mav> {
        let mavs = vehicles(links);
        if !self.streams_requested {
            self.streams_requested = true;
            for m in &mavs {
                links[m.key.0].send(LinkCommand::Send(Box::new(MavMessage::REQUEST_DATA_STREAM(REQUEST_DATA_STREAM_DATA {
                    req_message_rate: 10,
                    target_system: m.key.1,
                    target_component: m.key.2,
                    req_stream_id: MavDataStream::MAV_DATA_STREAM_POSITION,
                    start_stop: 1,
                }))));
            }
        }
        mavs
    }

    fn run_step(&mut self, links: &[&Link]) {
        if self.step >= self.seq.steps.len() {
            return;
        }
        let name = self.seq.steps[self.step].clone();
        let mavs = self.start(links);
        if self.startpos.is_none() {
            self.startpos = mavs.first().map(|m| (m.lat, m.lng));
        }
        self.label = format!("{name} : {}", self.step);
        let Some(layout) = self.seq.layouts.iter().find(|l| l.id == name) else { return };
        for (sysid, lat, lng, alt) in step_targets(layout, self.startpos.unwrap_or_default()) {
            if let Some(m) = mavs.iter().find(|m| m.key.1 as u32 == sysid) {
                // SendPositionVelocity(newpos, Vector3.Zero)
                links[m.key.0].send(LinkCommand::Send(Box::new(MavMessage::SET_POSITION_TARGET_GLOBAL_INT(SET_POSITION_TARGET_GLOBAL_INT_DATA {
                    lat_int: (lat * 1e7) as i32,
                    lon_int: (lng * 1e7) as i32,
                    alt: alt as f32,
                    target_system: m.key.1,
                    target_component: m.key.2,
                    coordinate_frame: MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    type_mask: PositionTargetTypemask::from_bits_truncate(u16::MAX - (1 << 9) - 0b111 - 0b111_000),
                    ..Default::default()
                }))));
            }
        }
        self.step += 1;
    }

    fn takeoff(&mut self, links: &[&Link]) {
        self.streams_requested = false;
        let mavs = self.start(links);
        for m in &mavs {
            if !m.mode.eq_ignore_ascii_case("guided") {
                set_mode(links[m.key.0], m, "GUIDED");
            }
        }
        for m in &mavs {
            if !m.armed {
                command(links[m.key.0], m.key.1, m.key.2, MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
            }
        }
        for m in &mavs {
            command(links[m.key.0], m.key.1, m.key.2, MavCmd::MAV_CMD_NAV_TAKEOFF, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0]);
        }
    }

    fn select_layout(&mut self, i: usize) {
        self.working = Some(i);
        self.grid.clear();
        self.update_display();
    }

    pub fn ui(&mut self, ctx: &egui::Context, links: &[&Link], toasts: &mut Toasts) {
        if !self.open {
            return;
        }
        let _ = &toasts;
        let mut open = true;
        egui::Window::new(crate::i18n::t("Layout Editor")).id(egui::Id::new("swarm_layout_editor")).open(&mut open).collapsible(false).resizable(false).fixed_size([899.0, 494.0]).show(ctx, |ui| {
            let (area, _) = ui.allocate_exact_size(Vec2::new(899.0, 494.0), Sense::hover());
            let o = area.min;
            let r = |x: f32, y: f32, w: f32, h: f32| Rect::from_min_size(o + Vec2::new(x, y), Vec2::new(w, h));
            let button = |ui: &mut egui::Ui, rect: Rect, text: &str| ui.put(rect, |ui: &mut egui::Ui| theme::button_sized(ui, text, rect.size())).clicked();
            if button(ui, r(12.0, 3.0, 75.0, 23.0), "Load") {
                self.file_picker = Some((SequenceFile::Load, crate::platform::FilePicker::new("Load sequence", &["txt", "json"], None)));
            }
            // comboBox1: the layouts by Id
            let cur = self.working.and_then(|i| self.seq.layouts.get(i)).map(|l| l.id.clone()).unwrap_or_default();
            let cr = r(93.0, 3.0, 121.0, 21.0);
            let mut pick = None;
            ui.put(cr, |ui: &mut egui::Ui| {
                egui::ComboBox::from_id_salt("seq_layouts").width(cr.width() - 8.0).selected_text(cur.clone()).show_ui(ui, |ui| {
                    for (i, l) in self.seq.layouts.iter().enumerate() {
                        if ui.selectable_label(self.working == Some(i), &l.id).clicked() {
                            pick = Some(i);
                        }
                    }
                }).response
            });
            if let Some(i) = pick {
                self.select_layout(i);
            }
            if button(ui, r(220.0, 3.0, 75.0, 23.0), "New Layout") {
                self.name_prompt = Some("Layout X".into());
            }
            let mut n = self.num_drones as f64;
            let nr = r(301.0, 4.0, 75.0, 20.0);
            if theme::numeric_updown_fill(ui, nr, &mut n, 1.0, 1000.0, 1.0, 0, theme::control_bg()).changed() {
                self.num_drones = n.round().clamp(1.0, 1000.0) as u32;
                set_drone_count(&mut self.seq, self.num_drones, &mut self.known);
                self.update_display();
            }
            if button(ui, r(382.0, 3.0, 75.0, 23.0), "Add Step") {
                self.seq.steps.push(cur.clone());
            }
            if button(ui, r(463.0, 3.0, 59.0, 23.0), "Takeoff") {
                self.takeoff(links);
            }
            if button(ui, r(528.0, 3.0, 75.0, 23.0), "Reset") {
                self.step = 0;
            }
            if button(ui, r(609.0, 3.0, 75.0, 23.0), "Run Step") {
                self.run_step(links);
            }
            ui.painter().text(o + Vec2::new(691.0, 10.0), egui::Align2::LEFT_TOP, &self.label, crate::fonts::ui_id(crate::fonts::UI_SIZE), Color32::WHITE);
            if button(ui, r(690.0, 26.0, 75.0, 23.0), "set image") {
                self.file_picker = Some((SequenceFile::Image, crate::platform::FilePicker::new("Background image", &["png", "jpg", "jpeg"], None)));
            }
            if button(ui, r(812.0, 3.0, 75.0, 23.0), "Save") {
                self.file_picker = Some((SequenceFile::Save, crate::platform::FilePicker::new("Save sequence", &["txt"], Some("sequence.txt"))));
            }
            let grid_rect = r(12.0, 52.0, 725.0, 430.0);
            if ui.rect_contains_pointer(grid_rect) {
                let wheel: f32 = ui.input(|i| i.events.iter().filter_map(|e| if let egui::Event::MouseWheel { delta, .. } = e { Some(delta.y) } else { None }).sum());
                if wheel != 0.0 {
                    let scale = self.grid.xdist + if wheel < 0.0 { 4 } else { -4 };
                    self.grid.set_scale(scale, grid_rect.size());
                }
            }
            let (moved, alt) = self.grid.ui(ui, grid_rect);
            // grid_UpdateOffsets: the working layout takes the dragged offset
            if let (Some((k, x, y, z)), Some(l)) = (moved, self.working.and_then(|i| self.seq.layouts.get_mut(i))) {
                l.offset.insert(k.1 as u32, Xyz { x: x as f64, y: y as f64, z: z as f64 });
            }
            if let Some(k) = alt {
                let _ = k;
            }
            // listBox1: the steps; select picks the layout, Delete removes, drag reorders
            let lr = r(744.0, 52.0, 143.0, 433.0);
            ui.painter().rect_filled(lr, 0.0, theme::control_bg());
            ui.painter().rect_stroke(lr, 0.0, Stroke::new(1.0, Color32::from_gray(0x82)), egui::StrokeKind::Inside);
            let row_h = 13.0;
            let mut drop_at = None;
            for (i, step) in self.seq.steps.clone().iter().enumerate() {
                let rr = Rect::from_min_size(lr.min + Vec2::new(1.0, 1.0 + i as f32 * row_h), Vec2::new(lr.width() - 2.0, row_h));
                if rr.bottom() > lr.bottom() {
                    break;
                }
                let resp = ui.interact(rr, ui.id().with(("seq_step", i)), Sense::click_and_drag());
                if self.selected_step == Some(i) {
                    ui.painter().rect_filled(rr, 0.0, Color32::from_rgb(0x33, 0x99, 0xff));
                }
                ui.painter().text(rr.left_center() + Vec2::new(2.0, 0.0), egui::Align2::LEFT_CENTER, step, crate::fonts::ui_id(crate::fonts::UI_SIZE), Color32::WHITE);
                if resp.is_pointer_button_down_on() && self.dragging_step.is_none() {
                    self.selected_step = Some(i);
                    if let Some(li) = self.seq.layouts.iter().position(|l| &l.id == step) {
                        if self.working != Some(li) {
                            self.select_layout(li);
                        }
                    }
                }
                if resp.drag_started() {
                    self.dragging_step = Some(i);
                }
                if self.dragging_step.is_some() && ui.input(|inp| inp.pointer.any_released()) && ui.rect_contains_pointer(rr) {
                    drop_at = Some(i);
                }
            }
            if let Some(from) = self.dragging_step {
                if ui.input(|i| i.pointer.any_released()) {
                    // listBox1_DragDrop: below the last row drops at the end
                    let to = drop_at.unwrap_or(self.seq.steps.len().saturating_sub(1));
                    if from < self.seq.steps.len() {
                        let item = self.seq.steps.remove(from);
                        let to = to.min(self.seq.steps.len());
                        self.seq.steps.insert(to, item);
                        self.selected_step = Some(to);
                    }
                    self.dragging_step = None;
                }
            }
            if ui.input(|i| i.key_released(egui::Key::Delete)) {
                if let Some(i) = self.selected_step.filter(|i| *i < self.seq.steps.len()) {
                    self.seq.steps.remove(i);
                    self.selected_step = None;
                }
            }
        });
        if !open {
            self.open = false;
        }
        if let Some(mut name) = self.name_prompt.take() {
            let mut done = None;
            egui::Modal::new(egui::Id::new("seq_layout_name")).show(ctx, |ui| {
                theme::label(ui, "Layout Name");
                ui.text_edit_singleline(&mut name);
                ui.horizontal(|ui| {
                    if theme::button(ui, "OK").clicked() {
                        done = Some(true);
                    }
                    if theme::button(ui, "Cancel").clicked() {
                        done = Some(false);
                    }
                });
            });
            match done {
                // BUT_new_Click adds the layout whatever the InputBox returns
                Some(_) => {
                    let offset = self.working.and_then(|i| self.seq.layouts.get(i)).map(|l| l.offset.clone()).unwrap_or_default();
                    self.seq.layouts.push(Layout { id: name, offset, ..Default::default() });
                    let i = self.seq.layouts.len() - 1;
                    self.select_layout(i);
                }
                None => self.name_prompt = Some(name),
            }
        }
        if let Some((action, mut picker)) = self.file_picker.take() {
            match picker.show(ctx) {
                None => self.file_picker = Some((action, picker)),
                Some(path) => match action {
                    SequenceFile::Load => {
                        if let Some(load) = path.and_then(|p| Sequence::load(&p)) {
                            self.seq = load;
                            if let Some(first) = self.seq.layouts.first() {
                                self.num_drones = first.offset.len() as u32;
                                self.known = first.offset.keys().copied().collect();
                                self.select_layout(0);
                            }
                        }
                    }
                    SequenceFile::Save => {
                        if let Some(path) = path { if let Err(e) = self.seq.save(&path) { toasts.error(e.to_string()); } }
                    }
                    SequenceFile::Image => {
                        if let Some(path) = path {
                            match image::open(path) {
                                Ok(img) => {
                                    let rgba = img.to_rgba8();
                                    let size = [rgba.width() as usize, rgba.height() as usize];
                                    let color = egui::ColorImage::from_rgba_unmultiplied(size, rgba.as_raw());
                                    self.grid.bg = Some(ctx.load_texture("swarm_bg", color, Default::default()));
                                    self.image_buttons = true;
                                }
                                Err(e) => toasts.error(e.to_string()),
                            }
                        }
                    }
                },
            }
        }
        if self.image_buttons {
            let mut open = true;
            egui::Window::new(crate::i18n::t("Move Image")).id(egui::Id::new("swarm_move_image")).open(&mut open).resizable(false).show(ctx, |ui| {
                let g = &mut self.grid;
                let s = g.bg_step;
                for (text, f) in [
                    ("Up", [0.0, -s, 0.0, 0.0]),
                    ("Down", [0.0, s, 0.0, 0.0]),
                    ("Left", [-s, 0.0, 0.0, 0.0]),
                    ("Right", [s, 0.0, 0.0, 0.0]),
                    ("X+", [0.0, 0.0, s, 0.0]),
                    ("X-", [0.0, 0.0, -s, 0.0]),
                    ("Y+", [0.0, 0.0, 0.0, s]),
                    ("Y-", [0.0, 0.0, 0.0, -s]),
                ] {
                    if theme::button(ui, text).clicked() {
                        for (v, d) in g.bg_rect.iter_mut().zip(f) {
                            *v += d;
                        }
                    }
                }
                if theme::button(ui, "Scale 0.1").clicked() {
                    g.bg_step = 0.1;
                }
                if theme::button(ui, "Scale 1").clicked() {
                    g.bg_step = 1.0;
                }
            });
            self.image_buttons = open;
        }
    }
}

#[cfg(test)]
mod sequence_tests {
    use super::*;

    #[test]
    fn cancelling_background_selection_keeps_the_existing_image() {
        let ctx = egui::Context::default();
        let mut editor = LayoutEditor::default();
        editor.show();
        editor.grid.bg = Some(ctx.load_texture("existing_background", egui::ColorImage::from_rgba_unmultiplied([1,1], &[1,2,3,255]), Default::default()));
        let before = editor.grid.bg.as_ref().unwrap().id();
        editor.image_buttons = true;
        editor.file_picker = Some((SequenceFile::Image, crate::platform::FilePicker::in_directory("Background image", std::env::temp_dir(), &["png"], None)));
        let mut toasts = Toasts::default();
        let mut out = ctx.run_ui(egui::RawInput {
            screen_rect: Some(Rect::from_min_size(egui::Pos2::ZERO, Vec2::new(1280.0,800.0))),
            events: vec![egui::Event::Key { key: egui::Key::Escape, physical_key: None, pressed: true, repeat: false, modifiers: Default::default() }],
            ..Default::default()
        }, |ui| editor.ui(ui.ctx(), &[], &mut toasts));
        out.textures_delta.clear();
        assert!(editor.file_picker.is_none());
        assert_eq!(editor.grid.bg.as_ref().unwrap().id(), before);
        assert!(editor.image_buttons && toasts.0.is_empty());
    }

    #[test]
    fn sequence_json_matches_the_original_file_format() {
        let text = r#"{
  "Layouts": [
    {
      "Id": "Layout X",
      "DelayStart": 0,
      "DelayEnd": 0,
      "Offset": {
        "1": { "x": 1.0, "y": 0.0, "z": 0.0 },
        "2": { "x": 2.0, "y": 5.0, "z": 10.0 }
      }
    }
  ],
  "Steps": [ "Layout X", "Layout X" ]
}"#;
        let seq: Sequence = serde_json::from_str(text).unwrap();
        assert_eq!(seq.layouts[0].offset[&2], Xyz { x: 2.0, y: 5.0, z: 10.0 });
        assert_eq!(seq.steps.len(), 2);
        let back: serde_json::Value = serde_json::from_str(&serde_json::to_string_pretty(&seq).unwrap()).unwrap();
        assert_eq!(back, serde_json::from_str::<serde_json::Value>(text).unwrap());
    }

    #[test]
    fn drone_count_and_step_targets_follow_the_editor() {
        let mut seq = Sequence { layouts: vec![Layout { id: "a".into(), ..Default::default() }, Layout { id: "b".into(), ..Default::default() }], steps: vec![] };
        let mut known = Default::default();
        set_drone_count(&mut seq, 1, &mut known);
        set_drone_count(&mut seq, 2, &mut known);
        assert_eq!(seq.layouts[1].offset[&2], Xyz { x: 2.0, y: 0.0, z: 0.0 });
        // shrinking removes the sysid equal to the old count
        set_drone_count(&mut seq, 1, &mut known);
        assert!(seq.layouts.iter().all(|l| l.offset.keys().copied().collect::<Vec<_>>() == [1]));
        let l = Layout { id: "x".into(), offset: [(3, Xyz { x: 30.0, y: 40.0, z: 12.0 })].into_iter().collect(), ..Default::default() };
        let t = step_targets(&l, (-35.0, 149.0));
        assert_eq!((t[0].0, t[0].3), (3, 12.0));
        assert!((distance((-35.0, 149.0), (t[0].1, t[0].2)) - 50.0).abs() < 0.1);
        assert!((bearing((-35.0, 149.0), (t[0].1, t[0].2)) - 36.87).abs() < 0.1, "x east, y north");
    }
}

#[cfg(test)]
mod sequence_sim_tests {
    use super::*;
    use mp_core::sim;

    /// Takeoff then Run Step: each simulated Copter flies to startpos + its layout offset.
    #[test]
    fn run_step_sends_each_drone_to_its_layout_offset() {
        let port = std::net::UdpSocket::bind("127.0.0.1:0").unwrap().local_addr().unwrap().port();
        let link = Link::connect_with_params(ConnectionSpec::UdpListen { port }, None, false);
        let _sims: Vec<_> = (1..=3u8)
            .map(|i| sim::spawn_with(sim::SimOptions { gcs_port: port, sysid: i, kind: sim::SimKind::Copter, offset: (0.0, i as f64 * 10.0) }).unwrap())
            .collect();
        let links = [&link];
        let mut ed = LayoutEditor::default();
        ed.show();
        let poll = |what: &str, secs: u64, f: &dyn Fn(&[Mav]) -> bool, retry: &mut dyn FnMut()| {
            let end = Instant::now() + Duration::from_secs(secs);
            let mut next = Instant::now();
            loop {
                let mavs = vehicles(&links);
                if f(&mavs) {
                    return mavs;
                }
                assert!(Instant::now() < end, "timed out: {what}: {mavs:?}");
                if Instant::now() >= next {
                    retry();
                    next = Instant::now() + Duration::from_secs(1);
                }
                std::thread::sleep(Duration::from_millis(50));
            }
        };
        poll("three vehicles", 15, &|m| m.len() == 3 && m.iter().all(|v| v.lat != 0.0), &mut || {});
        // but_takeoff_Click (resent like the original's retries over a lossy link)
        poll("airborne", 30, &|m| m.iter().all(|v| v.alt > 1.5), &mut || ed.takeoff(&links));
        let mut layout = Layout { id: "line".into(), ..Default::default() };
        for (sysid, x) in [(1u32, 0.0), (2, 15.0), (3, 30.0)] {
            layout.offset.insert(sysid, Xyz { x, y: 20.0, z: 8.0 });
        }
        ed.seq = Sequence { layouts: vec![layout.clone()], steps: vec!["line".into()] };
        ed.run_step(&links);
        assert_eq!(ed.label, "line : 0");
        assert_eq!(ed.step, 1);
        let start = ed.startpos.unwrap();
        let targets = step_targets(&layout, start);
        poll("drones at their layout positions", 40, &|m| {
            targets.iter().all(|(sysid, lat, lng, alt)| {
                m.iter().find(|v| v.key.1 as u32 == *sysid).is_some_and(|v| distance((*lat, *lng), (v.lat, v.lng)).hypot(alt - v.alt) < 3.0)
            })
        }, &mut || {
            ed.step = 0;
            ed.run_step(&links);
        });
        // past the last step nothing more is sent
        ed.run_step(&links);
        assert_eq!(ed.step, 1);
    }
}

/// MAVLinkInterface.setGuidedModeWP(sysid, compid, ...) for one vehicle on a connection.
/// The frame is GLOBAL_RELATIVE_ALT: FollowPath's Locationwp leaves frame 0 (absolute) while
/// carrying the trail's relative altitude, which would send real followers to the ground.
fn guided_wp(l: &Link, m: &Mav, lat: f64, lng: f64, alt: f64) {
    if alt == 0.0 || lat == 0.0 || lng == 0.0 {
        return;
    }
    if !m.mode.eq_ignore_ascii_case("GUIDED") {
        set_mode(l, m, "GUIDED");
    }
    if m.firmware == Firmware::ArduPlane {
        let wp = mp_core::mission::Locationwp::waypoint(lat, lng, alt as f32, 3);
        if let Some(mut item) = wp.to_mission_item_int(0, m.key.1, m.key.2, MavMissionType::MAV_MISSION_TYPE_MISSION) {
            item.current = 2;
            l.send(LinkCommand::Send(Box::new(MavMessage::MISSION_ITEM_INT(item))));
        }
    } else {
        l.send(LinkCommand::Send(Box::new(MavMessage::SET_POSITION_TARGET_GLOBAL_INT(SET_POSITION_TARGET_GLOBAL_INT_DATA {
            lat_int: (lat * 1e7) as i32,
            lon_int: (lng * 1e7) as i32,
            alt: alt as f32,
            target_system: m.key.1,
            target_component: m.key.2,
            coordinate_frame: MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            type_mask: PositionTargetTypemask::from_bits_truncate(u16::MAX - (1 << 9) - 0b111),
            ..Default::default()
        }))));
    }
}

/// FollowPath.cs: the active vehicle's trail and the points 2 m apart along it.
#[derive(Default)]
pub struct FollowPath {
    pub trail: Vec<(f64, f64, f64)>,
}

impl FollowPath {
    const FOLLOW_DISTANCE: f64 = 2.0;

    /// FindTrailPnt: the next trail point more than FollowDistance (2D) from `from`.
    fn find_trail_point(&self, from: usize) -> Option<usize> {
        let f = self.trail[from];
        (from + 1..self.trail.len()).find(|i| distance((f.0, f.1), (self.trail[*i].0, self.trail[*i].1)) > Self::FOLLOW_DISTANCE)
    }

    /// PlanMove: up to 20 points from the start of the trail.
    pub fn plan(&self) -> Vec<(f64, f64, f64)> {
        let mut out = Vec::new();
        if self.trail.is_empty() {
            return out;
        }
        let mut current = 0;
        for _ in 0..20 {
            if let Some(next) = self.find_trail_point(current) {
                out.push(self.trail[next]);
                current = next;
            }
        }
        out
    }
}

/// FollowPathControl ("Follow the leader")
pub struct FollowPathControl {
    pub open: bool,
    shown: bool,
    leader: Option<Key>,
    selected: Option<Key>,
    running: bool,
    start_enabled: bool,
    last: Instant,
    path: FollowPath,
    /// MAVState.GuidedMode per connection's current vehicle
    guided: HashMap<usize, (f64, f64, f64)>,
}

impl Default for FollowPathControl {
    fn default() -> Self {
        Self { open: false, shown: false, leader: None, selected: None, running: false, start_enabled: false, last: Instant::now(), path: FollowPath::default(), guided: HashMap::new() }
    }
}

impl FollowPathControl {
    pub fn show(&mut self) {
        *self = Self { open: true, ..Default::default() };
    }

    /// MainV2.comPort.MAV: the active connection's current vehicle.
    fn current(links: &[&Link], mavs: &[Mav]) -> Option<Mav> {
        let l = links.first()?;
        let (sys, comp) = {
            let s = l.shared.read();
            (s.cs.sysid, s.cs.compid)
        };
        mavs.iter().find(|m| m.key == (0, sys, comp)).cloned()
    }

    /// Update + SendCommand
    pub fn tick(&mut self, links: &[&Link], mavs: &[Mav]) {
        let Some(cur) = Self::current(links, mavs) else { return };
        if cur.lat == 0.0 || cur.lng == 0.0 {
            return;
        }
        if self.leader.is_none() {
            self.leader = Some(cur.key);
        }
        self.path.trail.push((cur.lat, cur.lng, cur.alt));
        let positions = self.path.plan();
        for m in mavs.iter().filter(|m| Some(m.key) != self.leader) {
            // the original never advances its index: every follower takes the first point
            if let Some(&(lat, lng, alt)) = positions.first() {
                guided_wp(links[m.key.0], m, lat, lng, alt);
                self.guided.insert(m.key.0, (lat, lng, alt));
            }
        }
    }

    pub fn ui(&mut self, ctx: &egui::Context, links: &[&Link], toasts: &mut Toasts, actions: &mut Vec<crate::app::AppAction>) {
        if !self.open {
            self.running = false;
            return;
        }
        let mavs = vehicles(links);
        if !self.shown {
            self.shown = true;
            if !mavs.is_empty() {
                self.selected = mavs.first().map(|m| m.key);
                toasts.error("this is beta, use at own risk");
            }
        }
        if self.running && self.last.elapsed() >= Duration::from_millis(200) {
            self.last = Instant::now();
            self.tick(links, &mavs);
        }
        if self.running {
            ctx.request_repaint_after(Duration::from_millis(200));
        }
        let mut open = true;
        egui::Window::new(crate::i18n::t("Control")).id(egui::Id::new("swarm_follow_path")).open(&mut open).collapsible(false).resizable(false).order(egui::Order::Foreground).fixed_size([551.0, 171.0]).show(ctx, |ui| {
            let (area, _) = ui.allocate_exact_size(Vec2::new(551.0, 171.0), Sense::hover());
            let o = area.min;
            let r = |x: f32, y: f32, w: f32, h: f32| Rect::from_min_size(o + Vec2::new(x, y), Vec2::new(w, h));
            let button = |ui: &mut egui::Ui, rect: Rect, text: &str, enabled: bool| -> bool {
                ui.put(rect, |ui: &mut egui::Ui| ui.add_enabled_ui(enabled, |ui| theme::button_sized(ui, text, rect.size())).inner).clicked()
            };
            if button(ui, r(12.0, 12.0, 75.0, 23.0), "Arm (exl leader)", true) {
                for m in mavs.iter().filter(|m| Some(m.key) != self.leader) {
                    command(links[m.key.0], m.key.1, m.key.2, MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
                }
            }
            if button(ui, r(93.0, 12.0, 75.0, 23.0), "Disarm (exl leader)", true) {
                for m in mavs.iter().filter(|m| Some(m.key) != self.leader) {
                    command(links[m.key.0], m.key.1, m.key.2, MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, [0.0; 7]);
                }
            }
            // CMB_mavs: its handler compares "MAV x on PORT" with "PORT x y", so choosing an
            // entry never changes the active vehicle
            let label = |m: &Mav| format!("{} {} {}", m.port, m.key.1, m.key.2);
            let cur = self.selected.and_then(|k| mavs.iter().find(|m| m.key == k)).map(label).unwrap_or_default();
            let cr = r(174.0, 12.0, 121.0, 21.0);
            ui.put(cr, |ui: &mut egui::Ui| {
                egui::ComboBox::from_id_salt("CMB_mavs_follow").width(cr.width() - 8.0).selected_text(cur).show_ui(ui, |ui| {
                    for m in &mavs {
                        if ui.selectable_label(self.selected == Some(m.key), label(m)).clicked() {
                            self.selected = Some(m.key);
                        }
                    }
                }).response
            });
            if button(ui, r(301.0, 12.0, 75.0, 23.0), "Connect MAVs", true) {
                // CommsSerialScan.Scan: find another MAVLink port
                actions.push(crate::app::AppAction::ConnectExtra(ConnectionSpec::Auto));
            }
            if button(ui, r(382.0, 12.0, 75.0, 23.0), "Set Leader", true) {
                if let Some(c) = Self::current(links, &mavs) {
                    self.leader = Some(c.key);
                    self.start_enabled = true;
                }
            }
            if button(ui, r(463.0, 12.0, 75.0, 23.0), if self.running { "Stop" } else { "Start" }, self.start_enabled) {
                self.running = !self.running;
                self.last = Instant::now() - Duration::from_millis(200);
            }
            // PNL_status: one Status block per connection, for its current vehicle
            let f = crate::fonts::ui_id(crate::fonts::UI_SIZE);
            for (i, l) in links.iter().enumerate() {
                let b = r(12.0 + i as f32 * 113.0, 41.0, 113.0, 116.0);
                if b.right() > o.x + 539.0 {
                    break;
                }
                let s = l.shared.read();
                let m = mavs.iter().find(|m| m.key == (i, s.cs.sysid, s.cs.compid));
                let guided = self.guided.get(&i).copied().unwrap_or_default();
                let at = |x: f32, y: f32| b.min + Vec2::new(x, y);
                let port = port_name(s.spec.as_ref());
                for (x, y, t) in [
                    (3.0, 0.0, format!("MAV {} on {port}", s.cs.sysid)),
                    (3.0, 13.0, "Armed".into()),
                    (46.0, 13.0, if s.cs.armed { "True" } else { "False" }.into()),
                    (3.0, 26.0, "Mode".into()),
                    (46.0, 26.0, m.map(|m| m.mode.clone()).unwrap_or_else(|| s.cs.mode.clone())),
                    (3.0, 39.0, "GPS".into()),
                    (46.0, 39.0, if s.cs.gpsstatus >= 3 { "OK" } else { "Bad" }.into()),
                    (3.0, 52.0, "Guided".into()),
                ] {
                    ui.painter().text(at(x, y), egui::Align2::LEFT_TOP, t, f.clone(), Color32::WHITE);
                }
                let mut job = egui::text::LayoutJob::simple(format!("{},{},{}", guided.0, guided.1, guided.2), f.clone(), Color32::WHITE, 67.0);
                job.wrap.break_anywhere = true;
                let galley = ui.painter().layout_job(job);
                ui.painter().with_clip_rect(Rect::from_min_size(at(46.0, 52.0), Vec2::new(67.0, 30.0))).galley(at(46.0, 52.0), galley, Color32::WHITE);
            }
        });
        if !open {
            self.open = false;
            self.running = false;
        }
    }
}

/// Swarm/WaypointLeader/DroneGroup.cs: mode
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum DroneGroupMode {
    #[default]
    Idle,
    Takeoff,
    FlyToUser,
    FollowUser,
    RTH,
    LandAlt,
    Land,
}

impl std::fmt::Display for DroneGroupMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Idle => write!(f, "idle"),
            Self::Takeoff => write!(f, "takeoff"),
            Self::FlyToUser => write!(f, "flytouser"),
            Self::FollowUser => write!(f, "followuser"),
            Self::RTH => write!(f, "RTH"),
            Self::LandAlt => write!(f, "LandAlt"),
            Self::Land => write!(f, "Land"),
        }
    }
}

/// A drone as tracked by WPControl's DroneGroup
#[derive(Clone, Debug, Default)]
pub struct DroneState {
    pub key: Key,
    pub path_index: usize,
    pub takeoff_done: bool,
    pub target_location: (f64, f64, f64),
    pub target_velocity: (f64, f64, f64),
}

/// Swarm/WaypointLeader/Path.cs: converts waypoints to 0.1m path increments
pub fn generate_path(wps: &[mp_core::mission::Locationwp]) -> Vec<(f64, f64, f64)> {
    let mut result = Vec::new();
    let mut prev: Option<(f64, f64, f64)> = None;
    for wp in wps {
        if prev.is_none() {
            prev = Some((wp.lat, wp.lng, 0.0));
            continue;
        }
        if wp.id != 16 && wp.id != 82 {
            continue;
        }
        let wp1 = prev.unwrap();
        let wp2 = (wp.lat, wp.lng, wp.alt as f64);
        let brng = bearing((wp1.0, wp1.1), (wp2.0, wp2.1));
        let dist = distance((wp1.0, wp1.1), (wp2.0, wp2.1));
        let start_alt = if wp1.2 == 0.0 { wp2.2 } else { wp1.2 };
        let end_alt = wp2.2;

        if dist > 5000.0 || dist < 0.1 {
            prev = Some(wp2);
            continue;
        }

        let mut d = 0.1;
        while d < dist {
            let p = newpos((wp1.0, wp1.1), brng, d);
            let alt = start_alt + (d / dist) * (end_alt - start_alt);
            result.push((p.0, p.1, alt));
            d += 0.1;
        }
        prev = Some(wp2);
    }
    result
}

/// DroneGroup.GetLocations: finds points along path spaced by separation
pub fn get_locations(path: &[(f64, f64, f64)], location: (f64, f64, f64), lead: f64, separation: f64) -> Vec<(f64, f64, f64)> {
    let mut result = Vec::new();
    if path.is_empty() {
        return result;
    }
    let mut min_dist = f64::MAX;
    let mut start = 0;
    for (idx, pt) in path.iter().enumerate() {
        let d = distance((location.0, location.1), (pt.0, pt.1));
        if d < min_dist {
            min_dist = d;
            start = idx;
        }
    }
    let closest = path[start];
    let mut a = 0;
    let mut i = start as isize;
    while i < (path.len() as isize - 1) {
        let target_distance = lead - a as f64 * separation;
        if target_distance < 0.0 {
            i -= 2;
        }
        if i < 0 {
            break;
        }
        let dist = distance((closest.0, closest.1), (path[i as usize].0, path[i as usize].1));
        if dist >= target_distance.abs() {
            result.push(path[i as usize]);
            i = start as isize;
            if result.len() > 20 || separation <= 0.0 {
                break;
            }
            a += 1;
        }
        i += 1;
    }
    result
}

/// DroneGroup.GetLocationsV: V-formation offsets from the path locations
pub fn get_locations_v(path: &[(f64, f64, f64)], location: (f64, f64, f64), lead: f64, separation: f64) -> Vec<(f64, f64, f64)> {
    let mut result = Vec::new();
    let list = get_locations(path, location, lead, separation);
    if list.is_empty() {
        return result;
    }
    let front = list[0];
    result.push(front);
    let mut a = 1;
    for pt in list.iter().skip(1) {
        let brng = bearing((pt.0, pt.1), (front.0, front.1));
        let left = newpos((pt.0, pt.1), brng + 90.0, (separation / 2.0) * a as f64);
        let right = newpos((pt.0, pt.1), brng - 90.0, (separation / 2.0) * a as f64);
        result.push((left.0, left.1, pt.2));
        result.push((right.0, right.1, pt.2));
        a += 1;
    }
    result
}

/// DroneGroup.GetOffPathDistance: cross-track error to flight plan
pub fn get_off_path_distance(path: &[(f64, f64, f64)], location: (f64, f64, f64)) -> f64 {
    let mut dist_total = f64::MAX;
    let mut line_start: Option<(f64, f64)> = None;
    for pathpoint in path {
        let Some(start) = line_start else {
            line_start = Some((pathpoint.0, pathpoint.1));
            continue;
        };
        let line_end = (pathpoint.0, pathpoint.1);
        let line_dist = distance(start, line_end);
        let dist_to_loc = distance(start, (location.0, location.1));
        let bear_to_loc = bearing(start, (location.0, location.1));
        let line_bear = bearing(start, line_end);

        let mut angle = bear_to_loc - line_bear;
        if angle < 0.0 {
            angle += 360.0;
        }
        let along_line = angle.to_radians().cos() * dist_to_loc;
        if along_line.abs() > line_dist {
            line_start = Some(line_end);
            continue;
        }
        let dxt2 = angle.to_radians().sin() * dist_to_loc;
        dist_total = dist_total.min(dxt2.abs());
        line_start = Some(line_end);
    }
    for pathpoint in path {
        let d = distance((pathpoint.0, pathpoint.1), (location.0, location.1));
        dist_total = dist_total.min(d);
    }
    dist_total
}

const ZED_COLORS: [Color32; 14] = [
    Color32::from_rgb(255, 0, 0),     // Red
    Color32::from_rgb(0, 255, 0),     // Green
    Color32::from_rgb(0, 128, 255),   // Blue
    Color32::from_rgb(255, 165, 0),   // Orange
    Color32::from_rgb(255, 255, 0),   // Yellow
    Color32::from_rgb(238, 130, 238), // Violet
    Color32::from_rgb(255, 192, 203), // Pink
    Color32::from_rgb(0, 128, 128),   // Teal
    Color32::from_rgb(245, 222, 179), // Wheat
    Color32::from_rgb(192, 192, 192), // Silver
    Color32::from_rgb(128, 0, 128),   // Purple
    Color32::from_rgb(0, 255, 255),   // Aqua
    Color32::from_rgb(165, 42, 42),   // Brown
    Color32::from_rgb(245, 245, 245), // WhiteSmoke
];

const WPCONTROL_HOW_TO: &str = "How to Use:
1. connect all drones including masters (right click top icon bar \"connection options\")
2. select the correct vehicle and click set ground master
3. select the correct vehicle and click set air master
4. upload the path to the air master drone.
5. set the seperation and lead options
6. allow all drones to get gps lock, and calibrate. (failing to do so means they might fail prearm checks, this could be tested beforhand to ensure smooth operation)
7. ensure your ground master is ready and waiting on the flight path
8. click start";

/// Swarm/WaypointLeader/WPControl.cs: Follow Swarm
pub struct WPControl {
    pub open: bool,
    pub shown: bool,
    pub groundmaster: Option<Key>,
    pub airmaster: Option<Key>,
    pub running: bool,
    pub start_enabled: bool,
    pub separation: f64,
    pub lead: f64,
    pub off_path_trigger: f64,
    pub takeoff_land_alt_sep: f64,
    pub wpnav_accel: f64,
    pub v_formation: bool,
    pub alt_interleave: bool,
    pub mode: DroneGroupMode,
    pub drones: Vec<DroneState>,
    pub path: Vec<(f64, f64, f64)>,
    pub guided: HashMap<Key, (f64, f64, f64)>,
    pub last: Instant,
}

impl Default for WPControl {
    fn default() -> Self {
        Self {
            open: false,
            shown: false,
            groundmaster: None,
            airmaster: None,
            running: false,
            start_enabled: false,
            separation: 5.0,
            lead: 20.0,
            off_path_trigger: 10.0,
            takeoff_land_alt_sep: 2.0,
            wpnav_accel: 1.0,
            v_formation: false,
            alt_interleave: false,
            mode: DroneGroupMode::Idle,
            drones: Vec::new(),
            path: Vec::new(),
            guided: HashMap::new(),
            last: Instant::now(),
        }
    }
}

impl WPControl {
    pub fn show(&mut self) {
        *self = Self { open: true, ..Default::default() };
    }

    /// MainV2.comPort.MAV: the active connection's current vehicle.
    fn current(links: &[&Link], mavs: &[Mav]) -> Option<Mav> {
        let l = links.first()?;
        let (sys, comp) = {
            let s = l.shared.read();
            (s.cs.sysid, s.cs.compid)
        };
        mavs.iter().find(|m| m.key == (0, sys, comp)).cloned()
    }

    pub fn update_positions(&mut self, links: &[&Link], mavs: &[Mav]) {
        let Some(am_key) = self.airmaster else { return };
        let Some(gm_key) = self.groundmaster else { return };
        let Some(airmaster) = mavs.iter().find(|m| m.key == am_key).cloned() else { return };
        let Some(groundmaster) = mavs.iter().find(|m| m.key == gm_key).cloned() else { return };

        if self.path.is_empty() {
            if let Some(l) = links.get(am_key.0) {
                let s = l.shared.read();
                let wps = s.missions.get(&(am_key.1, am_key.2)).unwrap_or(&s.mission);
                if !wps.is_empty() {
                    self.path = generate_path(wps);
                }
            }
        }
        if self.path.is_empty() {
            return;
        }

        let mut newlist = Vec::new();
        newlist.push(airmaster.clone());
        for m in mavs {
            if m.key != am_key && m.key != gm_key {
                newlist.push(m.clone());
            }
        }

        // Collision check
        for (i, d1) in newlist.iter().enumerate() {
            if !d1.armed { continue; }
            for d2 in newlist.iter().skip(i + 1) {
                if !d2.armed { continue; }
                let d = distance((d1.lat, d1.lng), (d2.lat, d2.lng));
                if d < self.separation / 2.0 && (d1.alt - d2.alt).abs() < 1.0 {
                    if d1.alt > d2.alt {
                        guided_wp(links[d1.key.0], d1, d1.lat, d1.lng, d1.alt + self.takeoff_land_alt_sep);
                    } else {
                        guided_wp(links[d2.key.0], d2, d2.lat, d2.lng, d2.alt + self.takeoff_land_alt_sep);
                    }
                    return;
                }
            }
        }

        // Update path indices
        for d in &mut self.drones {
            if let Some(m) = mavs.iter().find(|m| m.key == d.key) {
                let locs = get_locations(&self.path, (m.lat, m.lng, m.alt), 0.0, 0.0);
                if let Some(first) = locs.first() {
                    if let Some(pos) = self.path.iter().position(|p| (p.0 - first.0).abs() < 1e-6 && (p.1 - first.1).abs() < 1e-6) {
                        d.path_index = pos;
                    }
                }
            }
        }

        match self.mode {
            DroneGroupMode::Idle => {
                self.mode = DroneGroupMode::Takeoff;
                for m in &newlist {
                    if m.key != gm_key {
                        if let Some(l) = links.get(m.key.0) {
                            l.send(LinkCommand::Send(Box::new(MavMessage::REQUEST_DATA_STREAM(REQUEST_DATA_STREAM_DATA {
                                req_message_rate: 5,
                                target_system: m.key.1,
                                target_component: m.key.2,
                                req_stream_id: MavDataStream::MAV_DATA_STREAM_POSITION,
                                start_stop: 1,
                            }))));
                        }
                    }
                }
            }
            DroneGroupMode::Takeoff => {
                let lead = newlist.len() as f64 * self.separation;
                let home = (airmaster.lat, airmaster.lng, airmaster.alt);
                let newpositions = get_locations(&self.path, home, lead, self.separation);
                if newpositions.is_empty() { return; }

                for (a, m) in newlist.iter().enumerate() {
                    let l = links[m.key.0];
                    if !m.mode.eq_ignore_ascii_case("guided") {
                        set_mode(l, m, "GUIDED");
                    }
                    if !m.armed {
                        command(l, m.key.1, m.key.2, MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
                    }
                    let target_pos = newpositions[a % newpositions.len()];
                    let target_alt = target_pos.2 + self.takeoff_land_alt_sep * (a % 3) as f64;
                    let target = (target_pos.0, target_pos.1, target_alt);

                    let d_state = self.drones.iter_mut().find(|d| d.key == m.key);
                    let takeoff_done = d_state.as_ref().map(|d| d.takeoff_done).unwrap_or(false);
                    if m.alt < (target_alt - 0.5) && !takeoff_done {
                        command(l, m.key.1, m.key.2, MavCmd::MAV_CMD_NAV_TAKEOFF, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, target_alt as f32]);
                        if let Some(d) = d_state {
                            d.takeoff_done = true;
                        }
                    }
                    guided_wp(l, m, target.0, target.1, target.2);
                    self.guided.insert(m.key, target);
                }

                let all_close = newlist.iter().enumerate().all(|(a, m)| {
                    let target_pos = newpositions[a % newpositions.len()];
                    distance((m.lat, m.lng), (target_pos.0, target_pos.1)) <= self.separation
                });
                if all_close {
                    self.mode = DroneGroupMode::FlyToUser;
                }
            }
            DroneGroupMode::FlyToUser => {
                let newpositions2 = get_locations(&self.path, (airmaster.lat, airmaster.lng, airmaster.alt), self.separation, self.separation);
                if newpositions2.is_empty() {
                    if let Some(last_pt) = self.path.last() {
                        if distance((airmaster.lat, airmaster.lng), (last_pt.0, last_pt.1)) < self.separation {
                            self.mode = DroneGroupMode::RTH;
                        }
                    }
                    return;
                }
                for (b, m) in newlist.iter().enumerate() {
                    if b >= newpositions2.len() { break; }
                    let mut target = newpositions2[b];
                    if self.alt_interleave {
                        target.2 += self.takeoff_land_alt_sep * (b % 2) as f64;
                    }
                    guided_wp(links[m.key.0], m, target.0, target.1, target.2);
                    self.guided.insert(m.key, target);
                }

                let newpositions_follow = get_locations(&self.path, (groundmaster.lat, groundmaster.lng, groundmaster.alt), self.lead, self.separation);
                if let Some(first) = newpositions_follow.first() {
                    if distance((airmaster.lat, airmaster.lng), (first.0, first.1)) < self.separation {
                        self.mode = DroneGroupMode::FollowUser;
                    }
                }
            }
            DroneGroupMode::FollowUser => {
                let newpositions3 = if self.v_formation {
                    get_locations_v(&self.path, (groundmaster.lat, groundmaster.lng, groundmaster.alt), self.lead, self.separation)
                } else {
                    get_locations(&self.path, (groundmaster.lat, groundmaster.lng, groundmaster.alt), self.lead, self.separation)
                };
                if newpositions3.is_empty() { return; }

                if get_off_path_distance(&self.path, (groundmaster.lat, groundmaster.lng, groundmaster.alt)) > self.off_path_trigger {
                    self.mode = DroneGroupMode::RTH;
                    return;
                }

                for (c, m) in newlist.iter().enumerate() {
                    if m.key == gm_key { continue; }
                    if c >= newpositions3.len() { break; }
                    let mut target = newpositions3[c];
                    if self.alt_interleave {
                        target.2 += self.takeoff_land_alt_sep * (c % 2) as f64;
                    }
                    guided_wp(links[m.key.0], m, target.0, target.1, target.2);
                    self.guided.insert(m.key, target);
                }
            }
            DroneGroupMode::RTH => {
                let newpositions4 = get_locations(&self.path, (airmaster.lat, airmaster.lng, airmaster.alt), self.separation, self.separation);
                if newpositions4.is_empty() {
                    if let Some(last_pt) = self.path.last() {
                        if distance((airmaster.lat, airmaster.lng), (last_pt.0, last_pt.1)) < self.separation {
                            self.mode = DroneGroupMode::LandAlt;
                        }
                    }
                    return;
                }
                for (d, m) in newlist.iter().enumerate() {
                    if d >= newpositions4.len() { break; }
                    let mut target = newpositions4[d];
                    if self.alt_interleave {
                        target.2 += self.takeoff_land_alt_sep * (d % 2) as f64;
                    }
                    guided_wp(links[m.key.0], m, target.0, target.1, target.2);
                    self.guided.insert(m.key, target);
                }
            }
            DroneGroupMode::LandAlt => {
                for (e, m) in newlist.iter().enumerate() {
                    let target_alt = m.alt + self.takeoff_land_alt_sep * e as f64;
                    guided_wp(links[m.key.0], m, m.lat, m.lng, target_alt);
                    self.guided.insert(m.key, (m.lat, m.lng, target_alt));
                }
                self.mode = DroneGroupMode::Land;
            }
            DroneGroupMode::Land => {
                for m in &newlist {
                    set_mode(links[m.key.0], m, "RTL");
                }
            }
        }
    }

    pub fn ui(&mut self, ctx: &egui::Context, links: &[&Link], _toasts: &mut Toasts) {
        if !self.open {
            self.running = false;
            return;
        }
        let mavs = vehicles(links);
        if !self.shown {
            self.shown = true;
            if self.drones.is_empty() {
                for m in &mavs {
                    self.drones.push(DroneState { key: m.key, ..Default::default() });
                }
            }
        }
        if self.running && self.last.elapsed() >= Duration::from_millis(100) {
            self.last = Instant::now();
            self.update_positions(links, &mavs);
        }
        if self.running {
            ctx.request_repaint_after(Duration::from_millis(100));
        }

        let mut open = true;
        egui::Window::new(crate::i18n::t("WPControl"))
            .id(egui::Id::new("swarm_wp_control"))
            .open(&mut open)
            .collapsible(false)
            .resizable(false)
            .order(egui::Order::Foreground)
            .fixed_size([783.0, 509.0])
            .show(ctx, |ui| {
                let (area, _) = ui.allocate_exact_size(Vec2::new(783.0, 509.0), Sense::hover());
                let o = area.min;
                let r = |x: f32, y: f32, w: f32, h: f32| Rect::from_min_size(o + Vec2::new(x, y), Vec2::new(w, h));
                let button = |ui: &mut egui::Ui, rect: Rect, text: &str, enabled: bool| -> bool {
                    ui.put(rect, |ui: &mut egui::Ui| ui.add_enabled_ui(enabled, |ui| theme::button_sized(ui, text, rect.size())).inner).clicked()
                };
                let f = crate::fonts::ui_id(crate::fonts::UI_SIZE);

                // but_master: Set Ground Master (12, 12, 75, 37)
                if button(ui, r(12.0, 12.0, 75.0, 37.0), "Set Ground Master", true) {
                    if let Some(c) = Self::current(links, &mavs) {
                        self.groundmaster = Some(c.key);
                    }
                    self.drones.clear();
                    for m in &mavs {
                        self.drones.push(DroneState { key: m.key, ..Default::default() });
                    }
                }

                // but_airmaster: Set Air Master (109, 12, 75, 37)
                if button(ui, r(109.0, 12.0, 75.0, 37.0), "Set Air Master", true) {
                    if let Some(c) = Self::current(links, &mavs) {
                        self.airmaster = Some(c.key);
                        self.start_enabled = true;
                        if let Some(l) = links.get(c.key.0) {
                            let s = l.shared.read();
                            let wps = s.missions.get(&(c.key.1, c.key.2)).unwrap_or(&s.mission);
                            if !wps.is_empty() {
                                self.path = generate_path(wps);
                            }
                        }
                    }
                    self.drones.clear();
                    for m in &mavs {
                        self.drones.push(DroneState { key: m.key, ..Default::default() });
                    }
                }

                // txt_mode: (203, 12, 75, 13)
                ui.painter().text(o + Vec2::new(203.0, 12.0), egui::Align2::LEFT_TOP, self.mode.to_string(), f.clone(), Color32::WHITE);

                // but_resetmode: (282, 7, 75, 23)
                if button(ui, r(282.0, 7.0, 75.0, 23.0), "Reset Mode", true) {
                    self.mode = DroneGroupMode::Idle;
                }

                // but_start: (190, 36, 75, 23)
                let start_text = if self.running { "Stop" } else { "Start" };
                if button(ui, r(190.0, 36.0, 75.0, 23.0), start_text, self.start_enabled) {
                    self.running = !self.running;
                    self.last = Instant::now() - Duration::from_millis(100);
                }

                // but_rth: (282, 36, 75, 23)
                if button(ui, r(282.0, 36.0, 75.0, 23.0), "set mode rth", true) {
                    self.mode = DroneGroupMode::RTH;
                }

                // label1: Line Seperation (10, 57) & numericUpDown1 (109, 55, 75, 20)
                ui.painter().text(o + Vec2::new(10.0, 57.0), egui::Align2::LEFT_TOP, crate::i18n::t("Line Seperation"), f.clone(), Color32::WHITE);
                ui.put(r(109.0, 55.0, 75.0, 20.0), |ui: &mut egui::Ui| {
                    ui.add_sized([75.0, 20.0], egui::DragValue::new(&mut self.separation).speed(0.1).range(0.1..=100.0).fixed_decimals(1))
                });

                // chk_V: (190, 65, 33, 17)
                ui.put(r(190.0, 65.0, 40.0, 17.0), |ui: &mut egui::Ui| {
                    ui.checkbox(&mut self.v_formation, "V")
                });

                // but_setmoderltland: (282, 65, 75, 44)
                if button(ui, r(282.0, 65.0, 75.0, 44.0), "RTL (abandon mission)", true) {
                    self.mode = DroneGroupMode::LandAlt;
                }

                // label2: Lead (10, 83) & numericUpDown2 (109, 81, 75, 20)
                ui.painter().text(o + Vec2::new(10.0, 83.0), egui::Align2::LEFT_TOP, crate::i18n::t("Lead"), f.clone(), Color32::WHITE);
                ui.put(r(109.0, 81.0, 75.0, 20.0), |ui: &mut egui::Ui| {
                    ui.add_sized([75.0, 20.0], egui::DragValue::new(&mut self.lead).speed(0.1).range(-100.0..=100.0).fixed_decimals(1))
                });

                // chk_alt_interleave: (188, 84, 88, 17)
                ui.put(r(188.0, 84.0, 95.0, 17.0), |ui: &mut egui::Ui| {
                    ui.checkbox(&mut self.alt_interleave, crate::i18n::t("Alt Interleave"))
                });

                // label3: User Offline trig (10, 109) & num_useroffline (109, 107, 75, 20)
                ui.painter().text(o + Vec2::new(10.0, 109.0), egui::Align2::LEFT_TOP, crate::i18n::t("User Offline trig"), f.clone(), Color32::WHITE);
                ui.put(r(109.0, 107.0, 75.0, 20.0), |ui: &mut egui::Ui| {
                    ui.add_sized([75.0, 20.0], egui::DragValue::new(&mut self.off_path_trigger).speed(0.1).range(1.0..=100.0).fixed_decimals(1))
                });

                // label4: alt seperation (10, 135) & num_rtl_alt (109, 133, 75, 20)
                ui.painter().text(o + Vec2::new(10.0, 135.0), egui::Align2::LEFT_TOP, crate::i18n::t("alt seperation"), f.clone(), Color32::WHITE);
                ui.put(r(109.0, 133.0, 75.0, 20.0), |ui: &mut egui::Ui| {
                    ui.add_sized([75.0, 20.0], egui::DragValue::new(&mut self.takeoff_land_alt_sep).speed(0.1).range(1.0..=100.0).fixed_decimals(1))
                });

                // label5: WPNAV_ACCEL (190, 135) & num_wpnav_accel (282, 133, 75, 20)
                ui.painter().text(o + Vec2::new(190.0, 135.0), egui::Align2::LEFT_TOP, crate::i18n::t("WPNAV_ACCEL"), f.clone(), Color32::WHITE);
                ui.put(r(282.0, 133.0, 75.0, 20.0), |ui: &mut egui::Ui| {
                    ui.add_sized([75.0, 20.0], egui::DragValue::new(&mut self.wpnav_accel).speed(0.1).range(1.0..=100.0).fixed_decimals(1))
                });

                // textBox1: How to use (363, 4, 408, 138)
                let tb_rect = r(363.0, 4.0, 408.0, 138.0);
                ui.painter().rect_filled(tb_rect, 0.0, Color32::from_gray(30));
                ui.painter().rect_stroke(tb_rect, 0.0, Stroke::new(1.0, Color32::from_gray(70)), egui::StrokeKind::Inside);
                let mut tb_job = egui::text::LayoutJob::simple(WPCONTROL_HOW_TO.into(), f.clone(), Color32::WHITE, 396.0);
                tb_job.wrap.break_anywhere = true;
                let tb_galley = ui.painter().layout_job(tb_job);
                ui.painter().with_clip_rect(tb_rect.shrink(4.0)).galley(tb_rect.min + Vec2::new(4.0, 4.0), tb_galley, Color32::WHITE);

                // PNL_status: (13, 159, 758, 173)
                let pnl_rect = r(13.0, 159.0, 758.0, 173.0);
                ui.painter().rect_filled(pnl_rect, 0.0, Color32::from_gray(20));
                ui.painter().rect_stroke(pnl_rect, 0.0, Stroke::new(1.0, Color32::from_gray(60)), egui::StrokeKind::Inside);
                for (i, m) in mavs.iter().enumerate() {
                    let b = r(16.0 + i as f32 * 115.0, 162.0, 113.0, 128.0);
                    if b.right() > o.x + 768.0 { break; }
                    ui.painter().rect_filled(b, 0.0, Color32::from_gray(30));
                    ui.painter().rect_stroke(b, 0.0, Stroke::new(1.0, Color32::from_gray(70)), egui::StrokeKind::Inside);

                    let at = |x: f32, y: f32| b.min + Vec2::new(x, y);
                    let is_am = self.airmaster == Some(m.key);
                    let is_gm = self.groundmaster == Some(m.key);
                    let title = if is_am {
                        format!("MAV {}-{} airmaster", m.key.1, m.key.2)
                    } else if is_gm {
                        format!("MAV {}-{} groundmaster", m.key.1, m.key.2)
                    } else {
                        format!("MAV {}-{}", m.key.1, m.key.2)
                    };
                    let title_col = if is_am || is_gm { Color32::RED } else { Color32::WHITE };
                    ui.painter().text(at(3.0, 0.0), egui::Align2::LEFT_TOP, title, f.clone(), title_col);

                    for (x, y, t) in [
                        (3.0, 13.0, "Armed".into()),
                        (46.0, 13.0, if m.armed { "True" } else { "False" }.into()),
                        (3.0, 26.0, "Mode".into()),
                        (46.0, 26.0, m.mode.clone()),
                        (3.0, 39.0, "GPS".into()),
                        (46.0, 39.0, if m.gpsstatus >= 3 { format!("OK {}", m.gpsstatus) } else { "Bad".into() }),
                        (3.0, 52.0, "Guided".into()),
                        (3.0, 82.0, "Location".into()),
                        (3.0, 112.0, "Speed".into()),
                        (46.0, 112.0, format!("{:.2}", m.groundspeed)),
                    ] {
                        ui.painter().text(at(x, y), egui::Align2::LEFT_TOP, t, f.clone(), Color32::WHITE);
                    }

                    let guided = self.guided.get(&m.key).copied().unwrap_or((0.0, 0.0, 0.0));
                    let mut g_job = egui::text::LayoutJob::simple(format!("{:.5},{:.5},{:.2}", guided.0, guided.1, guided.2), f.clone(), Color32::WHITE, 67.0);
                    g_job.wrap.break_anywhere = true;
                    let g_galley = ui.painter().layout_job(g_job);
                    ui.painter().with_clip_rect(Rect::from_min_size(at(46.0, 52.0), Vec2::new(67.0, 30.0))).galley(at(46.0, 52.0), g_galley, Color32::WHITE);

                    let loc_str = format!("{:.5},{:.5},{:.2}", m.lat, m.lng, m.alt);
                    let mut l_job = egui::text::LayoutJob::simple(loc_str, f.clone(), Color32::WHITE, 67.0);
                    l_job.wrap.break_anywhere = true;
                    let l_galley = ui.painter().layout_job(l_job);
                    ui.painter().with_clip_rect(Rect::from_min_size(at(47.0, 82.0), Vec2::new(67.0, 30.0))).galley(at(47.0, 82.0), l_galley, Color32::WHITE);
                }

                // zedGraphControl1: Distance vs Altitude plot (12, 338, 759, 155)
                let graph_rect = r(12.0, 338.0, 759.0, 155.0);
                ui.painter().rect_filled(graph_rect, 0.0, Color32::from_rgb(20, 20, 20));
                let chart = Rect::from_min_max(graph_rect.min + Vec2::new(45.0, 10.0), graph_rect.max - Vec2::new(15.0, 25.0));
                ui.painter().rect_filled(chart, 0.0, Color32::BLACK);
                ui.painter().rect_stroke(chart, 0.0, Stroke::new(1.0, Color32::GRAY), egui::StrokeKind::Inside);

                // Axis titles
                ui.painter().text(Pos2::new(chart.center().x, chart.bottom() + 4.0), egui::Align2::CENTER_TOP, crate::i18n::t("Distance"), f.clone(), Color32::WHITE);
                ui.painter().text(Pos2::new(graph_rect.left() + 4.0, chart.center().y), egui::Align2::LEFT_CENTER, crate::i18n::t("Altitude"), f.clone(), Color32::WHITE);

                // Range
                let max_dist = (self.path.len() as f64 * 0.1).max(50.0);
                let (mut min_alt, mut max_alt) = (0.0f64, 30.0f64);
                for p in &self.path {
                    min_alt = min_alt.min(p.2);
                    max_alt = max_alt.max(p.2);
                }
                for m in &mavs {
                    min_alt = min_alt.min(m.alt);
                    max_alt = max_alt.max(m.alt);
                }
                if (max_alt - min_alt).abs() < 1.0 {
                    max_alt += 5.0;
                }
                let alt_span = max_alt - min_alt;

                let to_pos = |dist: f64, alt: f64| -> Pos2 {
                    let x = chart.left() + ((dist / max_dist).clamp(0.0, 1.0) as f32) * chart.width();
                    let y = chart.bottom() - (((alt - min_alt) / alt_span).clamp(0.0, 1.0) as f32) * chart.height();
                    Pos2::new(x, y)
                };

                // Grid lines & labels
                for k in 0..=3 {
                    let frac = k as f32 / 3.0;
                    let alt_val = min_alt + alt_span * frac as f64;
                    let y = chart.bottom() - frac * chart.height();
                    ui.painter().line_segment([Pos2::new(chart.left(), y), Pos2::new(chart.right(), y)], Stroke::new(1.0, Color32::from_gray(40)));
                    ui.painter().text(Pos2::new(chart.left() - 3.0, y), egui::Align2::RIGHT_CENTER, format!("{alt_val:.0}"), f.clone(), Color32::WHITE);

                    let dist_val = max_dist * frac as f64;
                    let x = chart.left() + frac * chart.width();
                    ui.painter().line_segment([Pos2::new(x, chart.top()), Pos2::new(x, chart.bottom())], Stroke::new(1.0, Color32::from_gray(40)));
                    ui.painter().text(Pos2::new(x, chart.bottom() + 1.0), egui::Align2::CENTER_TOP, format!("{dist_val:.0}"), f.clone(), Color32::WHITE);
                }

                // Path curve (Red)
                if !self.path.is_empty() {
                    let step = (self.path.len() / 200).max(1);
                    let pts: Vec<Pos2> = self.path.iter().enumerate().step_by(step).map(|(i, p)| to_pos(i as f64 * 0.1, p.2)).collect();
                    if pts.len() > 1 {
                        ui.painter().add(egui::Shape::line(pts, Stroke::new(1.5, Color32::RED)));
                    }
                }

                // Drones on graph
                for (i, d) in self.drones.iter().enumerate() {
                    let alt = mavs.iter().find(|m| m.key == d.key).map(|m| m.alt).unwrap_or(0.0);
                    let dist = d.path_index as f64 * 0.1;
                    let pt = to_pos(dist, alt);
                    let col = ZED_COLORS[i % ZED_COLORS.len()];
                    let tri = vec![
                        pt + Vec2::new(0.0, -5.0),
                        pt + Vec2::new(-4.0, 4.0),
                        pt + Vec2::new(4.0, 4.0),
                    ];
                    ui.painter().add(egui::Shape::convex_polygon(tri, col, Stroke::NONE));
                    ui.painter().text(pt + Vec2::new(0.0, -7.0), egui::Align2::CENTER_BOTTOM, format!("MAV {}", d.key.1), f.clone(), col);
                }
            });

        if !open {
            self.open = false;
            self.running = false;
        }
    }
}

#[cfg(test)]
mod follow_path_tests {
    use super::*;

    #[test]
    fn plan_walks_the_trail_in_two_metre_steps_from_its_start() {
        let mut p = FollowPath::default();
        assert!(p.plan().is_empty());
        // a straight trail north, one point every metre
        for i in 0..30 {
            p.trail.push((-35.0 + i as f64 / 111_320.0, 149.0, 10.0));
        }
        let plan = p.plan();
        assert_eq!(plan.len(), 9, "20 searches, but the trail ends after 9 hops of 3 m");
        let first = plan[0];
        assert!((distance((p.trail[0].0, p.trail[0].1), (first.0, first.1)) - 3.0).abs() < 0.01, "the first point beyond 2 m");
    }
}

#[cfg(test)]
mod wp_control_tests {
    use super::*;

    #[test]
    fn generate_path_interpolates_in_tenth_metre_increments() {
        let wps = vec![
            mp_core::mission::Locationwp::waypoint(-35.0, 149.0, 0.0, 0),
            mp_core::mission::Locationwp::waypoint(-35.0009, 149.0, 20.0, 3),
        ];
        let path = generate_path(&wps);
        assert!(!path.is_empty(), "path should not be empty");
        assert!(path.len() > 500, "100m at 0.1m step is ~1000 points, got {}", path.len());
        assert!((path.first().unwrap().2 - 20.0).abs() < 1.0);
        assert!((path.last().unwrap().2 - 20.0).abs() < 1.0);
    }

    #[test]
    fn get_locations_and_v_offsets() {
        let mut path = Vec::new();
        for i in 0..1000 {
            path.push((-35.0 + i as f64 / 111_320.0, 149.0, 10.0));
        }
        let locs = get_locations(&path, path[0], 20.0, 5.0);
        assert!(!locs.is_empty());
        assert_eq!(locs.len(), 5);

        let v_locs = get_locations_v(&path, path[0], 20.0, 5.0);
        assert!(!v_locs.is_empty());
        assert_eq!(v_locs.len(), 9);
    }

    #[test]
    fn get_off_path_distance_measures_crosstrack() {
        let path = vec![
            (-35.0, 149.0, 10.0),
            (-35.001, 149.0, 10.0),
        ];
        let on_path = (-35.0005, 149.0, 10.0);
        let dist = get_off_path_distance(&path, on_path);
        assert!(dist < 0.1, "point directly on line should have near-zero distance, got {}", dist);

        let off_path = (-35.0005, 149.001, 10.0);
        let dist2 = get_off_path_distance(&path, off_path);
        assert!(dist2 > 50.0 && dist2 < 120.0, "crosstrack distance, got {}", dist2);
    }
}

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
}

impl Default for Grid {
    fn default() -> Self {
        Self { xdist: 40, ydist: 40, centerx: 0.0, centery: 0.0, vertical: false, icons: BTreeMap::new(), hover: None, drag: None }
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
        if self.drag.is_none() {
            self.hover = pointer.and_then(|pos| hit(self, pos));
        }
        let mut alt = None;
        resp.context_menu(|ui| {
            if ui.button("Change Alt").clicked() {
                alt = self.hover;
                ui.close();
            }
        });
        let r = Rect::from_min_size(rect.max - Vec2::new(64.0, 20.0), Vec2::new(61.0, 17.0));
        ui.put(r, egui::Checkbox::new(&mut self.vertical, "Vertical"));
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
        egui::Window::new("Control").id(egui::Id::new("swarm_formation")).open(&mut open).collapsible(false).resizable(false).order(egui::Order::Foreground).fixed_size([931.0, 471.0]).show(ctx, |ui| {
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
            ui.painter().text(tab.min + Vec2::new(8.0, 4.0), egui::Align2::LEFT_TOP, "Stage 1", crate::fonts::ui_id(crate::fonts::UI_SIZE), Color32::WHITE);
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
            // PNL_status: one Status block per vehicle
            let mut y = 61.0;
            for m in &mavs {
                let b = r(783.0, y, 113.0, 128.0);
                let color = if Some(m.key) == self.leader { Color32::RED } else { Color32::WHITE };
                let f = crate::fonts::ui_id(crate::fonts::UI_SIZE);
                let at = |x: f32, yy: f32| b.min + Vec2::new(x, yy);
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
                ] {
                    p.text(at(x, yy), egui::Align2::LEFT_TOP, t, f.clone(), color);
                }
                for (yy, t) in [(52.0, format!("{},{},{}", guided.0, guided.1, guided.2)), (82.0, format!("{},{},{}", m.lat, m.lng, m.alt))] {
                    let mut job = egui::text::LayoutJob::simple(t, f.clone(), color, 67.0);
                    job.wrap.break_anywhere = true;
                    let galley = ui.painter().layout_job(job);
                    ui.painter().with_clip_rect(Rect::from_min_size(at(46.0, yy), Vec2::new(67.0, 30.0))).galley(at(46.0, yy), galley, color);
                }
                y += 128.0;
                if y > 61.0 + 398.0 - 128.0 {
                    break;
                }
            }
        });
        if !open {
            // Control_FormClosing
            self.open = false;
            self.running = false;
        }
        if let Some((k, mut text)) = self.alt_prompt.take() {
            let mut done = None;
            egui::Modal::new(egui::Id::new("swarm_alt")).show(ctx, |ui| {
                ui.heading("Alt");
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

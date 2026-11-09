//! Direct port of `ExtLibs/Controls/HUD.cs` `doPaint()` (GDI path, displayicons = false).
//!
//! Layout maths keeps the original integer divisions (`Height / 30`, `Height / 65`, ...) so the
//! HUD lines up with Mission Planner at any size.

use egui::epaint::{Mesh, TextShape, Vertex, WHITE_UV};
use egui::{Color32, Pos2, Rect, Sense, Shape, Stroke, Ui, Vec2};

use mp_core::state::CurrentState;

use crate::fonts;

pub struct HudInput<'a> {
    pub cs: &'a CurrentState,
    pub connected: bool,
    /// When the mode last changed (mode text is red for 2 s afterwards).
    pub mode_changed_secs: f64,
    /// When the vehicle last armed ("ARMED" shows for 8 s).
    pub armed_secs: f64,
    pub opts: &'a HudOptions,
    /// Video frame drawn behind the HUD (MJPEG source).
    pub video: Option<egui::TextureId>,
}

/// contextMenuStripHud settings.
#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct HudOptions {
    /// Russian HUD: the horizon stays level and the aircraft symbol rolls.
    pub russian: bool,
    /// Ground Color: gradient top / bottom (None = theme colours).
    pub ground: Option<[u8; 6]>,
    /// Battery Cell Voltage: cells in the pack (0 = show pack voltage).
    pub cells: u32,
    /// User Items: CurrentState fields listed on the HUD.
    pub items: Vec<String>,
    /// Show icons instead of the battery / GPS text prefixes.
    pub icons: bool,
}

// theme / HUD.cs colours
const SKY1: Color32 = Color32::from_rgb(0, 0, 255); // Blue
const SKY2: Color32 = Color32::from_rgb(0xad, 0xd8, 0xe6); // LightBlue
const GROUND1: Color32 = Color32::from_rgb(0x9b, 0xb8, 0x24);
const GROUND2: Color32 = Color32::from_rgb(0x41, 0x4f, 0x07);
const HUDCOLOR: Color32 = Color32::from_rgb(0xd3, 0xd3, 0xd3); // LightGray
const GREEN: Color32 = Color32::from_rgb(0, 128, 0); // Color.Green
const RED: Color32 = Color32::from_rgb(255, 0, 0);
const ORANGE: Color32 = Color32::from_rgb(255, 165, 0);
const YELLOW: Color32 = Color32::from_rgb(255, 255, 0);
const ALICEBLUE: Color32 = Color32::from_rgb(0xf0, 0xf8, 0xff);
const BLUE: Color32 = Color32::from_rgb(0, 0, 255);
const SOLID: Color32 = Color32::from_rgba_unmultiplied_const(0xff, 0xff, 0xff, 0x55);
const SLIGHTLY_WHITE: Color32 = Color32::from_rgba_unmultiplied_const(255, 255, 255, 220);
const ALT_GROUND: Color32 = Color32::from_rgba_unmultiplied_const(0xde, 0xb8, 0x87, 100); // BurlyWood
const OUTLINE: Color32 = Color32::from_rgb(0x26, 0x27, 0x28);

/// GDI-style transform: rotate (degrees, clockwise in y-down) then translate.
#[derive(Clone, Copy)]
struct Xf {
    origin: Pos2,
    angle: f32,
}

impl Xf {
    fn at(origin: Pos2) -> Self {
        Self { origin, angle: 0.0 }
    }
    fn rot(origin: Pos2, deg: f32) -> Self {
        Self { origin, angle: deg.to_radians() }
    }
    fn p(&self, x: f32, y: f32) -> Pos2 {
        let (s, c) = self.angle.sin_cos();
        Pos2::new(self.origin.x + x * c - y * s, self.origin.y + x * s + y * c)
    }
}

struct Hud<'a> {
    painter: egui::Painter,
    ctx: &'a egui::Context,
    w: i32,
    h: i32,
    o: Pos2,
}

impl Hud<'_> {
    fn line(&self, xf: Xf, x1: f32, y1: f32, x2: f32, y2: f32, width: f32, col: Color32) {
        self.painter.line_segment([xf.p(x1, y1), xf.p(x2, y2)], Stroke::new(width, col));
    }

    fn poly(&self, xf: Xf, pts: &[(f32, f32)], width: f32, col: Color32) {
        let v: Vec<Pos2> = pts.iter().map(|&(x, y)| xf.p(x, y)).collect();
        self.painter.add(Shape::closed_line(v, Stroke::new(width, col)));
    }

    fn fill_poly(&self, xf: Xf, pts: &[(f32, f32)], col: Color32) {
        let v: Vec<Pos2> = pts.iter().map(|&(x, y)| xf.p(x, y)).collect();
        // concave-safe: fan from the first point works for these simple shapes
        let mut m = Mesh::default();
        for p in &v {
            m.vertices.push(Vertex { pos: *p, uv: WHITE_UV, color: col });
        }
        for i in 1..v.len().saturating_sub(1) {
            m.indices.extend_from_slice(&[0, i as u32, i as u32 + 1]);
        }
        self.painter.add(Shape::mesh(m));
    }

    fn fill_rect(&self, xf: Xf, x: f32, y: f32, w: f32, h: f32, col: Color32) {
        self.fill_poly(xf, &[(x, y), (x + w, y), (x + w, y + h), (x, y + h)], col);
    }

    fn grad_rect(&self, xf: Xf, x: f32, y: f32, w: f32, h: f32, top: Color32, bottom: Color32) {
        let mut m = Mesh::default();
        for (px, py, c) in [(x, y, top), (x + w, y, top), (x + w, y + h, bottom), (x, y + h, bottom)] {
            m.vertices.push(Vertex { pos: xf.p(px, py), uv: WHITE_UV, color: c });
        }
        m.indices.extend_from_slice(&[0, 1, 2, 0, 2, 3]);
        self.painter.add(Shape::mesh(m));
    }

    fn rect_outline(&self, xf: Xf, x: f32, y: f32, w: f32, h: f32, width: f32, col: Color32) {
        self.poly(xf, &[(x, y), (x + w, y), (x + w, y + h), (x, y + h)], width, col);
    }

    /// `drawstring`: Arial at `fontsize + 5` px with a 2px #262728 outline, top-left at (x, y).
    fn text(&self, xf: Xf, s: &str, fontsize: f32, col: Color32, x: f32, y: f32) {
        if s.is_empty() {
            return;
        }
        let font = fonts::arial_id(fontsize + 5.0);
        let pos = xf.p(x, y);
        let outline = self.painter.layout_no_wrap(s.to_string(), font.clone(), OUTLINE);
        for (dx, dy) in [(-1.0, 0.0), (1.0, 0.0), (0.0, -1.0), (0.0, 1.0), (-1.0, -1.0), (1.0, 1.0), (-1.0, 1.0), (1.0, -1.0)] {
            let off = Xf { origin: pos, angle: xf.angle }.p(dx, dy);
            self.painter.add(TextShape::new(off, outline.clone(), OUTLINE).with_angle(xf.angle));
        }
        let g = self.painter.layout_no_wrap(s.to_string(), font, col);
        self.painter.add(TextShape::new(pos, g, col).with_angle(xf.angle));
    }

    fn text_width(&self, s: &str, fontsize: f32) -> f32 {
        self.painter.layout_no_wrap(s.to_string(), fonts::arial_id(fontsize + 5.0), Color32::WHITE).size().x
    }
}

fn pad(n: impl std::fmt::Display, width: usize) -> String {
    format!("{n:>width$}")
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HudClick {
    Ekf,
    Vibe,
    PreArm,
}

/// Draws the HUD; returns which status item was clicked (HUD.cs ekfhitzone / vibehitzone /
/// prearmhitzone).
pub fn show(ui: &mut Ui, input: &HudInput, size: Vec2) -> (Option<HudClick>, egui::Response) {
    let (resp, painter) = ui.allocate_painter(size, Sense::click());
    let click = clicked_zone(&resp, input, painter);
    (click, resp)
}

fn clicked_zone(resp: &egui::Response, input: &HudInput, painter: egui::Painter) -> Option<HudClick> {
    let rect = resp.rect;
    let painter = painter.with_clip_rect(rect);
    painter.rect_filled(rect, 0.0, Color32::BLACK);
    let cs = input.cs;
    let hud = Hud { painter, ctx: &resp.ctx, w: rect.width() as i32, h: rect.height() as i32, o: rect.min };
    draw(&hud, cs, input);

    let pos = resp.interact_pointer_pos().filter(|_| resp.clicked())?;
    let (w, h) = (rect.width(), rect.height());
    let fontsize = (rect.height() as i32 / 30) as f32;
    let ybo = if fontsize >= 8.0 { (fontsize as i32 / 3) as f32 } else { 2.0 };
    let yto = fontsize + ybo + 2.0;
    let ypos = [h - 2.0 * yto - ybo - 4.0, h - yto - ybo - 4.0];
    let zone = |x: f32, y: f32, zw: f32, zh: f32| Rect::from_min_size(rect.min + Vec2::new(x, y), Vec2::new(zw, zh));
    if zone(w - 23.0 * fontsize, ypos[1], 40.0, fontsize * 2.0).contains(pos) {
        return Some(HudClick::Ekf);
    }
    if zone(w - 18.0 * fontsize, ypos[1], 40.0, fontsize * 2.0).contains(pos) {
        return Some(HudClick::Vibe);
    }
    if zone(w - 26.0 * fontsize, ypos[0] - 4.0, 16.0 * fontsize, fontsize * 2.0).contains(pos) && !cs.armed {
        return Some(HudClick::PreArm);
    }
    None
}

fn draw(g: &Hud, cs: &CurrentState, input: &HudInput) {
    let (w, h) = (g.w, g.h);
    let (wf, hf) = (w as f32, h as f32);
    let o = g.o;
    let at = |x: f32, y: f32| Pos2::new(o.x + x, o.y + y);
    let base = Xf::at(o);

    let roll = cs.roll;
    let pitch = cs.pitch;
    let heading = cs.yaw;
    let white = HUDCOLOR;

    let fontsize = (h / 30) as f32;
    let fontoffset = fontsize - 10.0;
    let every5deg = -((h / 65) as f32);
    let pitchoffset = -pitch * every5deg;
    let halfwidth = (w / 2) as f32;
    let halfheight = (h / 2) as f32;
    let center = at(wf / 2.0, hf / 2.0);

    // ---- sky / ground (rotated by -roll)
    let opts = input.opts;
    let hz = Xf::rot(center, if opts.russian { 0.0 } else { -roll });
    let (g1, g2) = match opts.ground {
        Some(c) => (Color32::from_rgb(c[0], c[1], c[2]), Color32::from_rgb(c[3], c[4], c[5])),
        None => (GROUND1, GROUND2),
    };
    let sky_h = halfheight * 2.0 + pitchoffset;
    let gnd_h = halfheight * 2.0 - pitchoffset;
    if let Some(v) = input.video {
        g.painter.image(v, Rect::from_min_size(o, Vec2::new(wf, hf)), Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)), Color32::WHITE);
    } else {
        if sky_h != 0.0 {
            g.grad_rect(hz, -halfwidth * 2.0, -halfheight * 2.0, wf * 2.0, sky_h, SKY1, SKY2);
        }
        if gnd_h != 0.0 {
            g.grad_rect(hz, -halfwidth * 2.0, pitchoffset, wf * 2.0, gnd_h, g1, g2);
        }
    }
    g.line(hz, -halfwidth * 2.0, pitchoffset, halfwidth * 2.0, pitchoffset, 2.0, white);

    // ---- pitch ladder, clipped below the heading tape
    {
        let clip = Rect::from_min_max(at(0.0, (h / 14) as f32), at(wf, hf));
        let gc = Hud { painter: g.painter.with_clip_rect(clip.intersect(g.painter.clip_rect())), ctx: g.ctx, w, h, o };
        let lengthshort = (w / 14) as f32;
        let lengthlong = (w / 10) as f32;
        let mut a = -90;
        while a <= 90 {
            let af = a as f32;
            if af >= pitch - 29.0 && af <= pitch + 20.0 {
                let y = pitchoffset + af * every5deg;
                if a % 10 == 0 {
                    let col = if a == 0 { GREEN } else { white };
                    gc.line(hz, -lengthlong, y, lengthlong, y, 2.0, col);
                    gc.text(hz, &a.to_string(), fontsize + 2.0, white, -lengthlong - 30.0 - (fontoffset * 1.7).trunc(), y - 8.0 - fontoffset);
                } else {
                    gc.line(hz, -lengthshort, y, lengthshort, y, 2.0, white);
                }
            }
            a += 5;
        }

        // roll needle (fixed)
        let ll = (h / 66) as f32;
        let extra = ((hf / 15.0) * 4.9).trunc();
        let lle = ll + 2.0;
        let c0 = Xf::at(center);
        let needle_w = if roll.abs() > 45.0 { 4.0 } else { 2.0 };
        gc.poly(c0, &[(0.0, -lle * 2.0 - extra), (-lle, -lle - extra), (lle, -lle - extra)], needle_w, RED);

        // roll scale (moves with roll)
        for a in [-60i32, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60] {
            let xf = Xf::rot(center, a as f32 - roll);
            gc.text(xf, &pad(a.abs(), 2), fontsize, white, -6.0 - fontoffset, -ll * 8.0 - extra);
            gc.line(xf, 0.0, -ll * 3.0 - extra, 0.0, -ll * 3.0 - extra - ll, 2.0, white);
        }
        let r = ll * 3.0 + extra;
        let start = 210.0 - roll;
        let pts: Vec<Pos2> = (0..=60)
            .map(|i| {
                let t = (start + i as f32 * 2.0).to_radians();
                Pos2::new(center.x + r * t.cos(), center.y + r * t.sin())
            })
            .collect();
        gc.painter.add(Shape::line(pts, Stroke::new(2.0, white)));

        // plane wings (rolling with the aircraft on the Russian HUD)
        let c0 = if opts.russian { Xf::rot(center, roll) } else { c0 };
        let red200 = Color32::from_rgba_unmultiplied(255, 0, 0, 200);
        let (l, rr) = (-halfwidth / 2.0, halfwidth / 2.0);
        gc.line(c0, l - (w / 2 / 5) as f32, 0.0, l, 0.0, 4.0, red200);
        gc.line(c0, rr, 0.0, rr + (w / 2 / 5) as f32, 0.0, 4.0, red200);
        let third = ((w / 2) / 3) as f32;
        let tenth = ((h / 2) / 10) as f32;
        gc.line(c0, -1.0, 0.0, rr - third, tenth, 4.0, red200);
        gc.line(c0, 1.0, 0.0, l + third, tenth, 4.0, red200);
    }

    // ---- heading tape
    let head_h = (h / 14) as f32;
    let headbg = Rect::from_min_size(o, Vec2::new(wf, head_h));
    g.rect_outline(base, 0.0, 0.0, wf, head_h, 2.0, Color32::BLACK);
    g.fill_rect(base, 0.0, 0.0, wf, head_h, SOLID);
    let hb = headbg.height();
    g.line(base, 5.0, hb - 5.0, wf - 5.0, hb - 5.0, 2.0, white);
    let space = (wf - 10.0) / 120.0;
    let start = (heading - 60.0).round() as i32;
    let target = cs.nav_bearing;
    if target < start as f32 {
        g.line(base, 5.0, hb, 5.0, 0.0, 6.0, GREEN);
    }
    if target > heading + 60.0 {
        g.line(base, 5.0 + space * 60.0, hb, 5.0 + space * 60.0, 0.0, 6.0, GREEN);
    }
    let mut a = start;
    while a as f32 <= heading + 60.0 {
        let x = 5.0 + space * (a - start) as f32;
        let norm = (a + 360) % 360;
        if norm == target as i32 {
            g.line(base, x, hb, x, 0.0, 6.0, GREEN);
        }
        if norm == cs.groundcourse as i32 {
            g.line(base, x, hb, x, 0.0, 6.0, Color32::BLACK);
        }
        let tx = -5.0 + space * (a - start) as f32 - fontoffset;
        let ty = hb - 24.0 - (fontoffset * 1.7).trunc();
        if a % 15 == 0 {
            g.line(base, x, hb - 5.0, x, hb - 10.0, 2.0, white);
            let disp = (a + 360 * 4) % 360;
            let card = match disp {
                0 => Some("N"),
                45 => Some("NE"),
                90 => Some("E"),
                135 => Some("SE"),
                180 => Some("S"),
                225 => Some("SW"),
                270 => Some("W"),
                315 => Some("NW"),
                _ => None,
            };
            match card {
                Some(c) => g.text(base, &pad(c, 2), fontsize + 4.0, white, tx, ty),
                None => g.text(base, &pad(disp, 3), fontsize, white, tx, ty),
            }
        } else if a % 5 == 0 {
            g.line(base, x, hb - 5.0, x, hb - 10.0, 2.0, white);
        }
        a += 1;
    }
    g.fill_rect(base, wf / 2.0 - (fontsize * 2.4) / 2.0, 0.0, fontsize * 2.4, hb, SLIGHTLY_WHITE);
    g.text(base, &pad((heading as i32).rem_euclid(360), 3), fontsize, white, wf / 2.0 - fontsize, hb - 24.0 - (fontoffset * 1.7).trunc());

    // ---- xtrack error and rate of turn
    {
        let xtspace = wf / 10.0 / 3.0;
        let padv = 10.0;
        let xt = cs.xtrack_error.clamp(-40.0, 40.0);
        let loc = xt / 20.0 * xtspace;
        let x10 = (w / 10) as f32;
        let gcol = if xt.abs() == 40.0 { GREEN.gamma_multiply(0.5) } else { GREEN };
        g.line(base, x10 + loc, hb + 5.0, x10 + loc, hb + (h / 10) as f32, 2.0, gcol);
        g.line(base, x10, hb + 5.0, x10, hb + (h / 10) as f32, 2.0, white);
        for k in [-2.0f32, -1.0, 1.0, 2.0] {
            g.line(base, x10 + xtspace * k, hb + 5.0 + padv, x10 + xtspace * k, hb + (h / 10) as f32 - padv, 2.0, white);
        }
        let yb = hb + (h / 10) as f32 + 10.0;
        for k in [-2.0f32, 0.0, 2.0] {
            g.line(base, x10 + xtspace * k - xtspace / 2.0, yb, x10 + xtspace * k - xtspace / 2.0 + xtspace, yb, 4.0, white);
        }
        let trwidth = (x10 + xtspace * 2.0 - xtspace / 2.0) - (x10 - xtspace * 2.0 - xtspace / 2.0);
        let range = 12.0;
        let tr = cs.yawspeed.clamp(-range / 2.0, range / 2.0);
        let loc = tr / range * trwidth;
        let gcol = if tr.abs() == range / 2.0 { GREEN.gamma_multiply(0.5) } else { GREEN };
        g.line(base, x10 + loc - xtspace / 2.0, yb + 3.0, x10 + loc + xtspace / 2.0, yb + 3.0, 4.0, gcol);
        g.line(base, x10 + loc, yb + 3.0, x10 + loc, yb + 10.0, 4.0, gcol);
    }

    // ---- left scroller (speed)
    let sb_top = halfheight - (h / 2 / 2) as f32;
    let sb_w = (w / 10) as f32;
    let sb_h = (h / 2) as f32;
    let speedunit = crate::units::speed_unit();
    {
        g.rect_outline(base, 0.0, sb_top, sb_w, sb_h, 2.0, white);
        g.fill_rect(base, 0.0, sb_top, sb_w, sb_h, SOLID);
        let tr = Xf::at(at(0.0, (h / 2) as f32));
        let viewrange = 26.0;
        let speed = crate::units::speed(if cs.airspeed == 0.0 { cs.groundspeed } else { cs.airspeed });
        let space = sb_h / viewrange;
        let start = (speed - viewrange / 2.0).trunc();
        let end = (speed + viewrange / 2.0).trunc() as i64;
        let mut a = start as i64;
        while a <= end {
            let y = sb_top - space * (a as f32 - start);
            if a % 5 == 0 {
                g.line(tr, sb_w, y, sb_w - 10.0, y, 2.0, white);
                g.text(tr, &pad(a, 5), fontsize, white, 0.0, y - 6.0 - fontoffset);
            }
            a += 1;
        }
        let arrow = [(0.0, -10.0), (sb_w - 10.0, -10.0), (sb_w - 5.0, 0.0), (sb_w - 10.0, 10.0), (0.0, 10.0)];
        g.fill_poly(tr, &arrow, Color32::BLACK);
        g.poly(tr, &arrow, 2.0, Color32::BLACK);
        g.text(tr, &format!("{:.0}{speedunit}", speed), 10.0, ALICEBLUE, 0.0, -9.0);
        let sb_bottom = sb_top + sb_h;
        g.text(base, &format!("AS {:.1}{speedunit}", crate::units::speed(cs.airspeed)), fontsize, white, 1.0, sb_bottom + 5.0);
        g.text(base, &format!("GS {:.1}{speedunit}", crate::units::speed(cs.groundspeed)), fontsize, white, 1.0, sb_bottom + fontsize + 2.0 + 10.0);
    }

    // ---- right scroller (altitude) + VSI
    let rl = wf - sb_w;
    let altunit = crate::units::alt_unit();
    {
        g.rect_outline(base, rl, sb_top, sb_w, sb_h, 2.0, white);
        g.fill_rect(base, rl, sb_top, sb_w, sb_h, SOLID);
        let tr = Xf::at(at(0.0, (h / 2) as f32));
        let viewrange = 26i64;
        let space = sb_h / viewrange as f32;
        let alt = crate::units::alt(cs.alt);
        let start = alt as i64 - viewrange / 2;
        let mut a = start;
        while (a as f32) <= alt + (viewrange / 2) as f32 {
            let y = sb_top - space * (a - start) as f32;
            if a == 0 && cs.home.is_some() {
                g.fill_rect(tr, rl, y, sb_w, space * (a - start) as f32, ALT_GROUND);
            }
            if a % 5 == 0 {
                g.line(tr, rl, y, rl + 10.0, y, 2.0, white);
                g.text(tr, &pad(a, 5), fontsize, white, rl, y - 6.0 - fontoffset);
            }
            a += 1;
        }

        // vsi
        let q = sb_w / 4.0;
        let sb_bottom = sb_top + sb_h;
        let vs = cs.verticalspeed.clamp(-6.0, 6.0);
        let scaled = vs / -12.0 * sb_h;
        let mid = sb_top + sb_h / 2.0;
        let mut peak = 0.0;
        if scaled > 0.0 {
            peak = -q;
            if mid + scaled + peak < mid {
                peak = -scaled;
            }
        } else if scaled < 0.0 {
            peak = q;
            if mid + scaled + peak > mid {
                peak = -scaled;
            }
        }
        g.fill_poly(base, &[(rl, mid), (rl - q, mid), (rl - q, mid + scaled + peak), (rl, mid + scaled)], BLUE);
        g.poly(base, &[(rl, sb_top), (rl - q, sb_top + q), (rl - q, sb_bottom - q), (rl, sb_bottom)], 2.0, white);
        let linespace = 1.0 / -12.0 * sb_h;
        for a in 1..12 {
            let y = sb_top - linespace * a as f32;
            g.line(base, rl - q, y, rl - sb_w / 8.0, y, 2.0, white);
        }

        let mirror = Xf::rot(at(wf, (h / 2) as f32), 180.0);
        let arrow = [(0.0, -10.0), (sb_w - 10.0, -10.0), (sb_w - 5.0, 0.0), (sb_w - 10.0, 10.0), (0.0, 10.0)];
        g.fill_poly(mirror, &arrow, Color32::BLACK);
        g.poly(mirror, &arrow, 2.0, Color32::BLACK);
        g.text(tr, &format!("{} {altunit}", alt as i32), 10.0, ALICEBLUE, rl + 10.0, -9.0);

        // mode, wp dist
        let mode_col = if input.mode_changed_secs < 2.0 { RED } else { white };
        g.text(base, &cs.mode, fontsize, mode_col, rl - 30.0, sb_bottom + 5.0);
        let wpd = crate::units::dist(cs.wp_dist);
        let (dist, unit) = if wpd >= 1000.0 {
            (format!("{}", (wpd / 1000.0 * 10.0).round() / 10.0), "k")
        } else {
            (format!("{}", wpd as i32), crate::units::dist_unit())
        };
        g.text(base, &format!("{dist}{unit}>{}", cs.wpno), fontsize, white, rl - 30.0, sb_bottom + fontsize + 2.0 + 10.0);

        // connection info
        let lq = cs.linkqualitygcs;
        let ytop = sb_top - (fontsize * 2.2).trunc() - 2.0;
        let ybot = sb_top - fontsize.trunc() - 2.0 - 20.0;
        if lq > 80.0 {
            g.line(base, rl - 5.0, ytop - 20.0, rl - 5.0, ybot, 2.0, GREEN);
        }
        if lq > 50.0 {
            g.line(base, rl - 10.0, ytop - 15.0, rl - 10.0, ybot, 2.0, GREEN);
        }
        if lq > 20.0 {
            g.line(base, rl - 15.0, ytop - 10.0, rl - 15.0, ybot, 2.0, GREEN);
        }
        g.text(base, &format!("{:.0}%", lq), fontsize, white, rl, ytop - 20.0);
        if lq == 0.0 {
            g.line(base, rl, ytop - 20.0, rl + 50.0, ytop, 2.0, RED);
            g.line(base, rl, ytop, rl + 50.0, ytop - 20.0, 2.0, RED);
        }
        let now = chrono_now();
        g.text(base, &now, fontsize, white, rl - 30.0, sb_top - fontsize - 2.0 - 20.0);
    }

    // ---- bottom lines
    let ybo = if fontsize >= 8.0 { (fontsize as i32 / 3) as f32 } else { 2.0 };
    let yto = fontsize + ybo + 2.0;
    let xpos = fontsize;
    let ypos = [hf - 2.0 * yto - ybo - 4.0, hf - yto - ybo - 4.0];

    let batt_col = if cs.battery_remaining > 0 && cs.battery_remaining < 15 { RED } else { white };
    let volts = if opts.cells > 0 { cs.battery_voltage / opts.cells as f32 } else { cs.battery_voltage };
    let bat_text = if opts.icons {
        // battery glyph in place of the "Bat1" prefix
        let bx = xpos;
        let by = ypos[1] + 2.0;
        g.rect_outline(base, bx, by, fontsize * 1.6, fontsize * 0.8, 1.5, batt_col);
        g.fill_rect(base, bx + fontsize * 1.6, by + fontsize * 0.2, fontsize * 0.2, fontsize * 0.4, batt_col);
        g.fill_rect(base, bx + 2.0, by + 2.0, (fontsize * 1.6 - 4.0) * (cs.battery_remaining.clamp(0, 100) as f32 / 100.0), fontsize * 0.8 - 4.0, batt_col);
        format!("       {:.2}v {:.1} A {}%", volts, cs.current, cs.battery_remaining)
    } else {
        format!("Bat1 {:.2}v {:.1} A {}%", volts, cs.current, cs.battery_remaining)
    };
    g.text(base, &bat_text, fontsize, batt_col, xpos, ypos[1]);
    // User Items, listed down the left side under the speed tape
    for (k, name) in opts.items.iter().enumerate() {
        if let Some(v) = cs.field(name) {
            g.text(base, &format!("{name}: {v:.2}"), fontsize, white, xpos, hf * 0.62 + k as f32 * (fontsize + 4.0));
        }
    }

    let (gps, gcol) = match cs.gpsstatus {
        0 => ("GPS: No GPS".to_string(), RED),
        1 => ("GPS: No Fix".to_string(), RED),
        2 => ("GPS: 2D Fix".to_string(), white),
        3 => ("GPS: 3D Fix".to_string(), white),
        4 => ("GPS: 3D dgps".to_string(), white),
        5 => ("GPS: rtk Float".to_string(), white),
        6 => ("GPS: rtk Fixed".to_string(), white),
        n => (n.to_string(), white),
    };
    g.text(base, &gps, fontsize, gcol, wf - 13.0 * fontsize, ypos[1]);

    // armed / disarmed / message (origin at centre)
    let c0 = Xf::at(center);
    if !cs.armed {
        let s = "DISARMED";
        let tw = g.text_width(s, fontsize + 10.0);
        g.text(c0, s, fontsize + 10.0, RED, -tw / 2.0, -halfheight / 3.0);
    } else if input.armed_secs < 8.0 {
        let s = "ARMED";
        let tw = g.text_width(s, fontsize + 20.0);
        g.text(c0, s, fontsize + 20.0, RED, -tw / 2.0, -halfheight / 3.0);
    }
    let message_high = cs.message_high();
    if !message_high.is_empty() {
        let col = if cs.message_high_severity <= 3 {
            RED
        } else if cs.message_high_severity <= 4 {
            YELLOW
        } else {
            Color32::WHITE
        };
        // calcfontsize: shrink until it fits Width - 100
        let mut fs = fontsize + 10.0;
        while fs > 6.0 && g.text_width(&message_high, fs) > wf - 100.0 {
            fs -= 1.0;
        }
        let tw = g.text_width(&message_high, fs);
        g.text(c0, &message_high, fs, col, -tw / 2.0, halfheight / 3.0);
    }

    // vibe / ekf / prearm
    let vmax = cs.vibex.max(cs.vibey).max(cs.vibez);
    let vcol = if vmax > 60.0 { RED } else if vmax > 30.0 { ORANGE } else { white };
    g.text(base, "Vibe", fontsize + 2.0, vcol, wf - 18.0 * fontsize, ypos[1]);
    if cs.load >= 100.0 {
        g.text(base, "CPU", fontsize + 2.0, RED, wf - 18.0 * fontsize + 40.0, ypos[1]);
    }
    let ecol = if cs.ekfstatus > 0.8 { RED } else if cs.ekfstatus > 0.5 { ORANGE } else { white };
    g.text(base, "EKF", fontsize + 2.0, ecol, wf - 23.0 * fontsize, ypos[1]);
    if !cs.armed && input.connected {
        let ready = cs.sensors_health & 0x1000_0000 != 0;
        let mut x = wf - 24.0 * fontsize;
        if !ready {
            x -= 2.0 * fontsize;
        }
        let (s, c) = if ready { ("Ready to Arm", white) } else { ("Not Ready to Arm", RED) };
        g.text(base, s, fontsize + 2.0, c, x, ypos[0] - 4.0);
    }
}

fn chrono_now() -> String {
    // local wall clock, HH:mm:ss
    let now = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs() as i64).unwrap_or(0);
    let off = local_offset_secs();
    let t = (now + off).rem_euclid(86400);
    format!("{:02}:{:02}:{:02}", t / 3600, (t / 60) % 60, t % 60)
}

fn local_offset_secs() -> i64 {
    use std::sync::OnceLock;
    static OFF: OnceLock<i64> = OnceLock::new();
    *OFF.get_or_init(|| mp_core::local_utc_offset_secs())
}

//! Images taken from Mission Planner's `Resources/` and GMap.NET (GPL-3.0, same as this port).

use egui::{ColorImage, TextureHandle, TextureOptions, TextureWrapMode};

pub struct Assets {
    pub fd: TextureHandle,
    pub fp: TextureHandle,
    pub setup: TextureHandle,
    pub config: TextureHandle,
    pub sim: TextureHandle,
    pub help: TextureHandle,
    pub connect: TextureHandle,
    pub disconnect: TextureHandle,
    pub bg: TextureHandle,
    pub logo: TextureHandle,
    pub up: TextureHandle,
    pub down: TextureHandle,
    pub marker_green: TextureHandle,
    pub marker_red: TextureHandle,
    pub marker_shadow: TextureHandle,
}

fn load(ctx: &egui::Context, name: &str, bytes: &[u8], repeat: bool) -> TextureHandle {
    let img = image::load_from_memory(bytes).expect("bundled image").to_rgba8();
    let ci = ColorImage::from_rgba_unmultiplied([img.width() as usize, img.height() as usize], img.as_raw());
    let mut opts = TextureOptions::LINEAR;
    if repeat {
        opts.wrap_mode = TextureWrapMode::Repeat;
    }
    ctx.load_texture(name, ci, opts)
}

macro_rules! asset {
    ($ctx:expr, $file:literal) => {
        load($ctx, $file, include_bytes!(concat!("../assets/", $file)), false)
    };
}

impl Assets {
    pub fn load(ctx: &egui::Context) -> Self {
        Self {
            fd: asset!(ctx, "light_flightdata_icon.png"),
            fp: asset!(ctx, "light_flightplan_icon.png"),
            setup: asset!(ctx, "light_initialsetup_icon.png"),
            config: asset!(ctx, "light_tuningconfig_icon.png"),
            sim: asset!(ctx, "light_simulation_icon.png"),
            help: asset!(ctx, "light_help_icon.png"),
            connect: asset!(ctx, "light_connect_icon.png"),
            disconnect: asset!(ctx, "light_disconnect_icon.png"),
            bg: load(ctx, "bgdark.jpg", include_bytes!("../assets/bgdark.jpg"), true),
            logo: asset!(ctx, "ardupilot_logo.png"),
            up: asset!(ctx, "up.png"),
            down: asset!(ctx, "down.png"),
            marker_green: asset!(ctx, "marker_green.png"),
            marker_red: asset!(ctx, "marker_red.png"),
            marker_shadow: asset!(ctx, "marker_shadow.png"),
        }
    }
}

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
    pub sitl_plane: TextureHandle,
    pub sitl_plane_over: TextureHandle,
    pub sitl_rover: TextureHandle,
    pub sitl_rover_over: TextureHandle,
    pub sitl_quad: TextureHandle,
    pub sitl_quad_over: TextureHandle,
    pub sitl_heli: TextureHandle,
    pub sitl_heli_over: TextureHandle,
    pub fw_rover: TextureHandle,
    pub fw_plane: TextureHandle,
    pub fw_quad: TextureHandle,
    pub fw_hexa: TextureHandle,
    pub fw_octaquad: TextureHandle,
    pub fw_sub: TextureHandle,
    pub fw_tracker: TextureHandle,
    pub fw_heli: TextureHandle,
    pub fw_tri: TextureHandle,
    pub fw_y6: TextureHandle,
    pub fw_octa: TextureHandle,
    pub fw_cube: TextureHandle,
    pub fc_quad: TextureHandle,
    pub fc_hexa: TextureHandle,
    pub fc_octa: TextureHandle,
    pub fc_octaquad: TextureHandle,
    pub fc_y6: TextureHandle,
    pub fc_heli: TextureHandle,
    pub fc_tri: TextureHandle,
    pub ft_plus: TextureHandle,
    pub ft_x: TextureHandle,
    pub ft_v: TextureHandle,
    pub ft_h: TextureHandle,
    pub ft_y6b: TextureHandle,
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
            sitl_plane: asset!(ctx, "sitl_plane_ImageNormal.png"),
            sitl_plane_over: asset!(ctx, "sitl_plane_ImageOver.png"),
            sitl_rover: asset!(ctx, "sitl_rover_ImageNormal.png"),
            sitl_rover_over: asset!(ctx, "sitl_rover_ImageOver.png"),
            sitl_quad: asset!(ctx, "sitl_quad_ImageNormal.png"),
            sitl_quad_over: asset!(ctx, "sitl_quad_ImageOver.png"),
            sitl_heli: asset!(ctx, "sitl_heli_ImageNormal.png"),
            sitl_heli_over: asset!(ctx, "sitl_heli_ImageOver.png"),
            fw_rover: asset!(ctx, "fw_rover.png"),
            fw_plane: asset!(ctx, "fw_plane.png"),
            fw_quad: asset!(ctx, "fw_quad.png"),
            fw_hexa: asset!(ctx, "fw_hexa.png"),
            fw_octaquad: asset!(ctx, "fw_octaquad.png"),
            fw_sub: asset!(ctx, "fw_sub.png"),
            fw_tracker: asset!(ctx, "fw_tracker.png"),
            fw_heli: asset!(ctx, "fw_heli.png"),
            fw_tri: asset!(ctx, "fw_tri.png"),
            fw_y6: asset!(ctx, "fw_y6.png"),
            fw_octa: asset!(ctx, "fw_octa.png"),
            fw_cube: asset!(ctx, "fw_cube.jpg"),
            fc_quad: asset!(ctx, "fc_quad.png"),
            fc_hexa: asset!(ctx, "fc_hexa.png"),
            fc_octa: asset!(ctx, "fc_octa.png"),
            fc_octaquad: asset!(ctx, "fc_octaquad.png"),
            fc_y6: asset!(ctx, "fc_y6.png"),
            fc_heli: asset!(ctx, "fc_heli.png"),
            fc_tri: asset!(ctx, "fc_tri.png"),
            ft_plus: asset!(ctx, "ft_plus.png"),
            ft_x: asset!(ctx, "ft_x.png"),
            ft_v: asset!(ctx, "ft_v.png"),
            ft_h: asset!(ctx, "ft_h.png"),
            ft_y6b: asset!(ctx, "ft_y6b.png"),
        }
    }
}

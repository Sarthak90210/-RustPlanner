//! Fonts. Mission Planner uses "Microsoft Sans Serif 8.25pt" for the UI and Arial for the HUD.
//! They are loaded from the OS at runtime (not redistributed). Metric-compatible fallbacks are
//! used where they are missing (Liberation Sans on Linux, Roboto on Android).

use std::sync::Arc;

use egui::{FontData, FontDefinitions, FontFamily};

pub const UI: &str = "mp-ui";
pub const ARIAL: &str = "mp-arial";
pub const BOLD: &str = "mp-bold";

/// 8.25pt at 96 dpi.
pub const UI_SIZE: f32 = 11.0;

fn first_existing(paths: &[&str]) -> Option<Vec<u8>> {
    paths.iter().find_map(|p| std::fs::read(p).ok())
}

fn ui_font() -> Option<Vec<u8>> {
    first_existing(&[
        "C:\\Windows\\Fonts\\micross.ttf",
        "/System/Library/Fonts/Supplemental/Microsoft Sans Serif.ttf",
        "/Library/Fonts/Microsoft Sans Serif.ttf",
        "/usr/share/fonts/truetype/msttcorefonts/Arial.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
        "/usr/share/fonts/liberation-sans/LiberationSans-Regular.ttf",
        "/usr/share/fonts/TTF/LiberationSans-Regular.ttf",
        "/system/fonts/Roboto-Regular.ttf",
    ])
}

fn arial() -> Option<Vec<u8>> {
    first_existing(&[
        "C:\\Windows\\Fonts\\arial.ttf",
        "/System/Library/Fonts/Supplemental/Arial.ttf",
        "/Library/Fonts/Arial.ttf",
        "/usr/share/fonts/truetype/msttcorefonts/Arial.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
        "/usr/share/fonts/liberation-sans/LiberationSans-Regular.ttf",
        "/usr/share/fonts/TTF/LiberationSans-Regular.ttf",
        "/system/fonts/Roboto-Regular.ttf",
    ])
}

fn arial_bold() -> Option<Vec<u8>> {
    first_existing(&[
        "C:\\Windows\\Fonts\\arialbd.ttf",
        "/System/Library/Fonts/Supplemental/Arial Bold.ttf",
        "/Library/Fonts/Arial Bold.ttf",
        "/usr/share/fonts/truetype/msttcorefonts/Arial_Bold.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
        "/usr/share/fonts/liberation-sans/LiberationSans-Bold.ttf",
        "/usr/share/fonts/TTF/LiberationSans-Bold.ttf",
        "/system/fonts/Roboto-Bold.ttf",
    ])
}

pub fn install(ctx: &egui::Context) {
    let mut defs = FontDefinitions::default();
    let mut defaults: Vec<String> = defs.families.get(&FontFamily::Proportional).cloned().unwrap_or_default();
    // GDI font linking gives WinForms CJK/Arabic glyphs; here the UI language's system font
    // follows every family
    if let Some(d) = first_existing(crate::i18n::script_font_paths(crate::i18n::culture())) {
        defs.font_data.insert("mp-script".into(), Arc::new(FontData::from_owned(d)));
        defaults.push("mp-script".into());
    }

    let mut add = |key: &str, data: Option<Vec<u8>>, family: FontFamily| {
        let mut chain = Vec::new();
        if let Some(d) = data {
            defs.font_data.insert(key.to_string(), Arc::new(FontData::from_owned(d)));
            chain.push(key.to_string());
        }
        chain.extend(defaults.iter().cloned());
        defs.families.insert(family, chain);
    };
    if crate::i18n::is_rtl(crate::i18n::culture()) {
        // egui shapes each font-face run in one direction, so Latin comes from a face with no
        // Arabic (egui's own) and Arabic from the script font; i18n::visual orders the runs
        for family in [FontFamily::Proportional, FontFamily::Name(ARIAL.into()), FontFamily::Name(BOLD.into())] {
            defs.families.insert(family, defaults.clone());
        }
    } else {
        add(UI, ui_font(), FontFamily::Proportional);
        add(ARIAL, arial(), FontFamily::Name(ARIAL.into()));
        add(BOLD, arial_bold(), FontFamily::Name(BOLD.into()));
    }
    ctx.set_fonts(defs);
}

pub fn arial_id(size: f32) -> egui::FontId {
    egui::FontId::new(size, FontFamily::Name(ARIAL.into()))
}

pub fn bold_id(size: f32) -> egui::FontId {
    egui::FontId::new(size, FontFamily::Name(BOLD.into()))
}

pub fn ui_id(size: f32) -> egui::FontId {
    egui::FontId::proportional(size)
}


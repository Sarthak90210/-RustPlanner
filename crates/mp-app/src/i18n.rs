//! UI languages (ConfigPlanner CMB_language / MainV2.changelanguage). Each culture's table is
//! exported from Mission Planner's .resx files by `scripts/export_translations.py`:
//! per-form control texts (ComponentResourceManager.ApplyResources) and an English-to-culture
//! map for the hand-built views. The culture is read once at startup; changing it asks for a
//! restart, as the original does.
use std::borrow::Cow;
use std::collections::HashMap;
use std::sync::OnceLock;

/// ConfigPlanner cultureCodes with CultureInfo.DisplayName.
pub const CULTURES: &[(&str, &str)] = &[
    ("en-US", "English (United States)"),
    ("zh-Hans", "Chinese (Simplified)"),
    ("zh-TW", "Chinese (Traditional, Taiwan)"),
    ("ru-RU", "Russian (Russia)"),
    ("fr", "French"),
    ("pl", "Polish"),
    ("it-IT", "Italian (Italy)"),
    ("es-ES", "Spanish (Spain)"),
    ("de-DE", "German (Germany)"),
    ("ja-JP", "Japanese (Japan)"),
    ("id-ID", "Indonesian (Indonesia)"),
    ("ko-KR", "Korean (Korea)"),
    ("ar", "Arabic"),
    ("pt", "Portuguese"),
    ("tr", "Turkish"),
    ("ru-KZ", "Russian (Kazakhstan)"),
    ("uk", "Ukrainian"),
];

macro_rules! tables {
    ($($c:literal),*) => { &[$(($c, include_str!(concat!("../assets/i18n/", $c, ".json")))),*] };
}

const TABLES: &[(&str, &str)] = tables!(
    "zh-Hans", "zh-TW", "ru-RU", "fr", "pl", "it-IT", "es-ES", "de-DE", "ja-JP", "id-ID", "ko-KR", "ar", "pt", "tr", "ru-KZ", "uk"
);

#[derive(serde::Deserialize, Default)]
pub struct Table {
    pub culture: String,
    #[serde(default)]
    pub forms: HashMap<String, HashMap<String, String>>,
    /// per original view (.resx type): English -> translation
    #[serde(default)]
    pub views: HashMap<String, HashMap<String, String>>,
    /// text every .resx translates the same way, for views without their own table
    #[serde(default)]
    pub text: HashMap<String, String>,
}

/// The original types (resx) whose resources apply to the UI drawn by a source file, as
/// ComponentResourceManager applies each type's own resources.
fn views_for(file: &str) -> &'static [&'static str] {
    let f = file.replace('\\', "/");
    let name = f.rsplit('/').next().unwrap_or("");
    if f.contains("/flight_plan") {
        &["FlightPlanner"]
    } else if matches!(name, "flight_data.rs" | "fd_tabs.rs" | "hud.rs" | "checklist.rs") {
        &["FlightData"]
    } else if name == "app.rs" {
        &["MainV2", "ConnectionControl"]
    } else if matches!(name, "setup.rs" | "setup_pages.rs") {
        &["InitialSetup", "ConfigMandatory", "ConfigOptional"]
    } else if name == "config.rs" {
        &["SoftwareConfig", "ConfigPlanner", "ConfigRawParams", "ConfigFriendlyParams"]
    } else if matches!(name, "tuning.rs" | "vehicle_tuning.rs") {
        &["ConfigArducopter", "ConfigSimplePids", "ConfigAC_Fence", "ConfigArduplane", "ConfigArdurover"]
    } else if name == "compass.rs" {
        &["ConfigHWCompass2", "ConfigHWCompass"]
    } else if matches!(name, "simulation.rs") {
        &["SITL"]
    } else if matches!(name, "more_tools.rs" | "tools.rs" | "tool_windows.rs" | "hotkey_tools.rs") {
        &["temp", "ConfigAdvanced"]
    } else if name == "swarm.rs" {
        &["FormationControl", "FollowPathControl", "LayoutEditor", "WPControl"]
    } else if name == "help.rs" {
        &["Help"]
    } else if name == "mavlog.rs" || name == "log_browse.rs" {
        &["LogBrowse", "MavlinkLog", "LogDownloadMavLink"]
    } else {
        &[]
    }
}

static ACTIVE: OnceLock<Option<Table>> = OnceLock::new();

/// Utilities.L10NU: the key=value tables for zh-Hans and uk (CurrentState DisplayText and
/// DisplayFieldName), chosen by CurrentUICulture.Name.
fn l10nu_table(culture: &str) -> HashMap<&'static str, &'static str> {
    let text: &'static str = match culture {
        "zh-CN" | "zh-Hans" => include_str!("../assets/i18n/l10nu_zhHans.txt"),
        "uk" | "uk-UA" => include_str!("../assets/i18n/l10nu_uk.txt"),
        _ => return HashMap::new(),
    };
    text.lines()
        .filter(|l| !l.starts_with('#'))
        .filter_map(|l| {
            let mut kv = l.split('=');
            match (kv.next(), kv.next(), kv.next()) {
                (Some(k), Some(v), None) => Some((k, v)),
                _ => None,
            }
        })
        .collect()
}

static L10NU: OnceLock<HashMap<&'static str, &'static str>> = OnceLock::new();

/// L10NU.GetString(key): None when the culture has no entry.
pub fn l10nu(key: &str) -> Option<&'static str> {
    L10NU.get_or_init(|| l10nu_table(culture())).get(key).copied()
}

/// The table for a culture name (case-insensitive, as CultureInfo names are).
pub fn table(culture: &str) -> Option<Table> {
    TABLES.iter().find(|(c, _)| c.eq_ignore_ascii_case(culture)).and_then(|(_, json)| serde_json::from_str(json).ok())
}

/// Settings "language": the culture for this run. Later calls keep the first one.
pub fn init(culture: &str) {
    let _ = ACTIVE.set(table(culture));
}

pub fn culture() -> &'static str {
    ACTIVE.get().and_then(|t| t.as_ref()).map(|t| t.culture.as_str()).unwrap_or("en-US")
}

fn lookup(table: &Table, file: &str, text: &str, fallback: bool) -> Option<String> {
    let key = text.trim();
    views_for(file)
        .iter()
        .find_map(|v| table.views.get(*v).and_then(|m| m.get(key)))
        .or_else(|| fallback.then(|| table.text.get(key)).flatten())
        .cloned()
}

/// The translated text for an English UI string drawn by the caller's view (unchanged when
/// there is none).
#[track_caller]
pub fn tr(text: &str) -> Cow<'_, str> {
    let file = std::panic::Location::caller().file();
    match ACTIVE.get().and_then(|t| t.as_ref()).and_then(|t| lookup(t, file, text, true)) {
        Some(t) => Cow::Owned(t),
        None => Cow::Borrowed(text),
    }
}

/// `tr` as an owned String, for widget constructors.
#[track_caller]
pub fn t(text: &str) -> String {
    tr(text).into_owned()
}

/// For labels, which also show data: only the caller view's own resources, never the
/// shared map (a value such as a mode name or "0" must not be rewritten).
#[track_caller]
pub fn tr_view(text: &str) -> Cow<'_, str> {
    let file = std::panic::Location::caller().file();
    match ACTIVE.get().and_then(|t| t.as_ref()).and_then(|t| lookup(t, file, text, false)) {
        Some(t) => Cow::Owned(t),
        None => Cow::Borrowed(text),
    }
}

/// Thread.CurrentUICulture without a saved setting: the OS display language, resolved with
/// .NET resource fallback (the culture itself, else its neutral parent) to a table.
pub fn resolve(os: &str) -> &'static str {
    let os = os.replace('_', "-");
    let os = os.split('.').next().unwrap_or("").split('@').next().unwrap_or("");
    let lower = os.to_ascii_lowercase();
    let special = match lower.as_str() {
        "zh-cn" | "zh-sg" | "zh-hans" | "zh-hans-cn" => Some("zh-Hans"),
        "zh-tw" | "zh-hant" | "zh-hant-tw" => Some("zh-TW"),
        _ => None,
    };
    if let Some(s) = special {
        return s;
    }
    let neutral = lower.split('-').next().unwrap_or("");
    CULTURES
        .iter()
        .map(|(c, _)| *c)
        .find(|c| c.eq_ignore_ascii_case(&lower))
        .or_else(|| CULTURES.iter().map(|(c, _)| *c).find(|c| c.eq_ignore_ascii_case(neutral)))
        .unwrap_or("en-US")
}

/// The OS display language (CultureInfo.CurrentUICulture.Name).
pub fn os_ui_culture() -> String {
    #[cfg(windows)]
    unsafe {
        use windows::Win32::Globalization::{GetUserDefaultUILanguage, LCIDToLocaleName};
        let mut buf = [0u16; 85];
        let n = LCIDToLocaleName(GetUserDefaultUILanguage() as u32, Some(&mut buf), 0);
        if n > 1 {
            return String::from_utf16_lossy(&buf[..n as usize - 1]);
        }
    }
    ["LC_ALL", "LC_MESSAGES", "LANG"].iter().find_map(|k| std::env::var(k).ok().filter(|v| !v.is_empty() && v != "C" && v != "POSIX")).unwrap_or_else(|| "en-US".into())
}

/// ComponentResourceManager.ApplyResources: a Designer form control's translated Text.
pub fn form_text(form: &str, control: &str) -> Option<&'static str> {
    ACTIVE.get()?.as_ref()?.forms.get(form)?.get(control).map(String::as_str)
}

/// The script font a culture needs beyond the Latin UI fonts (Windows links these itself).
pub fn script_font_paths(culture: &str) -> &'static [&'static str] {
    let c = culture.to_ascii_lowercase();
    if c.starts_with("zh-hans") || c == "zh-cn" {
        &[
            "C:\\Windows\\Fonts\\msyh.ttc",
            "C:\\Windows\\Fonts\\simsun.ttc",
            "/System/Library/Fonts/PingFang.ttc",
            "/System/Library/Fonts/Hiragino Sans GB.ttc",
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
            "/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc",
            "/usr/share/fonts/google-noto-cjk/NotoSansCJK-Regular.ttc",
            "/system/fonts/NotoSansCJK-Regular.ttc",
        ]
    } else if c.starts_with("zh") {
        &[
            "C:\\Windows\\Fonts\\msjh.ttc",
            "C:\\Windows\\Fonts\\mingliu.ttc",
            "/System/Library/Fonts/PingFang.ttc",
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
            "/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc",
            "/system/fonts/NotoSansCJK-Regular.ttc",
        ]
    } else if c.starts_with("ja") {
        &[
            "C:\\Windows\\Fonts\\YuGothR.ttc",
            "C:\\Windows\\Fonts\\meiryo.ttc",
            "C:\\Windows\\Fonts\\msgothic.ttc",
            "/System/Library/Fonts/ヒラギノ角ゴシック W3.ttc",
            "/System/Library/Fonts/Hiragino Sans GB.ttc",
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
            "/system/fonts/NotoSansCJK-Regular.ttc",
        ]
    } else if c.starts_with("ko") {
        &[
            "C:\\Windows\\Fonts\\malgun.ttf",
            "C:\\Windows\\Fonts\\gulim.ttc",
            "/System/Library/Fonts/AppleSDGothicNeo.ttc",
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
            "/system/fonts/NotoSansCJK-Regular.ttc",
        ]
    } else if c.starts_with("ar") {
        &[
            "C:\\Windows\\Fonts\\arial.ttf",
            "C:\\Windows\\Fonts\\tahoma.ttf",
            "/System/Library/Fonts/GeezaPro.ttc",
            "/usr/share/fonts/truetype/noto/NotoSansArabic-Regular.ttf",
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
            "/system/fonts/NotoNaskhArabic-Regular.ttf",
        ]
    } else {
        &[]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_offered_culture_has_a_table_with_menu_translations() {
        for (code, _) in CULTURES.iter().skip(1) {
            let t = table(code).unwrap_or_else(|| panic!("{code}"));
            assert!(!t.text.is_empty(), "{code}");
        }
        let zh = table("zh-hans").unwrap();
        assert_eq!(zh.text["DATA"], "飞行数据");
        assert_eq!(zh.text["Basic Tuning"], "基本调参");
        assert!(zh.forms["rangefinder"].contains_key("label1"));
        assert!(table("xx").is_none());
    }

    #[test]
    fn l10nu_tables_parse_like_the_original() {
        let uk = l10nu_table("uk");
        assert_eq!(uk["roll.Field"], "(fieldName) (крен)");
        assert_eq!(uk["Roll (deg)"], "Крен (deg)");
        assert!(l10nu_table("zh-Hans").contains_key("Roll (deg)"));
        assert!(l10nu_table("fr").is_empty());
    }

    #[test]
    fn views_keep_their_own_translation_and_labels_skip_the_shared_map() {
        let zh = table("zh-Hans").unwrap();
        // the PLAN context menu's Loiter, not another view's "Manual"
        assert_eq!(lookup(&zh, "src/views/flight_plan/menu.rs", "Loiter", true).as_deref(), Some("Loiter留待"));
        let ja = table("ja-JP").unwrap();
        assert!(!ja.text.contains_key("0"));
        assert_eq!(lookup(&ja, "src/views/config.rs", "0", false), None);
        let ko = table("ko-KR").unwrap();
        assert_eq!(lookup(&ko, "crates\\mp-app\\src\\views\\flight_plan.rs", "Load File", true).as_deref(), Some("파일 로드"));
        // no "&" mnemonics in any table
        for (c, _) in CULTURES.iter().skip(1) {
            let t = table(c).unwrap();
            assert!(t.text.values().chain(t.views.values().flat_map(|m| m.values())).all(|v| !v.contains('&') || v.contains("&&") || !v.chars().any(|ch| ch == '&')), "{c}");
        }
    }

    #[test]
    fn os_cultures_fall_back_like_resource_managers() {
        assert_eq!(resolve("zh-CN"), "zh-Hans");
        assert_eq!(resolve("zh_TW.UTF-8"), "zh-TW");
        assert_eq!(resolve("pt-BR"), "pt");
        assert_eq!(resolve("fr-CA"), "fr");
        assert_eq!(resolve("ja-JP"), "ja-JP");
        assert_eq!(resolve("de-AT"), "en-US", "no neutral de resources");
        assert_eq!(resolve("en-GB"), "en-US");
        assert_eq!(resolve("C"), "en-US");
    }

    #[test]
    fn english_passes_through() {
        // the process-wide culture is never set in tests
        assert_eq!(tr("DATA"), "DATA");
        assert_eq!(form_text("rangefinder", "label1"), None);
        assert_eq!(culture(), "en-US");
    }
}

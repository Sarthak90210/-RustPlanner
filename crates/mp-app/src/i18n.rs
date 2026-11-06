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
    #[serde(default)]
    pub text: HashMap<String, String>,
}

static ACTIVE: OnceLock<Option<Table>> = OnceLock::new();

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

/// The translated text for an English UI string (unchanged when there is none).
pub fn tr(text: &str) -> Cow<'_, str> {
    match ACTIVE.get().and_then(|t| t.as_ref()).and_then(|t| t.text.get(text.trim())) {
        Some(t) => Cow::Owned(t.clone()),
        None => Cow::Borrowed(text),
    }
}

/// `tr` as an owned String, for widget constructors.
pub fn t(text: &str) -> String {
    tr(text).into_owned()
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
    fn english_passes_through() {
        // the process-wide culture is never set in tests
        assert_eq!(tr("DATA"), "DATA");
        assert_eq!(form_text("rangefinder", "label1"), None);
        assert_eq!(culture(), "en-US");
    }
}

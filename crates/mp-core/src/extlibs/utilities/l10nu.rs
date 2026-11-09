//! Port of `ExtLibs/Utilities/L10NU.cs`
//!
//! `key=translation` tables embedded as resources, chosen once from the UI culture when the
//! class is first used: Simplified Chinese and Ukrainian.

use std::collections::HashMap;
use std::sync::OnceLock;

/// `Resources.strings_zhHans`
const STRINGS_ZHHANS: &str = include_str!("../../../assets/utilities/strings_zhHans.txt");
/// `Resources.strings_uk`
const STRINGS_UK: &str = include_str!("../../../assets/utilities/strings_uk.txt");

static STRINGS: OnceLock<HashMap<String, String>> = OnceLock::new();

/// `public static class L10NU`
pub struct L10NU;

impl L10NU {
    /// `public static Dictionary<string, string> strings`, built by the static constructor.
    pub fn strings() -> &'static HashMap<String, String> {
        STRINGS.get_or_init(|| {
            let mut strings = HashMap::new();
            let lang = crate::dotnet::current_ui_culture_name();
            let text = match lang.as_str() {
                "zh-CN" | "zh-Hans" => Some(STRINGS_ZHHANS),
                "uk" | "uk-UA" => Some(STRINGS_UK),
                _ => None,
            };
            if let Some(text) = text {
                // Regex.Split(text, "\r\n|\r|\n")
                let normalized = text.replace("\r\n", "\n").replace('\r', "\n");
                for line in normalized.split('\n') {
                    if line.starts_with('#') {
                        continue;
                    }
                    let kv: Vec<&str> = line.split('=').collect();
                    if kv.len() == 2 {
                        strings.insert(kv[0].to_string(), kv[1].to_string());
                    }
                }
            }
            strings
        })
    }

    /// `GetString(string key, string defaultTo)`
    pub fn get_string_or(key: &str, default_to: &str) -> String {
        match Self::strings().get(key) {
            Some(v) => v.clone(),
            None => default_to.to_string(),
        }
    }

    /// `GetString(string key)`: the key itself when there is no translation.
    pub fn get_string(key: &str) -> String {
        Self::get_string_or(key, key)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tables_parse() {
        assert!(STRINGS_UK.contains("Roll (deg)=Крен (deg)"));
        assert!(STRINGS_ZHHANS.contains("Mode=模式"));
        // with no translation table for the test culture the key comes back
        assert_eq!(L10NU::get_string_or("no such key", "x"), "x");
    }
}

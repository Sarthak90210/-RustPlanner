//! Port of `ExtLibs/Utilities/Settings.cs`
//!
//! Loads and saves app-level settings (`config.xml`) so UI state is preserved across sessions.
//! `Settings.config` is a static dictionary shared by every `Settings` object; `Instance` loads
//! it once.

use std::sync::{Mutex, MutexGuard, OnceLock, RwLock};

use super::displayview::{DisplayView, DisplayViewExtensions};
use crate::dotnet::collections::Dictionary;
use crate::dotnet::environment::{get_folder_path, SpecialFolder, DIRECTORY_SEPARATOR_CHAR};
use crate::dotnet::xml::{escape_text, read_nodes, XmlNode};
use crate::dotnet::{
    bool_try_parse, byte_try_parse, culture_compare, decimal_try_parse, double_try_parse, float_try_parse, int_try_parse, url_decode,
    url_encode, DateTime, DateTimeKind, Exception,
};

static INSTANCE: OnceLock<Settings> = OnceLock::new();
static APP_CONFIG_NAME: RwLock<String> = RwLock::new(String::new());
static FILE_NAME: RwLock<String> = RwLock::new(String::new());
static CUSTOM_USER_DATA_DIRECTORY: RwLock<String> = RwLock::new(String::new());
static GET_RUNNING_DIRECTORY: Mutex<String> = Mutex::new(String::new());
/// `public static Dictionary<string, string> config`
static CONFIG: Mutex<Option<Dictionary<String, String>>> = Mutex::new(None);

fn read(l: &RwLock<String>) -> String {
    l.read().unwrap_or_else(|e| e.into_inner()).clone()
}

/// The `config` dictionary, locked.
pub struct ConfigGuard(MutexGuard<'static, Option<Dictionary<String, String>>>);

impl std::ops::Deref for ConfigGuard {
    type Target = Dictionary<String, String>;
    fn deref(&self) -> &Dictionary<String, String> {
        self.0.as_ref().unwrap()
    }
}

impl std::ops::DerefMut for ConfigGuard {
    fn deref_mut(&mut self) -> &mut Dictionary<String, String> {
        self.0.as_mut().unwrap()
    }
}

/// `Path.Combine(a, b)` for a relative `b`.
fn path_combine(a: &str, b: &str) -> String {
    if a.is_empty() {
        return b.to_string();
    }
    if a.ends_with('\\') || a.ends_with('/') {
        format!("{a}{b}")
    } else {
        format!("{a}{DIRECTORY_SEPARATOR_CHAR}{b}")
    }
}

/// `Path.GetDirectoryName(path)`
fn get_directory_name(path: &str) -> String {
    match path.rfind(['\\', '/']) {
        Some(0) => path[..1].to_string(),
        Some(i) => path[..i].to_string(),
        None => String::new(),
    }
}

/// `MissionPlanner.Utilities.Settings`
pub struct Settings {
    user_agent: RwLock<String>,
}

impl Default for Settings {
    fn default() -> Self {
        Self::new()
    }
}

impl Settings {
    pub fn new() -> Self {
        Settings { user_agent: RwLock::new("MissionPlanner".into()) }
    }

    /// `public static Settings Instance`: created and loaded on first use; a load failure is
    /// swallowed.
    pub fn instance() -> &'static Settings {
        INSTANCE.get_or_init(|| {
            let s = Settings::new();
            let _ = s.load();
            s
        })
    }

    /// `AppConfigName` (default "Mission Planner")
    pub fn app_config_name() -> String {
        let s = read(&APP_CONFIG_NAME);
        if s.is_empty() {
            "Mission Planner".into()
        } else {
            s
        }
    }

    pub fn set_app_config_name(v: &str) {
        *APP_CONFIG_NAME.write().unwrap_or_else(|e| e.into_inner()) = v.to_string();
    }

    /// `FileName` (default "config.xml")
    pub fn file_name() -> String {
        let s = read(&FILE_NAME);
        if s.is_empty() {
            "config.xml".into()
        } else {
            s
        }
    }

    pub fn set_file_name(v: &str) {
        *FILE_NAME.write().unwrap_or_else(|e| e.into_inner()) = v.to_string();
    }

    /// `public static string CustomUserDataDirectory`
    pub fn custom_user_data_directory() -> String {
        read(&CUSTOM_USER_DATA_DIRECTORY)
    }

    pub fn set_custom_user_data_directory(v: &str) {
        *CUSTOM_USER_DATA_DIRECTORY.write().unwrap_or_else(|e| e.into_inner()) = v.to_string();
    }

    /// `Settings.config`
    pub fn config() -> ConfigGuard {
        let mut g = CONFIG.lock().unwrap_or_else(|e| e.into_inner());
        if g.is_none() {
            *g = Some(Dictionary::new());
        }
        ConfigGuard(g)
    }

    /// `this[string key]` getter: null when absent.
    pub fn get(&self, key: &str) -> Option<String> {
        Self::config().get(&key.to_string()).cloned()
    }

    /// `this[string key]` setter.
    pub fn set(&self, key: &str, value: &str) {
        Self::config().insert(key.to_string(), value.to_string());
    }

    /// `this[string key, string defaultvalue]`
    pub fn get_or(&self, key: &str, defaultvalue: &str) -> String {
        self.get(key).unwrap_or_else(|| defaultvalue.to_string())
    }

    /// `Keys`: a snapshot, so callers may add while enumerating.
    pub fn keys(&self) -> Vec<String> {
        Self::config().keys().cloned().collect()
    }

    pub fn contains_key(&self, key: &str) -> bool {
        Self::config().contains_key(&key.to_string())
    }

    pub fn user_agent(&self) -> String {
        read(&self.user_agent)
    }

    pub fn set_user_agent(&self, v: &str) {
        *self.user_agent.write().unwrap_or_else(|e| e.into_inner()) = v.to_string();
    }

    /// `ComPort` (key "comport")
    pub fn com_port(&self) -> Option<String> {
        self.get("comport")
    }

    pub fn set_com_port(&self, value: &str) {
        self.set("comport", value);
    }

    /// `APMFirmware`
    pub fn apm_firmware(&self) -> Option<String> {
        self.get("APMFirmware")
    }

    pub fn set_apm_firmware(&self, value: &str) {
        self.set("APMFirmware", value);
    }

    /// `GetString(key, default = "")`
    pub fn get_string(&self, key: &str, default: &str) -> String {
        self.get(key).unwrap_or_else(|| default.to_string())
    }

    /// `BaudRate`: the baud stored under `<ComPort>_BAUD` (a null port gives "_BAUD").
    pub fn baud_rate(&self) -> Option<String> {
        self.get(&format!("{}_BAUD", self.com_port().unwrap_or_default()))
    }

    pub fn set_baud_rate(&self, value: &str) {
        self.set(&format!("{}_BAUD", self.com_port().unwrap_or_default()), value);
    }

    /// `LogDir`: "logdirectory", else the default log directory.
    pub fn log_dir(&self) -> String {
        match self.get("logdirectory") {
            Some(d) if !d.is_empty() => d,
            _ => Self::get_default_log_dir(),
        }
    }

    pub fn set_log_dir(&self, value: &str) {
        self.set("logdirectory", value);
    }

    pub fn count(&self) -> i32 {
        Self::config().len() as i32
    }

    /// `GetDefaultLogDir()`: `<user data>logs`, created if missing.
    pub fn get_default_log_dir() -> String {
        let directory = format!("{}logs", Self::get_user_data_directory());
        if !std::path::Path::new(&directory).is_dir() {
            let _ = std::fs::create_dir_all(&directory);
        }
        directory
    }

    /// `GetList(key)`: ';' separated, URL-decoded, distinct.
    pub fn get_list(&self, key: &str) -> Vec<String> {
        match self.get(key) {
            Some(v) => {
                let mut out: Vec<String> = vec![];
                for a in v.split(';').map(url_decode) {
                    if !out.contains(&a) {
                        out.push(a);
                    }
                }
                out
            }
            None => vec![],
        }
    }

    /// `SetList(key, list)`: an empty list leaves the key unchanged.
    pub fn set_list(&self, key: &str, list: &[String]) {
        if list.is_empty() {
            return;
        }
        let mut distinct: Vec<&String> = vec![];
        for a in list {
            if !distinct.contains(&a) {
                distinct.push(a);
            }
        }
        let mut encoded: Vec<String> = vec![];
        for a in distinct.into_iter().map(|a| url_encode(a)) {
            if !encoded.contains(&a) {
                encoded.push(a);
            }
        }
        Self::config().insert(key.to_string(), encoded.join(";"));
    }

    /// `AppendList(key, item)`
    pub fn append_list(&self, key: &str, item: &str) {
        let mut list = self.get_list(key);
        list.push(item.to_string());
        self.set_list(key, &list);
    }

    /// `RemoveList(key, item)`: removes the key when nothing is left.
    pub fn remove_list(&self, key: &str, item: &str) {
        let list: Vec<String> = self.get_list(key).into_iter().filter(|a| a != item).collect();
        if list.is_empty() {
            Self::config().remove(&key.to_string());
            return;
        }
        self.set_list(key, &list);
    }

    pub fn get_int32(&self, key: &str, defaulti: i32) -> i32 {
        self.get(key).and_then(|v| int_try_parse(&v)).unwrap_or(defaulti)
    }

    /// `GetDisplayView(key)`
    pub fn get_display_view(&self, key: &str) -> DisplayView {
        self.get(key).and_then(|v| DisplayViewExtensions::try_parse(&v)).unwrap_or_default()
    }

    pub fn get_boolean(&self, key: &str, defaultb: bool) -> bool {
        self.get(key).and_then(|v| bool_try_parse(&v)).unwrap_or(defaultb)
    }

    pub fn get_float(&self, key: &str, defaultv: f32) -> f32 {
        self.get(key).and_then(|v| float_try_parse(&v)).unwrap_or(defaultv)
    }

    pub fn get_double(&self, key: &str, defaultd: f64) -> f64 {
        self.get(key).and_then(|v| double_try_parse(&v)).unwrap_or(defaultd)
    }

    pub fn get_decimal(&self, key: &str, defaultd: rust_decimal::Decimal) -> rust_decimal::Decimal {
        self.get(key).and_then(|v| decimal_try_parse(&v)).unwrap_or(defaultd)
    }

    pub fn get_byte(&self, key: &str, defaultb: u8) -> u8 {
        self.get(key).and_then(|v| byte_try_parse(&v)).unwrap_or(defaultb)
    }

    /// `GetRunningDirectory()`: the install directory (the executable's), with a trailing
    /// separator; cached.
    pub fn get_running_directory() -> String {
        {
            let g = GET_RUNNING_DIRECTORY.lock().unwrap_or_else(|e| e.into_inner());
            if !g.is_empty() {
                return g.clone();
            }
        }
        let location = match std::env::current_exe() {
            Ok(p) => p.to_string_lossy().to_string(),
            Err(_) => {
                let custom = Self::custom_user_data_directory();
                if !custom.is_empty() {
                    return format!("{custom}{DIRECTORY_SEPARATOR_CHAR}{}{DIRECTORY_SEPARATOR_CHAR}", Self::app_config_name());
                }
                return format!(".{DIRECTORY_SEPARATOR_CHAR}");
            }
        };
        let mut path = get_directory_name(&location);
        if path.is_empty() {
            path = get_directory_name(&Self::get_data_directory());
        }
        let r = format!("{path}{DIRECTORY_SEPARATOR_CHAR}");
        *GET_RUNNING_DIRECTORY.lock().unwrap_or_else(|e| e.into_inner()) = r.clone();
        r
    }

    /// `isMono()`: Mission Planner runs on Mono everywhere but Windows.
    fn is_mono() -> bool {
        !cfg!(windows)
    }

    /// `public static bool isUnix`
    pub fn is_unix() -> bool {
        cfg!(unix)
    }

    /// `GetDataDirectory()`: shared data (CommonApplicationData), or the user directory on Mono.
    pub fn get_data_directory() -> String {
        if Self::is_mono() {
            return Self::get_user_data_directory();
        }
        format!(
            "{}{DIRECTORY_SEPARATOR_CHAR}{}{DIRECTORY_SEPARATOR_CHAR}",
            get_folder_path(SpecialFolder::CommonApplicationData),
            Self::app_config_name()
        )
    }

    /// `GetUserDataDirectory()`: user data (MyDocuments, or LocalApplicationData on Unix when
    /// the old Documents folder does not exist), with a trailing separator.
    pub fn get_user_data_directory() -> String {
        let custom = Self::custom_user_data_directory();
        let app = Self::app_config_name();
        if !custom.is_empty() {
            return format!("{custom}{DIRECTORY_SEPARATOR_CHAR}{app}{DIRECTORY_SEPARATOR_CHAR}");
        }
        let old_approach_path = format!("{}{DIRECTORY_SEPARATOR_CHAR}{app}{DIRECTORY_SEPARATOR_CHAR}", get_folder_path(SpecialFolder::MyDocuments));
        let path = if Self::is_unix() && !std::path::Path::new(&old_approach_path).is_dir() {
            // Do not use new AppData path if old path already exists
            get_folder_path(SpecialFolder::LocalApplicationData)
        } else {
            get_folder_path(SpecialFolder::MyDocuments)
        };
        format!("{path}{DIRECTORY_SEPARATOR_CHAR}{app}{DIRECTORY_SEPARATOR_CHAR}")
    }

    /// `GetConfigFullPath()`: the user config path, migrating a config.xml found beside the
    /// program (and its sibling .xml files) on first run.
    fn get_config_full_path() -> Result<String, Exception> {
        // old path details
        let directory = Self::get_running_directory();
        if !std::path::Path::new(&directory).is_dir() {
            std::fs::create_dir_all(&directory)?;
        }
        let path = path_combine(&directory, &Self::file_name());

        // get new path details
        let newdir = Self::get_user_data_directory();
        if !std::path::Path::new(&newdir).is_dir() {
            std::fs::create_dir_all(&newdir)?;
        }
        let newpath = path_combine(&newdir, &Self::file_name());

        // check if oldpath config exists
        if std::path::Path::new(&path).is_file() && !std::path::Path::new(&newpath).is_file() {
            // move to new path
            std::fs::rename(&path, &newpath)?;

            // copy other xmls as this will be first run
            for e in std::fs::read_dir(&directory)?.flatten() {
                let p = e.path();
                if p.is_file() && p.extension().is_some_and(|x| x.eq_ignore_ascii_case("xml")) {
                    let name = p.file_name().unwrap().to_string_lossy().to_string();
                    let dest = format!("{newdir}{name}");
                    if std::path::Path::new(&dest).exists() {
                        return Err(Exception::Io(format!("The file '{dest}' already exists.")));
                    }
                    std::fs::copy(&p, &dest)?;
                }
            }
        }

        Ok(newpath)
    }

    /// `GetConfigDefaultsFullPath()`: `custom.config.xml` beside the program.
    fn get_config_defaults_full_path() -> String {
        path_combine(&Self::get_running_directory(), "custom.config.xml")
    }

    /// The elements `XmlTextReader` + `ReadString` see in the C# load loop, and the exception
    /// the loop ends with.
    fn read_config_elements(text: &str) -> (Vec<(String, String)>, Option<Exception>) {
        let (nodes, err) = read_nodes(text);
        let mut out = vec![];
        let mut p = 0usize;
        while p < nodes.len() {
            if let XmlNode::Element { name, empty } = &nodes[p] {
                match name.as_str() {
                    "Config" | "xml" => {}
                    _ => {
                        // ReadString: the text up to the next markup; the reader is left on that node
                        let mut value = String::new();
                        if !*empty {
                            let mut q = p + 1;
                            while let Some(XmlNode::Text(t)) = nodes.get(q) {
                                value.push_str(t);
                                q += 1;
                            }
                            if q >= nodes.len() && err.is_some() {
                                // the error was hit inside ReadString: the entry is not stored
                                break;
                            }
                            p = q;
                        }
                        out.push((name.clone(), value));
                    }
                }
            }
            p += 1;
        }
        (out, err)
    }

    /// `Load()`: the shipped `custom.config.xml` defaults (keys with `____` become `/`), then
    /// the user's config. A malformed user config is copied aside as `.failed` and the error
    /// rethrown, keeping the entries read before it.
    pub fn load(&self) -> Result<(), Exception> {
        // load the defaults
        let defaults = Self::get_config_defaults_full_path();
        if std::path::Path::new(&defaults).is_file() {
            if let Ok(text) = std::fs::read_to_string(&defaults) {
                let (els, _) = Self::read_config_elements(&text);
                let mut cfg = Self::config();
                for (name, value) in els {
                    let key = if name.contains("____") { name.replace("____", "/") } else { name };
                    cfg.insert(key, value);
                }
            }
        }

        let full = Self::get_config_full_path()?;
        if !std::path::Path::new(&full).is_file() {
            return Ok(());
        }

        let text = std::fs::read(&full).map_err(Exception::from)?;
        let text = String::from_utf8_lossy(&text).into_owned();
        let (els, err) = Self::read_config_elements(&text);
        {
            let mut cfg = Self::config();
            for (name, value) in els {
                cfg.insert(name, value);
            }
        }
        if let Some(ex) = err {
            let unix = (DateTime::now() - DateTime::new(1970, 1, 1, 0, 0, 0, DateTimeKind::Unspecified)?).total_milliseconds() / 1000.0;
            let _ = std::fs::copy(&full, format!("{full}{}.failed", unix as i32));
            return Err(ex);
        }
        Ok(())
    }

    /// `Save()`: `XmlTextWriter` (UTF-8 with BOM, indented) of every key in culture order.
    /// Keys with '/' are renamed `____` and then looked up under the new name, which is absent,
    /// so they are skipped as in C#; keys with characters XML names cannot hold are reported.
    pub fn save(&self) -> Result<(), Exception> {
        let filename = Self::get_config_full_path()?;
        let nl = if cfg!(windows) { "\r\n" } else { "\n" };

        let mut out = String::from("\u{feff}<?xml version=\"1.0\" encoding=\"utf-8\"?>");
        out.push_str(nl);
        out.push_str("<Config>");

        let cfg = Self::config();
        let mut keys: Vec<String> = cfg.keys().cloned().collect();
        keys.sort_by(|a, b| culture_compare(a, b));
        for key2 in keys {
            let mut key = key2.clone();
            if key.contains('/') {
                key = key.replace('/', "____");
            }
            if key.is_empty() || key.contains(['/', ' ', '-', ':', ';', '@', '!', '#', '$', '%']) {
                crate::dotnet::io::console_write_line(&format!("Bad config key {key}"));
                continue;
            }
            // config[key] with the renamed key: KeyNotFoundException, caught
            let Some(value) = cfg.get(&key) else { continue };
            out.push_str(nl);
            if value.is_empty() {
                out.push_str(&format!("  <{key} />"));
            } else {
                out.push_str(&format!("  <{key}>{}</{key}>", escape_text(value)));
            }
        }
        drop(cfg);

        out.push_str(nl);
        out.push_str("</Config>");
        std::fs::write(&filename, out.as_bytes())?;
        Ok(())
    }

    /// `Remove(key)`
    pub fn remove(&self, key: &str) {
        Self::config().remove(&key.to_string());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn read_string_semantics() {
        let (els, err) = Settings::read_config_elements(
            "<?xml version=\"1.0\"?>\r\n<Config>\r\n  <a>1</a>\r\n  <b />\r\n  <c>x<d>y</d></c>\r\n</Config>",
        );
        assert!(err.is_none());
        assert_eq!(els, vec![("a".into(), "1".into()), ("b".into(), "".into()), ("c".into(), "x".into())]);
        let (els, err) = Settings::read_config_elements("<Config><a>1</a><b>2</b><c>3</d></Config>");
        assert!(err.is_some());
        assert_eq!(els[0], ("a".to_string(), "1".to_string()));
    }

    #[test]
    fn lists_and_typed_getters() {
        let s = Settings::new();
        s.set_list("t_list", &["a b".into(), "c;d".into(), "a b".into()]);
        assert_eq!(s.get("t_list").unwrap(), "a+b;c%3Bd");
        assert_eq!(s.get_list("t_list"), vec!["a b".to_string(), "c;d".to_string()]);
        s.remove_list("t_list", "a b");
        s.remove_list("t_list", "c;d");
        assert!(!s.contains_key("t_list"));
        s.set("t_bool", " TRUE ");
        assert!(s.get_boolean("t_bool", false));
        s.set("t_int", "12x");
        assert_eq!(s.get_int32("t_int", 7), 7);
        s.set("t_byte", "300");
        assert_eq!(s.get_byte("t_byte", 1), 1);
        assert_eq!(s.get_or("t_missing", "d"), "d");
    }
}

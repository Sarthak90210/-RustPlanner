//! Port of `ExtLibs/Utilities/Warnings/WarningEngine.cs`
//!
//! The custom warnings list (saved as `warnings.xml` in the user data folder) and the loop
//! that checks it every 250 ms, speaking or raising `WarningMessage` for SpeakAndText items
//! and raising `QuickPanelColoring` for Coloring items. The static constructor (loading the
//! config) runs on first use; `MainLoop` runs on its own thread.

#![allow(non_snake_case)]

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, Once, RwLock};

use super::customwarning::{CustomWarning, WarningType};
use crate::dotnet::io::console_write_line;
use crate::dotnet::Exception;
use crate::extlibs::utilities::ispeech::ISpeech;
use crate::extlibs::utilities::settings::Settings;

/// `public static event EventHandler<string> WarningMessage`
pub type WarningMessageHandler = Arc<dyn Fn(&str) + Send + Sync>;
/// `public static event Action<string, string> QuickPanelColoring`: (field name, colour name)
pub type QuickPanelColoringHandler = Arc<dyn Fn(&str, &str) + Send + Sync>;

static WARNINGS: Mutex<Vec<CustomWarning>> = Mutex::new(Vec::new());
static RUN: AtomicBool = AtomicBool::new(false);
static INIT: Once = Once::new();
static SPEECH: RwLock<Option<Arc<dyn ISpeech>>> = RwLock::new(None);
static WARNING_MESSAGE: RwLock<Vec<WarningMessageHandler>> = RwLock::new(Vec::new());
static QUICK_PANEL_COLORING: RwLock<Vec<QuickPanelColoringHandler>> = RwLock::new(Vec::new());

/// `public class WarningEngine`
pub struct WarningEngine;

impl WarningEngine {
    /// `static WarningEngine()`: load the config; a failure is reported and ignored.
    fn static_init() {
        INIT.call_once(|| {
            if Self::LoadConfig().is_err() {
                console_write_line(&format!("Failed to read Warning config file {}", Self::warningconfigfile()));
            }
        });
    }

    /// `public static List<CustomWarning> warnings`: runs `f` under the list's lock.
    pub fn warnings<R>(f: impl FnOnce(&mut Vec<CustomWarning>) -> R) -> R {
        Self::static_init();
        let mut w = WARNINGS.lock().unwrap();
        f(&mut w)
    }

    /// `warningconfigfile = Settings.GetUserDataDirectory() + "warnings.xml"`
    pub fn warningconfigfile() -> String {
        Settings::get_user_data_directory() + "warnings.xml"
    }

    /// `LoadConfig()`: replaces the list with the file's (missing file: unchanged).
    pub fn LoadConfig() -> Result<(), Exception> {
        let path = Self::warningconfigfile();
        if !std::path::Path::new(&path).exists() {
            return Ok(());
        }
        let text = crate::dotnet::io::read_all_text(&path)?;
        let doc = roxmltree::Document::parse(text.trim_start_matches('\u{feff}')).map_err(|e| Exception::InvalidOperation(format!("There is an error in XML document ({e}).")))?;
        let root = doc.root_element();
        if root.tag_name().name() != "ArrayOfCustomWarning" {
            return Err(Exception::InvalidOperation("There is an error in XML document (2, 2).".into()));
        }
        let mut list = Vec::new();
        for c in root.children().filter(|c| c.is_element() && c.tag_name().name() == "CustomWarning") {
            list.push(CustomWarning::from_xml(c)?);
        }
        *WARNINGS.lock().unwrap() = list;
        Ok(())
    }

    /// `SaveConfig()`: `XmlSerializer` output (UTF-8 without a BOM, 2-space indent, CRLF).
    pub fn SaveConfig() -> Result<(), Exception> {
        Self::static_init();
        let mut out = String::from("<?xml version=\"1.0\" encoding=\"utf-8\"?>\r\n");
        out.push_str("<ArrayOfCustomWarning xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xmlns:xsd=\"http://www.w3.org/2001/XMLSchema\">\r\n");
        for w in WARNINGS.lock().unwrap().iter() {
            w.to_xml(&mut out, "CustomWarning", 1);
        }
        out.push_str("</ArrayOfCustomWarning>");
        let path = Self::warningconfigfile();
        std::fs::write(&path, out).map_err(|e| crate::dotnet::io::file_error(&path, &e))
    }

    /// `WarningMessage += handler`
    pub fn add_WarningMessage(handler: WarningMessageHandler) {
        WARNING_MESSAGE.write().unwrap().push(handler);
    }

    /// `QuickPanelColoring += handler`
    pub fn add_QuickPanelColoring(handler: QuickPanelColoringHandler) {
        QUICK_PANEL_COLORING.write().unwrap().push(handler);
    }

    /// `Start(speech)`: starts the loop if it is not running, then takes the speech engine.
    pub fn Start(speech: Option<Arc<dyn ISpeech>>) {
        Self::static_init();
        if !RUN.load(Ordering::SeqCst) {
            Self::MainLoop();
        }
        *SPEECH.write().unwrap() = speech;
    }

    /// `Stop()`
    pub fn Stop() {
        RUN.store(false, Ordering::SeqCst);
    }

    /// `MainLoop()`: one pass over the list every 250 ms while running. Any exception ends
    /// that pass.
    pub fn MainLoop() {
        RUN.store(true, Ordering::SeqCst);
        std::thread::spawn(|| {
            while RUN.load(Ordering::SeqCst) {
                let _ = Self::tick();
                std::thread::sleep(std::time::Duration::from_millis(250));
            }
        });
    }

    /// One pass of `MainLoop`'s try block.
    pub fn tick() -> Result<(), Exception> {
        let mut warnings = WARNINGS.lock().unwrap();
        for item in warnings.iter_mut() {
            // check primary condition
            if Self::checkCond(item)? {
                // Check item type
                if item.type_ == WarningType::SpeakAndText {
                    let speech = SPEECH.read().unwrap().clone();
                    if let Some(speech) = speech {
                        while !speech.is_ready() {
                            std::thread::yield_now();
                        }
                        speech.speak_async(&item.SayText()?);
                    }
                    let text = item.SayText()?;
                    for h in WARNING_MESSAGE.read().unwrap().iter() {
                        h(&text);
                    }
                } else if item.type_ == WarningType::Coloring {
                    let color = item.color.clone().unwrap_or_default();
                    for h in QUICK_PANEL_COLORING.read().unwrap().iter() {
                        h(item.Name(), &color);
                    }
                }
            }
            // if condition is not met, then color back the QV panel to default BackGround
            else if item.type_ == WarningType::Coloring {
                for h in QUICK_PANEL_COLORING.read().unwrap().iter() {
                    h(item.Name(), "NoColor");
                }
            }
        }
        Ok(())
    }

    /// `checkCond(item)`: the item and, recursively, its child must both match.
    fn checkCond(item: &mut CustomWarning) -> Result<bool, Exception> {
        // if there is a child go recursive
        if item.Child.is_some() {
            if item.CheckValue(true)? && Self::checkCond(item.Child.as_mut().unwrap())? {
                return Ok(true);
            }
        } else {
            // is no child then simple check
            if item.CheckValue(true)? {
                return Ok(true);
            }
        }
        Ok(false)
    }
}

#[cfg(test)]
mod tests {
    use super::super::customwarning::tests::Src;
    use super::super::customwarning::Conditional;
    use super::*;

    #[test]
    fn xml_round_trip() {
        CustomWarning::set_defaultsrc(Some(Arc::new(Src)));
        let mut w = CustomWarning::new();
        w.set_Name("alt").unwrap();
        w.ConditionType = Conditional::LT;
        w.Warning = 10.5;
        let mut child = CustomWarning::new();
        child.set_Name("alt").unwrap();
        w.Child = Some(Box::new(child));
        let mut out = String::new();
        w.to_xml(&mut out, "CustomWarning", 1);
        assert!(out.contains("<Warning>10.5</Warning>") && out.contains("<ConditionType>LT</ConditionType>") && !out.contains("<color>"));
        let doc = roxmltree::Document::parse(&out).unwrap();
        let back = CustomWarning::from_xml(doc.root_element()).unwrap();
        assert_eq!(back.Name(), "alt");
        assert_eq!(back.Warning, 10.5);
        assert!(back.Child.is_some());
        // the child (NONE) never matches, so the parent and child together do not
        let mut back = back;
        assert!(!WarningEngine::checkCond(&mut back).unwrap());
    }
}

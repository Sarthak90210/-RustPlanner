//! Port of `ExtLibs/ArduPilot/Mavlink/MAVAuthKeys.cs`
//!
//! MAVLink signing keys, kept in `<user data>authkeys.xml`: the `DataContractSerializer`
//! form of the `AuthKeys` dictionary, encrypted with `Crypto` (so tied to this machine).
//! The keys load on first use, as the C# static constructor does.

use std::sync::{Mutex, MutexGuard};

use base64::Engine;
use sha2::{Digest, Sha256};

use crate::dotnet::collections::Dictionary;
use crate::dotnet::Exception;
use crate::extlibs::utilities::{Crypto, Settings};

const LOG: &str = "MissionPlanner.Mavlink.MAVAuthKeys";

/// `[DataContract(Name = "AuthKey", Namespace = "")] public struct AuthKey`
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct AuthKey {
    pub name: Option<String>,
    pub key: Option<Vec<u8>>,
}

/// `[CollectionDataContract(ItemName = "AuthKeys", Namespace = "")] class AuthKeys : Dictionary<string, AuthKey>`
pub type AuthKeys = Dictionary<String, AuthKey>;

static KEYS: Mutex<Option<AuthKeys>> = Mutex::new(None);

/// `public class MAVAuthKeys`
pub struct MAVAuthKeys;

/// The root element `DataContractSerializer` gives the nested `MAVAuthKeys.AuthKeys` type.
const ROOT: &str = "MAVAuthKeys.AuthKeys";
const XSI: &str = "http://www.w3.org/2001/XMLSchema-instance";

fn esc(s: &str) -> String {
    crate::dotnet::xml::escape_text(s)
}

impl MAVAuthKeys {
    /// `static string keyfile`
    pub fn keyfile() -> String {
        format!("{}authkeys.xml", Settings::get_user_data_directory())
    }

    /// `public static AuthKeys Keys`, loaded on first access.
    pub fn keys() -> MutexGuard<'static, Option<AuthKeys>> {
        let mut g = KEYS.lock().unwrap_or_else(|e| e.into_inner());
        if g.is_none() {
            *g = Some(AuthKeys::new());
            if let Some(k) = Self::load_from_file() {
                *g = Some(k);
            }
        }
        g
    }

    /// `AddKey(name, seed)`: the key is SHA-256 of the UTF-8 seed.
    pub fn add_key(name: &str, seed: &str) {
        let shauser = Sha256::digest(seed.as_bytes()).to_vec();
        let mut g = Self::keys();
        g.as_mut().unwrap().insert(name.to_string(), AuthKey { key: Some(shauser), name: Some(name.to_string()) });
    }

    /// `DataContractSerializer.WriteObject` output for the dictionary.
    pub fn to_data_contract_xml(keys: &AuthKeys) -> String {
        let mut s = format!("<{ROOT} xmlns:i=\"{XSI}\" xmlns=\"\">");
        for (k, v) in keys.iter() {
            s.push_str("<AuthKeys>");
            s.push_str(&format!("<Key>{}</Key>", esc(k)));
            s.push_str("<Value>");
            match &v.key {
                Some(b) => s.push_str(&format!("<Key>{}</Key>", base64::engine::general_purpose::STANDARD.encode(b))),
                None => s.push_str("<Key i:nil=\"true\"/>"),
            }
            match &v.name {
                Some(n) => s.push_str(&format!("<Name>{}</Name>", esc(n))),
                None => s.push_str("<Name i:nil=\"true\"/>"),
            }
            s.push_str("</Value></AuthKeys>");
        }
        s.push_str(&format!("</{ROOT}>"));
        s
    }

    /// `DataContractSerializer.ReadObject` for the dictionary.
    pub fn from_data_contract_xml(xml: &str) -> Result<AuthKeys, Exception> {
        let doc = roxmltree::Document::parse(xml).map_err(|e| Exception::Serialization(e.to_string()))?;
        let root = doc.root_element();
        if root.tag_name().name() != ROOT {
            return Err(Exception::Serialization(format!(
                "Error in line 1 position 2. Expecting element '{ROOT}' from namespace ''.. Encountered 'Element'  with name '{}', namespace ''. ",
                root.tag_name().name()
            )));
        }
        let mut keys = AuthKeys::new();
        let text = |n: roxmltree::Node| -> Option<String> {
            if n.attribute((XSI, "nil")) == Some("true") {
                return None;
            }
            Some(n.children().filter(|c| c.is_text()).map(|c| c.text().unwrap_or("")).collect())
        };
        for item in root.children().filter(|n| n.is_element() && n.tag_name().name() == "AuthKeys") {
            let mut key: Option<String> = None;
            let mut value = AuthKey::default();
            for c in item.children().filter(|n| n.is_element()) {
                match c.tag_name().name() {
                    "Key" => key = text(c),
                    "Value" => {
                        for f in c.children().filter(|n| n.is_element()) {
                            match f.tag_name().name() {
                                "Key" => {
                                    value.key = match text(f) {
                                        Some(t) => Some(
                                            base64::engine::general_purpose::STANDARD
                                                .decode(t.trim())
                                                .map_err(|e| Exception::Serialization(e.to_string()))?,
                                        ),
                                        None => None,
                                    }
                                }
                                "Name" => value.name = text(f),
                                _ => {}
                            }
                        }
                    }
                    _ => {}
                }
            }
            let key = key.ok_or_else(|| Exception::ArgumentNull("Value cannot be null.\r\nParameter name: key".into()))?;
            if keys.contains_key(&key) {
                return Err(Exception::Argument("An item with the same key has already been added.".into()));
            }
            keys.insert(key, value);
        }
        Ok(keys)
    }

    /// `Save()`: FileMode.Create, through the Crypto encryptor.
    pub fn save() -> Result<(), Exception> {
        let xml = {
            let g = Self::keys();
            Self::to_data_contract_xml(g.as_ref().unwrap())
        };
        let rij = Crypto::new();
        let enc = rij.encrypt_buffer(xml.as_bytes())?;
        std::fs::write(Self::keyfile(), enc)?;
        Ok(())
    }

    fn load_from_file() -> Option<AuthKeys> {
        let path = Self::keyfile();
        if !std::path::Path::new(&path).is_file() {
            return None;
        }
        let r = (|| -> Result<AuthKeys, Exception> {
            let data = std::fs::read(&path)?;
            let rij = Crypto::new();
            let plain = rij.decrypt_buffer(&data)?;
            Self::from_data_contract_xml(&String::from_utf8_lossy(&plain))
        })();
        match r {
            Ok(k) => Some(k),
            Err(ex) => {
                log::error!(target: LOG, "{ex}");
                None
            }
        }
    }

    /// `internal static void Load()`: reloads the file; errors are logged and the current keys
    /// kept.
    pub fn load() {
        if let Some(k) = Self::load_from_file() {
            *KEYS.lock().unwrap_or_else(|e| e.into_inner()) = Some(k);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn data_contract_round_trip() {
        let mut k = AuthKeys::new();
        k.insert("a & b".into(), AuthKey { name: Some("a & b".into()), key: Some(vec![1, 2, 3]) });
        k.insert("n".into(), AuthKey { name: None, key: None });
        let x = MAVAuthKeys::to_data_contract_xml(&k);
        assert!(x.starts_with("<MAVAuthKeys.AuthKeys xmlns:i=\"http://www.w3.org/2001/XMLSchema-instance\" xmlns=\"\"><AuthKeys><Key>a &amp; b</Key><Value><Key>AQID</Key>"));
        let back = MAVAuthKeys::from_data_contract_xml(&x).unwrap();
        assert_eq!(back.get(&"a & b".to_string()).unwrap().key, Some(vec![1, 2, 3]));
        assert_eq!(back.get(&"n".to_string()).unwrap().name, None);
    }

    #[test]
    fn add_key_is_sha256_of_seed() {
        MAVAuthKeys::add_key("test-key", "seed");
        let g = MAVAuthKeys::keys();
        let k = g.as_ref().unwrap().get(&"test-key".to_string()).unwrap();
        assert_eq!(k.key.as_ref().unwrap().len(), 32);
        assert_eq!(k.key.as_ref().unwrap()[..4], Sha256::digest(b"seed")[..4]);
    }
}

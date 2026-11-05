//! Port of `ParameterMetaDataRepository`, backed by the bundled ParameterMetaDataBackup.xml.

use std::collections::HashMap;
use std::sync::OnceLock;

use crate::firmware::Firmware;

const META_XML: &str = include_str!("../assets/ParameterMetaDataBackup.xml");

#[derive(Debug, Clone, Default)]
pub struct ParamMeta {
    pub display_name: String,
    pub description: String,
    pub units: String,
    pub range: Option<(f64, f64)>,
    pub increment: Option<f64>,
    pub values: Vec<(i64, String)>,
    pub bitmask: Vec<(u32, String)>,
    pub user: String,
    pub reboot_required: bool,
    pub read_only: bool,
}

type Section = HashMap<String, ParamMeta>;

fn parse_values(s: &str) -> Vec<(i64, String)> {
    s.split(',')
        .filter_map(|kv| {
            let (k, v) = kv.split_once(':')?;
            Some((k.trim().parse::<f64>().ok()? as i64, v.trim().to_string()))
        })
        .collect()
}

fn all() -> &'static HashMap<String, Section> {
    static ALL: OnceLock<HashMap<String, Section>> = OnceLock::new();
    ALL.get_or_init(|| {
        let mut out: HashMap<String, Section> = HashMap::new();
        let xml = META_XML.trim_start_matches('\u{feff}');
        let doc = match roxmltree::Document::parse(xml) {
            Ok(d) => d,
            Err(e) => {
                log::error!("param metadata parse: {e}");
                return out;
            }
        };
        for fw in doc.root_element().children().filter(|n| n.is_element()) {
            let mut section: Section = HashMap::new();
            let mut copy_from: Vec<(String, String)> = Vec::new();
            for p in fw.children().filter(|n| n.is_element()) {
                let mut m = ParamMeta::default();
                for f in p.children().filter(|n| n.is_element()) {
                    let t = f.text().unwrap_or("").trim();
                    match f.tag_name().name() {
                        "DisplayName" => m.display_name = t.to_string(),
                        "Description" => m.description = t.to_string(),
                        "Units" => m.units = t.to_string(),
                        "Range" => {
                            let mut it = t.split_whitespace().filter_map(|x| x.parse::<f64>().ok());
                            if let (Some(a), Some(b)) = (it.next(), it.next()) {
                                m.range = Some((a, b));
                            }
                        }
                        "Increment" => m.increment = t.parse().ok(),
                        "Values" => m.values = parse_values(t),
                        "Bitmask" => {
                            m.bitmask = parse_values(t)
                                .into_iter()
                                .map(|(k, v)| (k as u32, v))
                                .collect()
                        }
                        "User" => m.user = t.to_string(),
                        "RebootRequired" => m.reboot_required = t.eq_ignore_ascii_case("true"),
                        "ReadOnly" => m.read_only = t.eq_ignore_ascii_case("true"),
                        "CopyValuesFrom" => {
                            copy_from.push((p.tag_name().name().to_string(), t.to_string()))
                        }
                        _ => {}
                    }
                }
                section.insert(p.tag_name().name().to_string(), m);
            }
            for (dst, src) in copy_from {
                if let Some(vals) = section.get(&src).map(|m| m.values.clone()) {
                    if let Some(d) = section.get_mut(&dst) {
                        d.values = vals;
                    }
                }
            }
            out.insert(fw.tag_name().name().to_string(), section);
        }
        out
    })
}

/// Parse the metadata on a background thread so the first lookup doesn't stall the UI.
pub fn warm_up() {
    std::thread::spawn(|| {
        let _ = all();
    });
}

pub fn get(fw: Firmware, name: &str) -> Option<&'static ParamMeta> {
    all().get(fw.meta_key())?.get(name)
}

pub fn options(fw: Firmware, name: &str) -> Vec<(i64, String)> {
    get(fw, name).map(|m| m.values.clone()).unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn copter_modes() {
        let o = options(Firmware::ArduCopter2, "FLTMODE3");
        assert!(o.iter().any(|(k, v)| *k == 5 && v == "Loiter"));
        let m = get(Firmware::ArduCopter2, "ACCEL_Z_FILT").unwrap();
        assert_eq!(m.units, "Hz");
        assert_eq!(m.range, Some((1.0, 100.0)));
    }
}

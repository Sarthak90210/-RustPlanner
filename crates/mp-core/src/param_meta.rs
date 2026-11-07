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
    /// the metadata's Values / Range text as written (the Options column shows these)
    pub values_raw: String,
    pub range_raw: String,
}

type Section = HashMap<String, ParamMeta>;

/// GetParameterOptionsInt / GetParameterBitMaskInt: "key:label" entries whose key int.Parse
/// accepts (others are skipped as bad entries); an entry without ':' is its own label.
fn parse_values(s: &str) -> Vec<(i64, String)> {
    s.split(',')
        .filter(|e| !e.is_empty())
        .filter_map(|kv| {
            let mut parts = kv.split(':');
            let key = parts.next()?.trim();
            let label = parts.next().map(str::trim).unwrap_or(key);
            let k: i32 = key.parse().ok()?;
            Some((k as i64, label.to_string()))
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
                            m.range_raw = t.to_string();
                            let mut it = t.split_whitespace().filter_map(|x| x.parse::<f64>().ok());
                            if let (Some(a), Some(b)) = (it.next(), it.next()) {
                                m.range = Some((a, b));
                            }
                        }
                        "Increment" => m.increment = t.parse().ok(),
                        "Values" => {
                            m.values_raw = t.to_string();
                            m.values = parse_values(t);
                        }
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
                if let Some((vals, raw)) = section.get(&src).map(|m| (m.values.clone(), m.values_raw.clone())) {
                    if let Some(d) = section.get_mut(&dst) {
                        d.values = vals;
                        d.values_raw = raw;
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

/// Every parameter name in a firmware's metadata with a plausible default (first option,
/// range minimum, or 0). Used by the simulator to look like a real autopilot.
pub fn all_defaults(fw: Firmware) -> Vec<(String, f32)> {
    let Some(sec) = all().get(fw.meta_key()) else { return Vec::new() };
    sec.iter()
        .filter(|(k, _)| !k.contains(':') && k.len() <= 16)
        .map(|(k, m)| {
            // 0 (usually "Disabled"/"None") when it is a valid value, else the first option / range start
            let zero_ok = m.values.iter().any(|(v, _)| *v == 0) || m.range.map(|(lo, hi)| lo <= 0.0 && hi >= 0.0).unwrap_or(m.values.is_empty());
            let v = if zero_ok { 0.0 } else { m.values.first().map(|(v, _)| *v as f32).or(m.range.map(|(lo, _)| lo as f32)).unwrap_or(0.0) };
            (k.clone(), v)
        })
        .collect()
}

#[cfg(test)]
mod option_tests {
    use super::*;

    #[test]
    fn options_int_drops_fractional_keys_like_getparameteroptionsint() {
        assert_eq!(parse_values("0:Disabled,1:Enabled"), [(0, "Disabled".to_string()), (1, "Enabled".to_string())]);
        // ACRO_RP_EXPO style keys are not integers: skipped, never written as 0
        assert_eq!(parse_values("0:Disabled,0.1:Very Low,0.2:Low"), [(0, "Disabled".to_string())]);
        assert_eq!(parse_values(" 3 : Three ,7"), [(3, "Three".to_string()), (7, "7".to_string())]);
        let expo = get(Firmware::ArduCopter2, "ACRO_RP_EXPO").expect("metadata");
        assert!(expo.values.iter().all(|(k, _)| *k == 0 || *k == 1 || *k == -1 || *k > 1), "only integer keys");
        assert!(expo.values_raw.contains("0.1"));
    }
}

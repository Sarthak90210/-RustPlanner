//! Port of `ParamFile`: Mission Planner `.param` files ("NAME,VALUE" per line, `#` comments).

use std::collections::BTreeMap;

pub fn load(text: &str) -> BTreeMap<String, f32> {
    let mut out = BTreeMap::new();
    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let mut it = line
            .split(|c: char| c == ',' || c == '\t' || c == ' ')
            .filter(|s| !s.is_empty());
        if let (Some(name), Some(val)) = (it.next(), it.next()) {
            if let Ok(v) = val.parse::<f32>() {
                out.insert(name.to_string(), v);
            }
        }
    }
    out
}

pub fn save<'a>(params: impl IntoIterator<Item = (&'a String, f32)>) -> String {
    let mut s = format!(
        "#NOTE: {}\r\n",
        chrono::Local::now().format("%d/%m/%Y %H:%M:%S")
    );
    for (name, value) in params {
        s.push_str(&format!("{name},{}\r\n", format_value(value)));
    }
    s
}

/// Format like Mission Planner: integers without a decimal point, floats with up to 7 digits.
pub fn format_value(v: f32) -> String {
    if v.fract() == 0.0 && v.abs() < 1e9 {
        format!("{}", v as i64)
    } else {
        let s = format!("{v:.7}");
        s.trim_end_matches('0').trim_end_matches('.').to_string()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip() {
        let mut m = BTreeMap::new();
        m.insert("ANGLE_MAX".to_string(), 4500.0);
        m.insert("ATC_RAT_RLL_P".to_string(), 0.135);
        let text = save(m.iter().map(|(k, v)| (k, *v)));
        let back = load(&text);
        assert_eq!(back["ANGLE_MAX"], 4500.0);
        assert!((back["ATC_RAT_RLL_P"] - 0.135).abs() < 1e-6);
    }
}

//! Port of `ParamFile`: Mission Planner `.param` files ("NAME,VALUE" per line, `#` comments).

use std::collections::BTreeMap;

#[derive(Debug, thiserror::Error)]
pub enum LoadError {
    #[error("{0}")]
    Io(#[from] std::io::Error),
    #[error("Invalid number on param {name} : {value} (line {line})")]
    InvalidNumber { name: String, value: String, line: usize },
}

// ExtLibs/Utilities/ParamFile.cs filters these in the shared reader, so both
// ConfigRawParams Load and ParamCompare exclude them.
const EXCLUDED: &[&str] = &["SYSID_SW_MREV", "WP_TOTAL", "CMD_TOTAL", "FENCE_TOTAL", "SYS_NUM_RESETS", "ARSPD_OFFSET", "GND_ABS_PRESS", "GND_TEMP", "BARO1_GND_PRESS", "BARO2_GND_PRESS", "BARO3_GND_PRESS", "BARO_GND_TEMP", "CMD_INDEX", "LOG_LASTFILE", "FORMAT_VERSION"];

pub fn load(text: &str) -> Result<BTreeMap<String, f32>, LoadError> {
    let mut out = BTreeMap::new();
    let text = text.strip_prefix('\u{feff}').unwrap_or(text);
    for (number, line) in text.lines().enumerate() {
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let mut it = line
            .split(|c: char| c == ',' || c == '\t' || c == ' ')
            .filter(|s| !s.is_empty());
        if let (Some(name), Some(val)) = (it.next(), it.next()) {
            let value = val.parse::<f64>().map_err(|_| LoadError::InvalidNumber { name: name.into(), value: val.into(), line: number + 1 })?;
            if !EXCLUDED.contains(&name) { out.insert(name.to_string(), value as f32); }
        }
    }
    Ok(out)
}

/// StreamReader-style BOM detection. Like .NET's default decoder, malformed
/// code units become replacement characters, then numeric parsing decides validity.
pub fn read(path: &std::path::Path) -> Result<BTreeMap<String, f32>, LoadError> {
    let bytes = std::fs::read(path)?;
    let text = if bytes.starts_with(&[0xff, 0xfe, 0, 0]) || bytes.starts_with(&[0, 0, 0xfe, 0xff]) {
        let little = bytes[0] == 0xff;
        let mut text: String = bytes[4..].chunks_exact(4).map(|b| {
            let code = if little { u32::from_le_bytes(b.try_into().unwrap()) } else { u32::from_be_bytes(b.try_into().unwrap()) };
            char::from_u32(code).unwrap_or(char::REPLACEMENT_CHARACTER)
        }).collect();
        if (bytes.len() - 4) % 4 != 0 { text.push(char::REPLACEMENT_CHARACTER); }
        text
    } else if bytes.starts_with(&[0xff, 0xfe]) || bytes.starts_with(&[0xfe, 0xff]) {
        let little = bytes[0] == 0xff;
        let units: Vec<u16> = bytes[2..].chunks_exact(2).map(|b| if little { u16::from_le_bytes([b[0],b[1]]) } else { u16::from_be_bytes([b[0],b[1]]) }).collect();
        let mut text = String::from_utf16_lossy(&units);
        if (bytes.len() - 2) % 2 != 0 { text.push(char::REPLACEMENT_CHARACTER); }
        text
    } else { String::from_utf8_lossy(&bytes).into_owned() };
    load(&text)
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
        let back = load(&text).unwrap();
        assert_eq!(back["ANGLE_MAX"], 4500.0);
        assert!((back["ATC_RAT_RLL_P"] - 0.135).abs() < 1e-6);
    }

    #[test]
    fn reference_exclusions_apply_to_all_imports_and_bad_numbers_abort_the_file() {
        let mut text = "ALT,5\n".to_string();
        for name in EXCLUDED { text.push_str(&format!("{name},99\n")); }
        assert_eq!(load(&text).unwrap(), BTreeMap::from([("ALT".into(), 5.0)]));
        assert!(matches!(load("ALT,5\nOTHER,bad\n"), Err(LoadError::InvalidNumber { line: 2, .. })));
        // Source parses a value before deciding whether to exclude its name.
        assert!(load("FORMAT_VERSION,bad").is_err());
    }

    #[test]
    fn bom_encoded_parameter_files_decode_like_streamreader() {
        let id = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_nanos();
        let path = std::env::temp_dir().join(format!("mp-param-bom-{}-{id}.param", std::process::id()));
        let text = "ALT,5\r\nFORMAT_VERSION,99\r\n";
        let mut encodings = vec![[vec![0xef,0xbb,0xbf], text.as_bytes().to_vec()].concat()];
        for little in [true, false] {
            let mut utf16 = if little { vec![0xff,0xfe] } else { vec![0xfe,0xff] };
            for c in text.encode_utf16() { utf16.extend_from_slice(&if little { c.to_le_bytes() } else { c.to_be_bytes() }); }
            encodings.push(utf16);
            let mut utf32 = if little { vec![0xff,0xfe,0,0] } else { vec![0,0,0xfe,0xff] };
            for c in text.chars() { utf32.extend_from_slice(&if little { (c as u32).to_le_bytes() } else { (c as u32).to_be_bytes() }); }
            encodings.push(utf32);
        }
        for bytes in encodings { std::fs::write(&path, bytes).unwrap(); assert_eq!(read(&path).unwrap(), BTreeMap::from([("ALT".into(), 5.0)])); }
        std::fs::remove_file(path).unwrap();
    }
}

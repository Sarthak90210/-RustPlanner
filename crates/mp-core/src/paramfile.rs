//! The app's `.param` file entry points, as a layer over the file-by-file port of
//! `ExtLibs/Utilities/ParamFile.cs` (`extlibs::utilities::paramfile::ParamFile`): loading
//! goes through `ParamFile.loadParamFile`'s rules (comment lines, separators, the excluded
//! counter/calibration names, FormatException on a bad number).

use std::collections::BTreeMap;

use crate::extlibs::utilities::paramfile::ParamFile;

#[derive(Debug, thiserror::Error)]
pub enum LoadError {
    #[error("{0}")]
    Io(#[from] std::io::Error),
    /// The exception `ParamFile.loadParamFile` throws (its `Message`).
    #[error("{}", .0.message())]
    Exception(crate::dotnet::Exception),
}

fn to_map(d: crate::dotnet::collections::Dictionary<String, f64>) -> BTreeMap<String, f32> {
    d.iter().map(|(k, v)| (k.clone(), *v as f32)).collect()
}

/// `ParamFile.loadParamFile` over the file's text.
pub fn load(text: &str) -> Result<BTreeMap<String, f32>, LoadError> {
    let text = text.strip_prefix('\u{feff}').unwrap_or(text);
    ParamFile::load_text(text).map(to_map).map_err(LoadError::Exception)
}

/// `ParamFile.loadParamFile(path)`: StreamReader BOM detection, then the C# parsing rules.
pub fn read(path: &std::path::Path) -> Result<BTreeMap<String, f32>, LoadError> {
    ParamFile::loadParamFile(&path.to_string_lossy()).map(to_map).map_err(LoadError::Exception)
}

/// `ParamFile.SaveParamFile` as ConfigRawParams calls it: each row's value cell text goes
/// through `double.Parse`, keys are written in SortedList order, no header.
pub fn save<'a>(params: impl IntoIterator<Item = (&'a String, f32)>) -> String {
    // data[name] = double.Parse(cell text): the cell holds the float's ToString() (G7, current
    // culture), parsed to a double whose ToString() SaveParamFile parses back in the same
    // culture, so the file gets the cell's digits ("0.135", not the widened float's G15)
    let list: Vec<(String, String)> = params
        .into_iter()
        .map(|(name, value)| {
            let cell = crate::dotnet::float_to_string(value);
            let text = crate::dotnet::double_parse(&cell).map(crate::dotnet::double_to_string).unwrap_or(cell);
            (name.clone(), text)
        })
        .collect();
    ParamFile::save_text(&list).unwrap_or_else(|e| {
        log::error!("{e}");
        String::new()
    })
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
        let excluded = [
            "SYSID_SW_MREV", "WP_TOTAL", "CMD_TOTAL", "FENCE_TOTAL", "SYS_NUM_RESETS", "ARSPD_OFFSET", "GND_ABS_PRESS", "GND_TEMP",
            "BARO1_GND_PRESS", "BARO2_GND_PRESS", "BARO3_GND_PRESS", "BARO_GND_TEMP", "CMD_INDEX", "LOG_LASTFILE", "FORMAT_VERSION",
        ];
        let mut text = "ALT,5\n".to_string();
        for name in excluded { text.push_str(&format!("{name},99\n")); }
        assert_eq!(load(&text).unwrap(), BTreeMap::from([("ALT".into(), 5.0)]));
        match load("ALT,5\nOTHER,bad\n") {
            Err(LoadError::Exception(e)) => assert_eq!(e.message(), "Invalid number on param OTHER : bad"),
            other => panic!("{other:?}"),
        }
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

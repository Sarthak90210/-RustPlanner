//! Port of `ExtLibs/Utilities/ParamFile.cs`

use crate::dotnet::collections::Dictionary;
use crate::dotnet::io::{read_lines, StreamWriter};
use crate::dotnet::{culture_compare, double_parse, double_parse_culture, double_to_string_culture, Exception, INVARIANT_CULTURE};

/// `public class ParamFile`: Mission Planner's .param files ("NAME,value" per line).
pub struct ParamFile;

/// Parameters `loadParamFile` drops: counters, calibration and state values that must not be
/// copied to another vehicle.
const SKIPPED: &[&str] = &[
    "SYSID_SW_MREV",
    "WP_TOTAL",
    "CMD_TOTAL",
    "FENCE_TOTAL",
    "SYS_NUM_RESETS",
    "ARSPD_OFFSET",
    "GND_ABS_PRESS",
    "GND_TEMP",
    "BARO1_GND_PRESS",
    "BARO2_GND_PRESS",
    "BARO3_GND_PRESS",
    "BARO_GND_TEMP",
    "CMD_INDEX",
    "LOG_LASTFILE",
    "FORMAT_VERSION",
];

impl ParamFile {
    /// `FileMask`
    pub const FILE_MASK: &'static str = "Parameter File|*.param;*.parm|All Files|*.*";

    /// `loadParamFile(string Filename)`: lines starting with '#' and lines with fewer than two
    /// fields (split on space, comma and tab) are skipped; a bad number throws FormatException.
    #[allow(non_snake_case)]
    pub fn loadParamFile(filename: &str) -> Result<Dictionary<String, f64>, Exception> {
        Self::load_lines(read_lines(filename)?)
    }

    /// The body of `loadParamFile` over text already decoded the way `StreamReader` does
    /// (`decode_detect`), for callers that hold the file contents rather than a path.
    pub fn load_text(text: &str) -> Result<Dictionary<String, f64>, Exception> {
        Self::load_lines(crate::dotnet::io::split_lines(text))
    }

    fn load_lines(lines: Vec<String>) -> Result<Dictionary<String, f64>, Exception> {
        let mut param: Dictionary<String, f64> = Dictionary::new();

        for line in lines {
            if line.starts_with('#') {
                continue;
            }

            let items: Vec<&str> = line.split([' ', ',', '\t']).filter(|s| !s.is_empty()).collect();

            if items.len() < 2 {
                continue;
            }

            let name = items[0];
            let value = match double_parse_culture(items[1], &INVARIANT_CULTURE) {
                Ok(v) => v,
                Err(ex) => {
                    log::error!("{ex}");
                    return Err(Exception::Format(format!("Invalid number on param {name} : {}", items[1])));
                }
            };

            if SKIPPED.contains(&name) {
                continue;
            }

            param.insert(name.to_string(), value);
        }

        Ok(param)
    }

    /// `SaveParamFile(string fn, Hashtable paramlist)`: one "NAME,value" line per entry,
    /// sorted by name in the current culture (`SortedList`), values written in the invariant
    /// culture. Each `Hashtable` value is given as its `ToString()` text, which the C#
    /// re-parses with `double.Parse` in the current culture.
    #[allow(non_snake_case)]
    pub fn SaveParamFile(path: &str, paramlist: &[(String, String)]) -> Result<(), Exception> {
        let mut sw = StreamWriter::create(path)?;
        Self::write_param_lines(paramlist, &mut |line| sw.write_line(line))?;
        sw.close()
    }

    /// The text `SaveParamFile` writes (each line ended with `Environment.NewLine`), for
    /// callers that save through a dialog of their own. An exception stops at the bad entry,
    /// as the C# loop does.
    pub fn save_text(paramlist: &[(String, String)]) -> Result<String, Exception> {
        let mut out = String::new();
        Self::write_param_lines(paramlist, &mut |line| {
            out.push_str(line);
            out.push_str(crate::dotnet::io::NEW_LINE);
            Ok(())
        })?;
        Ok(out)
    }

    /// `SaveParamFile`'s loop: keys in `SortedList` (current culture) order, each value
    /// re-parsed with `double.Parse` and written in the invariant culture.
    fn write_param_lines(paramlist: &[(String, String)], write_line: &mut dyn FnMut(&str) -> Result<(), Exception>) -> Result<(), Exception> {
        let mut list: Vec<&(String, String)> = paramlist.iter().collect();
        list.sort_by(|a, b| culture_compare(&a.0, &b.0));

        for (item, text) in list {
            let value = double_parse(text)?;

            let valueasstring = double_to_string_culture(value, &INVARIANT_CULTURE);

            if valueasstring.contains('.') {
                write_line(&format!("{item},{}", double_to_string_culture(value, &INVARIANT_CULTURE)))?;
            } else {
                write_line(&format!("{item},{valueasstring}"))?;
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn load_and_save() {
        let dir = std::env::temp_dir().join(format!("mp_paramfile_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let p = dir.join("a.param");
        std::fs::write(&p, "#comment\r\nRTL_ALT,1500\r\nWP_TOTAL 5\r\nAHRS_EKF_TYPE\t3\r\nonlyname\r\n").unwrap();
        let d = ParamFile::loadParamFile(p.to_str().unwrap()).unwrap();
        assert_eq!(d.keys().cloned().collect::<Vec<_>>(), vec!["RTL_ALT", "AHRS_EKF_TYPE"]);
        std::fs::write(&p, "X,abc\n").unwrap();
        assert_eq!(
            ParamFile::loadParamFile(p.to_str().unwrap()).unwrap_err(),
            Exception::Format("Invalid number on param X : abc".into())
        );
        let out = dir.join("b.param");
        ParamFile::SaveParamFile(out.to_str().unwrap(), &[("B".into(), "2".into()), ("A".into(), "1".into())]).unwrap();
        let text = std::fs::read_to_string(&out).unwrap();
        assert_eq!(text, format!("A,1{nl}B,2{nl}", nl = crate::dotnet::io::NEW_LINE));
        let _ = std::fs::remove_dir_all(&dir);
    }
}

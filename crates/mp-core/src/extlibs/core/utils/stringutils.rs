//! Port of `ExtLibs/Core/Utils/StringUtils.cs`
//!
//! Indices and lengths are UTF-16 code units as in C#. C# `null` strings are `Option<&str>`
//! where the C# code tests for or dereferences null. `IndexOf`/`StartsWith`/`EndsWith` with a
//! string argument compare ordinally here; the culture-sensitive .NET versions differ only for
//! ignorable characters, which the callers never pass.

use std::sync::OnceLock;

use crate::dotnet::convert::{int_try_parse_allow_thousands_current, string_compare_to};
use crate::dotnet::{int_try_parse, substring, Exception};

fn utf16(s: &str) -> Vec<u16> {
    s.encode_utf16().collect()
}

fn from_utf16(u: &[u16]) -> String {
    String::from_utf16_lossy(u)
}

fn index_of(hay: &[u16], needle: &[u16]) -> i32 {
    if needle.is_empty() {
        return 0;
    }
    if needle.len() > hay.len() {
        return -1;
    }
    (0..=hay.len() - needle.len()).find(|&i| &hay[i..i + needle.len()] == needle).map_or(-1, |i| i as i32)
}

/// `str.Substring(start)`
fn substring_from(s: &str, start: i32) -> Result<String, Exception> {
    let len = s.encode_utf16().count() as i32;
    if start < 0 {
        return Err(Exception::ArgumentOutOfRange("StartIndex cannot be less than zero.\r\nParameter name: startIndex".into()));
    }
    if start > len {
        return Err(Exception::ArgumentOutOfRange(
            "startIndex cannot be larger than length of string.\r\nParameter name: startIndex".into(),
        ));
    }
    substring(s, start, len - start)
}

/// `string.ToLower()` in the current culture: Turkish and Azeri map `I` to `ı`; a character
/// whose lower case is more than one character is kept.
fn to_lower_current(s: &str) -> String {
    let c = crate::dotnet::current_culture();
    let turkic = matches!(c.name.split('-').next(), Some("tr") | Some("az"));
    s.chars()
        .map(|ch| {
            if turkic && ch == 'I' {
                return '\u{131}';
            }
            let mut lo = ch.to_lowercase();
            match (lo.next(), lo.next()) {
                (Some(l), None) => l,
                _ => ch,
            }
        })
        .collect()
}

fn list_index_out_of_range() -> Exception {
    Exception::ArgumentOutOfRange(
        "Index was out of range. Must be non-negative and less than the size of the collection.\r\nParameter name: index".into(),
    )
}

/// `List<string>[index]`
pub(crate) fn list_get(list: &[String], index: usize) -> Result<&str, Exception> {
    list.get(index).map(|s| s.as_str()).ok_or_else(list_index_out_of_range)
}

/// `public static class StringUtils`
pub struct StringUtils;

/// `StringUtils.WILDCARD_STRING`
pub const WILDCARD_STRING: u16 = b'*' as u16;
/// `StringUtils.WILDCARD_CHAR`
pub const WILDCARD_CHAR: u16 = b'?' as u16;

impl StringUtils {
    /// `Split(string str, string delim)`: `str.Split(new[] { delim }, StringSplitOptions.None)`;
    /// null for a null string. An empty delimiter matches nothing on .NET Framework, so the
    /// whole string comes back.
    pub fn split(str: Option<&str>, delim: &str) -> Option<Vec<String>> {
        let s = str?;
        if delim.is_empty() {
            return Some(vec![s.to_string()]);
        }
        Some(s.split(delim).map(|x| x.to_string()).collect())
    }

    /// `SplitToList(string str, string delim)`: a null string throws from `new List<string>(null)`.
    pub fn split_to_list(str: Option<&str>, delim: &str) -> Result<Vec<String>, Exception> {
        Self::split(str, delim).ok_or_else(|| Exception::ArgumentNull("Value cannot be null.\r\nParameter name: collection".into()))
    }

    /// `SplitToList(string str, string delim, bool trim)`
    pub fn split_to_list_trim(str: Option<&str>, delim: &str, trim: bool) -> Result<Vec<String>, Exception> {
        if !trim {
            return Self::split_to_list(str, delim);
        }
        // foreach over a null array throws NullReferenceException
        let bits = Self::split(str, delim).ok_or_else(|| Exception::NullReference("Object reference not set to an instance of an object.".into()))?;
        Ok(bits.iter().map(|b| b.trim().to_string()).collect())
    }

    /// `IsNumeric(object Expression)`: `expression` is `Convert.ToString(Expression)` (an empty
    /// string for null), parsed with `NumberStyles.Any` and the invariant culture.
    pub fn is_numeric(expression: Option<&str>) -> bool {
        parse_number_style_any_invariant(expression.unwrap_or("")).is_some()
    }

    /// `IsAlphaNumeric(char x)`: `Regex(@"\w")` on the character, i.e.
    /// `[\p{L}\p{Mn}\p{Nd}\p{Pc}]`. A lone surrogate never matches.
    pub fn is_alpha_numeric_char(x: u16) -> bool {
        static RE: OnceLock<regex::Regex> = OnceLock::new();
        let re = RE.get_or_init(|| regex::Regex::new(r"[\p{L}\p{Mn}\p{Nd}\p{Pc}]").expect("static regex"));
        match char::from_u32(x as u32) {
            Some(c) => re.is_match(c.encode_utf8(&mut [0u8; 4])),
            None => false,
        }
    }

    /// `IsAlphaNumeric(string x)`
    pub fn is_alpha_numeric(x: &str) -> bool {
        x.encode_utf16().all(Self::is_alpha_numeric_char)
    }

    /// `SubstringAlphaNumericAtPosition(string str, int pos)`
    pub fn substring_alpha_numeric_at_position(str: &str, pos: i32) -> Result<String, Exception> {
        let chars = utf16(str);
        if pos < 0 || pos > chars.len() as i32 - 1 {
            return Ok(String::new());
        }
        let mut last = pos - 1;
        for &c in &chars[pos as usize..] {
            if Self::is_alpha_numeric_char(c) {
                last += 1;
            } else {
                break;
            }
        }
        if last >= pos {
            return substring(str, pos, last - pos + 1);
        }
        Ok(String::new())
    }

    /// `SubstringBetween(string str, string start, string end)`: the start index only skips one
    /// character of `start`, whatever its length.
    pub fn substring_between(str: &str, start: &str, end: &str) -> Result<String, Exception> {
        let s = utf16(str);
        let mut start_pos = index_of(&s, &utf16(start));
        if start_pos < 0 {
            return Ok(String::new());
        }
        start_pos += 1;
        let mut end_pos = index_of(&s, &utf16(end));
        if end_pos < start_pos {
            return substring_from(str, start_pos);
        }
        end_pos -= 1;
        substring(str, start_pos, end_pos - start_pos + 1)
    }

    /// `GetSmartPlural(int num, string noun)`
    pub fn get_smart_plural(num: i32, noun: &str) -> String {
        Self::get_smart_plural_with(num, noun, "s")
    }

    /// `GetSmartPlural(int num, string noun, string addToMakePlural)`
    pub fn get_smart_plural_with(num: i32, noun: &str, add_to_make_plural: &str) -> String {
        match num {
            0 => format!("are no {noun}{add_to_make_plural}"),
            1 => format!("is one {noun}"),
            _ => format!("are {num} {noun}{add_to_make_plural}"),
        }
    }

    /// `GetSmartPluralPastTense(int num, string noun)`
    pub fn get_smart_plural_past_tense(num: i32, noun: &str) -> String {
        Self::get_smart_plural_past_tense_with(num, noun, "s")
    }

    /// `GetSmartPluralPastTense(int num, string noun, string addToMakePlural)`
    pub fn get_smart_plural_past_tense_with(num: i32, noun: &str, add_to_make_plural: &str) -> String {
        match num {
            0 => format!("no {noun}{add_to_make_plural} were"),
            1 => format!("one {noun} was"),
            _ => format!("{num} {noun}{add_to_make_plural} were"),
        }
    }

    /// `RenameFolder(string oldDirPath, string proposedName)`: throws when the path has no
    /// directory separator (`Substring(0, -1)`).
    pub fn rename_folder(old_dir_path: &str, proposed_name: &str) -> Result<String, Exception> {
        let sep = std::path::MAIN_SEPARATOR;
        let units = utf16(old_dir_path);
        let last = units.iter().rposition(|&u| u == sep as u16).map_or(-1, |i| i as i32);
        Ok(format!("{}{}{}", substring(old_dir_path, 0, last)?, sep, proposed_name))
    }

    /// `RemoveFromFront(string str, string frontStr)`
    pub fn remove_from_front(str: &str, front_str: &str) -> Result<String, Exception> {
        if !str.starts_with(front_str) {
            return Err(Exception::Argument(format!("Error in RemoveFromFront: {str} does not start with {front_str}")));
        }
        substring_from(str, front_str.encode_utf16().count() as i32)
    }

    /// `RemoveExcessWhiteSpace(string str)`
    pub fn remove_excess_white_space(str: &str) -> String {
        let mut ans = str.to_string();
        while ans.contains("  ") {
            ans = ans.replace("  ", " ");
        }
        ans
    }

    /// `CompareAlphaNumeric(string a, string b)`
    pub fn compare_alpha_numeric(a: Option<&str>, b: Option<&str>) -> Result<i32, Exception> {
        let empty = |s: Option<&str>| s.is_none_or(|s| s.is_empty());
        if empty(a) && empty(b) {
            return Ok(0);
        }
        let ans = Self::compare_alpha_numeric_nums(a, b)?;
        if ans == 0 {
            // a is non-empty here: an empty a returned -1 above
            return Ok(string_compare_to(a.unwrap_or_default(), b));
        }
        Ok(ans)
    }

    fn compare_alpha_numeric_nums(a: Option<&str>, b: Option<&str>) -> Result<i32, Exception> {
        let a = match a {
            Some(a) if !a.is_empty() => a,
            _ => return Ok(-1),
        };
        let b = match b {
            Some(b) if !b.is_empty() => b,
            _ => return Ok(1),
        };
        if a.encode_utf16().count() != b.encode_utf16().count() {
            if let Some(test_a) = int_try_parse_allow_thousands_current(a) {
                if let Some(test_b) = int_try_parse_allow_thousands_current(b) {
                    return Ok(test_a.cmp(&test_b) as i32);
                }
            }
            let num_strt_a = Self::find_numeric_start(a);
            let num_strt_b = Self::find_numeric_start(b);
            if num_strt_a == 0 && num_strt_b == 0 {
                let num_a = Self::extract_number(a);
                let num_b = Self::extract_number(b);
                if num_a > 0 && num_b > 0 {
                    return Ok(num_a.cmp(&num_b) as i32);
                }
            }
            if num_strt_a > 0 && num_strt_b == num_strt_a && substring(a, 0, num_strt_a)? == substring(b, 0, num_strt_a)? {
                let post_a = substring_from(a, num_strt_a)?;
                let post_b = substring_from(b, num_strt_a)?;
                let num_a = Self::extract_number(&post_a);
                let num_b = Self::extract_number(&post_b);
                if num_a != -1 && num_b != -1 {
                    let ans = num_a.cmp(&num_b) as i32;
                    if ans != 0 {
                        return Ok(ans);
                    }
                    return Ok(string_compare_to(&post_a, Some(&post_b)));
                }
            }
        }
        if let Some(tst_a) = int_try_parse_allow_thousands_current(a) {
            if let Some(tst_b) = int_try_parse_allow_thousands_current(b) {
                return Ok(tst_a.cmp(&tst_b) as i32);
            }
        }
        Ok(string_compare_to(a, Some(b)))
    }

    /// `FindNumericStart(string numStr)`: the first character `int.TryParse` accepts on its own.
    pub fn find_numeric_start(num_str: &str) -> i32 {
        utf16(num_str).iter().position(|&u| (b'0' as u16..=b'9' as u16).contains(&u)).map_or(-1, |i| i as i32)
    }

    /// `FindNumericEnd(string numStr)`
    pub fn find_numeric_end(num_str: &str) -> Result<i32, Exception> {
        Self::find_numeric_end_from(Self::find_numeric_start(num_str), num_str)
    }

    /// `FindNumericEnd(int start, string numStr)`: a negative start reads `Substring(-1, 1)`
    /// and throws.
    pub fn find_numeric_end_from(start: i32, num_str: &str) -> Result<i32, Exception> {
        let len = num_str.encode_utf16().count() as i32;
        let mut i = start;
        while i < len {
            let one = substring(num_str, i, 1)?;
            if int_try_parse(&one).is_none() {
                return Ok(i);
            }
            i += 1;
        }
        Ok(-1)
    }

    /// `ExtractNumber(string numStr)`: a failed `int.TryParse` leaves `ans` at 0, which the
    /// digit loop then returns if its first prefix already fails.
    pub fn extract_number(num_str: &str) -> i32 {
        if let Some(v) = int_try_parse(num_str) {
            return v;
        }
        let mut ans = 0;
        let num_strt = Self::find_numeric_start(num_str);
        if num_strt == -1 {
            return -1;
        }
        let units = utf16(num_str);
        let mut concat: Vec<u16> = Vec::new();
        for &u in &units[num_strt as usize..] {
            concat.push(u);
            match int_try_parse(&from_utf16(&concat)) {
                Some(test) => ans = test,
                None => return ans,
            }
        }
        ans
    }

    /// `Join(IEnumerable list, string delim)`: each item is `Convert.ToString(o)`. An empty
    /// list with a non-empty delimiter throws (`Substring(0, -delim.Length)`).
    pub fn join<I, S>(list: I, delim: &str) -> Result<String, Exception>
    where
        I: IntoIterator<Item = S>,
        S: AsRef<str>,
    {
        let mut ans = String::new();
        for o in list {
            ans.push_str(o.as_ref());
            ans.push_str(delim);
        }
        let n = ans.encode_utf16().count() as i32 - delim.encode_utf16().count() as i32;
        substring(&ans, 0, n)
    }

    /// `AppendIfExists(string div, string val)`
    pub fn append_if_exists(div: &str, val: Option<&str>) -> String {
        match val {
            Some(v) if !v.is_empty() => format!("{div}{v}"),
            _ => String::new(),
        }
    }

    /// `WildcardEquals(string pattern, string str)`
    pub fn wildcard_equals(pattern: Option<&str>, str: Option<&str>) -> Result<bool, Exception> {
        Self::wildcard_equals_case(pattern, str, false)
    }

    /// `WildcardEquals(string pattern, string str, bool ignoreCase)`: with `ignoreCase` a null
    /// argument throws from `ToLower()` before the null checks.
    pub fn wildcard_equals_case(pattern: Option<&str>, str: Option<&str>, ignore_case: bool) -> Result<bool, Exception> {
        let (pattern, str) = if ignore_case {
            let nre = || Exception::NullReference("Object reference not set to an instance of an object.".into());
            let p = to_lower_current(pattern.ok_or_else(nre)?);
            let s = to_lower_current(str.ok_or_else(nre)?);
            (Some(p), Some(s))
        } else {
            (pattern.map(|s| s.to_string()), str.map(|s| s.to_string()))
        };
        let (Some(pattern), Some(str)) = (pattern, str) else { return Ok(false) };
        let p = utf16(&pattern);
        let s = utf16(&str);
        if p.len() == 1 && p[0] == WILDCARD_STRING {
            Ok(true)
        } else if s.is_empty() && !p.is_empty() {
            Ok(false)
        } else if !p.contains(&WILDCARD_STRING) && !p.contains(&WILDCARD_CHAR) {
            Ok(p == s)
        } else {
            Ok(Self::wildcard_equals_at(&p, 0, &s, 0))
        }
    }

    /// `internal static bool WildcardEquals(string pattern, int patternIdx, string string_Renamed, int stringIdx)`
    pub fn wildcard_equals_at(pattern: &[u16], pattern_idx: usize, string_renamed: &[u16], string_idx: usize) -> bool {
        let mut p = pattern_idx;
        let mut s = string_idx;
        loop {
            let s_end = s >= string_renamed.len();
            let p_end = p >= pattern.len();
            if s_end {
                let mut just_wildcards_left = true;
                let mut wildcard_search_pos = p;
                while wildcard_search_pos < pattern.len() && just_wildcards_left {
                    let wildchar = pattern[wildcard_search_pos];
                    if wildchar != WILDCARD_CHAR && wildchar != WILDCARD_STRING {
                        just_wildcards_left = false;
                    } else {
                        if wildchar == WILDCARD_CHAR {
                            return false;
                        }
                        wildcard_search_pos += 1;
                    }
                }
                if just_wildcards_left {
                    return true;
                }
            }
            if s_end || p_end {
                break;
            }
            if pattern[p] == WILDCARD_CHAR {
                p += 1;
                s += 1;
                continue;
            }
            if pattern[p] == WILDCARD_STRING {
                p += 1;
                let mut i = string_renamed.len() as isize;
                while i >= s as isize {
                    if Self::wildcard_equals_at(pattern, p, string_renamed, i as usize) {
                        return true;
                    }
                    i -= 1;
                }
                break;
            }
            if pattern[p] != string_renamed[s] {
                break;
            }
            p += 1;
            s += 1;
        }
        false
    }
}

/// `double.TryParse(s, NumberStyles.Any, NumberFormatInfo.InvariantInfo, out v)`: white space,
/// a leading or trailing sign or parentheses, the invariant currency symbol `¤`, `,` group
/// separators, `.` decimal point and an exponent.
pub(crate) fn parse_number_style_any_invariant(s: &str) -> Option<f64> {
    let mut t = s.trim_matches(|c: char| c.is_whitespace() || c == '\0').to_string();
    match t.as_str() {
        "Infinity" => return Some(f64::INFINITY),
        "-Infinity" => return Some(f64::NEG_INFINITY),
        "NaN" => return Some(f64::NAN),
        _ => {}
    }
    let mut neg = false;
    let mut signs = 0;
    if t.starts_with('(') && t.ends_with(')') && t.len() >= 2 {
        t = t[1..t.len() - 1].to_string();
        neg = true;
        signs += 1;
    }
    let strip_currency = |t: &mut String| {
        let tt = t.trim_matches(|c: char| c.is_whitespace()).to_string();
        let tt = tt.strip_prefix('¤').map(|x| x.trim_start().to_string()).unwrap_or(tt);
        let tt = tt.strip_suffix('¤').map(|x| x.trim_end().to_string()).unwrap_or(tt);
        *t = tt;
    };
    strip_currency(&mut t);
    if let Some(r) = t.strip_prefix('-') {
        t = r.to_string();
        neg = !neg;
        signs += 1;
    } else if let Some(r) = t.strip_prefix('+') {
        t = r.to_string();
        signs += 1;
    } else if let Some(r) = t.strip_suffix('-') {
        t = r.to_string();
        neg = !neg;
        signs += 1;
    } else if let Some(r) = t.strip_suffix('+') {
        t = r.to_string();
        signs += 1;
    }
    if signs > 1 {
        return None;
    }
    strip_currency(&mut t);
    let inv = &crate::dotnet::INVARIANT_CULTURE;
    let v = crate::dotnet::double_try_parse_culture(&t, inv)?;
    if t.starts_with(['-', '+']) {
        return None;
    }
    Some(if neg { -v } else { v })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn split_and_join() {
        assert_eq!(StringUtils::split_to_list(Some("a,,b"), ",").unwrap(), vec!["a", "", "b"]);
        assert!(StringUtils::split_to_list(None, ",").is_err());
        assert_eq!(StringUtils::split(Some("abc"), ""), Some(vec!["abc".to_string()]));
        assert_eq!(StringUtils::split_to_list_trim(Some(" a , b"), ",", true).unwrap(), vec!["a", "b"]);
        assert_eq!(StringUtils::join(["a", "b"], ", ").unwrap(), "a, b");
        assert!(StringUtils::join(Vec::<String>::new(), ",").is_err());
        assert_eq!(StringUtils::join(Vec::<String>::new(), "").unwrap(), "");
    }

    #[test]
    fn quirks_preserved() {
        // start skips one char only
        assert_eq!(StringUtils::substring_between("x[[ab]]", "[[", "]]").unwrap(), "[ab");
        assert_eq!(StringUtils::substring_between("abc", "z", "c").unwrap(), "");
        assert_eq!(StringUtils::substring_between("a(bc", "(", ")").unwrap(), "bc");
        assert_eq!(StringUtils::extract_number("abc12x"), 12);
        assert_eq!(StringUtils::extract_number("abc"), -1);
        assert_eq!(StringUtils::extract_number(" 7 "), 7);
        assert_eq!(StringUtils::find_numeric_end("ab12c").unwrap(), 4);
        assert_eq!(StringUtils::find_numeric_end("ab12").unwrap(), -1);
        assert!(StringUtils::find_numeric_end("abc").is_err());
        assert_eq!(StringUtils::get_smart_plural(0, "box"), "are no boxs");
        assert_eq!(StringUtils::get_smart_plural_past_tense(5, "box"), "5 boxs were");
        assert_eq!(StringUtils::remove_excess_white_space("a    b"), "a b");
        assert!(StringUtils::remove_from_front("abc", "b").is_err());
        assert_eq!(StringUtils::remove_from_front("abc", "a").unwrap(), "bc");
        assert_eq!(StringUtils::substring_alpha_numeric_at_position("ab_c d", 1).unwrap(), "b_c");
        assert_eq!(StringUtils::substring_alpha_numeric_at_position("ab", 5).unwrap(), "");
        assert!(StringUtils::rename_folder("nosep", "x").is_err());
    }

    #[test]
    fn alpha_numeric_compare() {
        let c = |a: &str, b: &str| StringUtils::compare_alpha_numeric(Some(a), Some(b)).unwrap();
        assert_eq!(c("2", "11"), -1);
        assert_eq!(c("A2", "A11"), -1);
        assert_eq!(c("2nd floor", "10th floor"), -1);
        assert_eq!(StringUtils::compare_alpha_numeric(None, Some("")).unwrap(), 0);
        assert_eq!(StringUtils::compare_alpha_numeric(None, Some("a")).unwrap(), -1);
        assert_eq!(StringUtils::compare_alpha_numeric(Some("a"), None).unwrap(), 1);
    }

    #[test]
    fn wildcards() {
        let w = |p: &str, s: &str| StringUtils::wildcard_equals(Some(p), Some(s)).unwrap();
        assert!(w("*", ""));
        assert!(w("c*t", "cat"));
        assert!(w("c?t", "cat"));
        assert!(!w("ca??", "cat"));
        assert!(w("ca*", "ca"));
        assert!(!w("abc", "abd"));
        assert!(!w("a", ""));
        assert!(StringUtils::wildcard_equals_case(None, Some("a"), true).is_err());
        assert!(!StringUtils::wildcard_equals(None, Some("a")).unwrap());
        assert!(StringUtils::wildcard_equals_case(Some("A*"), Some("ab"), true).unwrap());
    }

    #[test]
    fn numeric_any_style() {
        assert!(StringUtils::is_numeric(Some(" (1,000.5) ")));
        assert!(StringUtils::is_numeric(Some("5-")));
        assert!(StringUtils::is_numeric(Some("¤3")));
        assert!(StringUtils::is_numeric(Some("1e3")));
        assert!(!StringUtils::is_numeric(Some("--5")));
        assert!(!StringUtils::is_numeric(None));
        assert!(StringUtils::is_alpha_numeric("a_1é"));
        assert!(!StringUtils::is_alpha_numeric("a-"));
    }
}

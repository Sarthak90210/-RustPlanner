//! Newtonsoft.Json (`JsonConvert`) output and input as the ported code uses it.
//!
//! Writing: `SerializeObject(obj)` and `SerializeObject(obj, Formatting.Indented)` with the
//! default settings. Properties come out in declaration order, null values are written,
//! doubles use .NET's "R" text with a forced ".0", and indented output uses two spaces and
//! `Environment.NewLine` (the `StringWriter` default).
//!
//! Reading: `DeserializeObject<T>` property matching is exact first and then
//! case-insensitive; `JsonObj` gives that lookup over a parsed document.

use serde_json::Value;

use super::{format_general, CultureInfo, Exception, INVARIANT_CULTURE};

/// .NET Framework "R" text for a double: G15 if it round-trips, else G17.
pub fn r_double(x: f64) -> String {
    let c: &CultureInfo = &INVARIANT_CULTURE;
    let s15 = format_general(x, 15, c);
    if s15.parse::<f64>().ok() == Some(x) {
        s15
    } else {
        format_general(x, 17, c)
    }
}

/// `JsonWriter` floating point output: NaN/Infinity as strings, otherwise the number text,
/// which always carries a decimal place.
pub fn json_float(text: String) -> String {
    if text == "NaN" || text == "Infinity" || text == "-Infinity" {
        return format!("\"{text}\"");
    }
    if text.contains('.') || text.contains('E') || text.contains('e') {
        text
    } else {
        format!("{text}.0")
    }
}

/// A JSON string literal as `JsonTextWriter` escapes it (`StringEscapeHandling.Default`):
/// short escapes for \b \t \n \f \r \" \\, lower-case `\u00xx` for the other control
/// characters, and `\u0085`, ` `, ` `. Everything else, including '/' and DEL, is
/// written as is.
pub fn json_str(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    out.push('"');
    for c in s.chars() {
        match c {
            '\u{8}' => out.push_str("\\b"),
            '\t' => out.push_str("\\t"),
            '\n' => out.push_str("\\n"),
            '\u{c}' => out.push_str("\\f"),
            '\r' => out.push_str("\\r"),
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            c if (c as u32) < 0x20 || c == '\u{85}' || c == '\u{2028}' || c == '\u{2029}' => {
                out.push_str(&format!("\\u{:04x}", c as u32));
            }
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

/// A value as `JsonSerializer` writes it.
#[derive(Debug, Clone, PartialEq)]
pub enum J {
    Null,
    Bool(bool),
    Int(i64),
    Double(f64),
    Str(String),
    Arr(Vec<J>),
    /// properties in declaration order
    Obj(Vec<(String, J)>),
}

impl J {
    fn scalar_text(&self) -> Option<String> {
        Some(match self {
            J::Null => "null".into(),
            J::Bool(b) => if *b { "true" } else { "false" }.into(),
            J::Int(i) => i.to_string(),
            J::Double(d) => json_float(r_double(*d)),
            J::Str(s) => json_str(s),
            _ => return None,
        })
    }

    /// `Formatting.None`
    pub fn to_compact(&self) -> String {
        if let Some(s) = self.scalar_text() {
            return s;
        }
        match self {
            J::Arr(a) => format!("[{}]", a.iter().map(J::to_compact).collect::<Vec<_>>().join(",")),
            J::Obj(o) => format!("{{{}}}", o.iter().map(|(k, v)| format!("{}:{}", json_str(k), v.to_compact())).collect::<Vec<_>>().join(",")),
            _ => unreachable!(),
        }
    }

    /// `Formatting.Indented`
    pub fn to_indented(&self) -> String {
        let mut out = String::new();
        self.write_indented(&mut out, 0);
        out
    }

    fn write_indented(&self, out: &mut String, depth: usize) {
        let nl = super::io::NEW_LINE;
        if let Some(s) = self.scalar_text() {
            out.push_str(&s);
            return;
        }
        match self {
            J::Arr(a) => {
                if a.is_empty() {
                    out.push_str("[]");
                    return;
                }
                out.push('[');
                for (i, v) in a.iter().enumerate() {
                    if i > 0 {
                        out.push(',');
                    }
                    out.push_str(nl);
                    out.push_str(&"  ".repeat(depth + 1));
                    v.write_indented(out, depth + 1);
                }
                out.push_str(nl);
                out.push_str(&"  ".repeat(depth));
                out.push(']');
            }
            J::Obj(o) => {
                if o.is_empty() {
                    out.push_str("{}");
                    return;
                }
                out.push('{');
                for (i, (k, v)) in o.iter().enumerate() {
                    if i > 0 {
                        out.push(',');
                    }
                    out.push_str(nl);
                    out.push_str(&"  ".repeat(depth + 1));
                    out.push_str(&json_str(k));
                    out.push_str(": ");
                    v.write_indented(out, depth + 1);
                }
                out.push_str(nl);
                out.push_str(&"  ".repeat(depth));
                out.push('}');
            }
            _ => unreachable!(),
        }
    }
}

fn reader_error(msg: String) -> Exception {
    Exception::JsonReader(msg)
}

fn conversion_error(v: &Value, ty: &str, path: &str) -> Exception {
    Exception::JsonSerialization(format!("Error converting value {} to type '{ty}'. Path '{path}'.", value_display(v)))
}

/// How Newtonsoft quotes a value in its conversion errors.
fn value_display(v: &Value) -> String {
    match v {
        Value::Null => "{null}".into(),
        Value::String(s) => format!("\"{s}\""),
        other => other.to_string(),
    }
}

/// `JsonConvert.DeserializeObject` entry: parse the text; an empty document gives null.
pub fn parse(text: &str) -> Result<Option<Value>, Exception> {
    if text.trim().is_empty() {
        return Ok(None);
    }
    serde_json::from_str::<Value>(text).map(Some).map_err(|e| reader_error(e.to_string()))
}

/// A JSON object read the way `JsonSerializer` populates a class.
pub struct JsonObj<'a> {
    pub map: &'a serde_json::Map<String, Value>,
    pub path: String,
}

impl<'a> JsonObj<'a> {
    /// The object at `v`, or a conversion error naming the target type.
    pub fn new(v: &'a Value, ty: &str, path: &str) -> Result<Self, Exception> {
        match v {
            Value::Object(m) => Ok(JsonObj { map: m, path: path.to_string() }),
            other => Err(conversion_error(other, ty, path)),
        }
    }

    fn child_path(&self, name: &str) -> String {
        if self.path.is_empty() {
            name.to_string()
        } else {
            format!("{}.{name}", self.path)
        }
    }

    /// The property value: exact name first, then case-insensitive.
    pub fn get(&self, name: &str) -> Option<(&'a Value, String)> {
        if let Some(v) = self.map.get(name) {
            return Some((v, self.child_path(name)));
        }
        self.map.iter().find(|(k, _)| k.eq_ignore_ascii_case(name)).map(|(k, v)| (v, self.child_path(k)))
    }

    /// An `int` property: absent keeps the default; an integral number reads; anything else
    /// (null, text, a fraction) fails as Newtonsoft does.
    pub fn int(&self, name: &str, default: i32) -> Result<i32, Exception> {
        match self.get(name) {
            None => Ok(default),
            Some((v, path)) => read_int(v, &path).and_then(|o| o.ok_or_else(|| conversion_error(v, "System.Int32", &path))),
        }
    }

    /// An `int?` property.
    pub fn opt_int(&self, name: &str) -> Result<Option<i32>, Exception> {
        match self.get(name) {
            None => Ok(None),
            Some((v, path)) => read_int(v, &path),
        }
    }

    /// A `bool` property.
    pub fn bool(&self, name: &str, default: bool) -> Result<bool, Exception> {
        match self.get(name) {
            None => Ok(default),
            Some((Value::Bool(b), _)) => Ok(*b),
            Some((v, path)) => Err(conversion_error(v, "System.Boolean", &path)),
        }
    }

    /// A `bool?` property.
    pub fn opt_bool(&self, name: &str) -> Result<Option<bool>, Exception> {
        match self.get(name) {
            None | Some((Value::Null, _)) => Ok(None),
            Some((Value::Bool(b), _)) => Ok(Some(*b)),
            Some((v, path)) => Err(conversion_error(v, "System.Nullable`1[System.Boolean]", &path)),
        }
    }

    /// A `string` property: numbers and booleans become their text, as Newtonsoft does.
    pub fn string(&self, name: &str) -> Result<Option<String>, Exception> {
        match self.get(name) {
            None | Some((Value::Null, _)) => Ok(None),
            Some((Value::String(s), _)) => Ok(Some(s.clone())),
            // JsonTextReader.ReadAsString returns JsonConvert.True/False for boolean tokens
            Some((Value::Bool(b), _)) => Ok(Some(if *b { "true" } else { "false" }.into())),
            Some((Value::Number(n), _)) => Ok(Some(n.to_string())),
            Some((v, path)) => Err(conversion_error(v, "System.String", &path)),
        }
    }
}

/// `int?` from a token: null, an integral number, or a number text.
pub fn read_int(v: &Value, path: &str) -> Result<Option<i32>, Exception> {
    match v {
        Value::Null => Ok(None),
        Value::Number(n) => match n.as_i64() {
            Some(i) if i >= i32::MIN as i64 && i <= i32::MAX as i64 => Ok(Some(i as i32)),
            Some(_) => Err(reader_error(format!("JSON integer {n} is too large or small for an Int32. Path '{path}'."))),
            None => Err(reader_error(format!("Input string '{n}' is not a valid integer. Path '{path}'."))),
        },
        Value::String(s) if s.is_empty() => Ok(None),
        Value::String(s) => s
            .parse::<i32>()
            .map(Some)
            .map_err(|_| reader_error(format!("Could not convert string to integer: {s}. Path '{path}'."))),
        other => Err(conversion_error(other, "System.Int32", path)),
    }
}

/// `double?` from a token.
pub fn read_double(v: &Value, path: &str) -> Result<Option<f64>, Exception> {
    match v {
        Value::Null => Ok(None),
        Value::Number(n) => Ok(n.as_f64()),
        Value::String(s) if s.is_empty() => Ok(None),
        Value::String(s) => super::double_parse_culture(s, &INVARIANT_CULTURE)
            .map(Some)
            .map_err(|_| reader_error(format!("Could not convert string to double: {s}. Path '{path}'."))),
        other => Err(conversion_error(other, "System.Double", path)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn indented_layout() {
        let j = J::Obj(vec![
            ("a".into(), J::Arr(vec![J::Double(1.0), J::Null])),
            ("b".into(), J::Obj(vec![])),
            ("c".into(), J::Arr(vec![])),
        ]);
        let nl = super::super::io::NEW_LINE;
        assert_eq!(j.to_indented(), format!("{{{nl}  \"a\": [{nl}    1.0,{nl}    null{nl}  ],{nl}  \"b\": {{}},{nl}  \"c\": []{nl}}}"));
        assert_eq!(j.to_compact(), "{\"a\":[1.0,null],\"b\":{},\"c\":[]}");
        assert_eq!(json_str("a\"b\\\n\u{1}\u{2028}/"), "\"a\\\"b\\\\\\n\\u0001\\u2028/\"");
    }
}

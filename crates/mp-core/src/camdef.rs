//! MAVLink camera definitions (ExtLibs/ArduPilot/Mavlink): CameraDefinition and
//! CameraParameter (the XML file a camera advertises in CAMERA_INFORMATION.cam_definition_uri),
//! CameraDefinitionLoader.ParseFtpAddress, and CameraSettingsSession (reading and writing the
//! settings over PARAM_EXT_*).

use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::time::{Duration, Instant};

use mavlink::dialects::ardupilotmega::*;
use num_traits::FromPrimitive;

pub const MAX_SIZE: usize = 4 * 1024 * 1024;
pub const MAX_PARAMETERS: usize = 512;
pub const MAX_OPTIONS: usize = 512;

const TYPES: [&str; 11] = ["uint8", "int8", "uint16", "int16", "uint32", "int32", "uint64", "int64", "float", "double", "custom"];
const TYPE_NAMES: [&str; 10] = ["Byte", "SByte", "UInt16", "Int16", "UInt32", "Int32", "UInt64", "Int64", "Single", "Double"];

/// A boxed setting value; values of different types are never equal (object.Equals).
#[derive(Clone, Debug, PartialEq)]
pub enum Value {
    U8(u8),
    I8(i8),
    U16(u16),
    I16(i16),
    U32(u32),
    I32(i32),
    U64(u64),
    I64(i64),
    F32(f32),
    F64(f64),
    /// a custom (vendor) parameter's raw 128 bytes
    Bytes(Vec<u8>),
    /// a range option value not converted (its target is custom or unknown)
    Text(String),
}

impl Value {
    /// IComparable.CompareTo between values of the same type; None where .NET would throw.
    pub fn compare(&self, o: &Value) -> Option<Ordering> {
        use Value::*;
        match (self, o) {
            (U8(a), U8(b)) => Some(a.cmp(b)),
            (I8(a), I8(b)) => Some(a.cmp(b)),
            (U16(a), U16(b)) => Some(a.cmp(b)),
            (I16(a), I16(b)) => Some(a.cmp(b)),
            (U32(a), U32(b)) => Some(a.cmp(b)),
            (I32(a), I32(b)) => Some(a.cmp(b)),
            (U64(a), U64(b)) => Some(a.cmp(b)),
            (I64(a), I64(b)) => Some(a.cmp(b)),
            // Single/Double.CompareTo order NaN first
            (F32(a), F32(b)) => Some(a.partial_cmp(b).unwrap_or_else(|| a.is_nan().cmp(&b.is_nan()).reverse())),
            (F64(a), F64(b)) => Some(a.partial_cmp(b).unwrap_or_else(|| a.is_nan().cmp(&b.is_nan()).reverse())),
            (Text(a), Text(b)) => Some(a.cmp(b)),
            _ => None,
        }
    }

    pub fn to_f64(&self) -> f64 {
        use Value::*;
        match self {
            U8(v) => *v as f64,
            I8(v) => *v as f64,
            U16(v) => *v as f64,
            I16(v) => *v as f64,
            U32(v) => *v as f64,
            I32(v) => *v as f64,
            U64(v) => *v as f64,
            I64(v) => *v as f64,
            F32(v) => *v as f64,
            F64(v) => *v,
            Bytes(_) | Text(_) => f64::NAN,
        }
    }

    fn as_i128(&self) -> Option<i128> {
        use Value::*;
        Some(match self {
            U8(v) => *v as i128,
            I8(v) => *v as i128,
            U16(v) => *v as i128,
            I16(v) => *v as i128,
            U32(v) => *v as i128,
            I32(v) => *v as i128,
            U64(v) => *v as i128,
            I64(v) => *v as i128,
            _ => return None,
        })
    }
}

/// .NET "R" text of a float: the shortest round-trip digits, exponent form outside 1e-5..1e15.
fn round_trip(v: f64, s: String, e: String) -> String {
    if v == 0.0 || !v.is_finite() {
        return if v.is_nan() { "NaN".into() } else if v.is_infinite() { if v > 0.0 { "Infinity".into() } else { "-Infinity".into() } } else { s };
    }
    let a = v.abs();
    if (1e-5..1e15).contains(&a) {
        return s;
    }
    // "1.5e-7" -> "1.5E-07"
    let (m, x) = e.split_once('e').unwrap_or((&e, "0"));
    let x: i32 = x.parse().unwrap_or(0);
    format!("{m}E{}{:02}", if x < 0 { '-' } else { '+' }, x.abs())
}

/// CameraParameter.Format
pub fn format(v: &Value) -> String {
    use Value::*;
    match v {
        U8(x) => x.to_string(),
        I8(x) => x.to_string(),
        U16(x) => x.to_string(),
        I16(x) => x.to_string(),
        U32(x) => x.to_string(),
        I32(x) => x.to_string(),
        U64(x) => x.to_string(),
        I64(x) => x.to_string(),
        F32(x) => round_trip(*x as f64, format!("{x}"), format!("{x:e}")),
        F64(x) => round_trip(*x, format!("{x}"), format!("{x:e}")),
        Bytes(b) => format!("System.Byte[{}]", b.len()),
        Text(s) => s.clone(),
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct CamOption {
    pub label: String,
    pub value: Value,
    pub exclusions: Vec<String>,
    pub ranges: Vec<Range>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Range {
    pub target: String,
    pub condition: String,
    pub options: Vec<CamOption>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct CameraParameter {
    pub name: String,
    pub description: String,
    pub kind: String,
    pub wire_type: u8,
    pub control: bool,
    pub read_only: bool,
    pub write_only: bool,
    pub minimum: Option<Value>,
    pub maximum: Option<Value>,
    pub step: Option<String>,
    pub options: Vec<CamOption>,
    pub updates: Vec<String>,
}

const FORMAT_ERROR: &str = "Input string was not in a correct format.";

fn elem<'a, 'i>(n: roxmltree::Node<'a, 'i>, name: &str) -> Option<roxmltree::Node<'a, 'i>> {
    n.children().find(|c| c.is_element() && c.tag_name().name() == name)
}

fn elems<'a, 'i: 'a>(n: roxmltree::Node<'a, 'i>, name: &'a str) -> impl Iterator<Item = roxmltree::Node<'a, 'i>> + 'a {
    n.children().filter(move |c| c.is_element() && c.tag_name().name() == name)
}

/// (string)XElement: all the text inside
fn text(n: roxmltree::Node) -> String {
    n.descendants().filter(|d| d.is_text()).filter_map(|d| d.text()).collect()
}

/// Flag(e, name, expected): "1"/"true" when expected, "0"/"false" otherwise.
fn flag(e: roxmltree::Node, name: &str, expected: bool) -> bool {
    let v = e.attribute(name).unwrap_or("").to_lowercase();
    if expected { v == "1" || v == "true" } else { v == "0" || v == "false" }
}

impl CameraParameter {
    fn parse(e: roxmltree::Node, translate: &dyn Fn(&str) -> String) -> Result<Self, String> {
        let name = e.attribute("name").unwrap_or("").to_string();
        if name.is_empty() || name.len() > 16 || name.chars().any(|c| c == '\0' || c as u32 > 127) {
            return Err("Invalid camera parameter name".into());
        }
        let kind = e.attribute("type").unwrap_or("").to_lowercase();
        let lookup = if kind == "bool" { "uint8" } else { kind.as_str() };
        let wire_type = TYPES.iter().position(|t| *t == lookup).map_or(0, |i| i as u8 + 1);
        if wire_type == 0 {
            return Err(format!("Unsupported parameter type: {kind}"));
        }
        let description = translate(elem(e, "description").map(text).unwrap_or_else(|| name.clone()).trim());
        let mut p = CameraParameter {
            name,
            description,
            kind,
            wire_type,
            control: !flag(e, "control", false) && wire_type != 11,
            read_only: flag(e, "readonly", true),
            write_only: flag(e, "writeonly", true),
            minimum: None,
            maximum: None,
            step: None,
            options: Vec::new(),
            updates: elem(e, "updates").map(|u| elems(u, "update").map(|x| text(x).trim().to_string()).collect()).unwrap_or_default(),
        };
        // vendor-specific binary structures need a custom controller
        if wire_type == 11 {
            return Ok(p);
        }
        if let Some(v) = e.attribute("min") {
            p.minimum = Some(p.convert(Some(v))?);
        }
        if let Some(v) = e.attribute("max") {
            p.maximum = Some(p.convert(Some(v))?);
        }
        if let (Some(a), Some(b)) = (&p.minimum, &p.maximum) {
            if a.compare(b) == Some(Ordering::Greater) {
                return Err(format!("Invalid limits for {}", p.name));
            }
        }
        if let Some(s) = e.attribute("step") {
            let v: f64 = s.trim().parse().map_err(|_| FORMAT_ERROR.to_string())?;
            if !(v > 0.0) || v.is_infinite() {
                return Err(format!("Invalid step for {}", p.name));
            }
            p.step = Some(s.to_string());
        }
        if let Some(opts) = elem(e, "options") {
            for o in elems(opts, "option") {
                if p.options.len() >= MAX_OPTIONS {
                    return Err(format!("Too many options for {}", p.name));
                }
                let mut option = CamOption { label: translate(o.attribute("name").unwrap_or("")), value: p.convert(o.attribute("value"))?, exclusions: Vec::new(), ranges: Vec::new() };
                option.exclusions = o.descendants().filter(|d| d.is_element() && d.tag_name().name() == "exclude").map(|x| text(x).trim().to_string()).collect();
                if let Some(rs) = elem(o, "parameterranges") {
                    for r in elems(rs, "parameterrange") {
                        if elems(r, "roption").take(MAX_OPTIONS + 1).count() > MAX_OPTIONS {
                            return Err(format!("Too many range options for {}", p.name));
                        }
                        option.ranges.push(Range {
                            target: r.attribute("parameter").unwrap_or("").to_string(),
                            condition: r.attribute("condition").unwrap_or("").to_string(),
                            options: elems(r, "roption")
                                .map(|x| CamOption { label: translate(x.attribute("name").unwrap_or("")), value: Value::Text(x.attribute("value").unwrap_or("").to_string()), exclusions: Vec::new(), ranges: Vec::new() })
                                .collect(),
                        });
                    }
                }
                p.options.push(option);
            }
        }
        if p.kind == "bool" && p.options.is_empty() {
            p.options.push(CamOption { label: translate("Off"), value: Value::U8(0), exclusions: Vec::new(), ranges: Vec::new() });
            p.options.push(CamOption { label: translate("On"), value: Value::U8(1), exclusions: Vec::new(), ranges: Vec::new() });
        }
        Ok(p)
    }

    /// ConvertValue: the text as this parameter's type (invariant culture).
    pub fn convert(&self, text: Option<&str>) -> Result<Value, String> {
        let Some(text) = text else { return Err("Value cannot be null.".into()) };
        let t = text.trim();
        let int = |min: i128, max: i128| -> Result<i128, String> {
            let v: i128 = t.parse().map_err(|_| FORMAT_ERROR.to_string())?;
            if v < min || v > max {
                return Err(format!("Value was either too large or too small for a {}.", TYPE_NAMES[self.wire_type as usize - 1]));
            }
            Ok(v)
        };
        let float = || -> Result<f64, String> { t.replace(',', "").parse::<f64>().map_err(|_| FORMAT_ERROR.to_string()) };
        let v = match self.wire_type {
            1 => Value::U8(int(0, u8::MAX as i128)? as u8),
            2 => Value::I8(int(i8::MIN as i128, i8::MAX as i128)? as i8),
            3 => Value::U16(int(0, u16::MAX as i128)? as u16),
            4 => Value::I16(int(i16::MIN as i128, i16::MAX as i128)? as i16),
            5 => Value::U32(int(0, u32::MAX as i128)? as u32),
            6 => Value::I32(int(i32::MIN as i128, i32::MAX as i128)? as i32),
            7 => Value::U64(int(0, u64::MAX as i128)? as u64),
            8 => Value::I64(int(i64::MIN as i128, i64::MAX as i128)? as i64),
            9 => Value::F32(float()? as f32),
            10 => Value::F64(float()?),
            _ => return Err("Custom parameter has no numeric editor".into()),
        };
        if self.wire_type >= 9 && !v.to_f64().is_finite() {
            return Err("Value must be finite".into());
        }
        if self.kind == "bool" && matches!(v, Value::U8(b) if b > 1) {
            return Err("Boolean must be 0 or 1".into());
        }
        Ok(v)
    }

    /// Validate: type, limits, the available options and the step.
    pub fn validate(&self, text: &str, options: &[CamOption]) -> Result<Value, String> {
        let value = self.convert(Some(text))?;
        if let Some(min) = &self.minimum {
            if value.compare(min) == Some(Ordering::Less) {
                return Err(format!("Minimum: {}", format(min)));
            }
        }
        if let Some(max) = &self.maximum {
            if value.compare(max) == Some(Ordering::Greater) {
                return Err(format!("Maximum: {}", format(max)));
            }
        }
        if (!self.options.is_empty() || !options.is_empty()) && !options.iter().any(|o| o.value == value) {
            return Err("Value is not an available option".into());
        }
        if let (Some(step), Some(min)) = (&self.step, &self.minimum) {
            let valid = if self.wire_type <= 8 {
                // decimal arithmetic: scale by the step's decimal places
                use rust_decimal::Decimal;
                let step = step.trim().parse::<Decimal>().or_else(|_| Decimal::from_scientific(step.trim())).map_err(|e| e.to_string())?;
                let diff = Decimal::from_i128_with_scale(value.as_i128().unwrap_or(0) - min.as_i128().unwrap_or(0), 0);
                diff.checked_rem(step) == Some(Decimal::ZERO)
            } else {
                let (v, min, step) = (value.to_f64(), min.to_f64(), step.trim().parse::<f64>().unwrap_or(1.0));
                let offset = ((v - min) / step).round_ties_even() * step;
                let tolerance = v.abs().max(min.abs()) * if self.wire_type == 9 { 1.2e-7 } else { 4.5e-16 };
                (v - (min + offset)).abs() <= tolerance.max(if self.wire_type == 9 { 1.4e-45 } else { f64::from_bits(1) })
            };
            if !valid {
                return Err(format!("Value must use steps of {step}"));
            }
        }
        Ok(value)
    }

    /// Encode: little endian into PARAM_EXT's 128 bytes.
    pub fn encode(&self, v: &Value) -> Result<[u8; 128], String> {
        use Value::*;
        let bytes: Vec<u8> = match v {
            U8(x) => vec![*x],
            I8(x) => vec![*x as u8],
            U16(x) => x.to_le_bytes().to_vec(),
            I16(x) => x.to_le_bytes().to_vec(),
            U32(x) => x.to_le_bytes().to_vec(),
            I32(x) => x.to_le_bytes().to_vec(),
            U64(x) => x.to_le_bytes().to_vec(),
            I64(x) => x.to_le_bytes().to_vec(),
            F32(x) => x.to_le_bytes().to_vec(),
            F64(x) => x.to_le_bytes().to_vec(),
            _ => return Err("Unsupported parameter type".into()),
        };
        let mut out = [0u8; 128];
        out[..bytes.len()].copy_from_slice(&bytes);
        Ok(out)
    }

    /// Decode: the received bytes as this type, round-tripped through Format/ConvertValue.
    pub fn decode(&self, bytes: &[u8], kind: u8) -> Result<Value, String> {
        if kind != self.wire_type || bytes.len() != 128 {
            return Err("Unexpected parameter type or size".into());
        }
        if self.wire_type == 11 {
            return Ok(Value::Bytes(bytes.to_vec()));
        }
        let b = |n: usize| -> [u8; 8] {
            let mut a = [0u8; 8];
            a[..n].copy_from_slice(&bytes[..n]);
            a
        };
        let v = match self.wire_type {
            1 => Value::U8(bytes[0]),
            2 => Value::I8(bytes[0] as i8),
            3 => Value::U16(u16::from_le_bytes([bytes[0], bytes[1]])),
            4 => Value::I16(i16::from_le_bytes([bytes[0], bytes[1]])),
            5 => Value::U32(u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])),
            6 => Value::I32(i32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])),
            7 => Value::U64(u64::from_le_bytes(b(8))),
            8 => Value::I64(i64::from_le_bytes(b(8))),
            9 => Value::F32(f32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])),
            _ => Value::F64(f64::from_le_bytes(b(8))),
        };
        self.convert(Some(&format(&v)))
    }

    /// The 16-byte PARAM_EXT id.
    pub fn id(&self) -> [u8; 16] {
        let mut id = [0u8; 16];
        id[..self.name.len()].copy_from_slice(self.name.as_bytes());
        id
    }

    /// CameraCustomSettingsForm.SliderRange. Decimal endpoints must round-trip
    /// to the original wire values; large/tiny ranges use the text editor.
    pub fn slider_steps(&self) -> Option<i32> {
        use rust_decimal::{Decimal, prelude::ToPrimitive};
        let parse = |s: &str| s.parse::<Decimal>().or_else(|_| Decimal::from_scientific(s)).ok();
        let minimum = parse(&format(self.minimum.as_ref()?))?;
        let maximum = parse(&format(self.maximum.as_ref()?))?;
        let step = parse(self.step.as_ref()?)?;
        if step <= Decimal::ZERO || (self.wire_type <= 8 && step.fract() != Decimal::ZERO) { return None; }
        if self.convert(Some(&minimum.to_string())).ok().as_ref() != self.minimum.as_ref() || self.convert(Some(&maximum.to_string())).ok().as_ref() != self.maximum.as_ref() { return None; }
        let count = maximum.checked_sub(minimum)?.checked_div(step)?.floor().to_i32()?;
        (1..=100_000).contains(&count).then_some(count)
    }

    pub fn slider_text(&self, position: i32) -> Result<String, String> {
        use rust_decimal::Decimal;
        let parse = |s: &str| s.parse::<Decimal>().or_else(|_| Decimal::from_scientific(s)).map_err(|e| e.to_string());
        let min = parse(&format(self.minimum.as_ref().ok_or("Missing minimum")?))?;
        let step = parse(self.step.as_deref().ok_or("Missing step")?)?;
        let value = step.checked_mul(Decimal::from(position)).and_then(|v| min.checked_add(v)).ok_or("Slider range overflow")?;
        self.convert(Some(&value.to_string())).map(|v| format(&v))
    }

    pub fn slider_position(&self, value: &Value) -> Option<i32> {
        use rust_decimal::{Decimal, RoundingStrategy, prelude::ToPrimitive};
        let parse=|s: &str| s.parse::<Decimal>().or_else(|_| Decimal::from_scientific(s)).ok();
        let minimum=parse(&format(self.minimum.as_ref()?))?;
        let maximum=parse(&format(self.maximum.as_ref()?))?;
        let step=parse(self.step.as_ref()?)?;
        let value=parse(&format(value))?.max(minimum).min(maximum);
        value.checked_sub(minimum)?.checked_div(step)?.round_dp_with_strategy(0,RoundingStrategy::MidpointNearestEven).to_i32()
    }
}

#[derive(Clone, Debug)]
pub struct CameraDefinition {
    pub model: String,
    /// in file order (the Dictionary's insertion order)
    pub parameters: Vec<CameraParameter>,
}

/// ReadLimited for an already complete buffer.
fn limited(data: Vec<u8>) -> Result<Vec<u8>, String> {
    if data.len() > MAX_SIZE {
        return Err("Camera definition is too large".into());
    }
    Ok(data)
}

struct LimitedOutput(Vec<u8>);
impl std::io::Write for LimitedOutput {
    fn write(&mut self, bytes: &[u8]) -> std::io::Result<usize> {
        if bytes.len() > MAX_SIZE.saturating_sub(self.0.len()) { return Err(std::io::Error::other("Camera definition is too large")); }
        self.0.extend_from_slice(bytes); Ok(bytes.len())
    }
    fn flush(&mut self) -> std::io::Result<()> { Ok(()) }
}

fn xml_text(data: &[u8]) -> Result<String, String> {
    if data.starts_with(&[0xff,0xfe,0,0]) || data.starts_with(&[0,0,0xfe,0xff]) || data.starts_with(&[b'<',0,0,0]) || data.starts_with(&[0,0,0,b'<']) {
        let little = data[0] == 0xff || data[0] == b'<';
        let offset = if data[0] == 0xff || data[2] == 0xfe { 4 } else { 0 };
        if (data.len()-offset) % 4 != 0 { return Err("Invalid UTF-32 camera definition".into()); }
        data[offset..].chunks_exact(4).map(|b| {
            let value=if little { u32::from_le_bytes(b.try_into().unwrap()) } else { u32::from_be_bytes(b.try_into().unwrap()) };
            char::from_u32(value).ok_or_else(|| "Invalid UTF-32 camera definition".to_string())
        }).collect()
    } else if data.starts_with(&[0xff, 0xfe]) || data.starts_with(&[0xfe, 0xff]) || data.starts_with(&[b'<',0]) || data.starts_with(&[0,b'<']) {
        if data.len() % 2 != 0 { return Err("Invalid UTF-16 camera definition".into()); }
        let little = data[0] == 0xff || data[0] == b'<';
        let offset = if data[0]==0xff || data[0]==0xfe { 2 } else { 0 };
        let units: Vec<u16> = data[offset..].chunks_exact(2).map(|b| if little { u16::from_le_bytes([b[0], b[1]]) } else { u16::from_be_bytes([b[0], b[1]]) }).collect();
        String::from_utf16(&units).map_err(|e| e.to_string())
    } else {
        std::str::from_utf8(data.strip_prefix(&[0xef, 0xbb, 0xbf]).unwrap_or(data)).map(str::to_owned).map_err(|e| e.to_string())
    }
}

impl CameraDefinition {
    /// The definition XML (plain, or xz-compressed), with `language` (CurrentUICulture, e.g.
    /// "de_DE") picking the <locale> translations.
    pub fn parse(data: &[u8], language: &str) -> Result<Self, String> {
        let mut data = limited(data.to_vec())?;
        if data.starts_with(&[0xfd, 0x37, 0x7a, 0x58, 0x5a, 0]) {
            let mut out = LimitedOutput(Vec::new());
            lzma_rs::xz_decompress(&mut std::io::Cursor::new(&data), &mut out).map_err(|e| format!("{e:?}"))?;
            data = out.0;
        }
        let xml = xml_text(&data)?;
        // DtdProcessing.Prohibit: roxmltree refuses DTDs unless asked
        let doc = roxmltree::Document::parse(&xml).map_err(|e| e.to_string())?;
        let root = doc.root_element();
        if root.tag_name().name() != "mavlinkcamera" || elem(root, "definition").is_none() {
            return Err("Not a MAVLink camera definition".into());
        }
        let model = elem(root, "definition").and_then(|d| elem(d, "model")).map(text).unwrap_or_else(|| "Camera".into());
        let locales: Vec<_> = elem(root, "localization").map(|l| elems(l, "locale").collect()).unwrap_or_default();
        let lang_base = language.split('_').next().unwrap_or("").to_lowercase();
        let locale = locales
            .iter()
            .find(|l| l.attribute("name").is_some_and(|n| n.eq_ignore_ascii_case(language)))
            .or_else(|| locales.iter().find(|l| l.attribute("name").unwrap_or("").split('_').next().unwrap_or("").to_lowercase() == lang_base));
        let mut translations: HashMap<String, String> = HashMap::new();
        if let Some(l) = locale {
            for s in elems(*l, "strings") {
                translations.insert(s.attribute("original").unwrap_or("").to_string(), s.attribute("translated").unwrap_or("").to_string());
            }
        }
        let translate = |s: &str| translations.get(s).cloned().unwrap_or_else(|| s.to_string());
        let mut parameters: Vec<CameraParameter> = Vec::new();
        if let Some(ps) = elem(root, "parameters") {
            for e in elems(ps, "parameter") {
                if parameters.len() >= MAX_PARAMETERS {
                    return Err("Camera definition has too many parameters".into());
                }
                let p = CameraParameter::parse(e, &translate)?;
                if parameters.iter().any(|q| q.name == p.name) {
                    return Err(format!("Duplicate parameter {}", p.name));
                }
                parameters.push(p);
            }
        }
        // range options take their target's type
        let targets: HashMap<String, CameraParameter> = parameters.iter().map(|p| (p.name.clone(), p.clone())).collect();
        for p in &mut parameters {
            for o in &mut p.options {
                for r in &mut o.ranges {
                    if let Some(t) = targets.get(&r.target).filter(|t| t.wire_type != 11) {
                        for ro in &mut r.options {
                            if let Value::Text(s) = &ro.value {
                                ro.value = t.convert(Some(s))?;
                            }
                        }
                    }
                }
            }
        }
        Ok(Self { model, parameters })
    }

    pub fn get(&self, name: &str) -> Option<&CameraParameter> {
        self.parameters.iter().find(|p| p.name == name)
    }

    /// Controls: the visible settings with their current choices; an option's exclusions hide
    /// settings and its parameterranges narrow other settings' options.
    pub fn controls(&self, values: &HashMap<String, Value>) -> BTreeMap<String, Vec<CamOption>> {
        let mut excluded: BTreeSet<String> = BTreeSet::new();
        let mut limited: BTreeSet<String> = BTreeSet::new();
        let mut choices: HashMap<String, Vec<CamOption>> = self.parameters.iter().map(|p| (p.name.clone(), p.options.clone())).collect();
        for p in &self.parameters {
            let Some(value) = values.get(&p.name) else { continue };
            for option in p.options.iter().filter(|o| &o.value == value) {
                excluded.extend(option.exclusions.iter().cloned());
                for r in &option.ranges {
                    if !limited.contains(&r.target) && self.condition(&r.condition, values) {
                        choices.insert(r.target.clone(), r.options.clone());
                        limited.insert(r.target.clone());
                    }
                }
            }
        }
        self.parameters
            .iter()
            .filter(|p| p.control && !excluded.contains(&p.name))
            .map(|p| (p.name.clone(), choices.get(&p.name).cloned().unwrap_or_default()))
            .collect()
    }

    /// A parameterrange condition: "NAME op VALUE" terms joined by AND / OR, left to right.
    fn condition(&self, condition: &str, values: &HashMap<String, Value>) -> bool {
        if condition.trim().is_empty() {
            return true;
        }
        // Regex.Split keeps the captured AND/OR between the terms
        let splitter = regex::RegexBuilder::new(r"\s+(AND|OR)\s+").case_insensitive(true).build().unwrap();
        let mut parts: Vec<String> = Vec::new();
        let trimmed = condition.trim();
        let mut last = 0;
        for c in splitter.captures_iter(trimmed) {
            let m = c.get(0).unwrap();
            parts.push(trimmed[last..m.start()].to_string());
            parts.push(c[1].to_string());
            last = m.end();
        }
        parts.push(trimmed[last..].to_string());
        let term = regex::Regex::new(r"^\s*([A-Za-z0-9_]+)\s*(!=|==|>=|<=|=|>|<)\s*(\S+)\s*$").unwrap();
        let mut result = false;
        for i in (0..parts.len()).step_by(2) {
            let Some(m) = term.captures(&parts[i]) else { return false };
            let name = &m[1];
            let (Some(value), Some(p)) = (values.get(name), self.get(name)) else { return false };
            let Ok(other) = p.convert(Some(&m[3])) else { return false };
            let Some(c) = value.compare(&other) else { return false };
            let test = match &m[2] {
                "=" | "==" => c == Ordering::Equal,
                "!=" => c != Ordering::Equal,
                ">" => c == Ordering::Greater,
                "<" => c == Ordering::Less,
                ">=" => c != Ordering::Less,
                _ => c != Ordering::Greater,
            };
            result = if i == 0 { test } else if parts[i - 1].eq_ignore_ascii_case("AND") { result && test } else { result || test };
        }
        result
    }
}

/// CameraDefinitionLoader.ParseFtpAddress: (system, component, path) of a mftp:// or mavftp://
/// definition URI.
pub fn parse_ftp_address(uri: &str, mut system: u32, mut component: u8) -> Result<(u32, u8, String), String> {
    let re = regex::RegexBuilder::new(r"^(mftp|mavftp)://(.*)$").case_insensitive(true).dot_matches_new_line(false).build().unwrap();
    let Some(m) = re.captures(uri) else { return Err("Invalid MAVFTP definition URI".into()) };
    let mut path = m[2].to_string();
    if m[1].eq_ignore_ascii_case("mftp") {
        let sel = regex::Regex::new(r"^/?\[;comp=(\d+)\]").unwrap();
        if let Some(s) = sel.captures(&path) {
            component = s[1].parse().map_err(|_| "Value was either too large or too small for an unsigned byte.".to_string())?;
            path = path[s.get(0).unwrap().end()..].to_string();
        } else if path.trim_start_matches('/').starts_with('[') {
            return Err("Invalid MAVFTP component selector".into());
        }
    } else if !path.starts_with('/') {
        let Some(slash) = path.find('/') else { return Err("Missing MAVFTP filename".into()) };
        let target: Vec<&str> = path[..slash].split(':').collect();
        if target.len() != 2 {
            return Err("Invalid MAVFTP address".into());
        }
        system = target[0].parse().map_err(|_| FORMAT_ERROR.to_string())?;
        component = target[1].parse().map_err(|_| FORMAT_ERROR.to_string())?;
        path = path[slash..].to_string();
    }
    let path = unescape(&format!("/{}", path.trim_start_matches('/')));
    if system == 0 || component == 0 || path == "/" || path.contains('\0') || path.len() > 238 {
        return Err("Invalid MAVFTP address or filename".into());
    }
    Ok((system, component, path))
}

/// Uri.UnescapeDataString
fn unescape(s: &str) -> String {
    let b = s.as_bytes();
    let mut out = Vec::with_capacity(b.len());
    let mut i = 0;
    while i < b.len() {
        if b[i] == b'%' && i + 2 < b.len() {
            let hex = |c: u8| match c { b'0'..=b'9' => Some(c - b'0'), b'a'..=b'f' => Some(c - b'a' + 10), b'A'..=b'F' => Some(c - b'A' + 10), _ => None };
            if let (Some(a), Some(v)) = (hex(b[i + 1]), hex(b[i + 2])) {
                out.push(a * 16 + v);
                i += 3;
                continue;
            }
        }
        out.push(b[i]);
        i += 1;
    }
    String::from_utf8_lossy(&out).into_owned()
}

struct Request {
    deadline: Instant,
    expires: Instant,
    attempts: u32,
    value: Option<Value>,
    in_progress: bool,
}

const RETRY: Duration = Duration::from_secs(2);

/// CameraSettingsSession: reads every readable setting, writes one at a time with retries,
/// and re-reads the settings an accepted change "updates".
pub struct CameraSettingsSession {
    pub definition: CameraDefinition,
    pub values: HashMap<String, Value>,
    pub errors: HashMap<String, String>,
    pub status: String,
    reads: BTreeMap<String, Request>,
    writes: BTreeMap<String, Request>,
    write_errors: BTreeSet<String>,
    system: u8,
    component: u8,
    next_batch: Option<Instant>,
    /// messages to send (the `send` callback)
    pub outbox: Vec<MavMessage>,
}

fn param_ack_name(r: u8) -> String {
    match r {
        0 => "ACCEPTED".into(),
        1 => "VALUE_UNSUPPORTED".into(),
        2 => "FAILED".into(),
        3 => "IN_PROGRESS".into(),
        n => n.to_string(),
    }
}

impl CameraSettingsSession {
    pub fn new(definition: CameraDefinition, system: u8, component: u8) -> Self {
        Self {
            definition,
            values: HashMap::new(),
            errors: HashMap::new(),
            status: "Reading camera settings".into(),
            reads: BTreeMap::new(),
            writes: BTreeMap::new(),
            write_errors: BTreeSet::new(),
            system,
            component,
            next_batch: None,
            outbox: Vec::new(),
        }
    }

    pub fn is_pending(&self, name: &str) -> bool {
        self.writes.contains_key(name)
    }

    pub fn can_edit(&self, name: &str, controls: Option<&BTreeMap<String, Vec<CamOption>>>) -> bool {
        let Some(p) = self.definition.get(name) else { return false };
        let owned;
        let controls = match controls {
            Some(c) => c,
            None => {
                owned = self.definition.controls(&self.values);
                &owned
            }
        };
        !p.read_only && !self.is_pending(name) && (self.values.contains_key(name) || p.write_only) && controls.contains_key(name)
    }

    pub fn request_all(&mut self, now: Instant) {
        self.reads.clear();
        self.errors.clear();
        self.write_errors.clear();
        for p in self.definition.parameters.iter().filter(|p| !p.write_only) {
            self.reads.insert(p.name.clone(), Request { deadline: now + RETRY, expires: now, attempts: 1, value: None, in_progress: false });
        }
        self.outbox.push(MavMessage::PARAM_EXT_REQUEST_LIST(PARAM_EXT_REQUEST_LIST_DATA { target_system: self.system, target_component: self.component }));
    }

    fn read(&mut self, name: &str, now: Instant) {
        if self.definition.get(name).is_some_and(|p| !p.write_only) && !self.writes.contains_key(name) && !self.reads.contains_key(name) {
            self.reads.insert(name.to_string(), Request { deadline: now, expires: now, attempts: 0, value: None, in_progress: false });
        }
    }

    pub fn set_value(&mut self, name: &str, text: &str, now: Instant) -> Result<(), String> {
        if !self.can_edit(name, None) {
            return Err("Setting is not currently editable".into());
        }
        let p = self.definition.get(name).unwrap().clone();
        let controls = self.definition.controls(&self.values);
        let value = p.validate(text, controls.get(name).map(|v| v.as_slice()).unwrap_or(&[]))?;
        self.reads.remove(name);
        self.errors.remove(name);
        self.write_errors.remove(name);
        self.writes.insert(name.to_string(), Request { deadline: now + RETRY, expires: now + Duration::from_secs(30), attempts: 1, value: Some(value), in_progress: false });
        self.status = format!("Applying {}", p.description);
        self.send_value(name);
        Ok(())
    }

    fn send_value(&mut self, name: &str) {
        let Some(p) = self.definition.get(name) else { return };
        let Some(Ok(bytes)) = self.writes.get(name).and_then(|w| w.value.as_ref()).map(|v| p.encode(v)) else { return };
        self.outbox.push(MavMessage::PARAM_EXT_SET(PARAM_EXT_SET_DATA {
            target_system: self.system,
            target_component: self.component,
            param_id: p.id().into(),
            param_value: bytes.into(),
            param_type: MavParamExtType::from_u8(p.wire_type).unwrap_or(MavParamExtType::MAV_PARAM_EXT_TYPE_UINT8),
        }));
    }

    fn received(&mut self, name: &str, value: Value, now: Instant, update: bool) {
        let changed = self.values.get(name).is_some_and(|old| *old != value);
        self.values.insert(name.to_string(), value);
        self.reads.remove(name);
        if !self.write_errors.contains(name) {
            self.errors.remove(name);
        } else if let Some(e) = self.errors.get_mut(name) {
            *e = e.replace("camera state is unconfirmed", "current camera value has been read back");
        }
        if update || changed {
            let targets = self.definition.get(name).map(|p| p.updates.clone()).unwrap_or_default();
            for t in targets {
                self.read(&t, now + Duration::from_millis(500));
            }
        }
    }

    /// Receive: a PARAM_EXT_VALUE or PARAM_EXT_ACK from the camera.
    pub fn receive(&mut self, msg: &MavMessage, now: Instant) {
        let (name, raw, kind, result, ack) = match msg {
            MavMessage::PARAM_EXT_ACK(m) => (cstr(&m.param_id[..]), m.param_value[..].to_vec(), m.param_type as u8, m.param_result as u8, true),
            MavMessage::PARAM_EXT_VALUE(m) => (cstr(&m.param_id[..]), m.param_value[..].to_vec(), m.param_type as u8, 0, false),
            _ => return,
        };
        let Some(p) = self.definition.get(&name).cloned() else { return };
        if kind != p.wire_type {
            self.errors.insert(name, "Camera returned a different parameter type".into());
            return;
        }
        let pending = self.writes.contains_key(&name);
        // a read response cannot confirm a write
        if !ack && pending {
            return;
        }
        if ack && result == 3 {
            if let Some(w) = self.writes.get_mut(&name) {
                w.in_progress = true;
                w.deadline = w.expires;
            }
            return;
        }
        if ack && result != 0 {
            if !pending {
                return;
            }
            self.writes.remove(&name);
            self.values.remove(&name);
            self.write_errors.insert(name.clone());
            let e = format!("Camera rejected setting: {}", param_ack_name(result));
            self.status = format!("{}: {e}", p.description);
            self.errors.insert(name.clone(), e);
            self.read(&name, now);
            return;
        }
        let value = match p.decode(&raw, kind) {
            Ok(v) => v,
            Err(e) => {
                self.errors.insert(name, e);
                return;
            }
        };
        let adjusted = self.writes.get(&name).and_then(|w| w.value.as_ref()).is_some_and(|v| *v != value);
        self.writes.remove(&name);
        if ack {
            self.errors.remove(&name);
        }
        self.received(&name, value, now, ack);
        if adjusted {
            self.read(&name, now);
        }
        if self.reads.is_empty() && self.writes.is_empty() {
            self.status = if self.errors.is_empty() { "Camera settings up to date".into() } else { "Some settings are unavailable or could not be applied".into() };
        }
    }

    /// Tick: up to five reads every 200 ms (three attempts each), and write retries.
    pub fn tick(&mut self, now: Instant) {
        let mut sent = 0;
        let names: Vec<String> = self.reads.keys().cloned().collect();
        for name in names {
            let r = &self.reads[&name];
            if now < r.deadline || self.next_batch.is_some_and(|b| now < b) || sent >= 5 {
                continue;
            }
            if r.attempts >= 3 {
                self.reads.remove(&name);
                self.errors.entry(name.clone()).or_insert_with(|| "Camera did not respond".into());
                self.status = "Some camera settings did not respond".into();
                continue;
            }
            let id = self.definition.get(&name).map(|p| p.id()).unwrap_or([0; 16]);
            self.outbox.push(MavMessage::PARAM_EXT_REQUEST_READ(PARAM_EXT_REQUEST_READ_DATA { param_index: -1, target_system: self.system, target_component: self.component, param_id: id.into() }));
            let r = self.reads.get_mut(&name).unwrap();
            r.attempts += 1;
            r.deadline = now + RETRY;
            sent += 1;
        }
        if sent > 0 {
            self.next_batch = Some(now + Duration::from_millis(200));
        }
        let names: Vec<String> = self.writes.keys().cloned().collect();
        for name in names {
            let w = &self.writes[&name];
            if now < w.deadline && now < w.expires {
                continue;
            }
            if w.in_progress || w.attempts >= 3 || now >= w.expires {
                self.writes.remove(&name);
                self.values.remove(&name);
                self.write_errors.insert(name.clone());
                self.errors.insert(name.clone(), "Write timed out; camera state is unconfirmed".into());
                self.status = format!("{name}: write timed out");
                self.read(&name, now);
            } else {
                self.send_value(&name);
                let w = self.writes.get_mut(&name).unwrap();
                w.attempts += 1;
                w.deadline = now + RETRY;
            }
        }
    }
}

fn cstr(b: &[u8]) -> String {
    let end = b.iter().position(|c| *c == 0).unwrap_or(b.len());
    String::from_utf8_lossy(&b[..end]).into_owned()
}

#[cfg(test)]
mod tests {
    use super::*;

    const XML: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<mavlinkcamera>
  <definition version="1"><model>Cam X</model><vendor>V</vendor></definition>
  <parameters>
    <parameter name="CAM_MODE" type="uint32" default="1" control="0">
      <description>Camera Mode</description>
      <options>
        <option name="Photo" value="0"><exclusions><exclude>CAM_VIDRES</exclude></exclusions></option>
        <option name="Video" value="1">
          <parameterranges><parameterrange parameter="CAM_EV" condition="CAM_WBMODE=0">
            <roption name="-1" value="-1.0"/><roption name="0" value="0.0"/>
          </parameterrange></parameterranges>
        </option>
      </options>
    </parameter>
    <parameter name="CAM_VIDRES" type="uint32"><description>Video Resolution</description>
      <options><option name="1080p" value="0"/><option name="4K" value="1"/></options></parameter>
    <parameter name="CAM_WBMODE" type="uint32"><description>White Balance</description>
      <options><option name="Auto" value="0"/><option name="Sun" value="1"/></options></parameter>
    <parameter name="CAM_EV" type="float" min="-2" max="2" step="0.5"><description>Exposure</description><updates><update>CAM_ISO</update></updates></parameter>
    <parameter name="CAM_ISO" type="uint16" min="100" max="3200" step="100" readonly="1"><description>ISO</description></parameter>
    <parameter name="CAM_FLIP" type="bool"><description>Flip</description></parameter>
  </parameters>
  <localization><locale name="de_DE"><strings original="White Balance" translated="Weißabgleich"/></locale></localization>
</mavlinkcamera>"#;

    #[test]
    fn definitions_parse_with_types_ranges_and_translations() {
        let d = CameraDefinition::parse(XML.as_bytes(), "de_AT").unwrap();
        assert_eq!(d.model, "Cam X");
        assert_eq!(d.parameters.len(), 6);
        assert_eq!(d.get("CAM_WBMODE").unwrap().description, "Weißabgleich", "falls back to the language");
        assert!(!d.get("CAM_MODE").unwrap().control, "control=\"0\"");
        let ev = d.get("CAM_EV").unwrap();
        assert_eq!((ev.wire_type, ev.minimum.clone(), ev.step.clone()), (9, Some(Value::F32(-2.0)), Some("0.5".into())));
        // range options take the target's type
        let r = &d.get("CAM_MODE").unwrap().options[1].ranges[0];
        assert_eq!(r.options[0].value, Value::F32(-1.0));
        let flip = d.get("CAM_FLIP").unwrap();
        assert_eq!(flip.options.iter().map(|o| o.label.as_str()).collect::<Vec<_>>(), ["Off", "On"]);
        assert!(CameraDefinition::parse(b"<x/>", "en").unwrap_err().contains("Not a MAVLink camera definition"));
        assert!(CameraDefinition::parse(b"<!DOCTYPE a [<!ENTITY e 'x'>]><mavlinkcamera><definition/></mavlinkcamera>", "en").is_err(), "DTDs are refused");
    }

    #[test]
    fn controls_follow_exclusions_and_conditional_ranges() {
        let d = CameraDefinition::parse(XML.as_bytes(), "en_US").unwrap();
        let mut v = HashMap::new();
        v.insert("CAM_MODE".to_string(), Value::U32(0));
        let c = d.controls(&v);
        assert!(!c.contains_key("CAM_VIDRES"), "Photo excludes the video resolution");
        assert!(!c.contains_key("CAM_MODE"), "not a control");
        v.insert("CAM_MODE".into(), Value::U32(1));
        v.insert("CAM_WBMODE".into(), Value::U32(0));
        let c = d.controls(&v);
        assert_eq!(c["CAM_EV"].len(), 2, "the range applies while CAM_WBMODE=0");
        v.insert("CAM_WBMODE".into(), Value::U32(1));
        assert!(d.controls(&v)["CAM_EV"].is_empty());
    }

    #[test]
    fn validation_and_wire_encoding() {
        let d = CameraDefinition::parse(XML.as_bytes(), "en").unwrap();
        let ev = d.get("CAM_EV").unwrap();
        assert_eq!(ev.validate("0.5", &[]), Ok(Value::F32(0.5)));
        assert_eq!(ev.validate("0.3", &[]), Err("Value must use steps of 0.5".into()));
        assert_eq!(ev.validate("3", &[]), Err("Maximum: 2".into()));
        assert_eq!(ev.validate("x", &[]), Err(FORMAT_ERROR.into()));
        let iso = d.get("CAM_ISO").unwrap();
        assert_eq!(iso.validate("250", &[]), Err("Value must use steps of 100".into()));
        assert_eq!(iso.convert(Some("70000")), Err("Value was either too large or too small for a UInt16.".into()));
        let bytes = iso.encode(&Value::U16(400)).unwrap();
        assert_eq!(&bytes[..3], &[0x90, 0x01, 0]);
        assert_eq!(iso.decode(&bytes, 3), Ok(Value::U16(400)));
        assert!(iso.decode(&bytes, 4).is_err());
        assert_eq!(&iso.id()[..8], b"CAM_ISO\0");
        assert_eq!(format(&Value::F32(0.1)), "0.1");
        assert_eq!(format(&Value::F32(1e-7)), "1E-07");
        assert_eq!(format(&Value::F64(1.5e-7)), "1.5E-07");
        let flip = d.get("CAM_FLIP").unwrap();
        assert_eq!(flip.convert(Some("2")), Err("Boolean must be 0 or 1".into()));
    }

    #[test]
    fn adversarial_steps_unicode_and_bounded_compression_do_not_panic() {
        let xml = br#"<mavlinkcamera><definition/><parameters><parameter name="BIG" type="uint64" min="0" step="0.0000000000000000000000000001"/></parameters></mavlinkcamera>"#;
        let d = CameraDefinition::parse(xml,"en").unwrap();
        assert_eq!(d.get("BIG").unwrap().validate("18446744073709551615",&[]), Ok(Value::U64(u64::MAX)));
        assert_eq!(parse_ftp_address("mftp://%€",1,100).unwrap().2,"/%€");
        let mut utf16 = vec![0xff,0xfe];
        for c in XML.encode_utf16() { utf16.extend_from_slice(&c.to_le_bytes()); }
        assert_eq!(CameraDefinition::parse(&utf16,"en").unwrap().model,"Cam X");
        let mut compressed = Vec::new();
        lzma_rs::xz_compress(&mut std::io::Cursor::new(vec![b'x'; MAX_SIZE+1]), &mut compressed).unwrap();
        assert!(CameraDefinition::parse(&compressed,"en").is_err());
    }

    #[test]
    fn full_width_integer_slider_positions_do_not_go_through_float() {
        let d=CameraDefinition::parse(br#"<mavlinkcamera><definition/><parameters><parameter name="HIGH" type="uint64" min="18446744073709551610" max="18446744073709551615" step="1"/></parameters></mavlinkcamera>"#,"en").unwrap();
        let p=d.get("HIGH").unwrap();
        assert_eq!(p.slider_steps(),Some(5));
        assert_eq!(p.slider_position(&Value::U64(18446744073709551614)),Some(4));
        assert_eq!(p.slider_text(4).unwrap(),"18446744073709551614");
    }

    #[test]
    fn ftp_addresses_follow_parseftpaddress() {
        assert_eq!(parse_ftp_address("mftp://[;comp=100]/cam.xml", 1, 1), Ok((1, 100, "/cam.xml".into())));
        assert_eq!(parse_ftp_address("mavftp://2:101/defs/cam%20x.xml.xz", 1, 1), Ok((2, 101, "/defs/cam x.xml.xz".into())));
        assert_eq!(parse_ftp_address("MFTP://cam.xml", 1, 100), Ok((1, 100, "/cam.xml".into())));
        assert_eq!(parse_ftp_address("http://x/cam.xml", 1, 1), Err("Invalid MAVFTP definition URI".into()));
        assert_eq!(parse_ftp_address("mftp://[bad]/x", 1, 1), Err("Invalid MAVFTP component selector".into()));
        assert_eq!(parse_ftp_address("mavftp://nofile", 1, 1), Err("Missing MAVFTP filename".into()));
        assert_eq!(parse_ftp_address("mftp:///", 1, 1), Err("Invalid MAVFTP address or filename".into()));
    }

    fn value_msg(p: &CameraParameter, v: &Value) -> MavMessage {
        MavMessage::PARAM_EXT_VALUE(PARAM_EXT_VALUE_DATA { param_count: 1, param_index: 0, param_id: p.id().into(), param_value: p.encode(v).unwrap().into(), param_type: MavParamExtType::from_u8(p.wire_type).unwrap() })
    }

    fn ack_msg(p: &CameraParameter, v: &Value, result: ParamAck) -> MavMessage {
        MavMessage::PARAM_EXT_ACK(PARAM_EXT_ACK_DATA { param_id: p.id().into(), param_value: p.encode(v).unwrap().into(), param_type: MavParamExtType::from_u8(p.wire_type).unwrap(), param_result: result })
    }

    #[test]
    fn session_reads_writes_and_retries() {
        let d = CameraDefinition::parse(XML.as_bytes(), "en").unwrap();
        let wb = d.get("CAM_WBMODE").unwrap().clone();
        let ev = d.get("CAM_EV").unwrap().clone();
        let mut s = CameraSettingsSession::new(d, 1, 100);
        let t0 = Instant::now();
        s.request_all(t0);
        assert!(matches!(s.outbox.remove(0), MavMessage::PARAM_EXT_REQUEST_LIST(_)));
        assert!(s.set_value("CAM_WBMODE", "1", t0).is_err(), "nothing read yet");
        s.receive(&value_msg(&wb, &Value::U32(0)), t0);
        assert_eq!(s.values["CAM_WBMODE"], Value::U32(0));
        s.set_value("CAM_WBMODE", "1", t0).unwrap();
        assert_eq!(s.status, "Applying White Balance");
        assert!(matches!(s.outbox.pop(), Some(MavMessage::PARAM_EXT_SET(_))));
        // a value message cannot confirm the write; the ACK does
        s.receive(&value_msg(&wb, &Value::U32(0)), t0);
        assert!(s.is_pending("CAM_WBMODE"));
        s.receive(&ack_msg(&wb, &Value::U32(1), ParamAck::PARAM_ACK_ACCEPTED), t0);
        assert_eq!(s.values["CAM_WBMODE"], Value::U32(1));
        // rejected writes are re-read
        s.receive(&value_msg(&ev, &Value::F32(0.0)), t0);
        s.set_value("CAM_EV", "1", t0).unwrap();
        s.receive(&ack_msg(&ev, &Value::F32(0.0), ParamAck::PARAM_ACK_FAILED), t0);
        assert_eq!(s.errors["CAM_EV"], "Camera rejected setting: FAILED");
        assert!(!s.values.contains_key("CAM_EV"));
        // unanswered reads give up after three attempts
        s.outbox.clear();
        let mut t = t0;
        for _ in 0..4 {
            t += Duration::from_secs(3);
            s.tick(t);
        }
        assert!(s.errors.values().any(|e| e == "Camera did not respond"));
        assert!(s.outbox.iter().any(|m| matches!(m, MavMessage::PARAM_EXT_REQUEST_READ(r) if r.param_index == -1)));
    }

    #[test]
    fn in_progress_has_absolute_deadline_and_timeout_readback_keeps_error_visible() {
        let definition=CameraDefinition::parse(br#"<mavlinkcamera><definition/><parameters><parameter name="BIG" type="uint64" min="0" step="1"/></parameters></mavlinkcamera>"#,"en").unwrap();
        let p=definition.get("BIG").unwrap().clone(); let t=Instant::now();
        let mut session=CameraSettingsSession::new(definition,42,100);
        session.request_all(t); session.outbox.clear(); session.receive(&value_msg(&p,&Value::U64(0)),t);
        session.tick(t+Duration::from_secs(10)); assert!(session.outbox.is_empty(),"no idle polling after values arrive");
        session.set_value("BIG","18446744073709551615",t).unwrap(); session.outbox.clear();
        session.receive(&ack_msg(&p,&Value::U64(0),ParamAck::PARAM_ACK_IN_PROGRESS),t+Duration::from_secs(1));
        session.tick(t+Duration::from_secs(20)); assert!(session.is_pending("BIG") && session.outbox.is_empty());
        session.tick(t+Duration::from_secs(31)); assert!(!session.is_pending("BIG"));
        assert!(session.errors["BIG"].contains("unconfirmed"));
        session.tick(t+Duration::from_secs(32)); assert!(session.outbox.iter().any(|m|matches!(m,MavMessage::PARAM_EXT_REQUEST_READ(_))));
        session.receive(&value_msg(&p,&Value::U64(u64::MAX)),t+Duration::from_secs(32));
        assert_eq!(session.values["BIG"],Value::U64(u64::MAX));
        assert!(session.errors["BIG"].contains("read back"));
    }
}

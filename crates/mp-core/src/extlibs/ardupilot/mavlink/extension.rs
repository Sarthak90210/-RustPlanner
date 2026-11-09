//! Port of `ExtLibs/ArduPilot/Mavlink/Extension.cs`
//!
//! `ToJSON` / `FromJSON` reproduce `JsonConvert.SerializeObject` / `DeserializeObject` on a
//! `MAVLinkMessage` with Newtonsoft's default settings: every public property in declaration
//! order, `byte[]` as base64, ISO dates (`yyyy-MM-ddTHH:mm:ss.FFFFFFFK`), floating point in
//! .NET "R" form with a forced decimal place, and on the way back only the publicly settable
//! `buffer` and `rxtime` applied to a new message.

use base64::Engine;
use serde_json::Value;

use crate::dotnet::{CultureInfo, DateTime, DateTimeKind, Exception, INVARIANT_CULTURE, TICKS_PER_SECOND};
use crate::extlibs::mavlink::{FieldValue, MAVLinkMessage};

/// .NET Framework "R" text for a double: G15 if it round-trips, else G17.
fn r_double(x: f64) -> String {
    let c: &CultureInfo = &INVARIANT_CULTURE;
    let s15 = crate::dotnet::format_general(x, 15, c);
    if s15.parse::<f64>().ok() == Some(x) {
        s15
    } else {
        crate::dotnet::format_general(x, 17, c)
    }
}

/// .NET Framework "R" text for a float: G7 if it round-trips, else G9.
fn r_float(x: f32) -> String {
    let c: &CultureInfo = &INVARIANT_CULTURE;
    let s7 = crate::dotnet::format_general(x as f64, 7, c);
    if s7.parse::<f32>().ok() == Some(x) {
        s7
    } else {
        crate::dotnet::format_general(x as f64, 9, c)
    }
}

/// `JsonWriter` floating point output: NaN/Infinity as strings, otherwise the number text,
/// which always carries a decimal place.
fn json_float(text: String) -> String {
    if text == "NaN" || text == "Infinity" || text == "-Infinity" {
        return format!("\"{text}\"");
    }
    if text.contains('.') || text.contains('E') || text.contains('e') {
        text
    } else {
        format!("{text}.0")
    }
}

/// A JSON string literal as `JsonTextWriter` escapes it.
fn json_str(s: &str) -> String {
    Value::String(s.to_string()).to_string()
}

fn json_b64(b: &[u8]) -> String {
    json_str(&base64::engine::general_purpose::STANDARD.encode(b))
}

fn json_list<T>(a: &[T], f: impl Fn(&T) -> String) -> String {
    format!("[{}]", a.iter().map(f).collect::<Vec<_>>().join(","))
}

/// Newtonsoft's ISO date: `yyyy-MM-ddTHH:mm:ss.FFFFFFFK`.
fn json_date(d: DateTime) -> String {
    let base = d.format("yyyy-MM-ddTHH:mm:ss", &INVARIANT_CULTURE);
    let frac = format!("{:07}", d.ticks.rem_euclid(TICKS_PER_SECOND));
    let frac = frac.trim_end_matches('0');
    let mut s = base;
    if !frac.is_empty() {
        s.push('.');
        s.push_str(frac);
    }
    match d.kind {
        DateTimeKind::Utc => s.push('Z'),
        DateTimeKind::Local => {
            let off = (d.ticks - d.to_universal_time().ticks) / TICKS_PER_SECOND;
            let sign = if off < 0 { '-' } else { '+' };
            let off = off.abs();
            s.push_str(&format!("{sign}{:02}:{:02}", off / 3600, (off / 60) % 60));
        }
        DateTimeKind::Unspecified => {}
    }
    s
}

fn field_json(v: &FieldValue) -> String {
    match v {
        FieldValue::Float(x) => json_float(r_float(*x)),
        FieldValue::Double(x) => json_float(r_double(*x)),
        FieldValue::Byte(x) => x.to_string(),
        FieldValue::SByte(x) => x.to_string(),
        FieldValue::Short(x) => x.to_string(),
        FieldValue::UShort(x) => x.to_string(),
        FieldValue::Int(x) => x.to_string(),
        FieldValue::UInt(x) => x.to_string(),
        FieldValue::Long(x) => x.to_string(),
        FieldValue::ULong(x) => x.to_string(),
        FieldValue::ByteArray(a) => json_b64(a),
        FieldValue::FloatArray(a) => json_list(a, |x| json_float(r_float(*x))),
        FieldValue::DoubleArray(a) => json_list(a, |x| json_float(r_double(*x))),
        FieldValue::SByteArray(a) => json_list(a, |x| x.to_string()),
        FieldValue::ShortArray(a) => json_list(a, |x| x.to_string()),
        FieldValue::UShortArray(a) => json_list(a, |x| x.to_string()),
        FieldValue::IntArray(a) => json_list(a, |x| x.to_string()),
        FieldValue::UIntArray(a) => json_list(a, |x| x.to_string()),
        FieldValue::LongArray(a) => json_list(a, |x| x.to_string()),
        FieldValue::ULongArray(a) => json_list(a, |x| x.to_string()),
    }
}

/// `public static string ToJSON(this MAVLink.MAVLinkMessage msg)`
pub fn to_json(msg: &MAVLinkMessage) -> String {
    let null = || "null".to_string();
    let name = msg.msgtypename();
    let data = match msg.data() {
        None => null(),
        Some(d) => {
            let parts: Vec<String> = d
                .message_type()
                .fields
                .iter()
                .filter_map(|f| d.get_field(f.name).map(|v| format!("{}:{}", json_str(f.name), field_json(&v))))
                .collect();
            format!("{{{}}}", parts.join(","))
        }
    };
    let props: Vec<(&str, String)> = vec![
        ("buffer", msg.buffer().map(json_b64).unwrap_or_else(null)),
        ("rxtime", json_str(&json_date(msg.rxtime))),
        ("header", msg.header().to_string()),
        ("payloadlength", msg.payloadlength().to_string()),
        ("incompat_flags", msg.incompat_flags().to_string()),
        ("compat_flags", msg.compat_flags().to_string()),
        ("seq", msg.seq().to_string()),
        ("sysid", msg.sysid().to_string()),
        ("compid", msg.compid().to_string()),
        ("headerlength", msg.headerlength().to_string()),
        ("target_system", msg.target_system().map(|t| t.to_string()).unwrap_or_else(null)),
        ("msgid", msg.msgid().to_string()),
        ("ismavlink2", msg.ismavlink2().to_string()),
        ("msgtypename", if name.is_empty() { null() } else { json_str(name) }),
        ("data", data),
        ("crc16", msg.crc16().to_string()),
        ("sig", msg.sig().map(json_b64).unwrap_or_else(null)),
        ("sigLinkid", msg.sig_linkid().to_string()),
        ("sigTimestamp", msg.sig_timestamp().to_string()),
        ("Length", msg.length().to_string()),
    ];
    let body: Vec<String> = props.into_iter().map(|(k, v)| format!("{}:{}", json_str(k), v)).collect();
    format!("{{{}}}", body.join(","))
}

/// Newtonsoft's reading of an ISO date string into a `DateTime` (`DateTimeZoneHandling.RoundtripKind`).
fn parse_json_date(s: &str) -> Option<DateTime> {
    let b = s.as_bytes();
    if b.len() < 19 {
        return None;
    }
    let num = |a: usize, n: usize| s.get(a..a + n)?.parse::<u32>().ok();
    let (y, mo, d, h, mi, se) = (num(0, 4)? as i32, num(5, 2)?, num(8, 2)?, num(11, 2)?, num(14, 2)?, num(17, 2)?);
    let mut rest = &s[19..];
    let mut frac_ticks = 0i64;
    if let Some(r) = rest.strip_prefix('.') {
        let digits: String = r.chars().take_while(|c| c.is_ascii_digit()).collect();
        rest = &r[digits.len()..];
        let mut f = digits.clone();
        f.truncate(7);
        while f.len() < 7 {
            f.push('0');
        }
        frac_ticks = f.parse().ok()?;
    }
    let (kind, offset_secs) = if rest == "Z" {
        (DateTimeKind::Utc, 0)
    } else if rest.is_empty() {
        (DateTimeKind::Unspecified, 0)
    } else {
        let sign = if rest.starts_with('-') { -1 } else { 1 };
        let hh: i64 = rest.get(1..3)?.parse().ok()?;
        let mm: i64 = rest.get(4..6)?.parse().ok()?;
        (DateTimeKind::Local, sign * (hh * 3600 + mm * 60))
    };
    let mut dt = DateTime::new(y, mo, d, h, mi, se, DateTimeKind::Utc).ok()?;
    dt.ticks += frac_ticks;
    if kind == DateTimeKind::Local {
        // an offset date becomes local time
        dt.ticks -= offset_secs * TICKS_PER_SECOND;
        return Some(dt.to_local_time());
    }
    dt.kind = kind;
    Some(dt)
}

/// `public static MAVLink.MAVLinkMessage FromJSON(this string msg)`
pub fn from_json(msg: &str) -> Result<Option<MAVLinkMessage>, Exception> {
    let v: Value = serde_json::from_str(msg).map_err(|e| Exception::JsonReader(e.to_string()))?;
    let o = match v {
        Value::Null => return Ok(None),
        Value::Object(o) => o,
        _ => return Err(Exception::JsonSerialization("Cannot deserialize the current JSON into type 'MAVLink+MAVLinkMessage'".into())),
    };
    let mut m = MAVLinkMessage::new();
    for (k, val) in o {
        match k.to_ascii_lowercase().as_str() {
            "buffer" => match val {
                Value::Null => m.set_buffer(None).map_err(|e| Exception::JsonSerialization(e.message().to_string()))?,
                Value::String(s) => {
                    let b = base64::engine::general_purpose::STANDARD.decode(s).map_err(|e| Exception::JsonSerialization(e.to_string()))?;
                    m.set_buffer(Some(b)).map_err(|e| Exception::JsonSerialization(e.message().to_string()))?;
                }
                _ => return Err(Exception::JsonSerialization("Error converting value to type 'System.Byte[]'.".into())),
            },
            "rxtime" => match val {
                Value::String(s) => {
                    m.rxtime = parse_json_date(&s)
                        .ok_or_else(|| Exception::JsonReader(format!("Could not convert string to DateTime: {s}")))?
                }
                _ => return Err(Exception::JsonSerialization("Error converting value to type 'System.DateTime'.".into())),
            },
            // every other property has no public setter and is skipped
            _ => {}
        }
    }
    Ok(Some(m))
}

/// `public static string WrapText(this string msg, int length, char[] spliton)`: after more
/// than `length` characters on a line, break after the next character found in `spliton`.
pub fn wrap_text(msg: &str, length: i32, spliton: &[char]) -> String {
    let units: Vec<u16> = msg.encode_utf16().collect();
    let split: Vec<u16> = spliton.iter().flat_map(|c| { let mut b = [0u16; 2]; c.encode_utf16(&mut b).to_vec() }).collect();
    let mut ans: Vec<u16> = Vec::with_capacity(units.len());
    let mut linecha = 0i32;
    for &u in &units {
        let mut splitline = false;
        if linecha > length {
            for &cha in &split {
                if u == cha {
                    ans.push(u);
                    ans.push(b'\n' as u16);
                    splitline = true;
                    linecha = -1;
                    break;
                }
            }
        }
        if !splitline {
            ans.push(u);
        }
        linecha += 1;
    }
    String::from_utf16_lossy(&ans)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::extlibs::mavlink::*;

    #[test]
    fn wrap_text_breaks_after_split_char() {
        assert_eq!(wrap_text("aaaa bbbb cccc", 4, &[' ']), "aaaa bbbb \ncccc");
        assert_eq!(wrap_text("ab", 0, &[' ']), "ab");
    }

    #[test]
    fn json_round_trip_keeps_buffer_and_time() {
        let mut p = MavlinkParse::new(false);
        let hb = mavlink_heartbeat_t { r#type: 2, custom_mode: 4, ..Default::default() };
        let pkt = p.generate_mavlink_packet20(MAVLINK_MSG_ID::HEARTBEAT, &hb, false, 1, 1, -1, None, 0).unwrap();
        let t = DateTime::new(2024, 3, 5, 13, 4, 9, DateTimeKind::Utc).unwrap();
        let m = MAVLinkMessage::from_buffer_at(pkt, t).unwrap();
        let j = to_json(&m);
        assert!(j.starts_with("{\"buffer\":\""), "{j}");
        assert!(j.contains("\"rxtime\":\"2024-03-05T13:04:09Z\""), "{j}");
        assert!(j.contains("\"msgtypename\":\"HEARTBEAT\""));
        assert!(j.contains("\"data\":{\"custom_mode\":4,\"type\":2,"), "{j}");
        let back = from_json(&j).unwrap().unwrap();
        assert_eq!(back.msgid(), 0);
        assert_eq!(back.rxtime, t);
        assert_eq!(back.to_structure::<mavlink_heartbeat_t>(), hb);
    }

    #[test]
    fn floats_carry_a_decimal_place() {
        assert_eq!(json_float(r_float(1.0)), "1.0");
        assert_eq!(json_float(r_float(0.8)), "0.8");
        assert_eq!(json_float(r_double(1e20)), "1E+20");
        assert_eq!(json_float(r_double(f64::NAN)), "\"NaN\"");
    }
}

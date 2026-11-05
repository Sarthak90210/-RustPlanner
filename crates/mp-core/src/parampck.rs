//! `@PARAM/param.pck` packed parameter format (`ExtLibs/ArduPilot/parampck.cs`,
//! AP_Filesystem_Param): fast parameter download over MAVFtp, optionally with defaults.
//!
//! Header: magic u16 (0x671b, or 0x671c with defaults), num_params u16, total_params u16.
//! Each entry: type:4 | flags:4, common_len:4 | name_len-1:4, name suffix, value
//! (and the default when flags bit 0 is set). Zero bytes between entries are padding.

pub const MAGIC: u16 = 0x671b;
pub const MAGIC_DEFAULTS: u16 = 0x671c;

/// One decoded parameter: (name, value, default when the file carries defaults).
pub type Entry = (String, f32, Option<f32>);

fn value(t: u8, d: &[u8]) -> Option<(f32, usize)> {
    Some(match t {
        1 => (*d.first()? as i8 as f32, 1),
        2 => (i16::from_le_bytes([*d.first()?, *d.get(1)?]) as f32, 2),
        3 => (i32::from_le_bytes(d.get(..4)?.try_into().ok()?) as f32, 4),
        4 => (f32::from_le_bytes(d.get(..4)?.try_into().ok()?), 4),
        _ => return None,
    })
}

pub fn unpack(data: &[u8]) -> Option<Vec<Entry>> {
    if data.len() < 6 {
        return None;
    }
    let magic = u16::from_le_bytes([data[0], data[1]]);
    let num = u16::from_le_bytes([data[2], data[3]]) as usize;
    if magic != MAGIC && magic != MAGIC_DEFAULTS {
        return None;
    }
    let with_defaults = magic == MAGIC_DEFAULTS;
    let mut i = 6;
    let mut last = String::new();
    let mut out = Vec::with_capacity(num);
    loop {
        while i < data.len() && data[i] == 0 {
            i += 1;
        }
        if i + 2 > data.len() {
            break;
        }
        let tb = data[i];
        let lb = data[i + 1];
        i += 2;
        let (t, flags) = (tb & 0x0f, tb >> 4);
        let name_len = ((lb >> 4) & 0x0f) as usize + 1;
        let common = (lb & 0x0f) as usize;
        let suffix = data.get(i..i + name_len)?;
        i += name_len;
        let name = format!("{}{}", last.get(..common.min(last.len()))?, String::from_utf8_lossy(suffix));
        let (v, n) = value(t, &data[i..])?;
        i += n;
        let def = if with_defaults {
            if flags & 1 != 0 {
                let (d, n) = value(t, &data[i..])?;
                i += n;
                Some(d)
            } else {
                Some(v)
            }
        } else {
            None
        };
        last = name.clone();
        out.push((name, v, def));
    }
    (out.len() == num).then_some(out)
}

/// Pack float parameters (sorted by name) with their defaults.
pub fn pack(params: &[(String, f32, f32)]) -> Vec<u8> {
    let mut v: Vec<&(String, f32, f32)> = params.iter().filter(|p| !p.0.is_empty() && p.0.len() <= 16).collect();
    v.sort_by(|a, b| a.0.cmp(&b.0));
    let mut out = Vec::new();
    out.extend_from_slice(&MAGIC_DEFAULTS.to_le_bytes());
    out.extend_from_slice(&(v.len() as u16).to_le_bytes());
    out.extend_from_slice(&(v.len() as u16).to_le_bytes());
    let mut last: &str = "";
    for (name, val, def) in v {
        let common = name.bytes().zip(last.bytes()).take_while(|(a, b)| a == b).count().min(15).min(name.len() - 1);
        let suffix = &name.as_bytes()[common..];
        let has_def = val.to_bits() != def.to_bits();
        out.push(4 | if has_def { 0x10 } else { 0 });
        out.push((((suffix.len() - 1) as u8) << 4) | common as u8);
        out.extend_from_slice(suffix);
        out.extend_from_slice(&val.to_le_bytes());
        if has_def {
            out.extend_from_slice(&def.to_le_bytes());
        }
        last = name;
    }
    out
}

#[cfg(test)]
mod tests {
    #[test]
    fn roundtrip() {
        let p = vec![("ATC_RAT_RLL_P".to_string(), 0.2, 0.135), ("ATC_RAT_RLL_I".to_string(), 0.135, 0.135), ("BATT_CAPACITY".to_string(), 5000.0, 3300.0)];
        let u = super::unpack(&super::pack(&p)).unwrap();
        assert_eq!(u.len(), 3);
        assert_eq!(u[0], ("ATC_RAT_RLL_I".to_string(), 0.135, Some(0.135)));
        assert_eq!(u[1], ("ATC_RAT_RLL_P".to_string(), 0.2, Some(0.135)));
        assert_eq!(u[2], ("BATT_CAPACITY".to_string(), 5000.0, Some(3300.0)));
    }
}

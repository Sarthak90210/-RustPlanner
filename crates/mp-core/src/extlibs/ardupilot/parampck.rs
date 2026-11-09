//! Port of `ExtLibs/ArduPilot/parampck.cs`
//!
//! The packed `@PARAM/param.pck` format ArduPilot serves over MAVFTP:
//!
//! ```text
//! file header:
//!   uint16_t magic = 0x671b (0x671c with defaults)
//!   uint16_t num_params
//!   uint16_t total_params
//! per-parameter:
//!   uint8_t type:4;         // AP_Param type NONE=0, INT8=1, INT16=2, INT32=3, FLOAT=4
//!   uint8_t flags:4;        // bit 0: includes default value for this param
//!   uint8_t common_len:4;   // number of name bytes in common with previous entry, 0..15
//!   uint8_t name_len:4;     // non-common length of param name -1 (0..15)
//!   uint8_t name[name_len]; // name
//!   uint8_t data[];         // value, length given by variable type, data length doubled if default is included
//! Any leading zero bytes after the header should be discarded as pad bytes.
//! ```

use crate::dotnet::{bit_converter, float_to_string, Exception};
use crate::extlibs::mavlink::{MAVLinkParam, MAVLinkParamList, MAV_PARAM_TYPE};

const LOG: &str = "MissionPlanner.ArduPilot.parampck";

const MAGIC: u16 = 0x671b;
const MAGIC_WITH_DEFAULTS: u16 = 0x671c;

/// `map`: AP_Param type -> (type_len, type_format)
fn map(ptype: u8) -> Option<(usize, char)> {
    Some(match ptype {
        1 => (1, 'b'),
        2 => (2, 'h'),
        3 => (4, 'i'),
        4 => (4, 'f'),
        _ => return None,
    })
}

/// A boxed `decode_value` result.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Decoded {
    SByte(i8),
    Short(i16),
    Int(i32),
    Single(f32),
}

impl Decoded {
    /// `Convert.ToDouble(v)`
    fn to_double(self) -> f64 {
        match self {
            Decoded::SByte(v) => v as f64,
            Decoded::Short(v) => v as f64,
            Decoded::Int(v) => v as f64,
            Decoded::Single(v) => v as f64,
        }
    }
}

impl std::fmt::Display for Decoded {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Decoded::SByte(v) => write!(f, "{v}"),
            Decoded::Short(v) => write!(f, "{v}"),
            Decoded::Int(v) => write!(f, "{v}"),
            Decoded::Single(v) => f.write_str(&float_to_string(*v)),
        }
    }
}

fn index_out_of_range() -> Exception {
    Exception::IndexOutOfRange("Index was outside the bounds of the array.".into())
}

fn decode_value(type_format: char, data: &[u8], start_index: usize) -> Result<Decoded, Exception> {
    let si = start_index as i32;
    Ok(match type_format {
        'b' => Decoded::SByte(*data.get(start_index).ok_or_else(index_out_of_range)? as i8),
        'h' => Decoded::Short(bit_converter::to_i16(data, si)?),
        'i' => Decoded::Int(bit_converter::to_i32(data, si)?),
        'f' => Decoded::Single(bit_converter::to_single(data, si)?),
        _ => return Err(Exception::Other(format!("Unexpected type_format: {type_format}"))),
    })
}

/// `public static class parampck`
#[allow(non_camel_case_types)]
pub struct parampck;

impl parampck {
    /// `unpack(byte[] data)`: `Ok(None)` is the C# `null` (bad magic, unknown type, or a count
    /// that does not match the header).
    pub fn unpack(data: &[u8]) -> Result<Option<MAVLinkParamList>, Exception> {
        let mut list = MAVLinkParamList::new();

        if data.len() < 6 {
            return Ok(None);
        }

        let mut data_pointer = 0usize;

        let magic2 = bit_converter::to_u16(data, data_pointer as i32)?;
        data_pointer += 2;
        let num_params = bit_converter::to_u16(data, data_pointer as i32)?;
        data_pointer += 2;
        let total_params = bit_converter::to_u16(data, data_pointer as i32)?;
        data_pointer += 2;

        if magic2 != MAGIC && magic2 != MAGIC_WITH_DEFAULTS {
            return Ok(None);
        }

        let with_defaults = magic2 == MAGIC_WITH_DEFAULTS;

        let pad_byte = 0u8;
        let mut count = 0i32;
        let mut last_name = String::new();
        loop {
            while data.len() - data_pointer > 0 && data[data_pointer] == pad_byte {
                data_pointer += 1;
            }

            if data.len() - data_pointer == 0 {
                break;
            }

            let mut ptype = *data.get(data_pointer).ok_or_else(index_out_of_range)?;
            data_pointer += 1;
            let plen = *data.get(data_pointer).ok_or_else(index_out_of_range)?;
            data_pointer += 1;
            let flags = (ptype >> 4) & 0x0F;
            ptype &= 0x0F;

            let Some((type_len, type_format)) = map(ptype) else { return Ok(None) };

            let name_len = (((plen >> 4) & 0x0F) + 1) as usize;
            let common_len = (plen & 0x0F) as usize;

            // last_name.Substring(0, common_len)
            let prefix: String = {
                let units: Vec<u16> = last_name.encode_utf16().collect();
                if common_len > units.len() {
                    return Err(Exception::ArgumentOutOfRange(
                        "Index and length must refer to a location within the string.\r\nParameter name: length".into(),
                    ));
                }
                String::from_utf16_lossy(&units[..common_len])
            };
            let mut name = prefix;
            for i in data_pointer..data_pointer + name_len {
                // (char)data[i]: one Latin-1 character per byte
                name.push(*data.get(i).ok_or_else(index_out_of_range)? as char);
            }
            data_pointer += name_len;

            last_name = name.clone();
            let v = decode_value(type_format, data, data_pointer)?;

            count += 1;
            log::debug!(target: LOG, "{:<16} {:<16} {:<16} {:<16}", name, v.to_string(), type_len, type_format);

            let mut vdata = [0u8; 4];
            if data_pointer + type_len > data.len() {
                return Err(Exception::Argument(
                    "Source array was not long enough. Check srcIndex and length, and the array's lower bounds.".into(),
                ));
            }
            vdata[..type_len].copy_from_slice(&data[data_pointer..data_pointer + type_len]);
            data_pointer += type_len;

            let mut default_value = None;
            if with_defaults {
                if (flags as u32 & 1) == 0 {
                    default_value = Some(v.to_double());
                } else {
                    default_value = Some(decode_value(type_format, data, data_pointer)?.to_double());
                    data_pointer += type_len;
                }
            }

            let t = match ptype {
                1 => MAV_PARAM_TYPE::INT8,
                2 => MAV_PARAM_TYPE::INT16,
                3 => MAV_PARAM_TYPE::INT32,
                4 => MAV_PARAM_TYPE::REAL32,
                _ => MAV_PARAM_TYPE(0),
            };
            list.add(MAVLinkParam::from_wire(&name, &vdata, t, t, default_value));
        }

        if count != num_params as i32 || count > total_params as i32 {
            return Ok(None);
        }

        Ok(Some(list))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pack(params: &[(&str, u8, &[u8])], with_defaults: bool) -> Vec<u8> {
        let mut out = vec![];
        out.extend((if with_defaults { MAGIC_WITH_DEFAULTS } else { MAGIC }).to_le_bytes());
        out.extend((params.len() as u16).to_le_bytes());
        out.extend((params.len() as u16).to_le_bytes());
        let mut last = String::new();
        for (name, t, data) in params {
            let common = last.bytes().zip(name.bytes()).take_while(|(a, b)| a == b).count().min(15);
            let rest = &name[common..];
            out.push(*t);
            out.push((((rest.len() - 1) as u8) << 4) | common as u8);
            out.extend(rest.bytes());
            out.extend(*data);
            last = name.to_string();
        }
        out
    }

    #[test]
    fn unpacks_shared_prefix_names() {
        let b = pack(&[("RC1_MAX", 2, &1900i16.to_le_bytes()), ("RC1_MIN", 2, &1100i16.to_le_bytes()), ("WPNAV_SPEED", 4, &500f32.to_le_bytes())], false);
        let l = parampck::unpack(&b).unwrap().unwrap();
        assert_eq!(l.len(), 3);
        assert_eq!(l.get("RC1_MIN").unwrap().value().unwrap(), 1100.0);
        assert_eq!(l.get("WPNAV_SPEED").unwrap().r#type, MAV_PARAM_TYPE::REAL32);
        assert_eq!(l.get("WPNAV_SPEED").unwrap().default_value(), None);
    }

    #[test]
    fn defaults_and_rejections() {
        let mut data = 5i8.to_le_bytes().to_vec();
        data.extend(7i8.to_le_bytes());
        let b = pack(&[("A", 1 | 0x10, &data)], true);
        let l = parampck::unpack(&b).unwrap().unwrap();
        assert_eq!(l.get("A").unwrap().default_value(), Some(7.0));
        assert!(parampck::unpack(&[0, 0, 0]).unwrap().is_none());
        let mut bad = b.clone();
        bad[0] = 0;
        assert!(parampck::unpack(&bad).unwrap().is_none());
        let mut badtype = b.clone();
        badtype[6] = 5;
        assert!(parampck::unpack(&badtype).unwrap().is_none());
    }
}

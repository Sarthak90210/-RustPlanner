//! Port of `ExtLibs/Mavlink/Mavlink.cs`
//!
//! `Mavlink.cs` is emitted by Mission Planner's forked pymavlink generator. Its Rust
//! counterpart is emitted by `crates/mp-core/build.rs`, a port of that generator run over the
//! same `all.xml` + `offspec.xml`, and included below. This file holds the pieces of the
//! generated C# that are fixed template text rather than per-message output: the protocol
//! constants, `message_info`, the enum and attribute plumbing, and the struct (de)serialisers
//! that C# gets from `Marshal.PtrToStructure` / `StructLayout(Pack=1)`.
#![allow(non_camel_case_types, non_upper_case_globals, non_snake_case, clippy::all, deprecated)]

use std::any::{Any, TypeId};
use std::fmt;

pub const MAVLINK_CORE_HEADER_LEN: u8 = 9;
pub const MAVLINK_CORE_HEADER_MAVLINK1_LEN: u8 = 5;
pub const MAVLINK_NUM_HEADER_BYTES: u8 = MAVLINK_CORE_HEADER_LEN + 1;
pub const MAVLINK_NUM_CHECKSUM_BYTES: u8 = 2;
pub const MAVLINK_NUM_NON_PAYLOAD_BYTES: u8 = MAVLINK_NUM_HEADER_BYTES + MAVLINK_NUM_CHECKSUM_BYTES;
pub const MAVLINK_MAX_HEADER_BYTES: i32 = MAVLINK_NUM_HEADER_BYTES as i32 + 7;
pub const MAVLINK_MAX_PACKET_LEN: i32 = MAVLINK_MAX_PAYLOAD_LEN
    + MAVLINK_MAX_HEADER_BYTES
    + MAVLINK_NUM_CHECKSUM_BYTES as i32
    + MAVLINK_SIGNATURE_BLOCK_LEN as i32;
pub const MAVLINK_SIGNATURE_BLOCK_LEN: u8 = 13;
pub const MAVLINK_LITTLE_ENDIAN: i32 = 1;
pub const MAVLINK_BIG_ENDIAN: i32 = 0;
pub const MAVLINK_STX_MAVLINK1: u8 = 0xFE;
pub const MAVLINK_ENDIAN: u8 = MAVLINK_LITTLE_ENDIAN as u8;
pub const MAVLINK_ALIGNED_FIELDS: bool = true;
pub const MAVLINK_CRC_EXTRA: u8 = 1;
pub const MAVLINK_COMMAND_24BIT: u8 = 1;
pub const MAVLINK_NEED_BYTE_SWAP: bool = MAVLINK_ENDIAN as i32 == MAVLINK_LITTLE_ENDIAN;

pub const MAVLINK_IFLAG_SIGNED: u8 = 0x01;
pub const MAVLINK_IFLAG_SYSID32: u8 = 0x02;
pub const MAVLINK_IFLAG_TARGET32: u8 = 0x04;
pub const MAVLINK_IFLAG_MASK: u8 = MAVLINK_IFLAG_SIGNED | MAVLINK_IFLAG_SYSID32 | MAVLINK_IFLAG_TARGET32;

/// `public struct message_info`. `type` is the generated struct; `None` is the C# `null`
/// that `GetMessageInfo` returns inside `new message_info()` for an unknown id.
#[derive(Clone, Copy)]
pub struct message_info {
    pub msgid: u32,
    pub name: &'static str,
    pub crc: u8,
    pub minlength: u32,
    pub length: u32,
    pub r#type: Option<&'static MessageType>,
}

impl message_info {
    /// `new message_info()`: every field zero, `name` and `type` null.
    pub const EMPTY: message_info = message_info { msgid: 0, name: "", crc: 0, minlength: 0, length: 0, r#type: None };
}

impl Default for message_info {
    fn default() -> Self {
        Self::EMPTY
    }
}

impl fmt::Display for message_info {
    /// `String.Format("{0} - {1}", name, msgid)`
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} - {}", self.name, self.msgid)
    }
}

impl fmt::Debug for message_info {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Display::fmt(self, f)
    }
}

/// `public static message_info[] MAVLINK_MESSAGE_INFOS`: a static field that code such as
/// `MavlinkMessagePlugin` replaces with an extended copy at runtime.
pub struct MessageInfoTable(std::sync::RwLock<&'static [message_info]>);

impl MessageInfoTable {
    /// The current table.
    pub fn get(&self) -> &'static [message_info] {
        *self.0.read().unwrap_or_else(|e| e.into_inner())
    }

    /// Assign a new array to the field.
    pub fn set(&self, table: &'static [message_info]) {
        *self.0.write().unwrap_or_else(|e| e.into_inner()) = table;
    }
}

pub static MAVLINK_MESSAGE_INFOS: MessageInfoTable = MessageInfoTable(std::sync::RwLock::new(MAVLINK_MESSAGE_INFOS_GENERATED));

/// What C# reaches through `System.Type` for a generated `mavlink_*_t` struct.
pub struct MessageType {
    /// C# type name, e.g. `mavlink_heartbeat_t`.
    pub name: &'static str,
    /// `Marshal.SizeOf(type)`: the full wire length.
    pub size: usize,
    /// `[Obsolete]` on the struct (a deprecated or work-in-progress message).
    pub obsolete: bool,
    /// `type.GetFields()` in declaration (wire) order.
    pub fields: &'static [FieldInfo],
    pub type_id: fn() -> TypeId,
    /// `Activator.CreateInstance` + `Marshal.PtrToStructure` over a zero-padded copy.
    pub deserialize: fn(&[u8]) -> Box<dyn MavlinkData>,
}

impl MessageType {
    /// `type.GetField(name)`
    pub fn get_field(&self, name: &str) -> Option<&'static FieldInfo> {
        self.fields.iter().find(|f| f.name == name)
    }
}

impl fmt::Debug for MessageType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name)
    }
}

impl PartialEq for MessageType {
    fn eq(&self, other: &Self) -> bool {
        (self.type_id)() == (other.type_id)()
    }
}

/// A generated field with its `[Units]` and `[Description]` attributes.
#[derive(Debug, Clone, Copy)]
pub struct FieldInfo {
    pub name: &'static str,
    /// The C# field type, e.g. `float`, `byte[]`.
    pub cs_type: &'static str,
    /// `Marshal.OffsetOf(type, name)`
    pub offset: usize,
    pub array_length: usize,
    pub units: &'static str,
    pub description: &'static str,
}

/// A boxed field value, as `FieldInfo.GetValue` returns an `object`.
#[derive(Debug, Clone, PartialEq)]
pub enum FieldValue {
    Float(f32),
    Double(f64),
    Byte(u8),
    SByte(i8),
    Short(i16),
    UShort(u16),
    Int(i32),
    UInt(u32),
    Long(i64),
    ULong(u64),
    FloatArray(Vec<f32>),
    DoubleArray(Vec<f64>),
    ByteArray(Vec<u8>),
    SByteArray(Vec<i8>),
    ShortArray(Vec<i16>),
    UShortArray(Vec<u16>),
    IntArray(Vec<i32>),
    UIntArray(Vec<u32>),
    LongArray(Vec<i64>),
    ULongArray(Vec<u64>),
}

impl FieldValue {
    /// `Convert.ToDouble(value)` for scalars; `None` for arrays (C# throws InvalidCastException).
    pub fn to_f64(&self) -> Option<f64> {
        Some(match *self {
            FieldValue::Float(v) => v as f64,
            FieldValue::Double(v) => v,
            FieldValue::Byte(v) => v as f64,
            FieldValue::SByte(v) => v as f64,
            FieldValue::Short(v) => v as f64,
            FieldValue::UShort(v) => v as f64,
            FieldValue::Int(v) => v as f64,
            FieldValue::UInt(v) => v as f64,
            FieldValue::Long(v) => v as f64,
            FieldValue::ULong(v) => v as f64,
            _ => return None,
        })
    }

    /// `Convert.ToUInt32(value)` as used on target_system fields.
    pub fn to_u32(&self) -> Option<u32> {
        Some(match *self {
            FieldValue::Byte(v) => v as u32,
            FieldValue::SByte(v) if v >= 0 => v as u32,
            FieldValue::UShort(v) => v as u32,
            FieldValue::UInt(v) => v,
            _ => return None,
        })
    }

    pub fn is_array(&self) -> bool {
        self.to_f64().is_none()
    }
}

impl fmt::Display for FieldValue {
    /// `value.ToString()`: scalars in .NET Framework format, arrays as their type name.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        use crate::dotnet::{double_to_string, float_to_string};
        match self {
            FieldValue::Float(v) => f.write_str(&float_to_string(*v)),
            FieldValue::Double(v) => f.write_str(&double_to_string(*v)),
            FieldValue::Byte(v) => write!(f, "{v}"),
            FieldValue::SByte(v) => write!(f, "{v}"),
            FieldValue::Short(v) => write!(f, "{v}"),
            FieldValue::UShort(v) => write!(f, "{v}"),
            FieldValue::Int(v) => write!(f, "{v}"),
            FieldValue::UInt(v) => write!(f, "{v}"),
            FieldValue::Long(v) => write!(f, "{v}"),
            FieldValue::ULong(v) => write!(f, "{v}"),
            FieldValue::FloatArray(_) => f.write_str("System.Single[]"),
            FieldValue::DoubleArray(_) => f.write_str("System.Double[]"),
            FieldValue::ByteArray(_) => f.write_str("System.Byte[]"),
            FieldValue::SByteArray(_) => f.write_str("System.SByte[]"),
            FieldValue::ShortArray(_) => f.write_str("System.Int16[]"),
            FieldValue::UShortArray(_) => f.write_str("System.UInt16[]"),
            FieldValue::IntArray(_) => f.write_str("System.Int32[]"),
            FieldValue::UIntArray(_) => f.write_str("System.UInt32[]"),
            FieldValue::LongArray(_) => f.write_str("System.Int64[]"),
            FieldValue::ULongArray(_) => f.write_str("System.UInt64[]"),
        }
    }
}

/// A generated message struct seen as `object` (what `MAVLinkMessage.data` returns).
pub trait MavlinkData: Any + Send + Sync + fmt::Debug {
    fn msgid(&self) -> u32;
    fn message_type(&self) -> &'static MessageType;
    /// `MavlinkUtil.StructureToByteArray`
    fn to_bytes(&self) -> Vec<u8>;
    fn as_any(&self) -> &dyn Any;
    fn as_any_mut(&mut self) -> &mut dyn Any;
    fn clone_box(&self) -> Box<dyn MavlinkData>;
    /// `type.GetField(name).GetValue(obj)`
    fn get_field(&self, name: &str) -> Option<FieldValue>;
    /// `type.GetField(name).SetValue(obj, value)`; false when the name or value type mismatch.
    fn set_field(&mut self, name: &str, value: &FieldValue) -> bool;
}

impl Clone for Box<dyn MavlinkData> {
    fn clone(&self) -> Self {
        self.clone_box()
    }
}

impl dyn MavlinkData {
    pub fn downcast_ref<T: 'static>(&self) -> Option<&T> {
        self.as_any().downcast_ref::<T>()
    }
}

/// One member of a generated enum, with its `[Description]`, `[Obsolete]` and `[hasLocation()]`.
#[derive(Debug, Clone, Copy)]
pub struct EnumEntry {
    pub name: &'static str,
    pub value: i64,
    pub description: &'static str,
    pub obsolete: bool,
    pub has_location: bool,
}

/// A generated enum as reflection sees it: `Enum.GetUnderlyingType`, `[Flags]`, the members.
#[derive(Debug, Clone, Copy)]
pub struct EnumInfo {
    pub name: &'static str,
    /// C# underlying type (`byte`, `ushort`, `uint`, `int`, ...).
    pub base_type: &'static str,
    pub flags: bool,
    pub entries: &'static [EnumEntry],
}

/// Behaviour shared by every generated enum (C# `System.Enum`).
pub trait CsEnum: Copy + Sized + 'static {
    const FLAGS: bool;
    const TYPE_NAME: &'static str;
    fn entries() -> &'static [EnumEntry];
    fn value_i64(self) -> i64;
    fn from_i64(v: i64) -> Self;

    /// `Enum.GetName(typeof(T), value)`
    fn get_name(self) -> Option<&'static str> {
        let v = self.value_i64();
        Self::entries().iter().find(|e| e.value == v).map(|e| e.name)
    }

    /// `Enum.IsDefined`
    fn is_defined(self) -> bool {
        self.get_name().is_some()
    }

    /// `Enum.Parse(typeof(T), name)` (case sensitive, accepts numbers and "A, B" for flags).
    fn parse(s: &str) -> Option<Self> {
        let s = s.trim();
        if let Ok(v) = s.parse::<i64>() {
            return Some(Self::from_i64(v));
        }
        let mut acc = 0i64;
        for part in s.split(',') {
            let part = part.trim();
            acc |= Self::entries().iter().find(|e| e.name == part)?.value;
        }
        Some(Self::from_i64(acc))
    }

    /// `Enum.GetValues(typeof(T))`, sorted by unsigned magnitude like .NET.
    fn get_values() -> Vec<Self> {
        let mut v: Vec<i64> = Self::entries().iter().map(|e| e.value).collect();
        v.sort_by_key(|x| *x as u64);
        v.into_iter().map(Self::from_i64).collect()
    }

    /// `Enum.GetNames(typeof(T))`, in the same order as `GetValues`.
    fn get_names() -> Vec<&'static str> {
        let mut v: Vec<&EnumEntry> = Self::entries().iter().collect();
        v.sort_by_key(|e| e.value as u64);
        v.into_iter().map(|e| e.name).collect()
    }

    /// The `[Description]` attribute text of the member with this value.
    fn description(self) -> Option<&'static str> {
        let v = self.value_i64();
        Self::entries().iter().find(|e| e.value == v).map(|e| e.description)
    }

    /// Whether the member with this value carries `[hasLocation()]`.
    fn has_location(self) -> bool {
        let v = self.value_i64();
        Self::entries().iter().any(|e| e.value == v && e.has_location)
    }

    /// `Enum.ToString()` with .NET Framework semantics, including `[Flags]` composition.
    fn to_cs_string(self) -> String {
        let v = self.value_i64();
        if let Some(n) = self.get_name() {
            return n.to_string();
        }
        if Self::FLAGS {
            if v == 0 {
                return "0".into();
            }
            let mut vals: Vec<&EnumEntry> = Self::entries().iter().filter(|e| e.value != 0).collect();
            vals.sort_by_key(|e| std::cmp::Reverse(e.value as u64));
            let mut result = v as u64;
            let mut names = vec![];
            for e in vals {
                let ev = e.value as u64;
                if result & ev == ev {
                    result -= ev;
                    names.push(e.name);
                }
                if result == 0 {
                    break;
                }
            }
            if result == 0 {
                names.reverse();
                return names.join(", ");
            }
        }
        v.to_string()
    }
}

macro_rules! impl_cs_enum {
    ($name:ident, $rt:ty, $flags:expr) => {
        impl CsEnum for $name {
            const FLAGS: bool = $flags;
            const TYPE_NAME: &'static str = stringify!($name);
            fn entries() -> &'static [EnumEntry] {
                $name::ENTRIES
            }
            fn value_i64(self) -> i64 {
                self.0 as i64
            }
            fn from_i64(v: i64) -> Self {
                $name(v as $rt)
            }
        }
        impl fmt::Display for $name {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                f.write_str(&self.to_cs_string())
            }
        }
        impl fmt::Debug for $name {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                f.write_str(&self.to_cs_string())
            }
        }
        impl From<$rt> for $name {
            fn from(v: $rt) -> Self {
                $name(v)
            }
        }
        impl From<$name> for $rt {
            fn from(v: $name) -> Self {
                v.0
            }
        }
        impl std::ops::BitOr for $name {
            type Output = $name;
            fn bitor(self, o: $name) -> $name {
                $name(self.0 | o.0)
            }
        }
        impl std::ops::BitAnd for $name {
            type Output = $name;
            fn bitand(self, o: $name) -> $name {
                $name(self.0 & o.0)
            }
        }
    };
}

/// Little-endian field access for `StructLayout(LayoutKind.Sequential, Pack=1)`.
pub mod wire {
    pub trait Wire: Copy + Default {
        const SIZE: usize;
        fn get(b: &[u8], off: usize) -> Self;
        fn put(self, b: &mut [u8], off: usize);
    }

    macro_rules! wire_prim {
        ($($t:ty),*) => {$(
            impl Wire for $t {
                const SIZE: usize = std::mem::size_of::<$t>();
                fn get(b: &[u8], off: usize) -> Self {
                    let mut a = [0u8; std::mem::size_of::<$t>()];
                    a.copy_from_slice(&b[off..off + Self::SIZE]);
                    <$t>::from_le_bytes(a)
                }
                fn put(self, b: &mut [u8], off: usize) {
                    b[off..off + Self::SIZE].copy_from_slice(&self.to_le_bytes());
                }
            }
        )*};
    }
    wire_prim!(u8, i8, u16, i16, u32, i32, u64, i64, f32, f64);

    pub fn get_arr<T: Wire, const N: usize>(b: &[u8], off: usize) -> [T; N] {
        let mut a = [T::default(); N];
        for (i, x) in a.iter_mut().enumerate() {
            *x = T::get(b, off + i * T::SIZE);
        }
        a
    }

    pub fn put_arr<T: Wire, const N: usize>(a: &[T; N], b: &mut [u8], off: usize) {
        for (i, x) in a.iter().enumerate() {
            x.put(b, off + i * T::SIZE);
        }
    }
}

include!(concat!(env!("OUT_DIR"), "/mavlink_generated.rs"));

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn message_table_matches_mission_planner_mavlink_cs() {
        // Spot checks against reference/MissionPlanner/ExtLibs/Mavlink/Mavlink.cs.
        let first = &MAVLINK_MESSAGE_INFOS.get()[0];
        assert_eq!((first.msgid, first.name, first.crc, first.minlength, first.length), (0, "HEARTBEAT", 50, 9, 9));
        let s = &MAVLINK_MESSAGE_INFOS.get()[1];
        assert_eq!((s.msgid, s.name, s.crc, s.minlength, s.length), (1, "SYS_STATUS", 124, 31, 43));
        let last = MAVLINK_MESSAGE_INFOS.get().last().unwrap();
        assert_eq!((last.msgid, last.name, last.crc, last.minlength, last.length), (26900, "zVIDEO_STREAM_INFORMATION", 124, 246, 246));
        assert_eq!(MAVLINK_MESSAGE_INFOS.get().len(), 354);
        assert_eq!(MAVLINK_MAX_PAYLOAD_LEN, 255);
    }

    #[test]
    fn enums_follow_mavgen_cs_naming() {
        assert_eq!(MAV_TYPE::QUADROTOR.0, 2u8);
        assert_eq!(MAV_TYPE::QUADROTOR.to_string(), "QUADROTOR");
        assert_eq!(MAV_CMD::WAYPOINT.0, 16);
        assert!(MAV_CMD::WAYPOINT.has_location());
        assert_eq!(MAV_TYPE(250).to_string(), "250");
    }

    #[test]
    fn heartbeat_round_trip() {
        let hb = mavlink_heartbeat_t { custom_mode: 5, r#type: 2, autopilot: 3, base_mode: 81, system_status: 4, mavlink_version: 3 };
        let b = hb.to_bytes();
        assert_eq!(b.len(), 9);
        assert_eq!(mavlink_heartbeat_t::from_bytes(&b), hb);
        // truncated payloads are zero-extended
        assert_eq!(mavlink_heartbeat_t::from_bytes(&b[..4]).r#type, 0);
    }
}

/// Whole-file comparison with Mission Planner's generated Mavlink.cs, through the digest
/// `scripts/mavlink_cs_digest.py` extracts into `tests/fixtures/` (the reference tree itself
/// is not committed).
#[cfg(test)]
mod digest_tests {
    use super::*;
    use serde_json::Value;

    fn digest() -> Value {
        let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/mavlink_cs_digest.json.gz");
        let f = std::fs::File::open(path).expect("mavlink_cs_digest.json.gz");
        serde_json::from_reader(flate2::read::GzDecoder::new(f)).unwrap()
    }

    #[test]
    fn build_date_and_message_infos_match() {
        let d = digest();
        assert_eq!(d["build_date"], MAVLINK_BUILD_DATE);
        let rows = d["message_infos"].as_array().unwrap();
        let table = MAVLINK_MESSAGE_INFOS.get();
        assert_eq!(rows.len(), table.len());
        for (row, r) in rows.iter().zip(table) {
            assert_eq!(row[0], r.msgid);
            assert_eq!(row[1], r.name);
            assert_eq!(row[2], r.crc as u64, "{}", r.name);
            assert_eq!(row[3], r.minlength, "{}", r.name);
            assert_eq!(row[4], r.length, "{}", r.name);
            assert_eq!(row[5], r.r#type.unwrap().name);
        }
    }

    #[test]
    fn every_enum_matches_with_attributes() {
        let d = digest();
        let enums = d["enums"].as_array().unwrap();
        assert_eq!(enums.len(), 232);
        for e in enums {
            let name = e["name"].as_str().unwrap();
            let ours = enum_info(name).unwrap_or_else(|| panic!("missing enum {name}"));
            assert_eq!(e["base"], ours.base_type, "{name} underlying type");
            assert_eq!(e["flags"], ours.flags, "{name} [Flags]");
            let members = e["members"].as_array().unwrap();
            assert_eq!(members.len(), ours.entries.len(), "{name} member count");
            for (m, o) in members.iter().zip(ours.entries) {
                assert_eq!(m[0], o.name, "{name}");
                assert_eq!(m[1], o.value, "{name}.{}", o.name);
                assert_eq!(m[2], o.description, "{name}.{} [Description]", o.name);
                assert_eq!(m[3], o.obsolete, "{name}.{} [Obsolete]", o.name);
                assert_eq!(m[4], o.has_location, "{name}.{} [hasLocation()]", o.name);
            }
        }
    }

    #[test]
    fn every_struct_field_matches_with_attributes() {
        let d = digest();
        let structs = d["structs"].as_array().unwrap();
        assert_eq!(structs.len(), MAVLINK_MESSAGE_INFOS.get().len());
        for st in structs {
            let name = st["name"].as_str().unwrap();
            let t = MAVLINK_MESSAGE_INFOS.get().iter().find_map(|i| i.r#type.filter(|t| t.name == name)).unwrap();
            assert_eq!(st["obsolete"], t.obsolete, "{name} [Obsolete]");
            let fields = st["fields"].as_array().unwrap();
            assert_eq!(fields.len(), t.fields.len(), "{name} field count");
            for (f, o) in fields.iter().zip(t.fields) {
                assert_eq!(f[0], o.name, "{name}");
                assert_eq!(f[1], o.cs_type, "{name}.{}", o.name);
                assert_eq!(f[2], o.array_length, "{name}.{} SizeConst", o.name);
                assert_eq!(f[3], o.offset, "{name}.{} FieldOffset", o.name);
                assert_eq!(f[4], o.units, "{name}.{} [Units]", o.name);
                assert_eq!(f[5], o.description, "{name}.{} [Description]", o.name);
            }
        }
    }
}

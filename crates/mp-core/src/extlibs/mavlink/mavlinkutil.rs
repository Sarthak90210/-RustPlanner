//! Port of `ExtLibs/Mavlink/MavlinkUtil.cs`
//!
//! C# marshals any blittable struct through `Marshal.PtrToStructure` / `StructureToPtr`. In
//! Rust the structs implement [`MarshalStruct`] (the generated MAVLink structs do so through
//! `build.rs`), which gives the same zero-padded little-endian layout.

use std::sync::atomic::{AtomicBool, Ordering};

use super::mavlink::{message_info, FieldInfo, MavlinkData, MessageType};
use crate::dotnet::Exception;

/// A `StructLayout(LayoutKind.Sequential, Pack=1)` struct.
pub trait MarshalStruct: Sized + Default + 'static {
    /// `Marshal.SizeOf(typeof(T))`
    const SIZE: usize;
    /// `typeof(T).GetFields()` with offsets.
    fn fields() -> &'static [FieldInfo];
    /// Reads the first `SIZE` bytes of `b`, zero-extending a shorter slice.
    fn from_bytes(b: &[u8]) -> Self;
    /// `StructureToByteArray`: exactly `SIZE` bytes.
    fn to_bytes(&self) -> Vec<u8>;
}

static USE_UNSAFE: AtomicBool = AtomicBool::new(true);

fn field_scalar_size(f: &FieldInfo) -> usize {
    match f.cs_type.trim_end_matches("[]") {
        "double" | "long" | "ulong" => 8,
        "float" | "int" | "uint" => 4,
        "short" | "ushort" => 2,
        _ => 1,
    }
}

/// `public static class MavlinkUtil`
pub struct MavlinkUtil;

impl MavlinkUtil {
    pub fn use_unsafe() -> bool {
        USE_UNSAFE.load(Ordering::Relaxed)
    }

    pub fn set_use_unsafe(v: bool) {
        USE_UNSAFE.store(v, Ordering::Relaxed)
    }

    /// `ByteArrayToStructure<T>(bytearray, startoffset = 6)`
    pub fn byte_array_to_structure<T: MarshalStruct>(bytearray: &[u8], startoffset: usize) -> T {
        if Self::use_unsafe() {
            Self::read_using_pointer::<T>(Some(bytearray), startoffset)
        } else {
            Self::byte_array_to_structure_gc::<T>(bytearray, startoffset)
        }
    }

    /// `ByteArrayToStructureBigEndian<T>(bytearray, startoffset = 6)`
    pub fn byte_array_to_structure_big_endian<T: MarshalStruct>(bytearray: &[u8], startoffset: usize) -> T {
        let mut new_packet = T::default();
        Self::byte_array_to_structure_endian(bytearray, &mut new_packet, startoffset);
        new_packet
    }

    /// `ByteArrayToStructure(byte[] bytearray, ref object obj, int startoffset, int payloadlength = 0)`
    pub fn byte_array_to_structure_ref<T: MarshalStruct>(bytearray: Option<&[u8]>, obj: &mut T, startoffset: usize, payloadlength: usize) {
        let Some(bytearray) = bytearray else { return };
        if bytearray.len() < startoffset + payloadlength || payloadlength == 0 {
            return;
        }
        let len = T::SIZE;
        let mut buf = vec![0u8; len];
        let payloadlength = payloadlength.min(len); // unknown mavlink extension...
        buf[..payloadlength].copy_from_slice(&bytearray[startoffset..startoffset + payloadlength]);
        *obj = T::from_bytes(&buf);
    }

    /// `ByteArrayToStructureT<T>(bytearray, startoffset)`. C# copies the tail of the array and
    /// marshals `SizeOf(T)` bytes from it; a short tail reads past the copy, which here reads zeros.
    pub fn byte_array_to_structure_t<T: MarshalStruct>(bytearray: Option<&[u8]>, startoffset: usize) -> T {
        match bytearray {
            Some(b) if b.len() >= startoffset => T::from_bytes(&b[startoffset..]),
            _ => T::default(),
        }
    }

    /// `trim_payload(ref byte[] payload)`: drop trailing zero bytes, keeping at least one byte.
    pub fn trim_payload(payload: &mut Vec<u8>) -> &mut Vec<u8> {
        let mut length = payload.len();
        while length > 1 && payload[length - 1] == 0 {
            length -= 1;
        }
        if length != payload.len() {
            payload.truncate(length);
        }
        payload
    }

    /// `ReadUsingPointer<T>(data, startoffset)`: `default(T)` unless a whole struct is present.
    pub fn read_using_pointer<T: MarshalStruct>(data: Option<&[u8]>, startoffset: usize) -> T {
        match data {
            Some(d) if d.len() >= startoffset && d.len() - startoffset >= T::SIZE => T::from_bytes(&d[startoffset..]),
            _ => T::default(),
        }
    }

    /// `ByteArrayToStructureGC<T>(bytearray, startoffset)`. C# reads straight out of the pinned
    /// array; past its end it reads whatever follows in memory, which here reads zeros.
    pub fn byte_array_to_structure_gc<T: MarshalStruct>(bytearray: &[u8], startoffset: usize) -> T {
        T::from_bytes(bytearray.get(startoffset..).unwrap_or(&[]))
    }

    /// `ByteArrayToStructureGC(byte[] bytearray, Type typeinfoType, byte startoffset, byte payloadlength)`:
    /// `payloadlength` bytes copied into a cleared struct-sized buffer.
    pub fn byte_array_to_structure_gc_dyn(bytearray: &[u8], typeinfo_type: &MessageType, startoffset: u8, payloadlength: u8) -> Box<dyn MavlinkData> {
        let start = startoffset as usize;
        let end = (start + payloadlength as usize).min(bytearray.len());
        (typeinfo_type.deserialize)(bytearray.get(start..end).unwrap_or(&[]))
    }

    /// `ByteArrayToStructureGCArray`: like the above, but `Array.Copy` throws when the payload is
    /// longer than the struct.
    pub fn byte_array_to_structure_gc_array(
        bytearray: &[u8],
        typeinfo_type: &MessageType,
        startoffset: u8,
        payloadlength: u8,
    ) -> Result<Box<dyn MavlinkData>, Exception> {
        let start = startoffset as usize;
        let n = payloadlength as usize;
        if n > typeinfo_type.size || start + n > bytearray.len() {
            return Err(Exception::Argument(
                "Destination array was not long enough. Check destIndex and length, and the array's lower bounds.".into(),
            ));
        }
        Ok((typeinfo_type.deserialize)(&bytearray[start..start + n]))
    }

    /// `ByteArrayToStructureEndian(bytearray, ref obj, startoffset)`: every scalar field's bytes
    /// reversed in place, arrays left as they are.
    pub fn byte_array_to_structure_endian<T: MarshalStruct>(bytearray: &[u8], obj: &mut T, startoffset: usize) {
        let len = T::SIZE;
        let mut temparray = bytearray.to_vec();
        let mut reversestartoffset = startoffset;
        for f in T::fields() {
            let size = field_scalar_size(f);
            if f.array_length == 0 {
                if reversestartoffset + size <= temparray.len() {
                    temparray[reversestartoffset..reversestartoffset + size].reverse();
                }
                reversestartoffset += size;
            } else {
                reversestartoffset += f.array_length * size;
            }
        }
        let end = (startoffset + len).min(temparray.len());
        *obj = T::from_bytes(temparray.get(startoffset..end).unwrap_or(&[]));
    }

    /// `StructureToByteArray(object obj)`
    pub fn structure_to_byte_array(obj: &dyn MavlinkData) -> Vec<u8> {
        obj.to_bytes()
    }

    /// `StructureToByteArray` for any marshalled struct.
    pub fn structure_to_byte_array_t<T: MarshalStruct>(obj: &T) -> Vec<u8> {
        obj.to_bytes()
    }

    /// `StructureToByteArrayBigEndian(params object[] list)`: scalars written MSB first. The C#
    /// switch has no `SByte` case, so sbyte fields stay 0; any array other than `byte[]` fails
    /// the `(byte[])` cast with InvalidCastException.
    pub fn structure_to_byte_array_big_endian<T: MarshalStruct>(obj: &T) -> Result<Vec<u8>, Exception> {
        let src = obj.to_bytes();
        let mut data = vec![0u8; T::SIZE];
        let mut offset = 0usize;
        for f in T::fields() {
            let size = field_scalar_size(f);
            if f.array_length != 0 {
                if f.cs_type != "byte[]" {
                    return Err(Exception::InvalidCast(format!(
                        "Unable to cast object of type 'System.{}' to type 'System.Byte[]'.",
                        cs_array_type_name(f.cs_type)
                    )));
                }
                let length = f.array_length;
                data[offset..offset + length].copy_from_slice(&src[f.offset..f.offset + length]);
                offset += length;
            } else {
                if f.cs_type != "sbyte" {
                    for i in 0..size {
                        data[offset + i] = src[f.offset + size - 1 - i];
                    }
                }
                offset += size;
            }
        }
        Ok(data)
    }
}

fn cs_array_type_name(cs: &str) -> &'static str {
    match cs {
        "float[]" => "Single[]",
        "double[]" => "Double[]",
        "sbyte[]" => "SByte[]",
        "short[]" => "Int16[]",
        "ushort[]" => "UInt16[]",
        "int[]" => "Int32[]",
        "uint[]" => "UInt32[]",
        "long[]" => "Int64[]",
        "ulong[]" => "UInt64[]",
        _ => "Byte[]",
    }
}

/// `GetMessageInfo(this message_info[] source, uint msgid)`: the first entry with that id;
/// otherwise prints "Unknown Packet" and returns an empty `message_info`. The scan is the C#
/// linear search, memoised per table (the table can be swapped at runtime).
pub fn get_message_info(source: &'static [message_info], msgid: u32) -> &'static message_info {
    use std::collections::HashMap;
    use std::sync::Mutex;
    type Index = Option<(usize, usize, HashMap<u32, usize>)>;
    static INDEX: Mutex<Index> = Mutex::new(None);
    let key = (source.as_ptr() as usize, source.len());
    let mut guard = INDEX.lock().unwrap_or_else(|e| e.into_inner());
    if guard.as_ref().is_none_or(|(p, l, _)| (*p, *l) != key) {
        let mut m = HashMap::new();
        for (i, item) in source.iter().enumerate() {
            m.entry(item.msgid).or_insert(i);
        }
        *guard = Some((key.0, key.1, m));
    }
    if let Some(&i) = guard.as_ref().unwrap().2.get(&msgid) {
        return &source[i];
    }
    drop(guard);
    crate::dotnet::io::console_write_line(&format!("Unknown Packet {msgid}"));
    &message_info::EMPTY
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::extlibs::mavlink::mavlink::*;

    #[test]
    fn trim_keeps_one_byte() {
        let mut v = vec![0u8, 0, 0];
        assert_eq!(MavlinkUtil::trim_payload(&mut v).len(), 1);
        let mut v = vec![1u8, 0, 2, 0, 0];
        assert_eq!(*MavlinkUtil::trim_payload(&mut v), vec![1, 0, 2]);
    }

    #[test]
    fn read_using_pointer_needs_whole_struct() {
        let hb = mavlink_heartbeat_t { custom_mode: 9, ..Default::default() };
        let mut frame = vec![0u8; 6];
        frame.extend(hb.to_bytes());
        let ok: mavlink_heartbeat_t = MavlinkUtil::byte_array_to_structure(&frame, 6);
        assert_eq!(ok.custom_mode, 9);
        let short: mavlink_heartbeat_t = MavlinkUtil::byte_array_to_structure(&frame[..10], 6);
        assert_eq!(short.custom_mode, 0, "ReadUsingPointer returns default(T) for a short buffer");
    }

    #[test]
    fn big_endian_round_trip() {
        let hb = mavlink_heartbeat_t { custom_mode: 0x01020304, r#type: 2, ..Default::default() };
        let be = MavlinkUtil::structure_to_byte_array_big_endian(&hb).unwrap();
        assert_eq!(&be[..4], &[1, 2, 3, 4]);
        let back: mavlink_heartbeat_t = MavlinkUtil::byte_array_to_structure_big_endian(&be, 0);
        assert_eq!(back, hb);
        // no SByte case in the C# switch: sbyte fields are written as 0
        let t = mavlink_named_value_int_t { value: 1, ..Default::default() };
        assert!(MavlinkUtil::structure_to_byte_array_big_endian(&t).is_ok());
        // a short[] field fails the (byte[]) cast
        let s = mavlink_servo_output_raw_t::default();
        assert!(MavlinkUtil::structure_to_byte_array_big_endian(&s).is_ok());
        let r = mavlink_rc_channels_override_t::default();
        assert!(MavlinkUtil::structure_to_byte_array_big_endian(&r).is_ok());
        let d = mavlink_debug_vect_t::default();
        assert!(MavlinkUtil::structure_to_byte_array_big_endian(&d).is_ok());
        let a = mavlink_attitude_quaternion_cov_t::default();
        assert!(matches!(MavlinkUtil::structure_to_byte_array_big_endian(&a), Err(Exception::InvalidCast(_))));
    }

    #[test]
    fn unknown_message_info_is_empty() {
        let i = get_message_info(MAVLINK_MESSAGE_INFOS.get(), 0xFFFFFF);
        assert!(i.r#type.is_none());
        assert_eq!(i.name, "");
    }
}

//! Port of `ExtLibs/Mavlink/MavlinkHeader.cs`

use super::mavlink::*;
use crate::dotnet::Exception;

pub const MAVLINK_SUPPORTED_IFLAGS: u8 = MAVLINK_IFLAG_SIGNED | MAVLINK_IFLAG_SYSID32 | MAVLINK_IFLAG_TARGET32;

/// `GetHeaderLength(flags)`
pub fn get_header_length(flags: u8) -> i32 {
    let wide = (flags & MAVLINK_IFLAG_SYSID32) != 0;
    MAVLINK_NUM_HEADER_BYTES as i32 + if wide { 3 } else { 0 } + if (flags & MAVLINK_IFLAG_TARGET32) != 0 { 4 } else { 0 }
}

/// `internal static uint ReadSystemId(byte[] buffer, ref int offset, bool wide)`
pub(crate) fn read_system_id(buffer: &[u8], offset: &mut usize, wide: bool) -> u32 {
    let mut value = buffer[*offset] as u32;
    *offset += 1;
    if wide {
        value |= (buffer[*offset] as u32) << 8;
        *offset += 1;
        value |= (buffer[*offset] as u32) << 16;
        *offset += 1;
        value |= (buffer[*offset] as u32) << 24;
        *offset += 1;
    }
    value
}

fn write_system_id(buffer: &mut [u8], offset: &mut usize, value: u32, wide: bool) {
    buffer[*offset] = value as u8;
    *offset += 1;
    if wide {
        buffer[*offset] = (value >> 8) as u8;
        *offset += 1;
        buffer[*offset] = (value >> 16) as u8;
        *offset += 1;
        buffer[*offset] = (value >> 24) as u8;
        *offset += 1;
    }
}

/// `GetTargetSystemField(type)`: `target_system`, or `target` on MANUAL_CONTROL.
pub fn get_target_system_field(r#type: &MessageType) -> Option<&'static FieldInfo> {
    r#type.get_field("target_system").or_else(|| {
        if r#type.name == "mavlink_manual_control_t" {
            r#type.get_field("target")
        } else {
            None
        }
    })
}

/// `SetPayloadTarget`: generated payload structs keep byte target fields, so write the full
/// target here before trimming the payload, keeping small targets and broadcast on the wire.
pub fn set_payload_target(message: &dyn MavlinkData, payload: &mut [u8], system: u32, component: u8) -> bool {
    let r#type = message.message_type();
    let Some(field) = get_target_system_field(r#type) else { return false };
    payload[field.offset] = if system > 255 { 255 } else { system as u8 };
    if let Some(component_field) = r#type.get_field("target_component") {
        payload[component_field.offset] = component;
    }
    true
}

/// .NET `ArgumentOutOfRangeException(paramName)` message.
pub(crate) fn arg_out_of_range(param: &str) -> Exception {
    Exception::ArgumentOutOfRange(format!("Specified argument was out of the range of valid values.\r\nParameter name: {param}"))
}

/// `WriteHeader`: source and target widths are independent; TARGET32 always adds four bytes.
#[allow(clippy::too_many_arguments)]
pub fn write_header(
    packet: &mut [u8],
    payload_length: u8,
    flags: u8,
    sequence: u8,
    system: u32,
    component: u8,
    message: u32,
    target_system: u32,
) -> Result<i32, Exception> {
    if (flags & !MAVLINK_SUPPORTED_IFLAGS) != 0 {
        return Err(arg_out_of_range("flags"));
    }
    if (flags & MAVLINK_IFLAG_SYSID32) == 0 && system > 255 {
        return Err(Exception::ArgumentOutOfRange("32-bit system IDs require SYSID32\r\nParameter name: flags".into()));
    }
    packet[0] = MAVLINK_STX;
    packet[1] = payload_length;
    packet[2] = flags;
    packet[3] = 0;
    packet[4] = sequence;
    let wide = (flags & MAVLINK_IFLAG_SYSID32) != 0;
    let mut offset = 5usize;
    write_system_id(packet, &mut offset, system, wide);
    packet[offset] = component;
    offset += 1;
    packet[offset] = message as u8;
    offset += 1;
    packet[offset] = (message >> 8) as u8;
    offset += 1;
    packet[offset] = (message >> 16) as u8;
    offset += 1;
    if (flags & MAVLINK_IFLAG_TARGET32) != 0 {
        write_system_id(packet, &mut offset, target_system, true);
    }
    Ok(offset as i32)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn header_lengths() {
        assert_eq!(get_header_length(0), 10);
        assert_eq!(get_header_length(MAVLINK_IFLAG_SYSID32), 13);
        assert_eq!(get_header_length(MAVLINK_IFLAG_SYSID32 | MAVLINK_IFLAG_TARGET32), 17);
        assert_eq!(get_header_length(MAVLINK_IFLAG_TARGET32), 14);
    }

    #[test]
    fn write_header_wide_ids() {
        let mut p = [0u8; 20];
        let n = write_header(&mut p, 9, MAVLINK_IFLAG_SYSID32 | MAVLINK_IFLAG_TARGET32, 7, 0x01020304, 1, 0x0A0B0C, 300).unwrap();
        assert_eq!(n, 17);
        assert_eq!(&p[..17], &[253, 9, 6, 0, 7, 4, 3, 2, 1, 1, 0x0C, 0x0B, 0x0A, 44, 1, 0, 0]);
        assert!(write_header(&mut p, 0, 0, 0, 256, 0, 0, 0).is_err());
        assert!(write_header(&mut p, 0, 0x80, 0, 1, 0, 0, 0).is_err());
    }
}

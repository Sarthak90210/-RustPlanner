//! Port of `ExtLibs/Utilities/Unicore.cs`
//!
//! Configures a Unicore receiver as an RTK base. Its `SendAck` only writes: it builds a
//! `StreamReader` it never reads. The C# methods are `async Task`s; here they run
//! synchronously.

use crate::dotnet::{ascii_get_bytes, Exception};
use crate::extlibs::interfaces::icommsserial::ICommsSerial;

/// `public class FailedAckException : Exception` (declared, never thrown)
pub fn failed_ack_exception() -> Exception {
    Exception::Custom(
        "MissionPlanner.Utilities.Unicore+FailedAckException",
        "Exception of type 'MissionPlanner.Utilities.Unicore+FailedAckException' was thrown.".into(),
    )
}

/// `public class Unicore`
pub struct Unicore;

impl Unicore {
    /// `ConfigureBaseReceiver(ICommsSerial receiverPort)`
    #[allow(non_snake_case)]
    pub fn ConfigureBaseReceiver(receiver_port: &mut dyn ICommsSerial) -> Result<(), Exception> {
        receiver_port.base_stream()?.flush()?;

        receiver_port.set_baud_rate(115200);
        receiver_port.set_read_timeout(200);
        receiver_port.set_write_timeout(200);

        for cmd in [
            "GPGGA COM3 1\r\n",
            "mode base time 60 2 2.5\r\n",
            "rtcm1006 com3 1\r\n",
            "rtcm1033 com3 1\r\n",
            "rtcm1074 com3 1\r\n",
            "rtcm1124 com3 1\r\n",
            "rtcm1084 com3 1\r\n",
            "rtcm1094 com3 1\r\n",
            "saveconfig\r\n",
        ] {
            Self::send_ack(receiver_port, cmd)?;
        }
        Ok(())
    }

    /// `SendAck(receiverPort, command)`: flush, then write the command bytes.
    fn send_ack(receiver_port: &mut dyn ICommsSerial, command: &str) -> Result<(), Exception> {
        let mut bs = receiver_port.base_stream()?;
        bs.flush()?;
        let bytes = ascii_get_bytes(command);
        let n = command.encode_utf16().count().min(bytes.len());
        bs.write(&bytes[..n])
    }
}

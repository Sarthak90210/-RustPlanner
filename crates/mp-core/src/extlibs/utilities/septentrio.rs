//! Port of `ExtLibs/Utilities/Septentrio.cs`
//!
//! Configures a Septentrio receiver as an RTK base. The C# methods are `async Task`s; here
//! they run synchronously on the caller's thread.

use std::time::Instant;

use crate::dotnet::io::StreamReaderAscii;
use crate::dotnet::{ascii_get_bytes, format_fixed_float, Exception, INVARIANT_CULTURE};
use crate::extlibs::interfaces::icommsserial::ICommsSerial;

/// `public class FailedAckException : Exception`
pub fn failed_ack_exception() -> Exception {
    Exception::Custom(
        "MissionPlanner.Utilities.Septentrio+FailedAckException",
        "Exception of type 'MissionPlanner.Utilities.Septentrio+FailedAckException' was thrown.".into(),
    )
}

crate::cs_enum! {
    /// `public enum RTCMLevel`: which MSM level to output.
    pub RTCMLevel: i32 {
        /// MSM3 messages.
        Lite = 3,
        /// MSM4 messages.
        Basic = 4,
        /// MSM7 messages.
        Full = 7,
    }
}

crate::cs_enum! {
    /// `[Flags] public enum RTCMSignals`: constellations to output RTCM messages for.
    pub RTCMSignals: i32 {
        None = 0b_0000_0000,
        Gps = 0b_0000_0001,
        Glonass = 0b_0000_0010,
        Beidou = 0b_0000_0100,
        Galileo = 0b_0000_1000,
    }
}

impl std::ops::BitOr for RTCMSignals {
    type Output = RTCMSignals;
    fn bitor(self, o: RTCMSignals) -> RTCMSignals {
        RTCMSignals(self.0 | o.0)
    }
}

impl std::ops::BitAnd for RTCMSignals {
    type Output = RTCMSignals;
    fn bitand(self, o: RTCMSignals) -> RTCMSignals {
        RTCMSignals(self.0 & o.0)
    }
}

/// `public class Septentrio`
pub struct Septentrio;

impl Septentrio {
    /// The maximum time to wait for the receiver to acknowledge a message (ms).
    const ACK_TIMEOUT: u128 = 1000;

    /// The default baud rate for Septentrio receivers.
    pub const DEFAULT_BAUDRATE: i32 = 115200;

    /// `ConfigureBaseReceiver(ICommsSerial receiverPort)`: set up the receiver as a base.
    /// Throws FailedAckException or the port's I/O exception.
    #[allow(non_snake_case)]
    pub fn ConfigureBaseReceiver(receiver_port: &mut dyn ICommsSerial) -> Result<(), Exception> {
        receiver_port.base_stream()?.flush()?;

        receiver_port.set_baud_rate(115200);
        receiver_port.set_read_timeout(200);
        receiver_port.set_write_timeout(200);

        Self::configure_baud(receiver_port)?;

        Self::send_ack(receiver_port, "setPVTMode,Static,All,Auto\n")?;
        Self::send_ack(receiver_port, "setDataInOut,USB1+USB2+COM1+COM2+COM3,Auto,RTCMv3\n")
    }

    /// `SetBasePosition(receiverPort, latitude, longitude, altitude)`: a fixed base position.
    #[allow(non_snake_case)]
    pub fn SetBasePosition(receiver_port: &mut dyn ICommsSerial, latitude: f32, longitude: f32, altitude: f32) -> Result<(), Exception> {
        receiver_port.base_stream()?.flush()?;

        let inv = &INVARIANT_CULTURE;
        let cmd = "setStaticPosGeodetic,Geodetic1,".to_string()
            + &format_fixed_float(latitude, 1, 9, inv)
            + ","
            + &format_fixed_float(longitude, 1, 9, inv)
            + ","
            + &format_fixed_float(altitude, 1, 4, inv)
            + ",WGS84"
            + "\n";
        Self::send_ack(receiver_port, &cmd)?;
        Self::send_ack(receiver_port, "setPVTMode,Static,,Geodetic1\n")
    }

    /// `SetAutoBasePosition(receiverPort)`: let the receiver work out its position.
    #[allow(non_snake_case)]
    pub fn SetAutoBasePosition(receiver_port: &mut dyn ICommsSerial) -> Result<(), Exception> {
        receiver_port.base_stream()?.flush()?;

        Self::send_ack(receiver_port, "setPVTMode,Static,,auto\n")
    }

    /// `ConfigureBaud(receiverPort)`: tries each likely baud rate until the receiver accepts
    /// the COM settings command, then switches the port to the default rate.
    fn configure_baud(receiver_port: &mut dyn ICommsSerial) -> Result<(), Exception> {
        let mut receiver_acknowledged = false;

        // All the baud rates we expect the receiver could be running at
        let bauds = [receiver_port.baud_rate(), 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800];

        for baud in bauds {
            receiver_port.set_baud_rate(baud);

            // Try to set the port settings on a best effort basis
            let cmd = format!("setCOMSettings,COM1+COM2+COM3,baud{},bits8,No,bit1,none\n", Self::DEFAULT_BAUDRATE);
            if Self::send_ack(receiver_port, &cmd).is_ok() {
                receiver_acknowledged = true;
                break;
            }
        }

        if !receiver_acknowledged {
            return Err(failed_ack_exception());
        }

        receiver_port.set_baud_rate(Self::DEFAULT_BAUDRATE);
        Ok(())
    }

    /// `SetEnabledRTCM(receiverPort, level, signals)`
    #[allow(non_snake_case)]
    pub fn SetEnabledRTCM(receiver_port: &mut dyn ICommsSerial, level: RTCMLevel, signals: RTCMSignals) -> Result<(), Exception> {
        let mut messages = String::from("RTCM1006+RTCM1033+RTCM1230");

        let message_level = match level {
            RTCMLevel::Lite => 3,
            RTCMLevel::Basic => 4,
            _ => 7,
        };

        if (signals & RTCMSignals::Gps) == RTCMSignals::Gps {
            messages += &format!("+RTCM107{message_level}");
        }
        if (signals & RTCMSignals::Glonass) == RTCMSignals::Glonass {
            messages += &format!("+RTCM108{message_level}");
        }
        if (signals & RTCMSignals::Galileo) == RTCMSignals::Galileo {
            messages += &format!("+RTCM109{message_level}");
        }
        if (signals & RTCMSignals::Beidou) == RTCMSignals::Beidou {
            messages += &format!("+RTCM112{message_level}");
        }

        Self::send_ack(receiver_port, &format!("setRTCMv3Output,COM1+COM2+COM3+USB1+USB2,{messages}\n"))
    }

    /// `SetRTCMInterval(receiverPort, interval)`
    #[allow(non_snake_case)]
    pub fn SetRTCMInterval(receiver_port: &mut dyn ICommsSerial, interval: f32) -> Result<(), Exception> {
        Self::send_ack(
            receiver_port,
            &("setRTCMv3Interval,MSM3+MSM4+MSM7+RTCM1005|6+RTCM1033+RTCM1230,".to_string()
                + &format_fixed_float(interval, 1, 1, &INVARIANT_CULTURE)
                + "\n"),
        )
    }

    /// `SendAck(receiverPort, command)`: writes the command, then reads lines until one
    /// contains it (without its final character). The timeout is only checked after a line
    /// arrives; the end of the stream or the timeout throws FailedAckException.
    fn send_ack(receiver_port: &mut dyn ICommsSerial, command: &str) -> Result<(), Exception> {
        let mut bs = receiver_port.base_stream()?;

        bs.flush()?;
        let bytes = ascii_get_bytes(command);
        let n = command.encode_utf16().count().min(bytes.len());
        bs.write(&bytes[..n])?;

        let sw = Instant::now();
        // command.Remove(command.Length - 1)
        let mut chars: Vec<char> = command.chars().collect();
        chars.pop();
        let needle: String = chars.into_iter().collect();

        let mut reader = StreamReaderAscii::new(&mut *bs);
        while let Some(line) = reader.read_line()? {
            if line.contains(&needle) {
                return Ok(());
            }

            // If the receiver never properly acknowledges the command, we need to manually time out
            if sw.elapsed().as_millis() > Self::ACK_TIMEOUT {
                log::error!("Waiting for command acknowledgement timed out");
                break;
            }
        }

        Err(failed_ack_exception())
    }
}

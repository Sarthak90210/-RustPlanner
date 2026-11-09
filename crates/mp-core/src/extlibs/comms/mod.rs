//! `ExtLibs/Comms` (`MissionPlanner.Comms.csproj`): the transports behind `ICommsSerial`.
//!
//! `System.IO.Ports/*` in the C# project is Mono's SerialPort source; the Rust build uses the
//! `serialport` crate in its place, through `crate::dotnet::ports` (see `port/MANIFEST.md`).

pub mod circularbuffer;
pub mod commsbase;
pub mod commsfile;
pub mod commsinjection;
pub mod commsntrip;
pub mod commsserialpipe;
pub mod commsserialport;
pub mod commsstream;
pub mod commstcpserial;
pub mod commsudpserial;
pub mod commsudpserialconnect;
pub mod commswebsocket;
pub mod serialtest;
pub mod udpstream;

pub use self::circularbuffer::CircularBuffer;
pub use self::commsbase::{inputboxreturn, CommsBase};
pub use self::commsfile::CommsFile;
pub use self::commsinjection::CommsInjection;
pub use self::commsntrip::CommsNTRIP;
pub use self::commsserialpipe::CommsSerialPipe;
pub use self::commsserialport::{SerialPort, WinSerialPort};
pub use self::commsstream::CommsStream;
pub use self::commstcpserial::TcpSerial;
pub use self::commsudpserial::UdpSerial;
pub use self::commsudpserialconnect::UdpSerialConnect;
pub use self::commswebsocket::WebSocket;
pub use self::udpstream::UDPStream;

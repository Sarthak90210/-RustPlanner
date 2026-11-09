//! Port of `ExtLibs/ArduPilot/Serial.cs`
//!
//! Arduino-style `Serial.print` / `println` to the console. A `byte[]` argument is decoded as
//! UTF-8; anything else is written with its `ToString()`.


/// `object v` as the two C# overload paths see it.
pub enum SerialArg<'a> {
    Bytes(&'a [u8]),
    Text(String),
}

/// `MissionPlanner.Serial`
pub struct Serial;

impl Serial {
    fn text(v: SerialArg<'_>) -> String {
        match v {
            SerialArg::Bytes(b) => String::from_utf8_lossy(b).into_owned(),
            SerialArg::Text(s) => s,
        }
    }

    /// `print(object v)`: `Console.Write`
    pub fn print(v: SerialArg<'_>) {
        crate::dotnet::io::console_write(&Self::text(v));
    }

    /// `println(object v)`: `Console.WriteLine`
    pub fn println(v: SerialArg<'_>) {
        crate::dotnet::io::console_write_line(&Self::text(v));
    }
}

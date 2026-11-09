//! Port of `ExtLibs/GMap.NET.Core/GMap.NET.Internals/Stuff.cs`
//!
//! etc functions. `EnumToString` reads a member's `[Description]` through the
//! `DescribedEnum` trait (reflection stand-in). The BouncyCastle SHA-1 and
//! `DESede/ECB/PKCS7Padding` cipher are the `dotnet::sha1` and `dotnet::tripledes` shims.
//! The PocketPC `Monitor` shim is not compiled for the desktop build and is not carried.

#![allow(non_snake_case, non_upper_case_globals)]

use std::sync::{LazyLock, Mutex};

use base64::Engine;

use crate::dotnet::random::Random;
use crate::dotnet::{sha1, tripledes, Exception, MemoryStream, SeekOrigin, Stream};
use crate::extlibs::gmap_net_core::gmap_net::languagetype::LanguageType;

/// An enum whose fields `Stuff.EnumToString` can reflect on.
pub trait DescribedEnum {
    /// `value.ToString()` names a field: the member name, `None` when the value is not a member
    fn field_name(&self) -> Option<&'static str>;
    /// the field's `[Description]`, if any
    fn description_attribute(&self) -> Option<&'static str>;
}

impl DescribedEnum for LanguageType {
    fn field_name(&self) -> Option<&'static str> {
        self.get_name()
    }
    fn description_attribute(&self) -> Option<&'static str> {
        self.description()
    }
}

/// `public static string EnumToString(Enum value)`: the `[Description]` text, else the
/// member name. A value that is no member has no field, so C# throws
/// `NullReferenceException`.
pub fn EnumToString<E: DescribedEnum>(value: &E) -> Result<String, Exception> {
    match value.field_name() {
        None => Err(Exception::NullReference("Object reference not set to an instance of an object.".into())),
        Some(name) => Ok(match value.description_attribute() {
            Some(d) => d.to_string(),
            None => name.to_string(),
        }),
    }
}

/// `SetCursorPos` (user32.dll)
#[cfg(windows)]
pub fn SetCursorPos(X: i32, Y: i32) -> Result<bool, Exception> {
    #[link(name = "user32")]
    extern "system" {
        #[link_name = "SetCursorPos"]
        fn set_cursor_pos(x: i32, y: i32) -> i32;
    }
    Ok(unsafe { set_cursor_pos(X, Y) } != 0)
}

/// `SetCursorPos` (user32.dll): the P/Invoke fails to load the library off Windows.
#[cfg(not(windows))]
pub fn SetCursorPos(_X: i32, _Y: i32) -> Result<bool, Exception> {
    Err(Exception::Custom("System.DllNotFoundException", "Unable to load DLL 'user32.dll': The specified module could not be found.".into()))
}

/// `public static readonly Random random = new System.Random()`
pub static random: LazyLock<Mutex<Random>> = LazyLock::new(|| Mutex::new(Random::new_default()));

/// `Shuffle<T>(List<T> deck)`
pub fn Shuffle<T>(deck: &mut [T]) {
    let n = deck.len() as i32;
    let mut rng = random.lock().unwrap_or_else(|e| e.into_inner());
    for i in 0..n {
        // N - i > 0, so Next cannot throw
        let r = i + rng.next_max(n - i).unwrap_or(0);
        deck.swap(r as usize, i as usize);
    }
}

/// `CopyStream(Stream inputStream, bool SeekOriginBegin)`: reads the stream to its end in
/// 32 KiB blocks into a new `MemoryStream`, optionally rewinds the input, and returns the copy
/// positioned at 0.
pub fn CopyStream(inputStream: &mut dyn Stream, SeekOriginBegin: bool) -> Result<MemoryStream, Exception> {
    const READ_SIZE: usize = 32 * 1024;
    let mut buffer = vec![0u8; READ_SIZE];
    let mut ms = MemoryStream::default();
    loop {
        let count = inputStream.read(&mut buffer)?;
        if count == 0 {
            break;
        }
        ms.write_from(&buffer, 0, count);
    }
    if SeekOriginBegin {
        inputStream.seek_to(0, SeekOrigin::Begin)?;
    }
    ms.seek(0, SeekOrigin::Begin)?;
    Ok(ms)
}

/// `Environment.OSVersion` as (is Win32NT, Major, Minor). Mission Planner's app.manifest
/// declares Windows 10 support, so .NET reports the real version (10.0 on Windows 10/11).
fn os_version() -> (bool, u32, u32) {
    #[cfg(windows)]
    {
        #[repr(C)]
        struct OsVersionInfoW {
            size: u32,
            major: u32,
            minor: u32,
            build: u32,
            platform: u32,
            csd: [u16; 128],
        }
        #[link(name = "ntdll")]
        extern "system" {
            fn RtlGetVersion(info: *mut OsVersionInfoW) -> i32;
        }
        let mut v = OsVersionInfoW { size: std::mem::size_of::<OsVersionInfoW>() as u32, major: 0, minor: 0, build: 0, platform: 0, csd: [0; 128] };
        unsafe { RtlGetVersion(&mut v) };
        (true, v.major, v.minor)
    }
    #[cfg(not(windows))]
    {
        (false, 0, 0)
    }
}

/// `IsRunningOnVistaOrLater()`
pub fn IsRunningOnVistaOrLater() -> bool {
    let (nt, major, minor) = os_version();
    let _ = minor;
    // vs.Minor >= 0 always holds
    nt && major >= 6
}

/// `IsRunningOnWin7orLater()`: `Major >= 6 && Minor > 0`, so Windows 10 (10.0) is false.
pub fn IsRunningOnWin7orLater() -> bool {
    let (nt, major, minor) = os_version();
    nt && major >= 6 && minor > 0
}

/// `Path.GetInvalidFileNameChars()`: .NET Framework on Windows, Mono's list elsewhere.
fn invalid_file_name_chars() -> Vec<char> {
    if cfg!(windows) {
        let mut v = vec!['"', '<', '>', '|'];
        v.extend((0u8..32).map(char::from));
        v.extend([':', '*', '?', '\\', '/']);
        v
    } else {
        vec!['\0', '/']
    }
}

/// `RemoveInvalidPathSymbols(ref string url)`
pub fn RemoveInvalidPathSymbols(url: &mut String) {
    for c in invalid_file_name_chars() {
        *url = url.replace(c, "_");
    }
}

// -- encryption --

fn tdes_key(passphrase: &str) -> Vec<u8> {
    // a 32-byte buffer receives the 20-byte digest, then is resized to 16
    let mut key = vec![0u8; 32];
    key[..20].copy_from_slice(&sha1::sha1(passphrase.as_bytes()));
    key.truncate(16);
    key
}

fn EncryptStringWith(Message: &str, Passphrase: &str) -> Result<String, Exception> {
    let results = tripledes::encrypt_ecb_pkcs7(&tdes_key(Passphrase), Message.as_bytes())?;
    Ok(base64::engine::general_purpose::STANDARD.encode(results))
}

fn DecryptStringWith(Message: &str, Passphrase: &str) -> Result<String, Exception> {
    let data = base64::engine::general_purpose::STANDARD
        .decode(Message)
        .map_err(|_| Exception::Format("The input is not a valid Base-64 string as it contains a non-base 64 character, more than two padding characters, or an illegal character among the padding characters. ".into()))?;
    let results = tripledes::decrypt_ecb_pkcs7(&tdes_key(Passphrase), &data)?;
    Ok(String::from_utf8_lossy(&results).into_owned())
}

/// `public static string EncryptString(string Message)`
pub fn EncryptString(Message: &str) -> Result<String, Exception> {
    EncryptStringWith(Message, MANIFESTO)
}

/// `public static string GString(string Message)`
pub fn GString(Message: &str) -> Result<String, Exception> {
    DecryptStringWith(Message, MANIFESTO)
}

const MANIFESTO: &str = "GMap.NET is great and Powerful, Free, cross platform, open source .NET control.";

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn enum_to_string_uses_description() {
        assert_eq!(EnumToString(&LanguageType::English).unwrap(), "en");
        assert_eq!(EnumToString(&LanguageType::PortugueseBrazil).unwrap(), "pt-BR");
        assert!(matches!(EnumToString(&LanguageType(500)), Err(Exception::NullReference(_))));
    }

    #[test]
    fn gstring_decrypts_provider_servers() {
        assert_eq!(GString("9gERyvblybF8iMuCt/LD6w==").unwrap(), "googleapis.com");
        assert_eq!(GString("gosr2U13BoS+bXaIxt6XWg==").unwrap(), "google.com");
        assert_eq!(GString("MECxW6okUK3Ir7a9ue/vIA==").unwrap(), "maps.yandex.net");
        let e = EncryptString("hello").unwrap();
        assert_eq!(GString(&e).unwrap(), "hello");
    }

    #[test]
    fn copy_stream_and_paths() {
        let mut src = MemoryStream::new((0..70000u32).map(|i| i as u8).collect());
        let c = CopyStream(&mut src, true).unwrap();
        assert_eq!(c.len(), 70000);
        assert_eq!(c.pos, 0);
        assert_eq!(src.pos, 0);
        let mut u = String::from("a/b");
        RemoveInvalidPathSymbols(&mut u);
        assert_eq!(u, "a_b");
        let mut v = vec![1, 2, 3, 4, 5];
        Shuffle(&mut v);
        v.sort();
        assert_eq!(v, vec![1, 2, 3, 4, 5]);
    }
}

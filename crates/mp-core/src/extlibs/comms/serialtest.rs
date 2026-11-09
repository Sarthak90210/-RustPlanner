//! Port of `ExtLibs/Comms/SerialTest.cs`
//!
//! A Windows-only diagnostic: opens `\\.\<port>` and prints the handle's `GetFileType`.

/// `SerialTest`
pub struct SerialTest;

/// `private enum FileType : uint`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(clippy::upper_case_acronyms)]
pub enum FileType {
    UNKNOWN = 0x0000,
    DISK = 0x0001,
    CHAR = 0x0002,
    PIPE = 0x0003,
    REMOTE = 0x8000,
}

impl FileType {
    fn from_u32(v: u32) -> Option<FileType> {
        Some(match v {
            0x0000 => FileType::UNKNOWN,
            0x0001 => FileType::DISK,
            0x0002 => FileType::CHAR,
            0x0003 => FileType::PIPE,
            0x8000 => FileType::REMOTE,
            _ => return None,
        })
    }
}

impl SerialTest {
    /// `public static void Mainold(string[] args)`
    #[cfg(windows)]
    pub fn mainold(args: &[String]) {
        use windows::core::HSTRING;
        use windows::Win32::Foundation::{CloseHandle, GENERIC_ACCESS_RIGHTS};
        use windows::Win32::Storage::FileSystem::{CreateFileW, GetFileType, FILE_FLAGS_AND_ATTRIBUTES, FILE_SHARE_MODE, OPEN_EXISTING};

        let port_name = format!("\\\\.\\{}", args[0]);
        // CreateFile(portName, 0, 0, IntPtr.Zero, 3 /* OPEN_EXISTING */, 0x80 /* NORMAL */, IntPtr.Zero)
        let handle = unsafe {
            CreateFileW(
                &HSTRING::from(port_name.as_str()),
                GENERIC_ACCESS_RIGHTS(0).0,
                FILE_SHARE_MODE(0),
                None,
                OPEN_EXISTING,
                FILE_FLAGS_AND_ATTRIBUTES(0x80),
                None,
            )
        };
        let handle = match handle {
            Ok(h) => h,
            Err(e) => {
                crate::dotnet::io::console_write_line(&format!("Could not open {}: {}", port_name, e.message()));
                read_key();
                return;
            }
        };

        let raw = unsafe { GetFileType(handle) }.0;
        let shown = FileType::from_u32(raw).map(|t| format!("{t:?}")).unwrap_or_else(|| raw.to_string());
        crate::dotnet::io::console_write_line(&format!("File {} reports its type as: {}", port_name, shown));
        unsafe {
            let _ = CloseHandle(handle);
        }

        read_key();
    }
}

/// `Console.ReadKey()`: waits for input on the console.
#[cfg(windows)]
fn read_key() {
    let mut s = String::new();
    let _ = std::io::stdin().read_line(&mut s);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn file_type_names() {
        assert_eq!(FileType::from_u32(2), Some(FileType::CHAR));
        assert_eq!(FileType::from_u32(7), None);
    }
}

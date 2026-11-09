//! `System.Environment` special folders and `NetworkInterface.GetAllNetworkInterfaces()` MAC
//! addresses, as .NET Framework (Windows) and Mono (Linux/macOS) report them.

/// `Environment.SpecialFolder` values the ported code uses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpecialFolder {
    MyDocuments,
    LocalApplicationData,
    CommonApplicationData,
    ApplicationData,
}

#[cfg(windows)]
pub fn get_folder_path(folder: SpecialFolder) -> String {
    #[link(name = "shell32")]
    #[allow(non_snake_case)]
    extern "system" {
        fn SHGetFolderPathW(hwnd: *mut std::ffi::c_void, csidl: i32, token: *mut std::ffi::c_void, flags: u32, path: *mut u16) -> i32;
    }
    // .NET Framework calls SHGetFolderPath with the CSIDL of the folder.
    let csidl = match folder {
        SpecialFolder::MyDocuments => 0x0005,
        SpecialFolder::LocalApplicationData => 0x001c,
        SpecialFolder::CommonApplicationData => 0x0023,
        SpecialFolder::ApplicationData => 0x001a,
    };
    let mut buf = [0u16; 260];
    let hr = unsafe { SHGetFolderPathW(std::ptr::null_mut(), csidl, std::ptr::null_mut(), 0, buf.as_mut_ptr()) };
    if hr < 0 {
        return String::new();
    }
    let n = buf.iter().position(|&c| c == 0).unwrap_or(buf.len());
    String::from_utf16_lossy(&buf[..n])
}

/// Mono's mapping on Unix: Personal/MyDocuments is `$HOME`; the application data folders
/// follow the XDG base directories.
#[cfg(not(windows))]
pub fn get_folder_path(folder: SpecialFolder) -> String {
    let home = std::env::var("HOME").unwrap_or_default();
    let xdg = |var: &str, default: &str| -> String {
        match std::env::var(var) {
            Ok(v) if !v.is_empty() => v,
            _ => format!("{home}/{default}"),
        }
    };
    match folder {
        SpecialFolder::MyDocuments => home,
        SpecialFolder::LocalApplicationData => xdg("XDG_DATA_HOME", ".local/share"),
        SpecialFolder::ApplicationData => xdg("XDG_CONFIG_HOME", ".config"),
        SpecialFolder::CommonApplicationData => "/usr/share".into(),
    }
}

/// `Path.DirectorySeparatorChar`
pub const DIRECTORY_SEPARATOR_CHAR: char = if cfg!(windows) { '\\' } else { '/' };

/// `NetworkInterface.GetAllNetworkInterfaces().FirstOrDefault()?.GetPhysicalAddress().GetAddressBytes()`
#[cfg(windows)]
pub fn first_interface_physical_address() -> Option<Vec<u8>> {
    use std::ffi::c_void;
    #[repr(C)]
    #[allow(non_snake_case, dead_code)]
    struct IpAdapterAddressesPrefix {
        Length: u32,
        IfIndex: u32,
        Next: *mut IpAdapterAddressesPrefix,
        AdapterName: *mut u8,
        FirstUnicastAddress: *mut c_void,
        FirstAnycastAddress: *mut c_void,
        FirstMulticastAddress: *mut c_void,
        FirstDnsServerAddress: *mut c_void,
        DnsSuffix: *mut u16,
        Description: *mut u16,
        FriendlyName: *mut u16,
        PhysicalAddress: [u8; 8],
        PhysicalAddressLength: u32,
    }
    #[link(name = "iphlpapi")]
    #[allow(non_snake_case)]
    extern "system" {
        fn GetAdaptersAddresses(family: u32, flags: u32, reserved: *mut c_void, addresses: *mut c_void, size: *mut u32) -> u32;
    }
    // .NET: AF_UNSPEC with GAA_FLAG_INCLUDE_WINS_INFO | GAA_FLAG_INCLUDE_GATEWAYS
    let flags = 0x0040 | 0x0080;
    let mut size: u32 = 15000;
    for _ in 0..3 {
        let mut buf = vec![0u64; (size as usize).div_ceil(8)];
        let r = unsafe { GetAdaptersAddresses(0, flags, std::ptr::null_mut(), buf.as_mut_ptr() as *mut c_void, &mut size) };
        if r == 111 {
            // ERROR_BUFFER_OVERFLOW: size now holds the required length
            continue;
        }
        if r != 0 {
            return None;
        }
        let first = buf.as_ptr() as *const IpAdapterAddressesPrefix;
        let a = unsafe { &*first };
        let n = (a.PhysicalAddressLength as usize).min(8);
        return Some(a.PhysicalAddress[..n].to_vec());
    }
    None
}

/// Mono on Linux lists interfaces by index (so `lo` first, whose address is all zeroes);
/// elsewhere the first interface has no hardware address.
#[cfg(not(windows))]
pub fn first_interface_physical_address() -> Option<Vec<u8>> {
    let dir = std::fs::read_dir("/sys/class/net").ok()?;
    let mut ifs: Vec<(i64, String)> = dir
        .filter_map(|e| e.ok())
        .filter_map(|e| {
            let name = e.file_name().to_string_lossy().to_string();
            let idx = std::fs::read_to_string(e.path().join("ifindex")).ok()?.trim().parse::<i64>().ok()?;
            Some((idx, name))
        })
        .collect();
    ifs.sort();
    let (_, name) = ifs.first()?;
    let addr = std::fs::read_to_string(format!("/sys/class/net/{name}/address")).ok()?;
    let bytes: Option<Vec<u8>> = addr.trim().split(':').filter(|s| !s.is_empty()).map(|h| u8::from_str_radix(h, 16).ok()).collect();
    bytes
}

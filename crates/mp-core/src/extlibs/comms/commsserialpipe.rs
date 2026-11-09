//! Port of `ExtLibs/Comms/CommsSerialPipe.cs`
//!
//! A raw kernel32 COM port: a reader thread copies `ReadFile` output into a `MemoryStream`
//! that `Read` serves from. The P/Invoke structs keep the C# declarations, including their
//! quirks: `DCB` lists its fields alphabetically, so `GetCommState`/`SetCommState` see the
//! Win32 bytes through that layout, and `SetDcbFlag` receives the DCB by value, so its flag
//! changes never reach the struct passed to `SetCommState`. Off Windows the kernel32 calls do
//! not exist and `Open` throws.

use std::any::Any;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use crate::dotnet::{ascii_get_bytes, ascii_get_string, DateTime, Exception, MemoryStream, SeekOrigin, Stream};
use crate::extlibs::interfaces::ICommsSerial;

const LOG: &str = "MissionPlanner.Comms.CommsSerialPipe";

fn null_ref() -> Exception {
    Exception::NullReference("Object reference not set to an instance of an object.".into())
}

fn disposed_stream() -> Exception {
    Exception::ObjectDisposed("Cannot access a closed Stream.".into())
}

/// `BaseStream`: a `MemoryStream` that can be disposed.
pub struct PipeStream {
    ms: MemoryStream,
    disposed: bool,
}

impl PipeStream {
    fn can_read(&self) -> bool {
        !self.disposed
    }
}

/// The C# `locker` guards the stream between the reader thread and the readers.
type SharedStream = Arc<Mutex<PipeStream>>;

/// `COMMTIMEOUTS` as declared in the C# (constant before multiplier).
#[repr(C)]
#[derive(Debug, Default, Clone, Copy)]
#[allow(non_snake_case)]
pub struct COMMTIMEOUTS {
    pub ReadIntervalTimeout: i32,
    pub ReadTotalTimeoutConstant: i32,
    pub ReadTotalTimeoutMultiplier: i32,
    pub WriteTotalTimeoutConstant: i32,
    pub WriteTotalTimeoutMultiplier: i32,
}

/// `COMMPROP` as declared in the C#; `char` marshals as one ANSI byte.
#[repr(C)]
#[derive(Debug, Default, Clone, Copy)]
#[allow(non_snake_case)]
pub struct COMMPROP {
    pub dwCurrentRxQueue: i32,
    pub dwCurrentTxQueue: i32,
    pub dwMaxBaud: i32,
    pub dwMaxRxQueue: i32,
    pub dwMaxTxQueue: i32,
    pub dwProvCapabilities: i32,
    pub dwProvSpec1: i32,
    pub dwProvSpec2: i32,
    pub dwProvSubType: i32,
    pub dwReserved1: i32,
    pub dwServiceMask: i32,
    pub dwSettableBaud: i32,
    pub dwSettableParams: i32,
    pub wcProvChar: u8,
    pub wPacketLength: u16,
    pub wPacketVersion: u16,
    pub wSettableData: u16,
    pub wSettableStopParity: u16,
}

/// `COMSTAT` as declared in the C#.
#[repr(C)]
#[derive(Debug, Default, Clone, Copy)]
#[allow(non_snake_case)]
pub struct COMSTAT {
    pub cbInQue: u32,
    pub cbOutQue: u32,
    pub Flags: u32,
}

/// `DCB` as declared in the C#: fields in alphabetical order, not the Win32 order.
#[repr(C)]
#[derive(Debug, Default, Clone, Copy)]
#[allow(non_snake_case)]
pub struct DCB {
    pub BaudRate: u32,
    pub ByteSize: u8,
    pub DCBlength: u32,
    pub EofChar: u8,
    pub ErrorChar: u8,
    pub EvtChar: u8,
    pub Flags: u32,
    pub Parity: u8,
    pub StopBits: u8,
    pub wReserved: u16,
    pub wReserved1: u16,
    pub XoffChar: u8,
    pub XoffLim: u16,
    pub XonChar: u8,
    pub XonLim: u16,
}

#[cfg(windows)]
#[allow(non_snake_case)]
mod native {
    use super::{COMMPROP, COMMTIMEOUTS, DCB};
    use std::ffi::c_void;

    pub type Handle = *mut c_void;
    pub const INVALID_HANDLE_VALUE: isize = -1;

    #[link(name = "kernel32")]
    extern "system" {
        pub fn CreateFileW(
            lpFileName: *const u16,
            dwDesiredAccess: i32,
            dwShareMode: i32,
            securityAttrs: *mut c_void,
            dwCreationDisposition: i32,
            dwFlagsAndAttributes: i32,
            hTemplateFile: Handle,
        ) -> Handle;
        pub fn GetCommProperties(hFile: Handle, lpCommProp: *mut COMMPROP) -> i32;
        pub fn GetCommModemStatus(hFile: Handle, lpModemStat: *mut i32) -> i32;
        pub fn GetCommState(hFile: Handle, lpDCB: *mut DCB) -> i32;
        pub fn SetCommState(hFile: Handle, lpDCB: *mut DCB) -> i32;
        pub fn SetCommTimeouts(hFile: Handle, lpCommTimeouts: *mut COMMTIMEOUTS) -> i32;
        pub fn SetCommMask(hFile: Handle, dwEvtMask: i32) -> i32;
        pub fn ReadFile(hFile: Handle, lpBuffer: *mut u8, n: u32, lpRead: *mut u32, lpOverlapped: *mut c_void) -> i32;
        pub fn WriteFile(hFile: Handle, lpBuffer: *const u8, n: i32, lpWritten: *mut i32, lpOverlapped: *mut c_void) -> i32;
        pub fn CloseHandle(h: Handle) -> i32;
    }
}

/// `SafeFileHandle`: closed once, by whichever of `Close` or the reader thread gets there first.
struct SafeFileHandle {
    raw: isize,
    closed: AtomicBool,
}

impl SafeFileHandle {
    fn is_invalid(&self) -> bool {
        self.raw == -1 || self.raw == 0
    }

    fn dispose(&self) {
        if !self.closed.swap(true, Ordering::SeqCst) && !self.is_invalid() {
            #[cfg(windows)]
            unsafe {
                native::CloseHandle(self.raw as native::Handle);
            }
        }
    }
}

/// `CommsSerialPipe : ICommsSerial`
pub struct CommsSerialPipe {
    comm_timeouts: COMMTIMEOUTS,
    safe_file_handle: Option<Arc<SafeFileHandle>>,
    base_stream: Option<SharedStream>,
    is_open: Arc<AtomicBool>,
    baud_rate: i32,
    data_bits: i32,
    dtr_enable: bool,
    port_name: String,
    read_buffer_size: i32,
    read_timeout: i32,
    rts_enable: bool,
    write_buffer_size: i32,
    write_timeout: i32,
}

impl Default for CommsSerialPipe {
    fn default() -> Self {
        Self::new()
    }
}

impl CommsSerialPipe {
    pub fn new() -> Self {
        CommsSerialPipe {
            comm_timeouts: COMMTIMEOUTS::default(),
            safe_file_handle: None,
            base_stream: None,
            is_open: Arc::new(AtomicBool::new(false)),
            baud_rate: 0,
            data_bits: 0,
            dtr_enable: false,
            port_name: String::new(),
            read_buffer_size: 0,
            read_timeout: 0,
            rts_enable: false,
            write_buffer_size: 0,
            write_timeout: 0,
        }
    }

    /// `SetDcbFlag(DCB dcb, int whichFlag, int setting)`: `dcb` is a copy, so the caller's DCB
    /// is unchanged.
    pub fn set_dcb_flag(mut dcb: DCB, which_flag: i32, setting: i32) {
        let setting = setting << which_flag;
        let num: u32 = if which_flag == 4 || which_flag == 12 {
            3
        } else if which_flag == 15 {
            131071
        } else {
            1
        };
        dcb.Flags &= !(num << which_flag);
        dcb.Flags |= setting as u32;
        let _ = dcb;
    }

    fn stream(&self) -> Result<&SharedStream, Exception> {
        self.base_stream.as_ref().ok_or_else(null_ref)
    }

    #[cfg(windows)]
    fn open_inner(&mut self) -> Result<(), Exception> {
        self.is_open.store(false, Ordering::SeqCst);
        let stream: SharedStream = Arc::new(Mutex::new(PipeStream { ms: MemoryStream::default(), disposed: false }));
        self.base_stream = Some(stream.clone());

        let dw_flags_and_attributes = 128;
        let path: Vec<u16> = format!("\\\\.\\{}", self.port_name).encode_utf16().chain(std::iter::once(0)).collect();
        let h = unsafe {
            native::CreateFileW(path.as_ptr(), -1073741824, 0, std::ptr::null_mut(), 3, dw_flags_and_attributes, std::ptr::null_mut())
        };
        let handle = Arc::new(SafeFileHandle { raw: h as isize, closed: AtomicBool::new(false) });
        self.safe_file_handle = Some(handle.clone());

        if handle.is_invalid() || handle.raw == native::INVALID_HANDLE_VALUE {
            return Err(Exception::Other("Invalid Port".into()));
        }
        let hf = handle.raw as native::Handle;

        let mut num1 = 0i32;
        let mut comm_prop = COMMPROP::default();
        unsafe {
            if native::GetCommProperties(hf, &mut comm_prop) == 0 || native::GetCommModemStatus(hf, &mut num1) == 0 {
                // the C# reads GetLastWin32Error and ignores it (87 / 6 checks are commented out)
            }
        }

        let mut dcb = DCB::default();
        unsafe {
            native::GetCommState(hf, &mut dcb);
        }

        dcb.BaudRate = self.baud_rate as u32;
        dcb.ByteSize = 8;
        dcb.StopBits = 0;
        dcb.Parity = 0;
        //https://msdn.microsoft.com/en-us/library/windows/desktop/aa363214(v=vs.85).aspx
        Self::set_dcb_flag(dcb, 0, 1); //fBinary
        Self::set_dcb_flag(dcb, 1, 0); //fParity
        Self::set_dcb_flag(dcb, 2, 1); //fOutxCtsFlow
        Self::set_dcb_flag(dcb, 3, 0); // fOutxDsrFlow
        Self::set_dcb_flag(dcb, 4, 0); //fDtrControl
        Self::set_dcb_flag(dcb, 5, 0); //fDsrSensitivity
        Self::set_dcb_flag(dcb, 6, 0); //fTXContinueOnXoff
        Self::set_dcb_flag(dcb, 7, 0); //fOutX
        Self::set_dcb_flag(dcb, 8, 0); //fInX
        Self::set_dcb_flag(dcb, 9, 0); //fErrorChar
        Self::set_dcb_flag(dcb, 10, 0); //fNull
        Self::set_dcb_flag(dcb, 11, 1); //fRtsControl
        Self::set_dcb_flag(dcb, 14, 0); //fAbortOnError

        unsafe {
            native::SetCommState(hf, &mut dcb);
        }

        self.comm_timeouts.ReadTotalTimeoutConstant = 0;
        self.comm_timeouts.ReadTotalTimeoutMultiplier = 0;
        self.comm_timeouts.ReadIntervalTimeout = -1;

        unsafe {
            native::SetCommTimeouts(hf, &mut self.comm_timeouts);
            native::SetCommMask(hf, 507);
        }

        self.is_open.store(true, Ordering::SeqCst);

        let is_open = self.is_open.clone();
        let th_handle = handle.clone();
        std::thread::Builder::new()
            .name("CommsSerialPipe reader".into())
            .spawn(move || {
                while is_open.load(Ordering::SeqCst) {
                    let mut buffer = vec![0u8; 4096];
                    let mut read: u32 = 0;
                    unsafe {
                        native::ReadFile(
                            th_handle.raw as native::Handle,
                            buffer.as_mut_ptr(),
                            buffer.len() as u32,
                            &mut read,
                            std::ptr::null_mut(),
                        );
                    }
                    if !is_open.load(Ordering::SeqCst) {
                        continue;
                    }
                    let mut s = stream.lock().unwrap_or_else(|e| e.into_inner());
                    if s.disposed {
                        continue;
                    }
                    // Writes at the read position, then seeks back to it.
                    let pos = s.ms.pos();
                    s.ms.write_from(&buffer, 0, read as usize);
                    let _ = s.ms.seek(pos, SeekOrigin::Begin);
                }
                th_handle.dispose();
            })
            .map_err(|e| Exception::Other(e.to_string()))?;
        Ok(())
    }

    #[cfg(not(windows))]
    fn open_inner(&mut self) -> Result<(), Exception> {
        self.is_open.store(false, Ordering::SeqCst);
        self.base_stream = Some(Arc::new(Mutex::new(PipeStream { ms: MemoryStream::default(), disposed: false })));
        Err(Exception::Other("Unable to find an entry point named 'CreateFile' in DLL 'kernel32.dll'.".into()))
    }
}

/// `BaseStream` seen from outside: the shared `MemoryStream`.
pub struct PipeStreamRef {
    s: SharedStream,
}

impl Stream for PipeStreamRef {
    fn can_seek(&self) -> bool {
        !self.s.lock().unwrap_or_else(|e| e.into_inner()).disposed
    }
    fn position(&self) -> Result<i64, Exception> {
        let s = self.s.lock().unwrap_or_else(|e| e.into_inner());
        if s.disposed {
            return Err(disposed_stream());
        }
        Ok(s.ms.pos())
    }
    fn length(&self) -> Result<i64, Exception> {
        let s = self.s.lock().unwrap_or_else(|e| e.into_inner());
        if s.disposed {
            return Err(disposed_stream());
        }
        Ok(s.ms.len())
    }
    fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Exception> {
        let mut s = self.s.lock().unwrap_or_else(|e| e.into_inner());
        if s.disposed {
            return Err(disposed_stream());
        }
        let n = buffer.len();
        Ok(s.ms.read_into(buffer, 0, n))
    }
    fn write(&mut self, buffer: &[u8]) -> Result<(), Exception> {
        let mut s = self.s.lock().unwrap_or_else(|e| e.into_inner());
        if s.disposed {
            return Err(disposed_stream());
        }
        s.ms.write_from(buffer, 0, buffer.len());
        Ok(())
    }
}

impl ICommsSerial for CommsSerialPipe {
    fn base_stream(&mut self) -> Result<Box<dyn Stream + '_>, Exception> {
        Ok(Box::new(PipeStreamRef { s: self.stream()?.clone() }))
    }

    fn baud_rate(&self) -> i32 {
        self.baud_rate
    }
    fn set_baud_rate(&mut self, value: i32) {
        self.baud_rate = value;
    }

    /// Unread bytes in the stream; an empty stream is truncated back to zero.
    fn bytes_to_read(&mut self) -> Result<i32, Exception> {
        let stream = self.stream()?;
        let mut s = stream.lock().unwrap_or_else(|e| e.into_inner());
        if !s.can_read() {
            return Ok(0);
        }
        let left = s.ms.len() - s.ms.pos();
        if left == 0 {
            s.ms.set_length(0);
        }
        Ok(left as i32)
    }

    fn bytes_to_write(&mut self) -> i32 {
        0
    }

    fn data_bits(&self) -> i32 {
        self.data_bits
    }
    fn set_data_bits(&mut self, value: i32) {
        self.data_bits = value;
    }

    fn dtr_enable(&self) -> bool {
        self.dtr_enable
    }
    fn set_dtr_enable(&mut self, value: bool) {
        self.dtr_enable = value;
    }

    fn is_open(&mut self) -> bool {
        self.is_open.load(Ordering::SeqCst)
    }

    fn port_name(&self) -> String {
        self.port_name.clone()
    }
    fn set_port_name(&mut self, value: &str) {
        self.port_name = value.to_string();
    }

    fn read_buffer_size(&self) -> i32 {
        self.read_buffer_size
    }
    fn set_read_buffer_size(&mut self, value: i32) {
        self.read_buffer_size = value;
    }

    fn read_timeout(&self) -> i32 {
        self.read_timeout
    }
    fn set_read_timeout(&mut self, value: i32) {
        self.read_timeout = value;
    }

    fn rts_enable(&self) -> bool {
        self.rts_enable
    }
    fn set_rts_enable(&mut self, value: bool) {
        self.rts_enable = value;
    }

    fn write_buffer_size(&self) -> i32 {
        self.write_buffer_size
    }
    fn set_write_buffer_size(&mut self, value: i32) {
        self.write_buffer_size = value;
    }

    fn write_timeout(&self) -> i32 {
        self.write_timeout
    }
    fn set_write_timeout(&mut self, value: i32) {
        self.write_timeout = value;
    }

    fn close(&mut self) -> Result<(), Exception> {
        self.is_open.store(false, Ordering::SeqCst);
        log::info!(target: LOG, "Closing port {}", self.port_name);
        self.stream()?.lock().unwrap_or_else(|e| e.into_inner()).disposed = true;
        self.safe_file_handle.as_ref().ok_or_else(null_ref)?.dispose();
        Ok(())
    }

    /// `DiscardInBuffer()`: `ReadExisting()`.
    fn discard_in_buffer(&mut self) -> Result<(), Exception> {
        self.read_existing().map(|_| ())
    }

    fn open(&mut self) -> Result<(), Exception> {
        // 500ms write timeout - win32 api default
        self.write_timeout = 500;

        if self.is_open() {
            return Ok(());
        }

        if self.port_name.starts_with('/') && !std::path::Path::new(&self.port_name).exists() {
            return Err(Exception::Other("No such device".into()));
        }

        match self.open_inner() {
            Ok(()) => Ok(()),
            Err(e) => {
                let _ = self.close();
                Err(e)
            }
        }
    }

    /// Waits the whole `ReadTimeout`, then serves what has arrived.
    fn read(&mut self, buffer: &mut [u8], offset: i32, count: i32) -> Result<i32, Exception> {
        let timeout = DateTime::now().add_milliseconds(self.read_timeout as f64)?;
        while timeout > DateTime::now() {
            std::thread::sleep(std::time::Duration::from_millis(1));
        }

        if self.bytes_to_read()? == 0 && self.read_timeout > 0 {
            return Err(Exception::Timeout("The operation has timed out.".into()));
        }

        let toread = count.min(self.bytes_to_read()?);

        let stream = self.stream()?;
        let mut s = stream.lock().unwrap_or_else(|e| e.into_inner());
        if s.disposed {
            return Err(disposed_stream());
        }
        Ok(s.ms.read_into(buffer, offset as usize, toread.max(0) as usize) as i32)
    }

    fn read_byte(&mut self) -> Result<i32, Exception> {
        let mut count = 0;
        while self.bytes_to_read()? == 0 {
            std::thread::sleep(std::time::Duration::from_millis(1));
            if count > self.read_timeout {
                return Err(Exception::Other("CommsSerialPipe Timeout on read".into()));
            }
            count += 1;
        }
        let mut buffer = [0u8; 1];
        self.read(&mut buffer, 0, 1)?;
        Ok(buffer[0] as i32)
    }

    fn read_char(&mut self) -> Result<i32, Exception> {
        self.read_byte()
    }

    fn read_existing(&mut self) -> Result<String, Exception> {
        let n = self.bytes_to_read()?;
        let mut data = vec![0u8; n.max(0) as usize];
        if !data.is_empty() {
            self.read(&mut data, 0, n)?;
        }
        Ok(ascii_get_string(&data))
    }

    fn read_line(&mut self) -> Result<String, Exception> {
        let mut temp = vec![0u8; 4000];
        let mut count = 0usize;
        let mut timeout = 0;

        while timeout <= 100 {
            if !self.is_open() {
                break;
            }
            if self.bytes_to_read()? > 0 {
                let letter = self.read_byte()? as u8;
                temp[count] = letter;
                if letter == b'\n' {
                    break;
                }
                count += 1;
                if count == temp.len() {
                    break;
                }
                timeout = 0;
            } else {
                timeout += 1;
                std::thread::sleep(std::time::Duration::from_millis(5));
            }
        }

        temp.resize(count + 1, 0);
        Ok(ascii_get_string(&temp))
    }

    fn write_str(&mut self, line: &str) -> Result<(), Exception> {
        let data = ascii_get_bytes(line);
        let n = data.len() as i32;
        self.write(&data, 0, n)
    }

    /// `WriteFile(handle, buffer.Skip(offset), count, ...)`; the result is ignored.
    fn write(&mut self, buffer: &[u8], offset: i32, count: i32) -> Result<(), Exception> {
        let h = self.safe_file_handle.as_ref().ok_or_else(|| {
            Exception::ArgumentNull("SafeHandle cannot be null.\r\nParameter name: pHandle".into())
        })?;
        let data: Vec<u8> = if offset != 0 { buffer.iter().skip(offset.max(0) as usize).copied().collect() } else { buffer.to_vec() };
        let _ = (&data, count, h);
        #[cfg(windows)]
        {
            if h.closed.load(Ordering::SeqCst) {
                return Err(Exception::ObjectDisposed("Safe handle has been closed".into()));
            }
            let n = (count.max(0) as usize).min(data.len()) as i32;
            let mut result = 0i32;
            unsafe {
                native::WriteFile(h.raw as native::Handle, data.as_ptr(), n, &mut result, std::ptr::null_mut());
            }
        }
        Ok(())
    }

    fn write_line(&mut self, line: &str) -> Result<(), Exception> {
        let line = format!("{line}\n");
        self.write_str(&line)
    }

    fn toggleDTR(&mut self) -> Result<(), Exception> {
        Ok(())
    }

    fn dispose(&mut self) {
        let _ = self.close();
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn csharp_struct_layouts() {
        // The alphabetical C# DCB is 36 bytes with natural alignment (the Win32 DCB is 28).
        assert_eq!(std::mem::size_of::<DCB>(), 36);
        assert_eq!(std::mem::size_of::<COMMTIMEOUTS>(), 20);
    }

    #[test]
    fn set_dcb_flag_leaves_callers_dcb() {
        let dcb = DCB::default();
        CommsSerialPipe::set_dcb_flag(dcb, 0, 1);
        assert_eq!(dcb.Flags, 0);
    }

    #[test]
    fn closed_and_unopened_behaviour() {
        let mut p = CommsSerialPipe::new();
        assert!(matches!(p.bytes_to_read(), Err(Exception::NullReference(_))));
        assert!(matches!(p.close(), Err(Exception::NullReference(_))));
        p.set_port_name("/no/such/port");
        assert!(matches!(p.open(), Err(Exception::Other(m)) if m == "No such device"));
        assert_eq!(p.write_timeout(), 500);
    }
}

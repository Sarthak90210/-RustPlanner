//! Port of `ExtLibs/Comms/CommsSerialPort.cs`
//!
//! `SerialPort` is Mission Planner's `ICommsSerial` facade that forwards to a platform port
//! chosen by `DefaultType` (a `WinSerialPort` on the desktop), plus the static port
//! enumeration helpers. `WinSerialPort` derives from the BCL `System.IO.Ports.SerialPort`
//! (`crate::dotnet::ports`). `SerialPortFixer` clears DCB.fAbortOnError through kernel32.
//!
//! The `ICommsSerial` property setters cannot report failure, while the BCL setters throw
//! (for example a non-positive `BaudRate`). Those exceptions are logged and the value is left
//! unchanged, which is where this port differs from C#: there the setter call itself throws.

use std::any::Any;
use std::collections::HashMap;
use std::sync::{Arc, Condvar, Mutex, RwLock};
use std::time::Duration;

use super::commsbase::StaticEvent;
use crate::dotnet::ports::{self, Handshake};
use crate::dotnet::{Exception, Stream};
use crate::extlibs::interfaces::ICommsSerial;

const LOG: &str = "MissionPlanner.Comms.SerialPort";
const LOG_WIN: &str = "MissionPlanner.Comms.WinSerialPort";
const LOG_FIXER: &str = "MissionPlanner.Comms.SerialPortFixer";

fn null_ref() -> Exception {
    Exception::NullReference("Object reference not set to an instance of an object.".into())
}

/// `Func<SerialPort, string, int, ICommsSerial>`. Rust cannot hand a callback the object that
/// is still being constructed, so the `SerialPort` argument is not passed.
pub type DefaultTypeFn = Arc<dyn Fn(Option<&str>, i32) -> Option<Box<dyn ICommsSerial>> + Send + Sync>;

/// `public static Func<SerialPort, string, int, ICommsSerial> DefaultType`; `None` is the
/// initial value, `(self, portname, baudrate) => new WinSerialPort()`.
static DEFAULT_TYPE: RwLock<Option<DefaultTypeFn>> = RwLock::new(None);

/// `public static Func<List<string>> GetCustomPorts`
pub type GetCustomPortsFn = Arc<dyn Fn() -> Result<Vec<String>, Exception> + Send + Sync>;
static GET_CUSTOM_PORTS: RwLock<Option<GetCustomPortsFn>> = RwLock::new(None);

/// `public static event EventArgsDeviceName GetDeviceName`
pub static GET_DEVICE_NAME: StaticEvent<dyn Fn(&str) -> Result<String, Exception> + Send + Sync> = StaticEvent::new();

static LOCKER: Mutex<()> = Mutex::new(());
static COMPORTNAMECACHE: Mutex<Option<HashMap<String, String>>> = Mutex::new(None);

const SYSTEM_PORT_ENUMERATION_TIMEOUT_MS: u64 = 2000;

type Enumeration = Arc<(Mutex<Option<Result<Vec<String>, Exception>>>, Condvar)>;

/// The last enumeration still running (a driver can stall it) and the last list it produced.
static SYSTEM_PORT_ENUMERATION: Mutex<Option<Enumeration>> = Mutex::new(None);
static LAST_SYSTEM_PORTS: Mutex<Vec<String>> = Mutex::new(Vec::new());

/// `MissionPlanner.Comms.SerialPort : ICommsSerial`
pub struct SerialPort {
    pub _baseport: Option<Box<dyn ICommsSerial>>,
    pub esp_fix: bool,
}

impl SerialPort {
    /// `DefaultType` getter.
    pub fn default_type() -> DefaultTypeFn {
        DEFAULT_TYPE
            .read()
            .unwrap_or_else(|e| e.into_inner())
            .clone()
            .unwrap_or_else(|| Arc::new(|_: Option<&str>, _: i32| Some(Box::new(WinSerialPort::new()) as Box<dyn ICommsSerial>)))
    }

    /// `DefaultType` setter.
    pub fn set_default_type(f: DefaultTypeFn) {
        *DEFAULT_TYPE.write().unwrap_or_else(|e| e.into_inner()) = Some(f);
    }

    /// `GetCustomPorts` assignment.
    pub fn set_get_custom_ports(f: Option<GetCustomPortsFn>) {
        *GET_CUSTOM_PORTS.write().unwrap_or_else(|e| e.into_inner()) = f;
    }

    /// `SerialPort()`
    pub fn new() -> Self {
        SerialPort { _baseport: Self::default_type()(None, 0), esp_fix: false }
    }

    /// `SerialPort(string portname, int baudrate)`
    pub fn with_port_baud(portname: &str, baudrate: i32) -> Self {
        let mut s = SerialPort { _baseport: Self::default_type()(Some(portname), baudrate), esp_fix: false };
        s.set_port_name(portname);
        s.set_baud_rate(baudrate);
        s
    }

    /// `SerialPort(string portname)`
    pub fn with_port(portname: &str) -> Self {
        let mut s = SerialPort { _baseport: Self::default_type()(Some(portname), 0), esp_fix: false };
        s.set_port_name(portname);
        s
    }

    fn bp(&mut self) -> Result<&mut Box<dyn ICommsSerial>, Exception> {
        self._baseport.as_mut().ok_or_else(null_ref)
    }

    /// `public new static string[] GetPortNames()`
    pub fn get_port_names() -> Vec<String> {
        // prevent hammering
        let _g = LOCKER.lock().unwrap_or_else(|e| e.into_inner());
        let mut all_ports: Vec<String> = vec![];

        if std::path::Path::new("/dev/").is_dir() {
            // mono is failing in here on linux "too many open files"
            if std::path::Path::new("/dev/serial/by-id/").is_dir() {
                if let Ok(f) = get_files("/dev/serial/by-id/", "*") {
                    all_ports.extend(f);
                }
            }
            for pattern in ["ttyACM*", "ttyUSB*", "rfcomm*", "*usb*", "tty.*", "cu.*"] {
                if let Ok(f) = get_files("/dev/", pattern) {
                    all_ports.extend(f);
                }
            }
        }

        // any exceptions will still result in a list
        let ports: Vec<String> = get_system_port_names()
            .into_iter()
            .map(|p| p.trim_end().to_string())
            .map(fix_blue_tooth_port_name_bug)
            .collect();
        all_ports.extend(ports);

        let custom = GET_CUSTOM_PORTS.read().unwrap_or_else(|e| e.into_inner()).clone();
        if let Some(custom) = custom {
            match custom() {
                Ok(v) => all_ports.extend(v),
                Err(ex) => log::error!(target: LOG, "{ex}"),
            }
        }

        // a port removed mid-enumeration can leave a null entry, which ComboBox.Items.AddRange rejects
        let mut out: Vec<String> = vec![];
        for p in all_ports {
            if !p.trim().is_empty() && !out.contains(&p) {
                out.push(p);
            }
        }
        out
    }

    /// `GetNiceName(string port)`: the device's friendly name from `GetDeviceName`, cached;
    /// a lookup that takes more than a second yields "" (and is left to finish on its own).
    pub fn get_nice_name(port: &str) -> String {
        // make sure we are exclusive
        let _g = LOCKER.lock().unwrap_or_else(|e| e.into_inner());

        if port == "AUTO" || port == "UDP" || port == "UDPCl" || port == "TCP" || port == "WS" {
            return String::new();
        }

        {
            let cache = COMPORTNAMECACHE.lock().unwrap_or_else(|e| e.into_inner());
            if let Some(v) = cache.as_ref().and_then(|c| c.get(port)) {
                log::info!(target: LOG, "done GetNiceName cache {} {}", port, v);
                return v.clone();
            }
        }

        log::info!(target: LOG, "start GetNiceName {}", port);

        let (tx, rx) = std::sync::mpsc::channel();
        let p = port.to_string();
        std::thread::spawn(move || {
            let _ = tx.send(get_name(&p));
        });
        let portnamenice = rx.recv_timeout(Duration::from_millis(1000)).unwrap_or_default();

        log::info!(target: LOG, "done GetNiceName {} = {}", port, portnamenice);

        COMPORTNAMECACHE
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .get_or_insert_with(HashMap::new)
            .insert(port.to_string(), portnamenice.clone());

        portnamenice
    }
}

impl Default for SerialPort {
    fn default() -> Self {
        Self::new()
    }
}

/// `Directory.GetFiles(dir, pattern)` for the simple `*` patterns used above: full paths of the
/// non-directory entries whose names match, in directory order.
fn get_files(dir: &str, pattern: &str) -> std::io::Result<Vec<String>> {
    let mut out = vec![];
    for e in std::fs::read_dir(dir)? {
        let e = e?;
        if e.file_type().map(|t| t.is_dir()).unwrap_or(true) {
            continue;
        }
        let name = e.file_name().to_string_lossy().to_string();
        if wildcard_match(pattern, &name) {
            out.push(format!("{}{}", dir, name));
        }
    }
    Ok(out)
}

fn wildcard_match(pattern: &str, name: &str) -> bool {
    let parts: Vec<&str> = pattern.split('*').collect();
    if parts.len() == 1 {
        return pattern == name;
    }
    let mut rest = name;
    if !rest.starts_with(parts[0]) {
        return false;
    }
    rest = &rest[parts[0].len()..];
    let last = parts[parts.len() - 1];
    for mid in &parts[1..parts.len() - 1] {
        match rest.find(mid) {
            Some(i) => rest = &rest[i + mid.len()..],
            None => return false,
        }
    }
    rest.ends_with(last)
}

/// `GetSystemPortNames()`: `System.IO.Ports.SerialPort.GetPortNames` with a timeout. On a
/// timeout or failure the previous list is returned, and while a stalled enumeration is still
/// running no new one is started.
fn get_system_port_names() -> Vec<String> {
    let mut slot = SYSTEM_PORT_ENUMERATION.lock().unwrap_or_else(|e| e.into_inner());
    if let Some(running) = slot.as_ref() {
        if running.0.lock().unwrap_or_else(|e| e.into_inner()).is_none() {
            return LAST_SYSTEM_PORTS.lock().unwrap_or_else(|e| e.into_inner()).clone();
        }
    }

    let task: Enumeration = Arc::new((Mutex::new(None), Condvar::new()));
    let t2 = task.clone();
    std::thread::spawn(move || {
        let r = ports::SerialPort::get_port_names();
        *t2.0.lock().unwrap_or_else(|e| e.into_inner()) = Some(r);
        t2.1.notify_all();
    });
    *slot = Some(task.clone());
    drop(slot);

    let guard = task.0.lock().unwrap_or_else(|e| e.into_inner());
    let (guard, to) = task
        .1
        .wait_timeout_while(guard, Duration::from_millis(SYSTEM_PORT_ENUMERATION_TIMEOUT_MS), |r| r.is_none())
        .unwrap_or_else(|e| e.into_inner());
    if to.timed_out() && guard.is_none() {
        log::warn!(
            target: LOG,
            "System.IO.Ports.SerialPort.GetPortNames did not return within {} ms, using the previous port list.",
            SYSTEM_PORT_ENUMERATION_TIMEOUT_MS
        );
        return LAST_SYSTEM_PORTS.lock().unwrap_or_else(|e| e.into_inner()).clone();
    }
    match guard.as_ref().unwrap() {
        Ok(v) => *LAST_SYSTEM_PORTS.lock().unwrap_or_else(|e| e.into_inner()) = v.clone(),
        Err(ex) => log::error!(target: LOG, "Failed to enumerate system serial ports, using the previous port list. {ex}"),
    }
    LAST_SYSTEM_PORTS.lock().unwrap_or_else(|e| e.into_inner()).clone()
}

/// `GetName(string port)`: the last `GetDeviceName` handler's answer, or "".
fn get_name(port: &str) -> String {
    let mut ans: Option<String> = None;
    for h in GET_DEVICE_NAME.list() {
        match h(port) {
            Ok(a) => ans = Some(a),
            Err(_) => return String::new(),
        }
    }
    match ans {
        Some(a) if !a.is_empty() => a,
        _ => String::new(),
    }
}

/// `FixBlueToothPortNameBug`: Bluetooth ports are sometimes enumerated with bogus characters
/// (`COM10c`); keep "COM" and the digits among the next three characters.
fn fix_blue_tooth_port_name_bug(port_name: String) -> String {
    if !port_name.starts_with("COM") {
        return port_name;
    }
    let mut new_port_name = "COM".to_string();
    for port_char in port_name[3..].chars().take(3) {
        // char.IsDigit: Unicode decimal digits
        if port_char.is_ascii_digit() || is_decimal_digit(port_char) {
            new_port_name.push(port_char);
        }
    }
    new_port_name
}

/// Unicode category Nd for the common non-ASCII digit blocks (Arabic-Indic, Devanagari, full width...).
fn is_decimal_digit(c: char) -> bool {
    let u = c as u32;
    const ZEROS: &[u32] = &[
        0x0660, 0x06F0, 0x07C0, 0x0966, 0x09E6, 0x0A66, 0x0AE6, 0x0B66, 0x0BE6, 0x0C66, 0x0CE6, 0x0D66, 0x0DE6, 0x0E50,
        0x0ED0, 0x0F20, 0x1040, 0x1090, 0x17E0, 0x1810, 0x1946, 0x19D0, 0x1A80, 0x1A90, 0x1B50, 0x1BB0, 0x1C40, 0x1C50,
        0xA620, 0xA8D0, 0xA900, 0xA9D0, 0xA9F0, 0xAA50, 0xABF0, 0xFF10,
    ];
    ZEROS.iter().any(|z| u >= *z && u < z + 10)
}

impl ICommsSerial for SerialPort {
    fn base_stream(&mut self) -> Result<Box<dyn Stream + '_>, Exception> {
        self.bp()?.base_stream()
    }

    fn baud_rate(&self) -> i32 {
        match &self._baseport {
            None => 0,
            Some(b) => b.baud_rate(),
        }
    }
    fn set_baud_rate(&mut self, value: i32) {
        if self._baseport.is_none() {
            let name = self.port_name();
            self._baseport = Self::default_type()(Some(name.as_str()), value);
        }
        // we are trying to set baudrate on a port that has no portname/doesnt exist
        let Some(b) = self._baseport.as_mut() else { return };
        b.set_baud_rate(value);
    }

    fn bytes_to_read(&mut self) -> Result<i32, Exception> {
        self.bp()?.bytes_to_read()
    }

    fn bytes_to_write(&mut self) -> i32 {
        self._baseport.as_mut().map(|b| b.bytes_to_write()).unwrap_or(0)
    }

    fn data_bits(&self) -> i32 {
        self._baseport.as_ref().map(|b| b.data_bits()).unwrap_or(0)
    }
    fn set_data_bits(&mut self, value: i32) {
        if let Some(b) = self._baseport.as_mut() {
            b.set_data_bits(value);
        }
    }

    fn dtr_enable(&self) -> bool {
        self._baseport.as_ref().map(|b| b.dtr_enable()).unwrap_or(false)
    }
    fn set_dtr_enable(&mut self, value: bool) {
        if let Some(b) = self._baseport.as_mut() {
            b.set_dtr_enable(value);
        }
    }

    fn is_open(&mut self) -> bool {
        match self._baseport.as_mut() {
            None => false,
            Some(b) => b.is_open(),
        }
    }

    fn port_name(&self) -> String {
        match &self._baseport {
            None => String::new(),
            Some(b) => b.port_name(),
        }
    }
    fn set_port_name(&mut self, value: &str) {
        if let Some(b) = self._baseport.as_mut() {
            b.set_port_name(value);
        }
        if self._baseport.is_none() {
            self._baseport = Self::default_type()(Some(value), 0);
        }
    }

    fn read_buffer_size(&self) -> i32 {
        self._baseport.as_ref().map(|b| b.read_buffer_size()).unwrap_or(0)
    }
    fn set_read_buffer_size(&mut self, value: i32) {
        if let Some(b) = self._baseport.as_mut() {
            b.set_read_buffer_size(value);
        }
    }

    fn read_timeout(&self) -> i32 {
        self._baseport.as_ref().map(|b| b.read_timeout()).unwrap_or(0)
    }
    fn set_read_timeout(&mut self, value: i32) {
        if let Some(b) = self._baseport.as_mut() {
            b.set_read_timeout(value);
        }
    }

    fn rts_enable(&self) -> bool {
        self._baseport.as_ref().map(|b| b.rts_enable()).unwrap_or(false)
    }
    fn set_rts_enable(&mut self, value: bool) {
        if let Some(b) = self._baseport.as_mut() {
            b.set_rts_enable(value);
        }
    }

    fn write_buffer_size(&self) -> i32 {
        self._baseport.as_ref().map(|b| b.write_buffer_size()).unwrap_or(0)
    }
    fn set_write_buffer_size(&mut self, value: i32) {
        if let Some(b) = self._baseport.as_mut() {
            b.set_write_buffer_size(value);
        }
    }

    fn write_timeout(&self) -> i32 {
        self._baseport.as_ref().map(|b| b.write_timeout()).unwrap_or(0)
    }
    fn set_write_timeout(&mut self, value: i32) {
        if let Some(b) = self._baseport.as_mut() {
            b.set_write_timeout(value);
        }
    }

    fn close(&mut self) -> Result<(), Exception> {
        self.bp()?.close()
    }

    fn discard_in_buffer(&mut self) -> Result<(), Exception> {
        self.bp()?.discard_in_buffer()
    }

    fn open(&mut self) -> Result<(), Exception> {
        let esp_fix = self.esp_fix;
        let bp = self.bp()?;
        if esp_fix {
            if let Some(w) = bp.as_any_mut().downcast_mut::<WinSerialPort>() {
                w.esp_fix = true;
            }
        }
        bp.open()
    }

    fn read(&mut self, buffer: &mut [u8], offset: i32, count: i32) -> Result<i32, Exception> {
        self.bp()?.read(buffer, offset, count)
    }

    fn read_byte(&mut self) -> Result<i32, Exception> {
        self.bp()?.read_byte()
    }

    fn read_char(&mut self) -> Result<i32, Exception> {
        self.bp()?.read_char()
    }

    fn read_existing(&mut self) -> Result<String, Exception> {
        self.bp()?.read_existing()
    }

    fn read_line(&mut self) -> Result<String, Exception> {
        self.bp()?.read_line()
    }

    fn write_str(&mut self, text: &str) -> Result<(), Exception> {
        self.bp()?.write_str(text)
    }

    fn write(&mut self, buffer: &[u8], offset: i32, count: i32) -> Result<(), Exception> {
        self.bp()?.write(buffer, offset, count)
    }

    fn write_line(&mut self, text: &str) -> Result<(), Exception> {
        self.bp()?.write_line(text)
    }

    fn toggleDTR(&mut self) -> Result<(), Exception> {
        self.bp()?.toggleDTR()
    }

    fn dispose(&mut self) {
        if let Some(b) = self._baseport.as_mut() {
            b.dispose();
        }
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}

/// `WinSerialPort : System.IO.Ports.SerialPort, ICommsSerial`
pub struct WinSerialPort {
    base: ports::SerialPort,
    pub esp_fix: bool,
}

impl Default for WinSerialPort {
    fn default() -> Self {
        Self::new()
    }
}

fn cs_bool(b: bool) -> &'static str {
    if b {
        "True"
    } else {
        "False"
    }
}

fn log_setter(r: Result<(), Exception>, what: &str) {
    if let Err(e) = r {
        log::warn!(target: LOG_WIN, "{what}: {e}");
    }
}

impl WinSerialPort {
    pub fn new() -> Self {
        WinSerialPort { base: ports::SerialPort::new(), esp_fix: false }
    }

    /// The BCL base object.
    pub fn base(&mut self) -> &mut ports::SerialPort {
        &mut self.base
    }
}

impl ICommsSerial for WinSerialPort {
    fn base_stream(&mut self) -> Result<Box<dyn Stream + '_>, Exception> {
        Ok(Box::new(self.base.base_stream()?))
    }

    fn baud_rate(&self) -> i32 {
        self.base.baud_rate()
    }
    fn set_baud_rate(&mut self, value: i32) {
        log_setter(self.base.set_baud_rate(value), "BaudRate");
    }

    fn bytes_to_read(&mut self) -> Result<i32, Exception> {
        self.base.bytes_to_read()
    }

    fn bytes_to_write(&mut self) -> i32 {
        self.base.bytes_to_write().unwrap_or(0)
    }

    fn data_bits(&self) -> i32 {
        self.base.data_bits()
    }
    fn set_data_bits(&mut self, value: i32) {
        log_setter(self.base.set_data_bits(value), "DataBits");
    }

    fn dtr_enable(&self) -> bool {
        self.base.dtr_enable()
    }
    /// `new bool DtrEnable { set; }`: logs, and skips an unchanged value.
    fn set_dtr_enable(&mut self, value: bool) {
        log::info!(target: LOG_WIN, "{} DtrEnable {}", self.base.port_name(), cs_bool(value));
        if self.base.dtr_enable() == value {
            return;
        }
        log_setter(self.base.set_dtr_enable(value), "DtrEnable");
    }

    fn is_open(&mut self) -> bool {
        self.base.is_open()
    }

    fn port_name(&self) -> String {
        self.base.port_name().to_string()
    }
    fn set_port_name(&mut self, value: &str) {
        log_setter(self.base.set_port_name(value), "PortName");
    }

    fn read_buffer_size(&self) -> i32 {
        self.base.read_buffer_size()
    }
    fn set_read_buffer_size(&mut self, value: i32) {
        log_setter(self.base.set_read_buffer_size(value), "ReadBufferSize");
    }

    fn read_timeout(&self) -> i32 {
        self.base.read_timeout()
    }
    fn set_read_timeout(&mut self, value: i32) {
        log_setter(self.base.set_read_timeout(value), "ReadTimeout");
    }

    fn rts_enable(&self) -> bool {
        self.base.rts_enable()
    }
    /// `new bool RtsEnable { set; }`: logs, and skips an unchanged value.
    fn set_rts_enable(&mut self, value: bool) {
        log::info!(target: LOG_WIN, "{} RtsEnable {}", self.base.port_name(), cs_bool(value));
        if self.base.rts_enable() == value {
            return;
        }
        log_setter(self.base.set_rts_enable(value), "RtsEnable");
    }

    fn write_buffer_size(&self) -> i32 {
        self.base.write_buffer_size()
    }
    fn set_write_buffer_size(&mut self, value: i32) {
        log_setter(self.base.set_write_buffer_size(value), "WriteBufferSize");
    }

    fn write_timeout(&self) -> i32 {
        self.base.write_timeout()
    }
    fn set_write_timeout(&mut self, value: i32) {
        log_setter(self.base.set_write_timeout(value), "WriteTimeout");
    }

    /// `new void Close()`
    fn close(&mut self) -> Result<(), Exception> {
        log::info!(target: LOG_WIN, "Closing port {}", self.base.port_name());
        self.base.close();
        Ok(())
    }

    fn discard_in_buffer(&mut self) -> Result<(), Exception> {
        self.base.discard_in_buffer()
    }

    /// `new void Open()`
    fn open(&mut self) -> Result<(), Exception> {
        // 500ms write timeout - win32 api default
        self.base.set_write_timeout(500)?;

        if self.base.is_open() {
            return Ok(());
        }

        // (the SerialPortFixer call for >= 115200 baud is commented out in the C#: it causes
        // "element not found" with bluetooth devices)

        let name = self.base.port_name().to_string();
        if name.starts_with('/') && !std::path::Path::new(&name).exists() {
            return Err(Exception::Other("No such device".into()));
        }

        if self.esp_fix {
            self.base.set_handshake(Handshake::RequestToSend)?;
        }

        if let Err(e) = self.base.open() {
            let _ = self.close();
            return Err(e);
        }
        if self.esp_fix {
            self.base.set_handshake(Handshake::None)?;
        }
        Ok(())
    }

    fn read(&mut self, buffer: &mut [u8], offset: i32, count: i32) -> Result<i32, Exception> {
        self.base.read(buffer, offset, count)
    }

    fn read_byte(&mut self) -> Result<i32, Exception> {
        self.base.read_byte()
    }

    fn read_char(&mut self) -> Result<i32, Exception> {
        self.base.read_char()
    }

    fn read_existing(&mut self) -> Result<String, Exception> {
        self.base.read_existing()
    }

    fn read_line(&mut self) -> Result<String, Exception> {
        self.base.read_line()
    }

    fn write_str(&mut self, text: &str) -> Result<(), Exception> {
        self.base.write_str(text)
    }

    fn write(&mut self, buffer: &[u8], offset: i32, count: i32) -> Result<(), Exception> {
        self.base.write(buffer, offset, count)
    }

    fn write_line(&mut self, text: &str) -> Result<(), Exception> {
        self.base.write_line(text)
    }

    /// Pulses DTR and RTS low for 50 ms, opening and re-closing the port if it was closed.
    fn toggleDTR(&mut self) -> Result<(), Exception> {
        let open = self.base.is_open();
        crate::dotnet::io::console_write_line(&format!("toggleDTR {}", cs_bool(self.base.is_open())));
        if !open {
            let _ = self.open();
        }

        self.base.set_dtr_enable(false)?;
        self.base.set_rts_enable(false)?;

        std::thread::sleep(Duration::from_millis(50));

        self.base.set_dtr_enable(true)?;
        self.base.set_rts_enable(true)?;

        std::thread::sleep(Duration::from_millis(50));

        if !open {
            let _ = self.close();
        }

        crate::dotnet::io::console_write_line(&format!("toggleDTR done {}", cs_bool(self.base.is_open())));
        Ok(())
    }

    /// `Component.Dispose()` -> `SerialPort.Dispose(true)`: closes the port.
    fn dispose(&mut self) {
        self.base.close();
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}

/// `public sealed class SerialPortFixer : IDisposable`: opens the COM port through kernel32 and
/// clears `DCB.fAbortOnError`, so a line error does not abort every later read and write.
#[cfg(windows)]
pub struct SerialPortFixer {
    m_handle: Option<native::Handle>,
}

#[cfg(windows)]
mod native {
    use std::ffi::c_void;

    pub type Handle = *mut c_void;
    pub const INVALID_HANDLE_VALUE: Handle = -1isize as Handle;

    /// `COMSTAT`
    #[repr(C)]
    #[derive(Default)]
    pub struct Comstat {
        pub flags: u32,
        pub cb_in_que: u32,
        pub cb_out_que: u32,
    }

    /// `DCB` (https://msdn.microsoft.com/en-us/library/windows/desktop/aa363214(v=vs.85).aspx)
    #[repr(C)]
    #[derive(Default)]
    pub struct Dcb {
        pub dcb_length: u32,
        pub baud_rate: u32,
        pub flags: u32,
        pub w_reserved: u16,
        pub xon_lim: u16,
        pub xoff_lim: u16,
        pub byte_size: u8,
        pub parity: u8,
        pub stop_bits: u8,
        pub xon_char: u8,
        pub xoff_char: u8,
        pub error_char: u8,
        pub eof_char: u8,
        pub evt_char: u8,
        pub w_reserved1: u16,
    }

    #[link(name = "kernel32")]
    #[allow(non_snake_case, clashing_extern_declarations)]
    extern "system" {
        pub fn CreateFileW(
            lpFileName: *const u16,
            dwDesiredAccess: u32,
            dwShareMode: u32,
            securityAttrs: *mut c_void,
            dwCreationDisposition: u32,
            dwFlagsAndAttributes: u32,
            hTemplateFile: Handle,
        ) -> Handle;
        pub fn GetFileType(hFile: Handle) -> u32;
        pub fn GetCommState(hFile: Handle, lpDcb: *mut Dcb) -> i32;
        pub fn SetCommState(hFile: Handle, lpDcb: *mut Dcb) -> i32;
        pub fn ClearCommError(hFile: Handle, lpErrors: *mut u32, lpStat: *mut Comstat) -> i32;
        pub fn CloseHandle(hObject: Handle) -> i32;
    }
}

#[cfg(windows)]
impl SerialPortFixer {
    const DCB_FLAG_ABORT_ON_ERROR: u32 = 14;
    const COMM_STATE_RETRIES: i32 = 10;

    /// `SerialPortFixer.Execute(portName)`
    pub fn execute(port_name: &str) -> Result<(), Exception> {
        let mut f = Self::new(Some(port_name))?;
        f.dispose();
        Ok(())
    }

    fn new(port_name: Option<&str>) -> Result<Self, Exception> {
        const DW_FLAGS_AND_ATTRIBUTES: u32 = 0x40000000;
        const DW_ACCESS: u32 = 0xC0000000;
        let port_name = match port_name {
            Some(p) if p.get(..3).is_some_and(|s| s.eq_ignore_ascii_case("COM")) => p,
            _ => return Err(Exception::Argument("Invalid Serial Port\r\nParameter name: portName".into())),
        };
        let path: Vec<u16> = format!("\\\\.\\{port_name}").encode_utf16().chain(std::iter::once(0)).collect();
        let h_file = unsafe {
            native::CreateFileW(path.as_ptr(), DW_ACCESS, 0, std::ptr::null_mut(), 3, DW_FLAGS_AND_ATTRIBUTES, std::ptr::null_mut())
        };
        if h_file == native::INVALID_HANDLE_VALUE {
            return Err(Self::win_io_error());
        }
        let file_type = unsafe { native::GetFileType(h_file) };
        if file_type != 2 && file_type != 0 {
            unsafe {
                native::CloseHandle(h_file);
            }
            return Err(Exception::Argument("Invalid Serial Port\r\nParameter name: portName".into()));
        }
        let mut f = SerialPortFixer { m_handle: Some(h_file) };
        if let Err(e) = f.initialize_dcb() {
            f.dispose();
            return Err(e);
        }
        Ok(f)
    }

    /// `Dispose()`
    pub fn dispose(&mut self) {
        if let Some(h) = self.m_handle.take() {
            unsafe {
                native::CloseHandle(h);
            }
        }
    }

    fn initialize_dcb(&mut self) -> Result<(), Exception> {
        let mut dcb = native::Dcb::default();
        self.get_comm_state_native(&mut dcb)?;
        log::info!(target: LOG_FIXER, "before dcb flags: {}", dcb.flags);
        dcb.flags &= !(1u32 << Self::DCB_FLAG_ABORT_ON_ERROR);
        log::info!(target: LOG_FIXER, "after dcb flags: {}", dcb.flags);
        self.set_comm_state_native(&mut dcb)
    }

    /// `GetMessage(errorCode)`: the system message text (FormatMessage keeps its trailing CRLF).
    fn get_message(error_code: i32) -> String {
        let s = std::io::Error::from_raw_os_error(error_code).to_string();
        match s.rfind(" (os error ") {
            Some(i) => format!("{}\r\n", &s[..i]),
            None => "Unknown Error".into(),
        }
    }

    /// `WinIoError()`: IOException with the last Win32 error's message.
    fn win_io_error() -> Exception {
        let error_code = std::io::Error::last_os_error().raw_os_error().unwrap_or(0);
        Exception::Io(Self::get_message(error_code))
    }

    fn get_comm_state_native(&mut self, lp_dcb: &mut native::Dcb) -> Result<(), Exception> {
        let h = self.m_handle.ok_or_else(null_ref)?;
        let mut comm_errors = 0u32;
        let mut com_stat = native::Comstat::default();
        for i in 0..Self::COMM_STATE_RETRIES {
            if unsafe { native::ClearCommError(h, &mut comm_errors, &mut com_stat) } == 0 {
                return Err(Self::win_io_error());
            }
            if unsafe { native::GetCommState(h, lp_dcb) } != 0 {
                break;
            }
            if i == Self::COMM_STATE_RETRIES - 1 {
                return Err(Self::win_io_error());
            }
        }
        Ok(())
    }

    fn set_comm_state_native(&mut self, lp_dcb: &mut native::Dcb) -> Result<(), Exception> {
        let h = self.m_handle.ok_or_else(null_ref)?;
        let mut comm_errors = 0u32;
        let mut com_stat = native::Comstat::default();
        for i in 0..Self::COMM_STATE_RETRIES {
            if unsafe { native::ClearCommError(h, &mut comm_errors, &mut com_stat) } == 0 {
                return Err(Self::win_io_error());
            }
            if unsafe { native::SetCommState(h, lp_dcb) } != 0 {
                break;
            }
            if i == Self::COMM_STATE_RETRIES - 1 {
                return Err(Self::win_io_error());
            }
        }
        Ok(())
    }
}

#[cfg(windows)]
impl Drop for SerialPortFixer {
    fn drop(&mut self) {
        self.dispose();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bluetooth_port_name_fix() {
        assert_eq!(fix_blue_tooth_port_name_bug("COM10c".into()), "COM10");
        assert_eq!(fix_blue_tooth_port_name_bug("COM3".into()), "COM3");
        assert_eq!(fix_blue_tooth_port_name_bug("COM1234".into()), "COM123");
        assert_eq!(fix_blue_tooth_port_name_bug("/dev/ttyACM0".into()), "/dev/ttyACM0");
    }

    #[test]
    fn wildcard() {
        assert!(wildcard_match("ttyACM*", "ttyACM0"));
        assert!(wildcard_match("*usb*", "cu.usbmodem1"));
        assert!(!wildcard_match("tty.*", "ttyS0"));
        assert!(wildcard_match("*", "anything"));
    }

    #[test]
    fn nice_name_skips_network_ports() {
        assert_eq!(SerialPort::get_nice_name("UDP"), "");
        assert_eq!(SerialPort::get_nice_name("TCP"), "");
    }

    #[test]
    fn facade_forwards_to_default_type() {
        let mut p = SerialPort::with_port_baud("COM42", 57600);
        assert_eq!(p.port_name(), "COM42");
        assert_eq!(p.baud_rate(), 57600);
        assert!(!p.is_open());
        assert!(p._baseport.as_ref().unwrap().as_any().downcast_ref::<WinSerialPort>().is_some());
        assert!(matches!(p.read(&mut [0u8; 1], 0, 1), Err(Exception::InvalidOperation(_))));
    }
}

//! Port of `ExtLibs/Comms/CommsInjection.cs`

use std::any::Any;
use std::sync::{Arc, Mutex, RwLock};

use super::circularbuffer::CircularBuffer;
use super::commsstream::{CommsStream, CommsStreamState};
use crate::dotnet::{ascii_get_bytes, ascii_get_string, Exception, Stream};
use crate::extlibs::interfaces::ICommsSerial;

/// `EventHandler<int>`
pub type ReadBufferUpdateHandler = Arc<dyn Fn(&CommsInjection, i32) + Send + Sync>;
/// `EventHandler<IEnumerable<byte>>`
pub type WriteCallbackHandler = Arc<dyn Fn(&CommsInjection, Vec<u8>) + Send + Sync>;

struct Props {
    baud_rate: i32,
    data_bits: i32,
    dtr_enable: bool,
    is_open: bool,
    port_name: String,
    read_buffer_size: i32,
    read_timeout: i32,
    rts_enable: bool,
    write_buffer_size: i32,
    write_timeout: i32,
}

struct Inner {
    _buffer_rx: Mutex<CircularBuffer<u8>>,
    base_stream: Arc<Mutex<CommsStreamState>>,
    read_buffer_update: RwLock<Option<ReadBufferUpdateHandler>>,
    write_callback: RwLock<Option<WriteCallbackHandler>>,
    props: Mutex<Props>,
}

/// `CommsInjection : ICommsSerial`: use `append_buffer` to populate the read buffer, and
/// `WriteCallback` to send. Clones share one object, as copies of a C# reference do.
#[derive(Clone)]
pub struct CommsInjection {
    inner: Arc<Inner>,
}

impl Default for CommsInjection {
    fn default() -> Self {
        Self::new()
    }
}

impl CommsInjection {
    pub fn new() -> Self {
        let c = CommsInjection {
            inner: Arc::new(Inner {
                _buffer_rx: Mutex::new(CircularBuffer::new(1024 * 100)),
                base_stream: Arc::new(Mutex::new(CommsStreamState::default())),
                read_buffer_update: RwLock::new(None),
                write_callback: RwLock::new(None),
                props: Mutex::new(Props {
                    baud_rate: 0,
                    data_bits: 0,
                    dtr_enable: false,
                    is_open: false,
                    port_name: String::new(),
                    read_buffer_size: 0,
                    read_timeout: 50,
                    rts_enable: false,
                    write_buffer_size: 0,
                    write_timeout: 0,
                }),
            }),
        };
        // BaseStream = new CommsStream(this, 0); Open();
        c.inner.base_stream.lock().unwrap_or_else(|e| e.into_inner()).len = 0;
        let mut o = c.clone();
        let _ = o.open();
        c
    }

    fn rx(&self) -> std::sync::MutexGuard<'_, CircularBuffer<u8>> {
        self.inner._buffer_rx.lock().unwrap_or_else(|e| e.into_inner())
    }

    fn props(&self) -> std::sync::MutexGuard<'_, Props> {
        self.inner.props.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// `AppendBuffer(byte[] indata)`
    pub fn append_buffer(&self, indata: &[u8]) {
        let mut rx = self.rx();
        for b in indata {
            rx.add(*b);
        }
        let mut st = self.inner.base_stream.lock().unwrap_or_else(|e| e.into_inner());
        st.len += indata.len() as i64;
    }

    /// `public EventHandler<int> ReadBufferUpdate` (assignment)
    pub fn set_read_buffer_update(&self, h: Option<ReadBufferUpdateHandler>) {
        *self.inner.read_buffer_update.write().unwrap_or_else(|e| e.into_inner()) = h;
    }

    /// `public EventHandler<IEnumerable<byte>> WriteCallback` (assignment)
    pub fn set_write_callback(&self, h: Option<WriteCallbackHandler>) {
        *self.inner.write_callback.write().unwrap_or_else(|e| e.into_inner()) = h;
    }

    /// `PortName { set; }` is public on this class.
    pub fn bytes_to_read_shared(&self) -> i32 {
        let h = self.inner.read_buffer_update.read().unwrap_or_else(|e| e.into_inner()).clone();
        if let Some(h) = h {
            h(self, 0);
        }
        self.rx().length()
    }
}

impl ICommsSerial for CommsInjection {
    fn base_stream(&mut self) -> Result<Box<dyn Stream + '_>, Exception> {
        let state = self.inner.base_stream.clone();
        Ok(Box::new(CommsStream::with_state(self, state)))
    }

    fn baud_rate(&self) -> i32 {
        self.props().baud_rate
    }
    fn set_baud_rate(&mut self, value: i32) {
        self.props().baud_rate = value;
    }

    fn bytes_to_read(&mut self) -> Result<i32, Exception> {
        Ok(self.bytes_to_read_shared())
    }

    fn bytes_to_write(&mut self) -> i32 {
        0
    }

    fn data_bits(&self) -> i32 {
        self.props().data_bits
    }
    fn set_data_bits(&mut self, value: i32) {
        self.props().data_bits = value;
    }

    fn dtr_enable(&self) -> bool {
        self.props().dtr_enable
    }
    fn set_dtr_enable(&mut self, value: bool) {
        self.props().dtr_enable = value;
    }

    fn is_open(&mut self) -> bool {
        self.props().is_open
    }

    fn port_name(&self) -> String {
        self.props().port_name.clone()
    }
    fn set_port_name(&mut self, value: &str) {
        self.props().port_name = value.to_string();
    }

    fn read_buffer_size(&self) -> i32 {
        self.props().read_buffer_size
    }
    fn set_read_buffer_size(&mut self, value: i32) {
        self.props().read_buffer_size = value;
    }

    fn read_timeout(&self) -> i32 {
        self.props().read_timeout
    }
    fn set_read_timeout(&mut self, value: i32) {
        self.props().read_timeout = value;
    }

    fn rts_enable(&self) -> bool {
        self.props().rts_enable
    }
    fn set_rts_enable(&mut self, value: bool) {
        self.props().rts_enable = value;
    }

    fn write_buffer_size(&self) -> i32 {
        self.props().write_buffer_size
    }
    fn set_write_buffer_size(&mut self, value: i32) {
        self.props().write_buffer_size = value;
    }

    fn write_timeout(&self) -> i32 {
        self.props().write_timeout
    }
    fn set_write_timeout(&mut self, value: i32) {
        self.props().write_timeout = value;
    }

    fn close(&mut self) -> Result<(), Exception> {
        self.rx().clear();
        self.props().is_open = false;
        Ok(())
    }

    fn discard_in_buffer(&mut self) -> Result<(), Exception> {
        self.rx().clear();
        Ok(())
    }

    fn open(&mut self) -> Result<(), Exception> {
        self.rx().clear();
        self.props().is_open = true;
        Ok(())
    }

    fn read(&mut self, buffer: &mut [u8], offset: i32, count: i32) -> Result<i32, Exception> {
        if !self.is_open() {
            return Err(Exception::Io("CommsInjection not open".into()));
        }

        let mut counttimeout = 0;
        while self.bytes_to_read_shared() == 0 {
            std::thread::sleep(std::time::Duration::from_millis(1));
            if counttimeout > self.read_timeout() {
                return Err(Exception::Timeout("CommsInjection Timeout on read".into()));
            }
            counttimeout += 1;
        }

        let mut rx = self.rx();
        let read = count.min(rx.length());
        for i in 0..read {
            buffer[(offset + i) as usize] = rx.read()?;
        }
        Ok(read)
    }

    fn read_byte(&mut self) -> Result<i32, Exception> {
        let mut buffer = [0u8; 1];
        self.read(&mut buffer, 0, 1)?;
        Ok(buffer[0] as i32)
    }

    fn read_char(&mut self) -> Result<i32, Exception> {
        self.read_byte()
    }

    /// Always "": the C# sizes its buffer as `new byte[0]`.
    fn read_existing(&mut self) -> Result<String, Exception> {
        let data: [u8; 0] = [];
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
            if self.bytes_to_read_shared() > 0 {
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

    /// `WriteCallback?.Invoke(this, buffer.Skip(offset).Take(count))`
    fn write(&mut self, buffer: &[u8], offset: i32, count: i32) -> Result<(), Exception> {
        let h = self.inner.write_callback.read().unwrap_or_else(|e| e.into_inner()).clone();
        if let Some(h) = h {
            let data: Vec<u8> = buffer.iter().skip(offset.max(0) as usize).take(count.max(0) as usize).copied().collect();
            h(self, data);
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
    fn append_read_and_callback() {
        let mut c = CommsInjection::new();
        assert!(c.is_open());
        let feeder = c.clone();
        feeder.append_buffer(b"ab\ncd");
        assert_eq!(c.base_stream().unwrap().length().unwrap(), 5);
        assert_eq!(c.read_line().unwrap(), "ab\n");
        let mut b = [0u8; 4];
        assert_eq!(c.read(&mut b, 1, 3).unwrap(), 2);
        assert_eq!(&b[1..3], b"cd");
        assert!(matches!(c.read(&mut b, 0, 1), Err(Exception::Timeout(_))));
        let sent = Arc::new(Mutex::new(vec![]));
        let s2 = sent.clone();
        c.set_write_callback(Some(Arc::new(move |_, d| s2.lock().unwrap().extend(d))));
        c.write(b"hello", 1, 3).unwrap();
        assert_eq!(*sent.lock().unwrap(), b"ell");
        assert_eq!(c.read_existing().unwrap(), "");
        c.close().unwrap();
        assert!(matches!(c.read(&mut b, 0, 1), Err(Exception::Io(_))));
    }
}

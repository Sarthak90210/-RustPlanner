//! MJPEG-over-HTTP video source for the HUD ("Set MJPEG source", `Utilities/CaptureMJPEG.cs`):
//! frames are split on JPEG SOI/EOI markers, decoded on a worker thread and shown behind the
//! HUD. "Record Hud to AVI" writes the received frames to a Motion-JPEG AVI.

use std::io::{Read, Seek, SeekFrom, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

pub struct Mjpeg {
    pub url: String,
    frame: Arc<Mutex<Option<egui::ColorImage>>>,
    stop: Arc<AtomicBool>,
    pub error: Arc<Mutex<Option<String>>>,
    recorder: Arc<Mutex<Option<AviWriter>>>,
    texture: Option<egui::TextureHandle>,
}

impl Mjpeg {
    pub fn start(url: &str, ctx: &egui::Context) -> Self {
        let frame = Arc::new(Mutex::new(None));
        let stop = Arc::new(AtomicBool::new(false));
        let error = Arc::new(Mutex::new(None));
        let recorder: Arc<Mutex<Option<AviWriter>>> = Arc::new(Mutex::new(None));
        let (f, s, e, r, c, u) = (frame.clone(), stop.clone(), error.clone(), recorder.clone(), ctx.clone(), url.to_string());
        std::thread::spawn(move || {
            let agent: ureq::Agent = ureq::Agent::config_builder().user_agent("MissionPlannerRS").build().into();
            let resp = match agent.get(&u).call() {
                Ok(r) => r,
                Err(err) => {
                    *e.lock().unwrap() = Some(err.to_string());
                    return;
                }
            };
            let mut reader = resp.into_body().into_reader();
            let mut buf: Vec<u8> = Vec::with_capacity(1 << 20);
            let mut chunk = [0u8; 16384];
            while !s.load(Ordering::Relaxed) {
                let n = match reader.read(&mut chunk) {
                    Ok(0) | Err(_) => break,
                    Ok(n) => n,
                };
                buf.extend_from_slice(&chunk[..n]);
                while let Some(start) = buf.windows(2).position(|w| w == [0xFF, 0xD8]) {
                    let Some(end) = buf[start + 2..].windows(2).position(|w| w == [0xFF, 0xD9]).map(|p| start + 2 + p + 2) else { break };
                    let jpeg = buf[start..end].to_vec();
                    buf.drain(..end);
                    if let Some(w) = r.lock().unwrap().as_mut() {
                        let _ = w.frame(&jpeg);
                    }
                    if let Ok(img) = image::load_from_memory(&jpeg) {
                        let img = img.to_rgba8();
                        *f.lock().unwrap() = Some(egui::ColorImage::from_rgba_unmultiplied([img.width() as usize, img.height() as usize], img.as_raw()));
                        c.request_repaint();
                    }
                }
                if buf.len() > 8 << 20 {
                    buf.clear();
                }
            }
        });
        Self { url: url.to_string(), frame, stop, error, recorder, texture: None }
    }

    /// The latest frame as a texture.
    pub fn texture(&mut self, ctx: &egui::Context) -> Option<egui::TextureId> {
        if let Some(img) = self.frame.lock().unwrap().take() {
            match &mut self.texture {
                Some(t) => t.set(img, egui::TextureOptions::LINEAR),
                None => self.texture = Some(ctx.load_texture("hud_video", img, egui::TextureOptions::LINEAR)),
            }
        }
        self.texture.as_ref().map(|t| t.id())
    }

    pub fn record(&self, path: Option<std::path::PathBuf>) -> std::io::Result<()> {
        let mut r = self.recorder.lock().unwrap();
        if let Some(w) = r.take() {
            w.finish()?;
        }
        if let Some(p) = path {
            *r = Some(AviWriter::create(&p)?);
        }
        Ok(())
    }

    pub fn recording(&self) -> bool {
        self.recorder.lock().unwrap().is_some()
    }
}

impl Drop for Mjpeg {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        let _ = self.record(None);
    }
}

/// Minimal RIFF AVI writer for MJPG frames (fixed 10 fps, size taken from the first frame).
pub struct AviWriter {
    f: std::fs::File,
    index: Vec<(u32, u32)>,
    movi_start: u64,
    size: (u32, u32),
}

fn jpeg_size(j: &[u8]) -> (u32, u32) {
    image::load_from_memory(j).map(|i| (i.width(), i.height())).unwrap_or((640, 480))
}

impl AviWriter {
    fn create(p: &std::path::Path) -> std::io::Result<Self> {
        let mut f = std::fs::File::create(p)?;
        f.write_all(&[0u8; 224])?; // header placeholder, rewritten by finish()
        f.write_all(b"LIST\0\0\0\0movi")?;
        Ok(Self { f, index: Vec::new(), movi_start: 224 + 8, size: (0, 0) })
    }

    fn frame(&mut self, jpeg: &[u8]) -> std::io::Result<()> {
        if self.size == (0, 0) {
            self.size = jpeg_size(jpeg);
        }
        let off = self.f.stream_position()? - self.movi_start;
        self.f.write_all(b"00dc")?;
        self.f.write_all(&(jpeg.len() as u32).to_le_bytes())?;
        self.f.write_all(jpeg)?;
        if jpeg.len() % 2 == 1 {
            self.f.write_all(&[0])?;
        }
        self.index.push((off as u32, jpeg.len() as u32));
        Ok(())
    }

    fn finish(mut self) -> std::io::Result<()> {
        let movi_end = self.f.stream_position()?;
        self.f.write_all(b"idx1")?;
        self.f.write_all(&((self.index.len() * 16) as u32).to_le_bytes())?;
        for (off, len) in &self.index {
            self.f.write_all(b"00dc")?;
            self.f.write_all(&0x10u32.to_le_bytes())?;
            self.f.write_all(&off.to_le_bytes())?;
            self.f.write_all(&len.to_le_bytes())?;
        }
        let total = self.f.stream_position()?;
        let (w, h) = self.size;
        let n = self.index.len() as u32;
        let le = |v: u32| v.to_le_bytes();
        let mut hd: Vec<u8> = Vec::new();
        hd.extend_from_slice(b"RIFF");
        hd.extend_from_slice(&le((total - 8) as u32));
        hd.extend_from_slice(b"AVI LIST");
        hd.extend_from_slice(&le(192));
        hd.extend_from_slice(b"hdrlavih");
        hd.extend_from_slice(&le(56));
        for v in [100_000u32, 0, 0, 0x10, n, 0, 1, 0, w, h, 0, 0, 0, 0] {
            hd.extend_from_slice(&le(v));
        }
        hd.extend_from_slice(b"LIST");
        hd.extend_from_slice(&le(116));
        hd.extend_from_slice(b"strlstrh");
        hd.extend_from_slice(&le(56));
        hd.extend_from_slice(b"vidsMJPG");
        for v in [0u32, 0, 0, 1, 10, 0, n, 0, 0xFFFF_FFFF, 0] {
            hd.extend_from_slice(&le(v));
        }
        hd.extend_from_slice(&[0u8; 8]);
        hd.extend_from_slice(b"strf");
        hd.extend_from_slice(&le(40));
        for v in [40u32, w, h] {
            hd.extend_from_slice(&le(v));
        }
        hd.extend_from_slice(&1u16.to_le_bytes());
        hd.extend_from_slice(&24u16.to_le_bytes());
        hd.extend_from_slice(b"MJPG");
        hd.extend_from_slice(&le(w * h * 3));
        hd.extend_from_slice(&[0u8; 16]);
        // pad the header out to the fixed 224 bytes with a JUNK chunk
        hd.extend_from_slice(b"JUNK");
        hd.extend_from_slice(&le((224 - hd.len() - 4) as u32));
        hd.resize(224, 0);
        self.f.seek(SeekFrom::Start(0))?;
        self.f.write_all(&hd)?;
        self.f.seek(SeekFrom::Start(224 + 4))?;
        self.f.write_all(&le((movi_end - 224 - 8) as u32))?;
        Ok(())
    }
}

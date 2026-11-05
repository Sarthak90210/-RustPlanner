//! Photo geotagging (`GeoRef/georefimage.cs`, `ExtLibs/Utilities/GeoRefImageBase.cs`):
//! read each photo's EXIF DateTimeOriginal, find the vehicle position at that time from a
//! telemetry or DataFlash log (time offset, or the CAM trigger messages in order), write
//! location.txt / location.kml, and insert EXIF GPS tags while keeping the existing tags.

use std::path::{Path, PathBuf};

/// A position fix: UTC seconds, lat, lng, alt (m), heading (deg).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Fix {
    pub t: f64,
    pub lat: f64,
    pub lng: f64,
    pub alt: f64,
    pub hdg: f64,
}

/// GPS time (week, ms of week) to UTC seconds (18 leap seconds).
pub fn gps_to_utc(week: f64, ms: f64) -> f64 {
    315_964_800.0 + week * 604_800.0 + ms / 1000.0 - 18.0
}

/// Days since 1970 for a civil date.
fn days_from_civil(y: i64, m: i64, d: i64) -> i64 {
    let y = if m <= 2 { y - 1 } else { y };
    let era = y.div_euclid(400);
    let yoe = y - era * 400;
    let mp = (m + 9) % 12;
    let doy = (153 * mp + 2) / 5 + d - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    era * 146_097 + doe - 719_468
}

/// "YYYY:MM:DD HH:MM:SS" (EXIF) to Unix seconds (camera clock, no zone).
pub fn exif_time(s: &str) -> Option<f64> {
    let s = s.trim_end_matches('\0').trim();
    let (d, t) = s.split_once(' ')?;
    let dv: Vec<i64> = d.split(':').filter_map(|x| x.parse().ok()).collect();
    let tv: Vec<i64> = t.split(':').filter_map(|x| x.parse().ok()).collect();
    if dv.len() != 3 || tv.len() != 3 {
        return None;
    }
    Some((days_from_civil(dv[0], dv[1], dv[2]) * 86400 + tv[0] * 3600 + tv[1] * 60 + tv[2]) as f64)
}

// ------------------------------------------------------------------ EXIF (TIFF) model

#[derive(Debug, Clone)]
struct Tag {
    id: u16,
    typ: u16,
    count: u32,
    /// value bytes, little endian
    data: Vec<u8>,
}

fn type_size(t: u16) -> usize {
    match t {
        1 | 2 | 6 | 7 => 1,
        3 | 8 => 2,
        4 | 9 | 11 => 4,
        5 | 10 | 12 => 8,
        _ => 1,
    }
}

struct Tiff<'a> {
    b: &'a [u8],
    le: bool,
}

impl Tiff<'_> {
    fn u16(&self, o: usize) -> Option<u16> {
        let v = self.b.get(o..o + 2)?;
        Some(if self.le { u16::from_le_bytes([v[0], v[1]]) } else { u16::from_be_bytes([v[0], v[1]]) })
    }
    fn u32(&self, o: usize) -> Option<u32> {
        let v = self.b.get(o..o + 4)?;
        let a = [v[0], v[1], v[2], v[3]];
        Some(if self.le { u32::from_le_bytes(a) } else { u32::from_be_bytes(a) })
    }
    /// Read one IFD; values are converted to little endian.
    fn ifd(&self, off: usize) -> Option<Vec<Tag>> {
        let n = self.u16(off)? as usize;
        let mut out = Vec::with_capacity(n);
        for k in 0..n {
            let e = off + 2 + k * 12;
            let id = self.u16(e)?;
            let typ = self.u16(e + 2)?;
            let count = self.u32(e + 4)?;
            let sz = type_size(typ);
            let len = sz * count as usize;
            let src = if len <= 4 { e + 8 } else { self.u32(e + 8)? as usize };
            let raw = self.b.get(src..src + len)?;
            let mut data = Vec::with_capacity(len);
            // byte-swap multi-byte components (rationals are two u32s)
            let comp = match sz {
                8 => 4,
                s => s,
            };
            for c in raw.chunks(comp) {
                if self.le || comp == 1 {
                    data.extend_from_slice(c);
                } else {
                    data.extend(c.iter().rev());
                }
            }
            out.push(Tag { id, typ, count, data });
        }
        Some(out)
    }
}

fn tag_u32(t: &Tag) -> Option<u32> {
    match t.typ {
        3 => Some(u16::from_le_bytes([*t.data.first()?, *t.data.get(1)?]) as u32),
        4 => Some(u32::from_le_bytes(t.data.get(..4)?.try_into().ok()?)),
        _ => None,
    }
}

/// The APP1 Exif segment of a JPEG: (segment start, segment end, TIFF bytes).
fn find_exif(j: &[u8]) -> Option<(usize, usize, &[u8])> {
    let mut i = 2;
    while i + 4 <= j.len() && j[i] == 0xFF {
        let marker = j[i + 1];
        let len = u16::from_be_bytes([j[i + 2], j[i + 3]]) as usize;
        if marker == 0xE1 && j.get(i + 4..i + 10) == Some(b"Exif\0\0") {
            return Some((i, i + 2 + len, &j[i + 10..i + 2 + len]));
        }
        if marker == 0xDA {
            break;
        }
        i += 2 + len;
    }
    None
}

/// EXIF DateTimeOriginal (0x9003) of a JPEG.
pub fn photo_time(jpeg: &[u8]) -> Option<f64> {
    let (_, _, t) = find_exif(jpeg)?;
    let tiff = Tiff { b: t, le: t.get(..2)? == b"II" };
    let ifd0 = tiff.ifd(tiff.u32(4)? as usize)?;
    let exif_off = ifd0.iter().find(|t| t.id == 0x8769).and_then(tag_u32)?;
    let exif = tiff.ifd(exif_off as usize)?;
    let dt = exif.iter().find(|t| t.id == 0x9003).or_else(|| ifd0.iter().find(|t| t.id == 0x0132))?;
    exif_time(&String::from_utf8_lossy(&dt.data))
}

fn rational(num: u32, den: u32) -> Vec<u8> {
    let mut v = num.to_le_bytes().to_vec();
    v.extend_from_slice(&den.to_le_bytes());
    v
}

fn dms(v: f64) -> Vec<u8> {
    let a = v.abs();
    let d = a.trunc();
    let m = ((a - d) * 60.0).trunc();
    let s = ((a - d) * 60.0 - m) * 60.0;
    let mut out = rational(d as u32, 1);
    out.extend(rational(m as u32, 1));
    out.extend(rational((s * 10000.0).round() as u32, 10000));
    out
}

fn write_ifd(out: &mut Vec<u8>, tags: &[Tag], next: u32) {
    // values that do not fit in 4 bytes go right after the entry table
    let start = out.len();
    let table = 2 + tags.len() * 12 + 4;
    let mut extra: Vec<u8> = Vec::new();
    out.extend_from_slice(&(tags.len() as u16).to_le_bytes());
    for t in tags {
        out.extend_from_slice(&t.id.to_le_bytes());
        out.extend_from_slice(&t.typ.to_le_bytes());
        out.extend_from_slice(&t.count.to_le_bytes());
        if t.data.len() <= 4 {
            let mut v = t.data.clone();
            v.resize(4, 0);
            out.extend_from_slice(&v);
        } else {
            let off = (start + table + extra.len()) as u32;
            out.extend_from_slice(&off.to_le_bytes());
            extra.extend_from_slice(&t.data);
            if extra.len() % 2 == 1 {
                extra.push(0);
            }
        }
    }
    out.extend_from_slice(&next.to_le_bytes());
    out.extend_from_slice(&extra);
}

fn ifd_size(tags: &[Tag]) -> usize {
    2 + tags.len() * 12 + 4 + tags.iter().filter(|t| t.data.len() > 4).map(|t| t.data.len() + t.data.len() % 2).sum::<usize>()
}

/// Insert (or replace) EXIF GPS tags, keeping IFD0 and the Exif IFD.
pub fn geotag(jpeg: &[u8], lat: f64, lng: f64, alt: f64) -> Option<Vec<u8>> {
    if jpeg.get(..2)? != [0xFF, 0xD8] {
        return None;
    }
    let (mut ifd0, mut exif, seg) = match find_exif(jpeg) {
        Some((a, b, t)) => {
            let tiff = Tiff { b: t, le: t.get(..2)? == b"II" };
            let ifd0 = tiff.ifd(tiff.u32(4)? as usize)?;
            let exif = ifd0.iter().find(|t| t.id == 0x8769).and_then(tag_u32).and_then(|o| tiff.ifd(o as usize)).unwrap_or_default();
            (ifd0, exif, Some((a, b)))
        }
        None => (Vec::new(), Vec::new(), None),
    };
    ifd0.retain(|t| !matches!(t.id, 0x8769 | 0x8825 | 0x0201 | 0x0202));
    exif.retain(|t| t.id != 0xA005);
    let ascii = |s: &str| s.as_bytes().iter().copied().chain(std::iter::once(0)).collect::<Vec<u8>>();
    let gps = vec![
        Tag { id: 0x0000, typ: 1, count: 4, data: vec![2, 3, 0, 0] },
        Tag { id: 0x0001, typ: 2, count: 2, data: ascii(if lat >= 0.0 { "N" } else { "S" }) },
        Tag { id: 0x0002, typ: 5, count: 3, data: dms(lat) },
        Tag { id: 0x0003, typ: 2, count: 2, data: ascii(if lng >= 0.0 { "E" } else { "W" }) },
        Tag { id: 0x0004, typ: 5, count: 3, data: dms(lng) },
        Tag { id: 0x0005, typ: 1, count: 1, data: vec![if alt < 0.0 { 1 } else { 0 }] },
        Tag { id: 0x0006, typ: 5, count: 1, data: rational((alt.abs() * 100.0).round() as u32, 100) },
    ];
    // layout: header 8 | IFD0 (+ 2 pointers) | Exif IFD | GPS IFD
    let has_exif = !exif.is_empty();
    let ptr = |id: u16, off: u32| Tag { id, typ: 4, count: 1, data: off.to_le_bytes().to_vec() };
    let mut probe = ifd0.clone();
    if has_exif {
        probe.push(ptr(0x8769, 0));
    }
    probe.push(ptr(0x8825, 0));
    let ifd0_len = ifd_size(&probe);
    let exif_off = 8 + ifd0_len;
    let gps_off = exif_off + if has_exif { ifd_size(&exif) } else { 0 };
    if has_exif {
        ifd0.push(ptr(0x8769, exif_off as u32));
    }
    ifd0.push(ptr(0x8825, gps_off as u32));
    ifd0.sort_by_key(|t| t.id);
    let mut tiff = b"II*\0".to_vec();
    tiff.extend_from_slice(&8u32.to_le_bytes());
    write_ifd(&mut tiff, &ifd0, 0);
    if has_exif {
        write_ifd(&mut tiff, &exif, 0);
    }
    write_ifd(&mut tiff, &gps, 0);
    let mut app1 = vec![0xFF, 0xE1];
    app1.extend_from_slice(&((tiff.len() + 8) as u16).to_be_bytes());
    app1.extend_from_slice(b"Exif\0\0");
    app1.extend_from_slice(&tiff);
    let mut out = Vec::with_capacity(jpeg.len() + app1.len());
    match seg {
        Some((a, b)) => {
            out.extend_from_slice(&jpeg[..a]);
            out.extend_from_slice(&app1);
            out.extend_from_slice(&jpeg[b..]);
        }
        None => {
            out.extend_from_slice(&jpeg[..2]);
            out.extend_from_slice(&app1);
            out.extend_from_slice(&jpeg[2..]);
        }
    }
    Some(out)
}

/// Read back the GPS position written by [`geotag`].
pub fn photo_gps(jpeg: &[u8]) -> Option<(f64, f64, f64)> {
    let (_, _, t) = find_exif(jpeg)?;
    let tiff = Tiff { b: t, le: t.get(..2)? == b"II" };
    let ifd0 = tiff.ifd(tiff.u32(4)? as usize)?;
    let g = tiff.ifd(ifd0.iter().find(|t| t.id == 0x8825).and_then(tag_u32)? as usize)?;
    let rat = |d: &[u8], k: usize| -> f64 {
        let n = u32::from_le_bytes(d[k * 8..k * 8 + 4].try_into().unwrap()) as f64;
        let m = u32::from_le_bytes(d[k * 8 + 4..k * 8 + 8].try_into().unwrap()) as f64;
        if m == 0.0 { 0.0 } else { n / m }
    };
    let deg = |id: u16, refid: u16, neg: u8| -> Option<f64> {
        let d = &g.iter().find(|t| t.id == id)?.data;
        let v = rat(d, 0) + rat(d, 1) / 60.0 + rat(d, 2) / 3600.0;
        let r = g.iter().find(|t| t.id == refid).and_then(|t| t.data.first().copied()).unwrap_or(0);
        Some(if r == neg { -v } else { v })
    };
    let lat = deg(2, 1, b'S')?;
    let lng = deg(4, 3, b'W')?;
    let alt = g.iter().find(|t| t.id == 6).map(|t| rat(&t.data, 0)).unwrap_or(0.0);
    Some((lat, lng, alt))
}

// ------------------------------------------------------------------ logs

/// Position fixes (UTC) and CAM trigger positions from a .tlog or .bin.
pub fn read_log(path: &Path) -> Result<(Vec<Fix>, Vec<Fix>), String> {
    let ext = path.extension().map(|e| e.to_string_lossy().to_lowercase()).unwrap_or_default();
    if ext == "tlog" {
        use mavlink::dialects::ardupilotmega::MavMessage;
        let recs = crate::tlog::read(path).map_err(|e| e.to_string())?;
        let mut fixes = Vec::new();
        let mut cams = Vec::new();
        for r in recs {
            match &r.msg {
                MavMessage::GLOBAL_POSITION_INT(g) if g.lat != 0 => fixes.push(Fix { t: r.time_us as f64 / 1e6, lat: g.lat as f64 * 1e-7, lng: g.lon as f64 * 1e-7, alt: g.alt as f64 / 1000.0, hdg: g.hdg as f64 / 100.0 }),
                MavMessage::CAMERA_FEEDBACK(c) => cams.push(Fix { t: c.time_usec as f64 / 1e6, lat: c.lat as f64 * 1e-7, lng: c.lng as f64 * 1e-7, alt: c.alt_msl as f64, hdg: c.yaw as f64 }),
                _ => {}
            }
        }
        return Ok((fixes, cams));
    }
    let data = std::fs::read(path).map_err(|e| e.to_string())?;
    let log = crate::dataflash::parse(&data);
    let pick = |name: &str, time: (&str, &str)| -> Vec<Fix> {
        let Some(t) = log.type_by_name(name) else { return Vec::new() };
        let f = &log.formats[&t];
        let idx = |n: &str| f.columns.iter().position(|c| c == n);
        let (Some(la), Some(lo)) = (idx("Lat"), idx("Lng")) else { return Vec::new() };
        let (al, yaw) = (idx("Alt"), idx("Yaw").or(idx("GCrs")));
        let (wk, ms) = (idx(time.0), idx(time.1));
        log.records
            .iter()
            .filter(|r| r.type_id == t)
            .filter_map(|r| {
                let v = |i: Option<usize>| i.and_then(|i| r.values.get(i)?.as_f64());
                let t = gps_to_utc(v(wk)?, v(ms)?);
                Some(Fix { t, lat: v(Some(la))?, lng: v(Some(lo))?, alt: v(al).unwrap_or(0.0), hdg: v(yaw).unwrap_or(0.0) })
            })
            .filter(|f| f.lat != 0.0 || f.lng != 0.0)
            .collect()
    };
    Ok((pick("GPS", ("GWk", "GMS")), pick("CAM", ("GPSWeek", "GPSTime"))))
}

/// Position at time `t` (linear interpolation between fixes).
pub fn position_at(fixes: &[Fix], t: f64) -> Option<Fix> {
    let i = fixes.partition_point(|f| f.t < t);
    if i == 0 || i >= fixes.len() {
        return None;
    }
    let (a, b) = (fixes[i - 1], fixes[i]);
    let k = if b.t > a.t { (t - a.t) / (b.t - a.t) } else { 0.0 };
    Some(Fix { t, lat: a.lat + (b.lat - a.lat) * k, lng: a.lng + (b.lng - a.lng) * k, alt: a.alt + (b.alt - a.alt) * k, hdg: a.hdg })
}

/// One matched photo.
#[derive(Debug, Clone)]
pub struct Match {
    pub file: PathBuf,
    pub fix: Fix,
}

pub fn photos(dir: &Path) -> Vec<(PathBuf, Option<f64>)> {
    let mut v: Vec<(PathBuf, Option<f64>)> = std::fs::read_dir(dir)
        .map(|rd| {
            rd.flatten()
                .map(|e| e.path())
                .filter(|p| p.extension().map(|e| matches!(e.to_string_lossy().to_lowercase().as_str(), "jpg" | "jpeg")).unwrap_or(false))
                .map(|p| {
                    let t = std::fs::read(&p).ok().and_then(|b| photo_time(&b));
                    (p, t)
                })
                .collect()
        })
        .unwrap_or_default();
    v.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal).then(a.0.cmp(&b.0)));
    v
}

/// Time-offset mode: photo time + offset (s) -> log time.
pub fn match_by_offset(photos: &[(PathBuf, Option<f64>)], fixes: &[Fix], offset: f64) -> Vec<Match> {
    photos.iter().filter_map(|(p, t)| Some(Match { file: p.clone(), fix: position_at(fixes, (*t)? + offset)? })).collect()
}

/// CAM message mode: the n-th photo is the n-th camera trigger.
pub fn match_by_cam(photos: &[(PathBuf, Option<f64>)], cams: &[Fix], drop_start: usize, drop_end: usize) -> Vec<Match> {
    let ph: Vec<&PathBuf> = photos.iter().map(|(p, _)| p).collect();
    let ph = &ph[drop_start.min(ph.len())..ph.len().saturating_sub(drop_end).max(drop_start.min(ph.len()))];
    ph.iter().zip(cams.iter()).map(|(p, c)| Match { file: (*p).clone(), fix: *c }).collect()
}

/// "Estimate Offset": first CAM trigger (or first fix) minus the first photo time.
pub fn estimate_offset(photos: &[(PathBuf, Option<f64>)], fixes: &[Fix], cams: &[Fix]) -> Option<f64> {
    let p0 = photos.iter().find_map(|(_, t)| *t)?;
    let l0 = cams.first().or(fixes.first())?.t;
    Some(l0 - p0)
}

pub fn location_txt(m: &[Match]) -> String {
    let mut s = String::from("#name latitude/Y longitude/X height/Z yaw pitch roll\r\n");
    for x in m {
        s.push_str(&format!("{} {} {} {:.2} {:.1} 0 0\r\n", x.file.file_name().map(|s| s.to_string_lossy().to_string()).unwrap_or_default(), x.fix.lat, x.fix.lng, x.fix.alt, x.fix.hdg));
    }
    s
}

pub fn location_kml(m: &[Match]) -> String {
    let mut s = String::from("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<kml xmlns=\"http://www.opengis.net/kml/2.2\"><Document><name>Geo-referenced images</name>\n");
    for x in m {
        let n = x.file.file_name().map(|s| s.to_string_lossy().to_string()).unwrap_or_default();
        s.push_str(&format!("<Placemark><name>{n}</name><description><![CDATA[<img src=\"{n}\" width=\"400\"/>]]></description><Point><altitudeMode>absolute</altitudeMode><coordinates>{},{},{}</coordinates></Point></Placemark>\n", x.fix.lng, x.fix.lat, x.fix.alt));
    }
    s.push_str("</Document></kml>\n");
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tiny_jpeg_with_time() -> Vec<u8> {
        // IFD0 -> Exif IFD with DateTimeOriginal, plus Make in IFD0 to check it survives
        let mut t: Vec<u8> = b"II*\0".to_vec();
        t.extend_from_slice(&8u32.to_le_bytes());
        // IFD0 at 8: 2 entries
        t.extend_from_slice(&2u16.to_le_bytes());
        t.extend_from_slice(&[0x0F, 0x01, 2, 0]);
        t.extend_from_slice(&4u32.to_le_bytes());
        t.extend_from_slice(b"Cam\0");
        t.extend_from_slice(&[0x69, 0x87, 4, 0]);
        t.extend_from_slice(&1u32.to_le_bytes());
        t.extend_from_slice(&38u32.to_le_bytes());
        t.extend_from_slice(&0u32.to_le_bytes());
        // Exif IFD at 38: DateTimeOriginal
        t.extend_from_slice(&1u16.to_le_bytes());
        t.extend_from_slice(&[0x03, 0x90, 2, 0]);
        t.extend_from_slice(&20u32.to_le_bytes());
        t.extend_from_slice(&56u32.to_le_bytes());
        t.extend_from_slice(&0u32.to_le_bytes());
        t.extend_from_slice(b"2024:05:06 07:08:09\0");
        let mut j = vec![0xFF, 0xD8, 0xFF, 0xE1];
        j.extend_from_slice(&((t.len() + 8) as u16).to_be_bytes());
        j.extend_from_slice(b"Exif\0\0");
        j.extend_from_slice(&t);
        j.extend_from_slice(&[0xFF, 0xDA, 0, 2, 0xFF, 0xD9]);
        j
    }

    #[test]
    fn exif_time_and_geotag_roundtrip() {
        let j = tiny_jpeg_with_time();
        let t = photo_time(&j).unwrap();
        assert_eq!(t, exif_time("2024:05:06 07:08:09").unwrap());
        assert_eq!(exif_time("1970:01:02 00:00:00"), Some(86400.0));
        let g = geotag(&j, -35.363261, 149.16523, 584.25).unwrap();
        let (la, lo, al) = photo_gps(&g).unwrap();
        assert!((la + 35.363261).abs() < 1e-6 && (lo - 149.16523).abs() < 1e-6 && (al - 584.25).abs() < 0.01);
        // the original DateTimeOriginal survived the rewrite
        assert_eq!(photo_time(&g), Some(t));
    }

    #[test]
    fn interpolate_and_offset() {
        let fixes = vec![Fix { t: 100.0, lat: 1.0, lng: 2.0, alt: 10.0, hdg: 0.0 }, Fix { t: 110.0, lat: 2.0, lng: 4.0, alt: 20.0, hdg: 0.0 }];
        let p = position_at(&fixes, 105.0).unwrap();
        assert_eq!((p.lat, p.lng, p.alt), (1.5, 3.0, 15.0));
        let photos = vec![(PathBuf::from("a.jpg"), Some(95.0))];
        assert_eq!(estimate_offset(&photos, &fixes, &[]), Some(5.0));
        assert_eq!(match_by_offset(&photos, &fixes, 10.0)[0].fix.lat, 1.5);
        assert!((gps_to_utc(2000.0, 0.0) - 1_525_564_782.0).abs() < 1.0);
    }
}

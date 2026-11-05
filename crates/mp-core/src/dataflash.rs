//! ArduPilot DataFlash `.bin` logs: self-describing via FMT records (port of
//! `ExtLibs/Utilities/DFLog*.cs`). Each record is `A3 95 <type>` followed by the fields
//! described by that type's FMT.

use std::collections::BTreeMap;

#[derive(Debug, Clone)]
pub struct Format {
    pub id: u8,
    pub len: usize,
    pub name: String,
    pub format: String,
    pub columns: Vec<String>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Value {
    Int(i64),
    Float(f64),
    Text(String),
}

impl Value {
    pub fn as_f64(&self) -> Option<f64> {
        match self {
            Value::Int(i) => Some(*i as f64),
            Value::Float(f) => Some(*f),
            Value::Text(_) => None,
        }
    }
}

impl std::fmt::Display for Value {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Value::Int(i) => write!(f, "{i}"),
            Value::Float(v) => write!(f, "{v}"),
            Value::Text(s) => write!(f, "{s}"),
        }
    }
}

#[derive(Debug, Clone)]
pub struct Record {
    pub type_id: u8,
    pub values: Vec<Value>,
}

#[derive(Debug, Default)]
pub struct Log {
    pub formats: BTreeMap<u8, Format>,
    pub records: Vec<Record>,
}

fn field_size(c: char) -> usize {
    match c {
        'b' | 'B' | 'M' => 1,
        'h' | 'H' | 'c' | 'C' => 2,
        'i' | 'I' | 'f' | 'e' | 'E' | 'L' | 'n' => match c {
            'n' => 4,
            _ => 4,
        },
        'd' | 'q' | 'Q' => 8,
        'N' => 16,
        'Z' => 64,
        'a' => 64,
        _ => 0,
    }
}

fn text(b: &[u8]) -> String {
    let end = b.iter().position(|&c| c == 0).unwrap_or(b.len());
    String::from_utf8_lossy(&b[..end]).trim().to_string()
}

fn decode(fmt: &Format, data: &[u8]) -> Vec<Value> {
    let mut out = Vec::with_capacity(fmt.format.len());
    let mut o = 0usize;
    for c in fmt.format.chars() {
        let n = field_size(c);
        if o + n > data.len() {
            break;
        }
        let d = &data[o..o + n];
        let v = match c {
            'b' => Value::Int(d[0] as i8 as i64),
            'B' | 'M' => Value::Int(d[0] as i64),
            'h' => Value::Int(i16::from_le_bytes([d[0], d[1]]) as i64),
            'H' => Value::Int(u16::from_le_bytes([d[0], d[1]]) as i64),
            'c' => Value::Float(i16::from_le_bytes([d[0], d[1]]) as f64 / 100.0),
            'C' => Value::Float(u16::from_le_bytes([d[0], d[1]]) as f64 / 100.0),
            'i' => Value::Int(i32::from_le_bytes(d.try_into().unwrap()) as i64),
            'I' => Value::Int(u32::from_le_bytes(d.try_into().unwrap()) as i64),
            'e' => Value::Float(i32::from_le_bytes(d.try_into().unwrap()) as f64 / 100.0),
            'E' => Value::Float(u32::from_le_bytes(d.try_into().unwrap()) as f64 / 100.0),
            'L' => Value::Float(i32::from_le_bytes(d.try_into().unwrap()) as f64 / 1e7),
            'f' => Value::Float(f32::from_le_bytes(d.try_into().unwrap()) as f64),
            'd' => Value::Float(f64::from_le_bytes(d.try_into().unwrap())),
            'q' => Value::Int(i64::from_le_bytes(d.try_into().unwrap())),
            'Q' => Value::Int(u64::from_le_bytes(d.try_into().unwrap()) as i64),
            'n' | 'N' | 'Z' => Value::Text(text(d)),
            'a' => Value::Text(format!("{:?}", d.chunks(2).map(|x| i16::from_le_bytes([x[0], x[1]])).collect::<Vec<_>>())),
            _ => Value::Int(0),
        };
        out.push(v);
        o += n;
    }
    out
}

/// Parse a DataFlash binary log.
pub fn parse(data: &[u8]) -> Log {
    let mut log = Log::default();
    // FMT is always type 128 with a fixed layout: BBnNZ (type, length, name, format, columns)
    log.formats.insert(128, Format { id: 128, len: 89, name: "FMT".into(), format: "BBnNZ".into(), columns: ["Type", "Length", "Name", "Format", "Columns"].iter().map(|s| s.to_string()).collect() });
    let mut i = 0usize;
    while i + 3 <= data.len() {
        if data[i] != 0xA3 || data[i + 1] != 0x95 {
            i += 1;
            continue;
        }
        let t = data[i + 2];
        let Some(fmt) = log.formats.get(&t).cloned() else {
            i += 1;
            continue;
        };
        if fmt.len < 3 || i + fmt.len > data.len() {
            break;
        }
        let body = &data[i + 3..i + fmt.len];
        let values = decode(&fmt, body);
        if t == 128 {
            if let (Some(Value::Int(id)), Some(Value::Int(len)), Some(Value::Text(name)), Some(Value::Text(format)), Some(Value::Text(cols))) =
                (values.first(), values.get(1), values.get(2), values.get(3), values.get(4))
            {
                log.formats.insert(
                    *id as u8,
                    Format { id: *id as u8, len: *len as usize, name: name.clone(), format: format.clone(), columns: cols.split(',').map(|s| s.to_string()).collect() },
                );
            }
        }
        log.records.push(Record { type_id: t, values });
        i += fmt.len;
    }
    log
}

impl Log {
    pub fn name(&self, t: u8) -> &str {
        self.formats.get(&t).map(|f| f.name.as_str()).unwrap_or("?")
    }

    pub fn type_by_name(&self, name: &str) -> Option<u8> {
        self.formats.values().find(|f| f.name == name).map(|f| f.id)
    }

    /// Message name -> column names, for the LogBrowse tree.
    pub fn tree(&self) -> Vec<(String, Vec<String>)> {
        let present: std::collections::BTreeSet<u8> = self.records.iter().map(|r| r.type_id).collect();
        let mut v: Vec<(String, Vec<String>)> = self.formats.values().filter(|f| present.contains(&f.id) && f.id != 128).map(|f| (f.name.clone(), f.columns.clone())).collect();
        v.sort();
        v
    }

    /// (time seconds, value) series for `MSG.Field`, using TimeUS / TimeMS when present.
    pub fn series(&self, msg: &str, field: &str) -> Vec<(f64, f64)> {
        let Some(t) = self.type_by_name(msg) else { return Vec::new() };
        let f = &self.formats[&t];
        let Some(col) = f.columns.iter().position(|c| c == field) else { return Vec::new() };
        let tcol = f.columns.iter().position(|c| c == "TimeUS").map(|c| (c, 1e-6)).or(f.columns.iter().position(|c| c == "TimeMS").map(|c| (c, 1e-3)));
        let mut out = Vec::new();
        for (k, r) in self.records.iter().filter(|r| r.type_id == t).enumerate() {
            let Some(v) = r.values.get(col).and_then(|v| v.as_f64()) else { continue };
            let time = tcol.and_then(|(c, s)| r.values.get(c).and_then(|v| v.as_f64()).map(|x| x * s)).unwrap_or(k as f64);
            out.push((time, v));
        }
        out
    }

    /// GPS track (lat, lng, alt) for the map / KML.
    pub fn gps_track(&self) -> Vec<(f64, f64, f64)> {
        let Some(t) = self.type_by_name("GPS").or(self.type_by_name("POS")) else { return Vec::new() };
        let f = &self.formats[&t];
        let idx = |n: &str| f.columns.iter().position(|c| c == n);
        let (Some(la), Some(lo)) = (idx("Lat"), idx("Lng")) else { return Vec::new() };
        let al = idx("Alt");
        self.records
            .iter()
            .filter(|r| r.type_id == t)
            .filter_map(|r| {
                let lat = r.values.get(la)?.as_f64()?;
                let lng = r.values.get(lo)?.as_f64()?;
                let alt = al.and_then(|a| r.values.get(a)?.as_f64()).unwrap_or(0.0);
                (lat != 0.0 || lng != 0.0).then_some((lat, lng, alt))
            })
            .collect()
    }

    /// `.bin to .log`: the text form ("NAME, v1, v2, ...").
    pub fn to_text(&self) -> String {
        let mut s = String::new();
        for r in &self.records {
            s.push_str(self.name(r.type_id));
            for v in &r.values {
                s.push_str(", ");
                s.push_str(&v.to_string());
            }
            s.push('\n');
        }
        s
    }

    /// "Create KML + gpx": track as KML and GPX documents.
    pub fn to_kml_gpx(&self, name: &str) -> (String, String) {
        let track = self.gps_track();
        let coords: String = track.iter().map(|(la, lo, al)| format!("{lo},{la},{al} ")).collect();
        let kml = format!(
            "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<kml xmlns=\"http://www.opengis.net/kml/2.2\"><Document><name>{name}</name><Placemark><name>Flight</name><Style><LineStyle><color>ff00ffff</color><width>3</width></LineStyle></Style><LineString><altitudeMode>absolute</altitudeMode><coordinates>{coords}</coordinates></LineString></Placemark></Document></kml>\n"
        );
        let pts: String = track.iter().map(|(la, lo, al)| format!("<trkpt lat=\"{la}\" lon=\"{lo}\"><ele>{al}</ele></trkpt>")).collect();
        let gpx = format!("<?xml version=\"1.0\"?>\n<gpx version=\"1.1\" creator=\"Mission Planner\"><trk><name>{name}</name><trkseg>{pts}</trkseg></trk></gpx>\n");
        (kml, gpx)
    }
}

impl Log {
    /// "Create Matlab File": MAT v4, one double matrix per message type (rows = records).
    pub fn to_mat_v4(&self) -> Vec<u8> {
        let mut out = Vec::new();
        for f in self.formats.values().filter(|f| f.id != 128) {
            let rows: Vec<&Record> = self.records.iter().filter(|r| r.type_id == f.id).collect();
            if rows.is_empty() {
                continue;
            }
            let numeric: Vec<usize> = (0..f.columns.len()).filter(|c| rows[0].values.get(*c).and_then(|v| v.as_f64()).is_some()).collect();
            if numeric.is_empty() {
                continue;
            }
            let name = format!("{} ", f.name);
            for v in [0i32, rows.len() as i32, numeric.len() as i32, 0, name.len() as i32] {
                out.extend_from_slice(&v.to_le_bytes());
            }
            out.extend_from_slice(name.as_bytes());
            // column-major doubles
            for c in &numeric {
                for r in &rows {
                    out.extend_from_slice(&r.values.get(*c).and_then(|v| v.as_f64()).unwrap_or(0.0).to_le_bytes());
                }
            }
        }
        out
    }

    /// "Auto Analysis": a few of LogAnalyzer's checks.
    pub fn analysis(&self) -> Vec<(String, String, String)> {
        let mut out = Vec::new();
        let max = |m: &str, f: &str| self.series(m, f).iter().map(|p| p.1).fold(f64::MIN, f64::max);
        let min = |m: &str, f: &str| self.series(m, f).iter().map(|p| p.1).fold(f64::MAX, f64::min);
        let vibe = ["VibeX", "VibeY", "VibeZ"].iter().map(|c| max("VIBE", c)).fold(f64::MIN, f64::max);
        if vibe > f64::MIN {
            out.push(("Vibration".into(), if vibe > 60.0 { "FAIL" } else if vibe > 30.0 { "WARN" } else { "GOOD" }.into(), format!("max {vibe:.1} m/s/s")));
        } else {
            out.push(("Vibration".into(), "NA".into(), "No VIBE data".into()));
        }
        let hdop = max("GPS", "HDop");
        let sats = min("GPS", "NSats");
        if hdop > f64::MIN {
            out.push(("GPS".into(), if hdop > 2.0 || sats < 6.0 { "WARN" } else { "GOOD" }.into(), format!("max HDop {hdop:.2}, min sats {sats:.0}")));
        } else if !self.gps_track().is_empty() {
            out.push(("GPS".into(), "GOOD".into(), format!("{} fixes", self.gps_track().len())));
        }
        let v = min("BAT", "Volt");
        if v < f64::MAX {
            out.push(("Battery".into(), "INFO".into(), format!("min {v:.2} V")));
        }
        let errs = self.type_by_name("ERR").map(|t| self.records.iter().filter(|r| r.type_id == t).count()).unwrap_or(0);
        out.push(("Event/Failsafe".into(), if errs > 0 { "FAIL" } else { "GOOD" }.into(), format!("{errs} ERR records")));
        out.push(("Log".into(), "INFO".into(), format!("{} records, {} message types", self.records.len(), self.formats.len())));
        out
    }
}

/// Build a minimal valid log (FMT + a few GPS/ATT records); used by tests and the simulator.
pub fn sample_log() -> Vec<u8> {
    fn fmt_rec(id: u8, len: u8, name: &str, format: &str, cols: &str) -> Vec<u8> {
        let mut v = vec![0xA3, 0x95, 128, id, len];
        let mut n = [0u8; 4];
        n[..name.len()].copy_from_slice(name.as_bytes());
        v.extend_from_slice(&n);
        let mut f = [0u8; 16];
        f[..format.len()].copy_from_slice(format.as_bytes());
        v.extend_from_slice(&f);
        let mut c = [0u8; 64];
        c[..cols.len()].copy_from_slice(cols.as_bytes());
        v.extend_from_slice(&c);
        v
    }
    let mut out = Vec::new();
    out.extend(fmt_rec(130, 3 + 8 + 4 + 4 + 4, "GPS", "QLLf", "TimeUS,Lat,Lng,Alt"));
    out.extend(fmt_rec(131, 3 + 8 + 4 + 4, "ATT", "Qff", "TimeUS,Roll,Pitch"));
    for k in 0..200u64 {
        let t = k * 100_000;
        let mut g = vec![0xA3, 0x95, 130];
        g.extend_from_slice(&t.to_le_bytes());
        g.extend_from_slice(&(((-35.363261 + k as f64 * 1e-5) * 1e7) as i32).to_le_bytes());
        g.extend_from_slice(&(((149.16523 + k as f64 * 1e-5) * 1e7) as i32).to_le_bytes());
        g.extend_from_slice(&(584.0f32 + k as f32 * 0.1).to_le_bytes());
        out.extend(g);
        let mut a = vec![0xA3, 0x95, 131];
        a.extend_from_slice(&t.to_le_bytes());
        a.extend_from_slice(&((k as f32 * 0.1).sin() * 20.0).to_le_bytes());
        a.extend_from_slice(&((k as f32 * 0.07).cos() * 10.0).to_le_bytes());
        out.extend(a);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_sample() {
        let log = parse(&sample_log());
        assert!(log.tree().iter().any(|(n, c)| n == "GPS" && c.contains(&"Lat".to_string())));
        let s = log.series("ATT", "Roll");
        assert_eq!(s.len(), 200);
        assert!((s[10].0 - 1.0).abs() < 1e-9);
        let trk = log.gps_track();
        assert_eq!(trk.len(), 200);
        assert!((trk[0].0 + 35.363261).abs() < 1e-6);
        assert!(log.to_text().starts_with("FMT, 130"));
        let (kml, gpx) = log.to_kml_gpx("t");
        assert!(kml.contains("<coordinates>149.16523"));
        assert!(gpx.contains("trkpt"));
    }
}

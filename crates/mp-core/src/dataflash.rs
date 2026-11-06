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
            if !v.is_finite() { continue; }
            let time = tcol.and_then(|(c, s)| r.values.get(c).and_then(|v| v.as_f64()).map(|x| x * s)).unwrap_or(k as f64);
            if !time.is_finite() { continue; }
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

    /// Partial native log checks. The reference launches an external LogAnalyzer runner;
    /// these heuristics do not implement that runner's complete vehicle-specific rules.
    pub fn analysis(&self) -> Vec<(String, String, String)> {
        let mut out = Vec::new();
        let max = |m: &str, f: &str| self.series(m, f).iter().map(|p| p.1).fold(f64::MIN, f64::max);
        let min = |m: &str, f: &str| self.series(m, f).iter().map(|p| p.1).fold(f64::MAX, f64::min);

        // 1. Log Metadata & Duration
        let (t_min, t_max) = self
            .records
            .iter()
            .flat_map(|r| {
                if let Some(fmt) = self.formats.get(&r.type_id) {
                    if let Some(pos) = fmt.columns.iter().position(|c| c == "TimeUS") {
                        return r.values.get(pos).and_then(|v| v.as_f64()).map(|x| x * 1e-6);
                    }
                    if let Some(pos) = fmt.columns.iter().position(|c| c == "TimeMS") {
                        return r.values.get(pos).and_then(|v| v.as_f64()).map(|x| x * 1e-3);
                    }
                }
                None
            })
            .fold((f64::MAX, f64::MIN), |(a, b), t| (a.min(t), b.max(t)));
        let dur_str = if t_min < t_max {
            let s = (t_max - t_min).round() as u64;
            format!("{}m {}s", s / 60, s % 60)
        } else {
            "unknown".into()
        };
        out.push(("Log Summary".into(), "INFO".into(), format!("{} records across {} types, duration {}", self.records.len(), self.formats.len(), dur_str)));

        // 2. Brownout & VCC Power Rail Check
        let vcc = {
            let p_vcc = min("POWR", "VCC");
            if p_vcc < f64::MAX {
                p_vcc
            } else {
                min("CURR", "Vcc")
            }
        };
        if (0.0..f64::MAX).contains(&vcc) {
            if vcc < 4.5 {
                out.push(("Brownout / Power".into(), "FAIL".into(), format!("Severe brownout risk: VCC dropped to {vcc:.2} V (<4.5 V critical)")));
            } else if vcc < 4.8 {
                out.push(("Brownout / Power".into(), "WARN".into(), format!("VCC sag detected: minimum {vcc:.2} V (<4.8 V nominal)")));
            } else {
                out.push(("Brownout / Power".into(), "GOOD".into(), format!("VCC power rail clean: minimum {vcc:.2} V")));
            }
        } else {
            out.push(("Brownout / Power".into(), "NA".into(), "No POWR/CURR VCC stream in log".into()));
        }

        // 3. IMU Vibration
        let vibe = ["VibeX", "VibeY", "VibeZ"].iter().map(|c| max("VIBE", c)).fold(f64::MIN, f64::max);
        if vibe.is_finite() && vibe >= 0.0 {
            if vibe > 60.0 {
                out.push(("Vibration".into(), "FAIL".into(), format!("Max vibration {vibe:.1} m/s/s exceeds 60 m/s/s heuristic threshold")));
            } else if vibe > 30.0 {
                out.push(("Vibration".into(), "WARN".into(), format!("Elevated vibration: max {vibe:.1} m/s/s (>30 m/s/s threshold)")));
            } else {
                out.push(("Vibration".into(), "GOOD".into(), format!("Max vibration {vibe:.1} m/s/s (well below 30 m/s/s)")));
            }
        } else {
            out.push(("Vibration".into(), "NA".into(), "No VIBE telemetry logged".into()));
        }

        // 4. Accelerometer Clipping
        // Counters are cumulative since boot. Count increases within the log,
        // including after resets, rather than blaming pre-log clipping on this flight.
        let clips = ["Clip0", "Clip1", "Clip2"].iter().filter_map(|c| {
            let s = self.series("VIBE", c);
            (s.len() >= 2).then(|| s.windows(2).map(|p| if p[1].1 >= p[0].1 { p[1].1 - p[0].1 } else { p[1].1.max(0.0) }).sum::<f64>())
        }).fold(f64::MIN, f64::max);
        if clips.is_finite() && clips >= 0.0 {
            if clips > 100.0 {
                out.push(("IMU Clipping".into(), "FAIL".into(), format!("Severe sensor clipping: max {clips:.0} clipping events recorded")));
            } else if clips > 0.0 {
                out.push(("IMU Clipping".into(), "WARN".into(), format!("Minor clipping: {clips:.0} clipping events recorded")));
            } else {
                out.push(("IMU Clipping".into(), "GOOD".into(), "Zero accelerometer clipping events".into()));
            }
        } else {
            out.push(("IMU Clipping".into(), "NA".into(), "No clipping telemetry logged".into()));
        }

        // 5. Compass Magnetic Interference Check
        let mag_x = self.series("MAG", "MagX");
        let mag_y = self.series("MAG", "MagY");
        let mag_z = self.series("MAG", "MagZ");
        if !mag_x.is_empty() && mag_x.len() == mag_y.len() && mag_y.len() == mag_z.len() {
            let mut mags = Vec::with_capacity(mag_x.len());
            for i in 0..mag_x.len() {
                let m = (mag_x[i].1.powi(2) + mag_y[i].1.powi(2) + mag_z[i].1.powi(2)).sqrt();
                mags.push(m);
            }
            let mean = mags.iter().sum::<f64>() / mags.len() as f64;
            let (m_min, m_max) = mags.iter().fold((f64::MAX, f64::MIN), |(a, b), &m| (a.min(m), b.max(m)));
            let spread_pct = if mean > 0.0 { ((m_max - m_min) / mean) * 100.0 } else { 0.0 };
            if mean == 0.0 {
                out.push(("Compass Interference".into(), "NA".into(), "Zero magnetic vectors; field variation cannot be evaluated".into()));
            } else if spread_pct > 50.0 {
                out.push(("Compass Interference".into(), "FAIL".into(), format!("Magnetic magnitude spread {spread_pct:.1}% (>50%) across mean {mean:.0} mGauss")));
            } else if spread_pct > 35.0 {
                out.push(("Compass Interference".into(), "WARN".into(), format!("Magnetic magnitude spread {spread_pct:.1}% (>35%); cause not determined")));
            } else {
                out.push(("Compass Interference".into(), "GOOD".into(), format!("Magnetic field clean: mean {mean:.0} mGauss, spread {spread_pct:.1}% (<35%)")));
            }
        } else {
            out.push(("Compass Interference".into(), "NA".into(), "No MAG vector records".into()));
        }

        // Servo channels require vehicle/frame/output mapping before motor analysis.
        out.push(("Motor Balance".into(), "NA".into(), "Requires vehicle frame, motor output mapping and an armed flight interval".into()));

        // Pair fields in the same record, including after missing or non-finite data.
        let mut max_err: Option<f64> = None;
        if let Some(t) = self.type_by_name("ATT") {
            let fmt = &self.formats[&t];
            for (actual, desired) in [("Roll", "DesRoll"), ("Pitch", "DesPitch")] {
                let cols = (fmt.columns.iter().position(|c| c == actual), fmt.columns.iter().position(|c| c == desired));
                if let (Some(a), Some(d)) = cols {
                    for r in self.records.iter().filter(|r| r.type_id == t) {
                        if let (Some(a), Some(d)) = (r.values.get(a).and_then(Value::as_f64), r.values.get(d).and_then(Value::as_f64)) {
                            if a.is_finite() && d.is_finite() {
                                let err = ((a - d + 180.0).rem_euclid(360.0) - 180.0).abs();
                                max_err = Some(max_err.unwrap_or(0.0).max(err));
                            }
                        }
                    }
                }
            }
        }
        if let Some(err) = max_err {
            out.push(("Attitude Divergence".into(), if err > 25.0 { "FAIL" } else if err > 12.0 { "WARN" } else { "GOOD" }.into(), format!("Maximum measured/desired attitude error {err:.1} deg; heuristic thresholds 12/25 deg")));
        } else {
            out.push(("Attitude Divergence".into(), "NA".into(), "No paired finite measured/desired ATT samples".into()));
        }

        // GPS is normally AMSL, BARO uses a local datum. Peak values cannot be compared.
        out.push(("Altitude Consistency".into(), "NA".into(), "Requires time-aligned altitudes with a common reference level".into()));

        // No fixed cutoff without knowing pack configuration and chemistry.
        let v = min("BAT", "Volt");
        if (0.0..f64::MAX).contains(&v) {
            out.push(("Battery".into(), "INFO".into(), format!("Minimum logged voltage {v:.2} V; pack configuration and failsafe thresholds not evaluated")));
        } else {
            out.push(("Battery".into(), "NA".into(), "No finite BAT voltage samples".into()));
        }

        // 10. GPS Fix Quality & HDOP
        let hdop = max("GPS", "HDop");
        let sats = min("GPS", "NSats");
        if hdop > f64::MIN && sats < f64::MAX {
            if hdop > 2.5 || sats < 5.0 {
                out.push(("GPS Fix & HDOP".into(), "FAIL".into(), format!("Poor GPS signal: max HDop {hdop:.2} (>2.5), min satellites {sats:.0} (<5)")));
            } else if hdop > 1.8 || sats < 7.0 {
                out.push(("GPS Fix & HDOP".into(), "WARN".into(), format!("Marginal GPS: max HDop {hdop:.2}, min satellites {sats:.0}")));
            } else {
                out.push(("GPS Fix & HDOP".into(), "GOOD".into(), format!("Max HDop {hdop:.2}, min satellites {sats:.0}; fix status not evaluated")));
            }
        } else if !self.gps_track().is_empty() {
            out.push(("GPS Fix & HDOP".into(), "INFO".into(), format!("{} GPS coordinates logged; fix quality fields unavailable", self.gps_track().len())));
        } else {
            out.push(("GPS Fix & HDOP".into(), "NA".into(), "No GPS telemetry".into()));
        }

        // 11. Event / Failsafe Records
        if let Some(t) = self.type_by_name("ERR") {
            let code = self.formats[&t].columns.iter().position(|c| c == "ECode");
            let mut faults = 0;
            let mut recoveries = 0;
            let mut unknown = 0;
            for r in self.records.iter().filter(|r| r.type_id == t) {
                match code.and_then(|c| r.values.get(c)).and_then(Value::as_f64) {
                    Some(0.0) => recoveries += 1,
                    Some(v) if v.is_finite() && v > 0.0 => faults += 1,
                    _ => unknown += 1,
                }
            }
            out.push(("Events & Failsafes".into(), if faults > 0 { "FAIL" } else if unknown > 0 { "WARN" } else { "INFO" }.into(), format!("{faults} nonzero ERR codes, {recoveries} recovery records, {unknown} undecodable records; historical events, not current fault state")));
        } else {
            out.push(("Events & Failsafes".into(), "NA".into(), "No ERR stream; absence does not establish vehicle health".into()));
        }

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

    fn stream(log: &mut Log, name: &str, columns: &[&str], rows: &[&[f64]]) {
        let id = log.formats.len() as u8 + 1;
        log.formats.insert(id, Format { id, len: 0, name: name.into(), format: "d".repeat(columns.len()), columns: columns.iter().map(|s| s.to_string()).collect() });
        log.records.extend(rows.iter().map(|row| Record { type_id: id, values: row.iter().map(|v| Value::Float(*v)).collect() }));
    }

    fn status(log: &Log, name: &str) -> String {
        log.analysis().into_iter().find(|r| r.0 == name).unwrap().1
    }

    #[test]
    fn missing_streams_do_not_certify_health() {
        let log = Log::default();
        assert!(log.analysis().iter().all(|r| r.1 == "NA" || r.1 == "INFO"));
    }

    #[test]
    fn battery_voltage_does_not_assume_cell_count() {
        for voltage in [7.4, 12.6, 25.2] {
            let mut log = Log::default();
            stream(&mut log, "BAT", &["Volt"], &[&[voltage]]);
            assert_eq!(status(&log, "Battery"), "INFO");
        }
    }

    #[test]
    fn error_recovery_is_not_a_new_fault() {
        let mut log = Log::default();
        stream(&mut log, "ERR", &["Subsys", "ECode"], &[&[5.0, 0.0]]);
        assert_eq!(status(&log, "Events & Failsafes"), "INFO");
        log.records.push(Record { type_id: 1, values: vec![Value::Int(5), Value::Int(1)] });
        assert_eq!(status(&log, "Events & Failsafes"), "FAIL");
    }

    #[test]
    fn perfect_attitude_tracking_and_wrapped_angles_are_valid() {
        let mut log = Log::default();
        stream(&mut log, "ATT", &["Roll", "DesRoll"], &[&[0.0, 0.0], &[179.0, -179.0]]);
        assert_eq!(status(&log, "Attitude Divergence"), "GOOD");
    }

    #[test]
    fn nonfinite_fields_cannot_pair_different_attitude_records() {
        let mut log = Log::default();
        stream(&mut log, "ATT", &["Roll", "DesRoll"], &[&[f64::NAN, 90.0], &[0.0, f64::NAN]]);
        assert_eq!(status(&log, "Attitude Divergence"), "NA");
        assert!(log.series("ATT", "Roll").iter().all(|p| p.1.is_finite()));
    }

    #[test]
    fn zero_power_is_reported_but_zero_compass_is_not_healthy() {
        let mut log = Log::default();
        stream(&mut log, "POWR", &["VCC"], &[&[0.0]]);
        stream(&mut log, "MAG", &["MagX", "MagY", "MagZ"], &[&[0.0, 0.0, 0.0]]);
        assert_eq!(status(&log, "Brownout / Power"), "FAIL");
        assert_eq!(status(&log, "Compass Interference"), "NA");
    }

    #[test]
    fn clipping_counts_increases_within_log_and_handles_resets() {
        let mut log = Log::default();
        stream(&mut log, "VIBE", &["Clip0"], &[&[1000.0], &[1000.0]]);
        assert_eq!(status(&log, "IMU Clipping"), "GOOD");
        log.records.push(Record { type_id: 1, values: vec![Value::Int(2)] });
        assert_eq!(status(&log, "IMU Clipping"), "WARN");
    }

    #[test]
    fn generic_servo_outputs_and_mixed_altitude_datums_are_not_diagnosed() {
        let mut log = Log::default();
        stream(&mut log, "RCOU", &["C1", "C2"], &[&[1000.0, 2000.0]]);
        stream(&mut log, "GPS", &["Alt"], &[&[600.0]]);
        stream(&mut log, "BARO", &["Alt"], &[&[20.0]]);
        assert_eq!(status(&log, "Motor Balance"), "NA");
        assert_eq!(status(&log, "Altitude Consistency"), "NA");
    }

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
        let a = log.analysis();
        assert!(a.iter().any(|(t, st, _)| t == "Log Summary" && st == "INFO"));
        assert!(a.iter().any(|(t, st, _)| t == "Events & Failsafes" && st == "NA"));
        assert!(a.iter().any(|(t, st, _)| t == "GPS Fix & HDOP" && st == "INFO"));
    }
}

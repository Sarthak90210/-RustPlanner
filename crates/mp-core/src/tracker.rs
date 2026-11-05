//! Antenna tracker outputs (Antenna/Maestro, ArduTracker, DegreeTracker): turn the
//! bearing and elevation from the tracker to the vehicle into servo commands.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Interface {
    Maestro,
    ArduTracker,
    DegreeTracker,
}

impl Interface {
    pub const ALL: [&'static str; 3] = ["Maestro", "ArduTracker", "DegreeTracker"];
    pub fn from_name(s: &str) -> Self {
        match s {
            "ArduTracker" => Self::ArduTracker,
            "DegreeTracker" => Self::DegreeTracker,
            _ => Self::Maestro,
        }
    }
}

#[derive(Clone, Debug)]
pub struct TrackerConfig {
    pub interface: Interface,
    pub pan_range: i32,
    pub tilt_range: i32,
    pub trim_pan: f64,
    pub trim_tilt: f64,
    pub pan_reverse: bool,
    pub tilt_reverse: bool,
    pub pan_pwm_range: i32,
    pub tilt_pwm_range: i32,
    pub pan_pwm_center: i32,
    pub tilt_pwm_center: i32,
    pub pan_speed: i32,
    pub pan_accel: i32,
    pub tilt_speed: i32,
    pub tilt_accel: i32,
}

impl Default for TrackerConfig {
    fn default() -> Self {
        Self {
            interface: Interface::Maestro,
            pan_range: 360,
            tilt_range: 90,
            trim_pan: 0.0,
            trim_tilt: 0.0,
            pan_reverse: false,
            tilt_reverse: false,
            pan_pwm_range: 1000,
            tilt_pwm_range: 1000,
            pan_pwm_center: 1500,
            tilt_pwm_center: 1500,
            pan_speed: 100,
            pan_accel: 5,
            tilt_speed: 100,
            tilt_accel: 5,
        }
    }
}

fn wrap_180(a: f64) -> f64 {
    if a > 180.0 {
        a - 360.0
    } else if a < -180.0 {
        a + 360.0
    } else {
        a
    }
}

const SET_TARGET: u8 = 0x84;
const SET_SPEED: u8 = 0x87;
const SET_ACCEL: u8 = 0x89;

/// Pololu compact protocol command with a 14 bit value.
pub fn maestro_cmd(cmd: u8, addr: u8, data: i32) -> [u8; 4] {
    [cmd, addr, (data & 0x7f) as u8, ((data >> 7) & 0x7f) as u8]
}

impl TrackerConfig {
    pub fn validate(&self) -> Result<(), &'static str> {
        if self.interface != Interface::DegreeTracker {
            if self.pan_range / 2 == 0 {
                return Err("Invalid Pan Range");
            }
            if self.tilt_range / 2 == 0 {
                return Err("Invalid Tilt Range");
            }
        }
        Ok(())
    }

    fn ranges(&self) -> (f64, f64, f64, f64) {
        let (ps, pe) = (-(self.pan_range / 2) as f64, (self.pan_range / 2) as f64);
        let (ts, te) = (-(self.tilt_range / 2) as f64, (self.tilt_range / 2) as f64);
        (ps, pe, ts, te)
    }

    /// Bytes sent once after opening the port.
    pub fn setup_bytes(&self) -> Vec<u8> {
        if self.interface != Interface::Maestro {
            return Vec::new();
        }
        [maestro_cmd(SET_SPEED, 0, self.pan_speed), maestro_cmd(SET_SPEED, 1, self.tilt_speed), maestro_cmd(SET_ACCEL, 0, self.pan_accel), maestro_cmd(SET_ACCEL, 1, self.tilt_accel)].concat()
    }

    /// PanAndTilt: the bytes to point the antenna at (pan, tilt) degrees.
    pub fn pan_and_tilt(&self, pan: f64, tilt: f64) -> Vec<u8> {
        let (ps, pe, ts, te) = self.ranges();
        match self.interface {
            Interface::Maestro => {
                let prev = if self.pan_reverse { -1.0 } else { 1.0 };
                let trev = if self.tilt_reverse { -1.0 } else { 1.0 };
                let pan_us = |a: f64| {
                    let pw = self.pan_pwm_range as f64 / (ps - pe).abs() * wrap_180(a - self.trim_pan) * prev + self.pan_pwm_center as f64;
                    pw.clamp((self.pan_pwm_center - self.pan_pwm_range / 2) as f64, (self.pan_pwm_center + self.pan_pwm_range / 2) as f64) as i32
                };
                let tilt_us = |a: f64| {
                    let pw = self.tilt_pwm_range as f64 / (ts - te).abs() * (a - self.trim_tilt) * trev + self.tilt_pwm_center as f64;
                    pw.clamp((self.tilt_pwm_center - self.tilt_pwm_range / 2) as f64, (self.tilt_pwm_center + self.tilt_pwm_range / 2) as f64) as i32
                };
                // a tilt range over 120 degrees flips over the top instead of panning past 90
                let (p, t) = if (ts - te).abs() > 120.0 && wrap_180(pan - self.trim_pan).abs() > 90.0 {
                    (pan_us(wrap_180(pan - self.trim_pan)), tilt_us(180.0 - tilt))
                } else {
                    (pan_us(pan), tilt_us(tilt))
                };
                [maestro_cmd(SET_TARGET, 1, t * 4), maestro_cmd(SET_TARGET, 0, p * 4)].concat()
            }
            Interface::ArduTracker => {
                let prev = if self.pan_reverse { -1.0 } else { 1.0 };
                let trev = if self.tilt_reverse { -1.0 } else { 1.0 };
                let pa = wrap_180(pan - self.trim_pan).clamp(ps, pe).trunc();
                let ta = (tilt - self.trim_tilt).clamp(ts, te).trunc();
                let p = (pa / (ps - pe).abs() * 2.0 * (self.pan_pwm_range / 2) as f64 * prev + self.pan_pwm_center as f64) as i32;
                let t = (ta / (ts - te).abs() * 2.0 * (self.tilt_pwm_range / 2) as f64 * trev + self.tilt_pwm_center as f64) as i32;
                format!("!!!PAN:{p:04},TLT:{t:04}\n").into_bytes()
            }
            Interface::DegreeTracker => format!("!!!PAN:{:04},TLT:{:04}\n", (pan * 10.0) as i32, (tilt * 10.0) as i32).into_bytes(),
        }
    }
}

/// CurrentState.AZToMAV / ELToMAV: bearing and elevation from the tracker to the vehicle.
pub fn az_el(tracker: (f64, f64, f64), mav: (f64, f64, f64)) -> (f64, f64) {
    let (tlat, tlng, talt) = tracker;
    let (lat, lng, alt) = mav;
    let r = 6_371_000.0f64;
    let (p1, p2) = (tlat.to_radians(), lat.to_radians());
    let dl = (lng - tlng).to_radians();
    let a = ((p2 - p1) / 2.0).sin().powi(2) + p1.cos() * p2.cos() * (dl / 2.0).sin().powi(2);
    let dist = 2.0 * r * a.sqrt().asin();
    if dist == 0.0 {
        return (0.0, 0.0);
    }
    let y = dl.sin() * p2.cos();
    let x = p1.cos() * p2.sin() - p1.sin() * p2.cos() * dl.cos();
    let az = (y.atan2(x).to_degrees() + 360.0) % 360.0;
    let el = ((alt - talt) / dist).atan().to_degrees();
    (az, el)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn maestro_center_and_edges() {
        let c = TrackerConfig::default();
        let b = c.pan_and_tilt(0.0, 0.0);
        // 1500us * 4 = 6000 quarter-us
        assert_eq!(b, [maestro_cmd(0x84, 1, 6000), maestro_cmd(0x84, 0, 6000)].concat());
        let b = c.pan_and_tilt(90.0, 45.0);
        assert_eq!(&b[..4], &maestro_cmd(0x84, 1, 2000 * 4));
        assert_eq!(&b[4..], &maestro_cmd(0x84, 0, 1750 * 4));
    }

    #[test]
    fn serial_trackers() {
        let mut c = TrackerConfig { interface: Interface::DegreeTracker, ..Default::default() };
        assert_eq!(c.pan_and_tilt(12.3, 4.5), b"!!!PAN:0123,TLT:0045\n");
        c.interface = Interface::ArduTracker;
        assert_eq!(c.pan_and_tilt(0.0, 0.0), b"!!!PAN:1500,TLT:1500\n");
        assert_eq!(c.pan_and_tilt(180.0, 45.0), b"!!!PAN:2000,TLT:2000\n");
    }

    #[test]
    fn bearing_and_elevation() {
        let (az, el) = az_el((0.0, 0.0, 0.0), (0.001, 0.0, 111.0));
        assert!(az.abs() < 0.01 || (az - 360.0).abs() < 0.01);
        assert!((el - 45.0).abs() < 0.5);
        let (az, _) = az_el((0.0, 0.0, 0.0), (0.0, 0.001, 0.0));
        assert!((az - 90.0).abs() < 0.01);
    }
}

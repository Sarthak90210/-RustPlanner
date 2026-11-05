//! CurrentState display multipliers (Planner "Distance / Altitude / Speed Units"):
//! `multiplierdist`, `multiplieralt`, `multiplierspeed` and their unit names.

use std::collections::BTreeMap;
use std::sync::RwLock;

#[derive(Clone, Copy)]
struct Units {
    dist: (f32, &'static str),
    alt: (f32, &'static str),
    speed: (f32, &'static str),
}

static UNITS: RwLock<Units> = RwLock::new(Units { dist: (1.0, "m"), alt: (1.0, "m"), speed: (1.0, "m/s") });

const FEET: f32 = 3.280_84;

/// MainV2 / ConfigPlanner: apply the unit choices.
pub fn set_from(planner: &BTreeMap<String, String>) {
    let get = |k: &str| planner.get(k).map(|s| s.as_str()).unwrap_or("");
    let len = |v: &str| if v == "Feet" { (FEET, "ft") } else { (1.0, "m") };
    let speed = match get("speedunits") {
        "fps" => (FEET, "fps"),
        "kph" => (3.6, "kph"),
        "mph" => (2.236_936, "mph"),
        "knots" => (1.943_844, "kn"),
        _ => (1.0, "m/s"),
    };
    *UNITS.write().unwrap() = Units { dist: len(get("distunits")), alt: len(get("altunits")), speed };
}

fn u() -> Units {
    *UNITS.read().unwrap()
}

pub fn dist(v: f32) -> f32 {
    v * u().dist.0
}
pub fn alt(v: f32) -> f32 {
    v * u().alt.0
}
pub fn speed(v: f32) -> f32 {
    v * u().speed.0
}
pub fn dist_unit() -> &'static str {
    u().dist.1
}
pub fn alt_unit() -> &'static str {
    u().alt.1
}
pub fn speed_unit() -> &'static str {
    u().speed.1
}

/// How a CurrentState field is displayed: converted value and unit (quick view / status).
pub fn field(name: &str, v: f64) -> (f64, Option<&'static str>) {
    match name {
        "alt" | "altasl" | "alt_error" | "target_alt" | "altoffsethome" | "HomeAlt" | "gpsalt" => (alt(v as f32) as f64, Some(alt_unit())),
        "wp_dist" | "DistToHome" | "distTraveled" | "xtrack_error" => (dist(v as f32) as f64, Some(dist_unit())),
        "groundspeed" | "airspeed" | "verticalspeed" | "wind_vel" | "targetairspeed" => (speed(v as f32) as f64, Some(speed_unit())),
        _ => (v, None),
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn feet_and_knots() {
        let mut p = std::collections::BTreeMap::new();
        p.insert("altunits".to_string(), "Feet".to_string());
        p.insert("speedunits".to_string(), "knots".to_string());
        super::set_from(&p);
        assert!((super::alt(100.0) - 328.084).abs() < 0.01);
        assert_eq!(super::speed_unit(), "kn");
        super::set_from(&Default::default());
        assert_eq!(super::alt(10.0), 10.0);
    }
}

//! GEO / UTM / MGRS conversions as Mission Planner's GeoUtility library performs them
//! (`ExtLibs/GeoUtility/Transformation/{WGSUTM,UTMWGS,UTMMGR,MGRUTM}.cs`, `GeoSystem/MGRS.cs`),
//! plus the signed-zone UTM of `PointLatLngAlt.GetUTMZone/ToUTM` and `utmpos.ToLLA`.

const A: f64 = 6_378_137.0;
const F: f64 = 3.352_810_664_747_48e-3;
const POL: f64 = A / (1.0 - F);
const E2: f64 = (2.0 * F - F * F) / ((1.0 - F) * (1.0 - F));
const E4: f64 = E2 * E2;
const E6: f64 = E4 * E2;
const E8: f64 = E4 * E4;
const K0: f64 = 0.9996;
const FALSE_EASTING: f64 = 500_000.0;
const UTM_BAND: &[u8] = b"CDEFGHJKLMNPQRSTUVWXX";
const MGRS_EAST0: &str = "STUVWXYZ";
const MGRS_EAST1: &str = "ABCDEFGH";
const MGRS_EAST2: &str = "JKLMNPQR";
const MGRS_EAST: &[u8] = b"ABCDEFGHJKLMNPQRSTUVWXYZ";
const MGRS_NORTH0: &str = "FGHJKLMNPQRSTUVABCDE";
const MGRS_NORTH1: &str = "ABCDEFGHJKLMNPQRSTUV";

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Utm {
    pub zone: i32,
    pub band: char,
    pub east: f64,
    pub north: f64,
}

/// .NET `Math.Round(x, d)` (banker's rounding).
fn round_even(x: f64, digits: i32) -> f64 {
    let m = 10f64.powi(digits);
    (x * m).round_ties_even() / m
}

/// .NET Framework `double.ToString()`: 15 significant digits, trailing zeros trimmed.
pub fn net_double(x: f64) -> String {
    if x == 0.0 || !x.is_finite() {
        return if x == 0.0 { "0".into() } else { x.to_string() };
    }
    let int_digits = x.abs().log10().floor() as i32 + 1;
    let decimals = (15 - int_digits).max(0) as usize;
    let s = format!("{x:.decimals$}");
    let s = if s.contains('.') { s.trim_end_matches('0').trim_end_matches('.').to_string() } else { s };
    if s == "-0" { "0".into() } else { s }
}

/// Transverse Mercator forward series used by GeoUtility, for a given zone's meridian.
fn tm_forward(lat: f64, lng: f64, zone: i32) -> (f64, f64) {
    let k0c = POL * (std::f64::consts::PI / 180.0) * (1.0 - 3.0 * E2 / 4.0 + 45.0 * E4 / 64.0 - 175.0 * E6 / 256.0 + 11025.0 * E8 / 16384.0);
    let k2 = POL * (-3.0 * E2 / 8.0 + 15.0 * E4 / 32.0 - 525.0 * E6 / 1024.0 + 2205.0 * E8 / 4096.0);
    let k4 = POL * (15.0 * E4 / 256.0 - 105.0 * E6 / 1024.0 + 2205.0 * E8 / 16384.0);
    let k6 = POL * (-35.0 * E6 / 3072.0 + 315.0 * E8 / 12288.0);
    let phi = lat.to_radians();
    let (t1, c1) = (phi.tan(), phi.cos());
    let (t2, t4) = (t1.powi(2), t1.powi(4));
    let (c2, c3, c4, c5) = (c1.powi(2), c1.powi(3), c1.powi(4), c1.powi(5));
    let eta = E2 * c2;
    let n = POL / (1.0 + eta).sqrt();
    let arc = k0c * lat + k2 * (2.0 * phi).sin() + k4 * (4.0 * phi).sin() + k6 * (6.0 * phi).sin();
    let merid = ((zone - 30) * 6 - 3) as f64;
    let dl = (lng - merid).to_radians();
    let north = K0 * (arc + n * c2 * t1 * dl.powi(2) / 2.0 + n * c4 * t1 * (5.0 - t2 + 9.0 * eta) * dl.powi(4) / 24.0);
    let north = if lat < 0.0 { 10e6 + north } else { north };
    let east = K0 * (n * c1 * dl + n * c3 * (1.0 - t2 + eta) * dl.powi(3) / 6.0 + n * c5 * (5.0 - 18.0 * t2 + t4) * dl.powi(5) / 120.0) + FALSE_EASTING;
    (east, north)
}

/// Transverse Mercator inverse series (`UTMWGS`). `south` subtracts the false northing.
fn tm_inverse(zone: i32, south: bool, east: f64, north: f64) -> (f64, f64) {
    let r = 180.0 / std::f64::consts::PI;
    let k0c = POL * (std::f64::consts::PI / 180.0) * (1.0 - 3.0 * E2 / 4.0 + 45.0 * E4 / 64.0 - 175.0 * E6 / 256.0 + 11025.0 * E8 / 16384.0);
    let k2 = r * (3.0 * E2 / 8.0 - 3.0 * E4 / 16.0 + 213.0 * E6 / 2048.0 - 255.0 * E8 / 4096.0);
    let k4 = r * (21.0 * E4 / 256.0 - 21.0 * E6 / 256.0 + 533.0 * E8 / 8192.0);
    let k6 = r * (151.0 * E6 / 6144.0 - 453.0 * E8 / 12288.0);
    let north = if south && north > 0.0 { north - 10e6 } else { north };
    let sig = (north / K0) / k0c;
    let sr = sig.to_radians();
    let fb = sig + k2 * (2.0 * sr).sin() + k4 * (4.0 * sr).sin() + k6 * (6.0 * sr).sin();
    let phi = fb.to_radians();
    let t1 = phi.tan();
    let (t2, c1) = (t1 * t1, phi.cos());
    let t4 = t2 * t2;
    let eta = E2 * c1 * c1;
    let n1 = POL / (1.0 + eta).sqrt();
    let merid = ((zone - 30) * 6 - 3) as f64;
    let d = (east - FALSE_EASTING) / K0;
    let b2 = -t1 * (1.0 + eta) / (2.0 * n1.powi(2));
    let b4 = t1 * (5.0 + 3.0 * t2 + 6.0 * eta * (1.0 - t2)) / (24.0 * n1.powi(4));
    let b6 = -t1 * (61.0 + 90.0 * t2 + 45.0 * t4) / (720.0 * n1.powi(6));
    let l1 = 1.0 / (n1 * c1);
    let l3 = -(1.0 + 2.0 * t2 + eta) / (6.0 * n1.powi(3) * c1);
    let l5 = (5.0 + 28.0 * t2 + 24.0 * t4) / (120.0 * n1.powi(5) * c1);
    let lat = fb + r * (b2 * d.powi(2) + b4 * d.powi(4) + b6 * d.powi(6));
    let lng = merid + r * (l1 * d + l3 * d.powi(3) + l5 * d.powi(5));
    (lat, lng)
}

/// `(UTM)new Geographic(lng, lat)`: GeoUtility's zone (with Norway/Svalbard exceptions) and band.
pub fn geo_to_utm(lat: f64, lng: f64) -> Option<Utm> {
    if lng <= -180.0 || lng > 180.0 || !(-80.0..=84.0).contains(&lat) {
        return None;
    }
    let mut zone = ((lng + 180.0) / 6.0) as i32 + 1;
    if (56.0..64.0).contains(&lat) && (3.0..12.0).contains(&lng) {
        zone = 32;
    } else if lat >= 72.0 {
        zone = match lng {
            l if (0.0..9.0).contains(&l) => 31,
            l if (9.0..21.0).contains(&l) => 33,
            l if (21.0..33.0).contains(&l) => 35,
            l if (33.0..42.0).contains(&l) => 37,
            _ => zone,
        };
    }
    let band = UTM_BAND[(1.0 + (lat + 80.0) / 8.0) as usize - 1] as char;
    let (east, north) = tm_forward(lat, lng, zone);
    Some(Utm { zone, band, east: round_even(east, 3), north: round_even(north, 3) })
}

/// `UTM.ToString()`: "55H 694420.123 6083869.456".
pub fn utm_string(u: &Utm) -> String {
    format!("{}{} {} {}", u.zone, u.band, net_double(u.east), net_double(u.north))
}

/// `UTM -> Geographic` (`UTMWGS`); bands below N are southern.
pub fn utm_to_geo(u: &Utm) -> (f64, f64) {
    tm_inverse(u.zone, u.band < 'N', u.east, u.north)
}

/// `PointLatLngAlt.GetUTMZone()`: negative in the southern hemisphere.
pub fn signed_zone(lat: f64, lng: f64) -> i32 {
    let zone = ((lng + 186.0) / 6.0) as i32;
    if lat < 0.0 { -zone } else { zone }
}

/// `PointLatLngAlt.ToUTM()`: [easting, northing] in the signed zone.
pub fn to_utm_signed(lat: f64, lng: f64) -> (i32, f64, f64) {
    let zone = signed_zone(lat, lng);
    let (e, n) = tm_forward(lat, lng, zone.abs());
    (zone, e, n)
}

/// UTM in a given signed zone (ProjNet's WGS84_UTM(zone, north)), for points near that zone.
pub fn to_utm_in_zone(zone: i32, lat: f64, lng: f64) -> (i32, f64, f64) {
    let (e, n) = tm_forward(lat, lng, zone.abs());
    let n = match (zone < 0, lat < 0.0) {
        (true, false) => n + 10e6,
        (false, true) => n - 10e6,
        _ => n,
    };
    (zone, e, n)
}

/// `new utmpos(east, north, zone).ToLLA()`: a negative zone is southern.
pub fn utm_signed_to_geo(zone: i32, east: f64, north: f64) -> (f64, f64) {
    tm_inverse(zone.abs(), zone < 0, east, north)
}

/// `((MGRS)new Geographic(lng, lat)).ToString()`, e.g. "55HFA9442083869".
pub fn geo_to_mgrs(lat: f64, lng: f64) -> Option<String> {
    let utm = geo_to_utm(lat, lng)?;
    // UTM.Format: EastString "000000", NorthString "0000000" (rounded half away from zero)
    let east_s = format!("{:06}", utm.east.round() as i64);
    let north_s = format!("{:07}", utm.north.round() as i64);
    let east_plan = east_s[..1].parse::<usize>().ok()?;
    let north_plan = north_s[..2].parse::<usize>().ok()?;
    let mut east = (utm.east.round_ties_even() as i64).to_string();
    if east.len() > 2 {
        east.remove(0);
    }
    let mut north = (utm.north.round_ties_even() as i64).to_string();
    if north.len() > 5 {
        north = north[north.len() - 5..].to_string();
    }
    if !(1..=60).contains(&utm.zone) || !('C'..='X').contains(&utm.band) {
        return None;
    }
    let eastgrid = match utm.zone % 3 {
        1 => east_plan.checked_sub(1)?,
        2 => east_plan + 7,
        _ => east_plan + 15,
    };
    let mut northgrid = if utm.zone % 2 != 1 { 5 } else { 0 } + north_plan % 20;
    if northgrid > 19 {
        northgrid -= 20;
    }
    let grid = format!("{}{}", *MGRS_EAST.get(eastgrid)? as char, MGRS_NORTH1.as_bytes()[northgrid] as char);
    // MGRS(zone, band, grid, east, north) then Format: pad to 5, strip shared trailing zeros
    let (ev, nv) = (east.parse::<i64>().ok()?, north.parse::<i64>().ok()?);
    let mut e = format!("{ev:05}");
    let mut n = format!("{nv:05}");
    while e.ends_with('0') && n.ends_with('0') {
        e.pop();
        n.pop();
    }
    Some(format!("{}{}{grid}{e}{n}", utm.zone, utm.band))
}

/// `new MGRS(text).ConvertTo<Geographic>()`; None where GeoUtility throws.
pub fn mgrs_to_geo(text: &str) -> Option<(f64, f64)> {
    let s: String = text.chars().filter(|c| *c != ' ').collect::<String>().to_uppercase();
    let b = s.as_bytes();
    if b.len() < 6 || !b[0].is_ascii_digit() {
        return None;
    }
    let zl = if b[1].is_ascii_digit() { 2 } else { 1 };
    let zone: i32 = s[..zl].parse().ok()?;
    let rest = &s[zl..];
    if rest.len() < 5 || !rest.is_ascii() {
        return None;
    }
    let band = rest.as_bytes()[0] as char;
    if !('C'..='X').contains(&band) || band == 'I' || band == 'O' {
        return None;
    }
    let g: Vec<char> = rest[1..3].chars().collect();
    if g.iter().any(|c| !('A'..'Z').contains(c) || *c == 'I' || *c == 'O') {
        return None;
    }
    let digits = &rest[3..];
    if digits.len() % 2 > 0 || !digits.bytes().all(|c| c.is_ascii_digit()) {
        return None;
    }
    let (e, n) = digits.split_at(digits.len() / 2);
    let fit = |v: &str| if v.len() > 5 { v[..5].to_string() } else { format!("{v:0<5}") };
    let (e, n) = (fit(e), fit(n));
    if !(1..=60).contains(&zone) {
        return None;
    }
    let east_idx = match zone % 3 {
        0 => MGRS_EAST0.find(g[0]),
        1 => MGRS_EAST1.find(g[0]),
        _ => MGRS_EAST2.find(g[0]),
    }
    .map(|i| i as i32 + 1)
    .unwrap_or(0);
    let mut sv = if zone % 2 == 0 { MGRS_NORTH0.find(g[1]) } else { MGRS_NORTH1.find(g[1]) }.map(|i| i as i32).unwrap_or(-1);
    let steps: [(char, i32, bool); 8] = if band >= 'N' {
        [('Q', 10, true), ('R', 0, false), ('S', 30, true), ('T', 0, false), ('U', 50, true), ('V', 0, false), ('W', 70, true), ('X', 0, false)]
    } else {
        [('C', 10, true), ('D', 0, false), ('F', 30, true), ('G', 0, false), ('H', 50, true), ('J', 0, false), ('K', 70, true), ('L', 0, false)]
    };
    for (c, limit, exact) in steps {
        if exact {
            if band == c && sv < limit {
                sv += 20;
            }
        } else if band >= c {
            sv += 20;
        }
    }
    let east: f64 = format!("{east_idx}{e}").parse().ok()?;
    let north_s = format!("{}{sv}{n}", if sv.to_string().len() == 1 { "0" } else { "" });
    let north: f64 = north_s.parse().ok()?;
    Some(utm_to_geo(&Utm { zone, band, east, north }))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn canberra_round_trips_through_utm_and_mgrs() {
        let (lat, lng) = (-35.363261, 149.165230);
        let u = geo_to_utm(lat, lng).unwrap();
        assert_eq!((u.zone, u.band), (55, 'H'));
        let s = utm_string(&u);
        assert!(s.starts_with("55H 6"), "{s}");
        let (la, lo) = utm_to_geo(&u);
        assert!((la - lat).abs() < 1e-7 && (lo - lng).abs() < 1e-7, "{la} {lo}");

        let m = geo_to_mgrs(lat, lng).unwrap();
        assert!(m.starts_with("55HFA"), "{m}");
        let (la, lo) = mgrs_to_geo(&m).unwrap();
        // one-metre MGRS cell
        assert!((la - lat).abs() < 2e-5 && (lo - lng).abs() < 2e-5, "{la} {lo}");
        assert_eq!(mgrs_to_geo(&m.to_lowercase().replace("fa", " fa ")), mgrs_to_geo(&m));

        let (z, e, n) = to_utm_signed(lat, lng);
        assert_eq!(z, -55);
        assert!((e - u.east).abs() < 1e-3 && (n - u.north).abs() < 1e-3);
        let (la, lo) = utm_signed_to_geo(z, e, n);
        assert!((la - lat).abs() < 1e-7 && (lo - lng).abs() < 1e-7);
    }

    #[test]
    fn northern_zone_exceptions_and_invalid_input() {
        assert_eq!(geo_to_utm(60.0, 5.0).unwrap().zone, 32);
        assert_eq!(geo_to_utm(78.0, 15.0).unwrap().zone, 33);
        assert!(geo_to_utm(85.0, 0.0).is_none() && geo_to_mgrs(-81.0, 0.0).is_none());
        for bad in ["", "55H", "55IFA1234", "55HFA123", "XXHFA1234"] {
            assert!(mgrs_to_geo(bad).is_none(), "{bad}");
        }
        assert_eq!(net_double(694420.1230000001), "694420.123");
        assert_eq!(net_double(6083869.0), "6083869");
    }
}

#[cfg(test)]
mod reference_tests {
    use super::*;

    /// UTM values from the independent PyPI `utm` package. MGRS digits are that package's metres
    /// rounded, as GeoUtility's UTMMGR rounds where standard MGRS truncates; GeoUtility also omits
    /// the zone's leading zero ("6W" not "06W") and strips shared trailing zeros.
    #[test]
    fn matches_independent_reference_points() {
        for (lat, lng, mgrs, east, north) in [
            (-35.363261, 149.165230, "55HFA9671984520", 696719.245, 6084519.587),
            (51.5007, -0.1246, "30UXC9956809428", 699567.540, 5709427.563),
            (40.7128, -74.0060, "18TWL8395907351", 583959.372, 4507350.998),
            (-33.8568, 151.2153, "56HLH3490152289", 334900.570, 6252288.753),
            (60.0, 5.0, "32VKM7698058157", 276979.926, 6658157.203),
            (0.0001, 0.0001, "31NAA6603300011", 166032.586, 11.068),
            (64.5, -147.7, "6WVS6637552918", 466375.136, 7152917.862),
        ] {
            let u = geo_to_utm(lat, lng).unwrap();
            assert!((u.east - east).abs() < 0.01 && (u.north - north).abs() < 0.01, "{lat},{lng}: {u:?}");
            assert_eq!(geo_to_mgrs(lat, lng).unwrap(), mgrs, "{lat},{lng}");
            let (la, lo) = mgrs_to_geo(mgrs).unwrap();
            assert!((la - lat).abs() < 2e-5 && (lo - lng).abs() < 2e-5, "{mgrs}: {la},{lo}");
        }
    }
}

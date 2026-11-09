//! Port of `ExtLibs/ProjNet/CoordinateSystems/Projections/MapProjection.cs`
//!
//! The abstract class is split in two: `MapProjection` holds the state the constructor
//! computes and the math helpers, and the `MapProjectionImpl` trait stands in for the
//! abstract `RadiansToMeters`/`MetersToRadians` and provides `DegreesToMeters`,
//! `MetersToDegrees` and `Transform`. The WKT and XML text members are not ported yet.

use std::f64::consts::PI as MATH_PI;

use super::projectionparameterset::{ProjectionParameter, ProjectionParameterSet};
use crate::dotnet::Exception;
use crate::extlibs::projnet::coordinatesystems::transformations::mathtransform::{degrees2radians, radians2degrees};

/// `PI`
pub const PI: f64 = MATH_PI;
/// `HALF_PI`
pub const HALF_PI: f64 = PI * 0.5;
/// `TWO_PI`
pub const TWO_PI: f64 = PI * 2.0;
/// `EPSLN`
pub const EPSLN: f64 = 1.0e-10;
/// `S2R`
pub const S2R: f64 = 4.848136811095359e-6;
/// `MAX_VAL`
pub const MAX_VAL: f64 = 4.0;
/// `prjMAXLONG`
pub const PRJ_MAXLONG: f64 = 2147483647.0;
/// `DBLLONG`
pub const DBLLONG: f64 = 4.61168601e18;

const C00: f64 = 1.0;
const C02: f64 = 0.25;
const C04: f64 = 0.046875;
const C06: f64 = 0.01953125;
const C08: f64 = 0.01068115234375;
const C22: f64 = 0.75;
const C44: f64 = 0.46875;
const C46: f64 = 0.01302083333333333333;
const C48: f64 = 0.00712076822916666666;
const C66: f64 = 0.36458333333333333333;
const C68: f64 = 0.00569661458333333333;
const C88: f64 = 0.3076171875;

/// The state of `public abstract class MapProjection : MathTransform, IProjection`.
#[allow(dead_code)]
#[derive(Debug, Clone)]
pub struct MapProjection {
    pub(crate) e: f64,
    pub(crate) es: f64,
    pub(crate) semi_major: f64,
    pub(crate) semi_minor: f64,
    pub(crate) meters_per_unit: f64,

    /// scale factor
    pub(crate) scale_factor: f64,
    /// Center longitude (projection center)
    pub(crate) central_meridian: f64,
    /// center latitude
    pub(crate) lat_origin: f64,
    /// y offset in meters
    pub(crate) false_northing: f64,
    /// x offset in meters
    pub(crate) false_easting: f64,

    pub(crate) en0: f64,
    pub(crate) en1: f64,
    pub(crate) en2: f64,
    pub(crate) en3: f64,
    pub(crate) en4: f64,

    pub(crate) parameters: ProjectionParameterSet,
    is_inverse: bool,

    pub name: String,
    pub authority: String,
    pub authority_code: i64,
    pub abbreviation: String,
    pub alias: String,
    pub remarks: String,
}

impl MapProjection {
    /// `protected MapProjection(IEnumerable<ProjectionParameter> parameters)`
    pub fn new(parameters: &[ProjectionParameter]) -> Result<Self, Exception> {
        let p = ProjectionParameterSet::new(parameters)?;

        let semi_major = p.get_parameter_value("semi_major", &[])?;
        let semi_minor = p.get_parameter_value("semi_minor", &[])?;

        let es = eccentricy_squared(semi_major, semi_minor);
        let e = es.sqrt();

        let scale_factor = p.get_optional_parameter_value("scale_factor", 1.0, &[]);

        let central_meridian = degrees2radians(p.get_parameter_value("central_meridian", &["longitude_of_center"])?);
        let lat_origin = degrees2radians(p.get_parameter_value("latitude_of_origin", &["latitude_of_center"])?);

        let meters_per_unit = p.get_parameter_value("unit", &[])?;

        let false_easting = p.get_optional_parameter_value("false_easting", 0.0, &[]) * meters_per_unit;
        let false_northing = p.get_optional_parameter_value("false_northing", 0.0, &[]) * meters_per_unit;

        // Compute constants for the mlfn
        let en0 = C00 - es * (C02 + es * (C04 + es * (C06 + es * C08)));
        let en1 = es * (C22 - es * (C04 + es * (C06 + es * C08)));
        let mut t = es * es;
        let en2 = t * (C44 - es * (C46 + es * C48));
        t *= es;
        let en3 = t * (C66 - es * C68);
        let en4 = t * es * C88;

        Ok(MapProjection {
            e,
            es,
            semi_major,
            semi_minor,
            meters_per_unit,
            scale_factor,
            central_meridian,
            lat_origin,
            false_northing,
            false_easting,
            en0,
            en1,
            en2,
            en3,
            en4,
            parameters: p,
            is_inverse: false,
            name: String::new(),
            authority: String::new(),
            authority_code: 0,
            abbreviation: String::new(),
            alias: String::new(),
            remarks: String::new(),
        })
    }

    /// `CloneParametersList(projectionParameters)`
    pub fn clone_parameters_list(projection_parameters: &[ProjectionParameter]) -> Vec<ProjectionParameter> {
        projection_parameters.iter().map(|pp| ProjectionParameter::new(&pp.name, pp.value)).collect()
    }

    /// `ClassName`
    pub fn class_name(&self) -> &str {
        &self.name
    }

    /// `GetParameter(int index)`
    pub fn get_parameter(&self, index: i32) -> Result<ProjectionParameter, Exception> {
        self.parameters.get_at_index(index)
    }

    /// `GetParameter(string name)`: case insensitive, `null` when absent.
    pub fn get_parameter_by_name(&self, name: &str) -> Option<ProjectionParameter> {
        self.parameters.find(name)
    }

    /// `NumParameters`
    pub fn num_parameters(&self) -> usize {
        self.parameters.count()
    }

    /// `DimSource` / `DimTarget`
    pub fn dim_source(&self) -> i32 {
        2
    }

    pub fn dim_target(&self) -> i32 {
        2
    }

    /// `IsInverse`
    pub fn is_inverse(&self) -> bool {
        self.is_inverse
    }

    /// `Invert()` on this instance. (The C# also flips the linked inverse instance; the Rust
    /// inverse is an independent copy, see `TransverseMercator::inverse`.)
    pub fn invert(&mut self) {
        self.is_inverse = !self.is_inverse;
    }

    pub(crate) fn set_inverse(&mut self, v: bool) {
        self.is_inverse = v;
    }

    /// `EqualParams(object obj)`: same parameter values and the same direction.
    pub fn equal_params(&self, proj: &MapProjection) -> bool {
        if self.parameters != proj.parameters {
            return false;
        }
        self.is_inverse() == proj.is_inverse()
    }

    // ---- Helper mathematical functions ----

    /// `CUBE(x)`
    pub fn cube(x: f64) -> f64 {
        x.powf(3.0)
    }

    /// `QUAD(x)`
    pub fn quad(x: f64) -> f64 {
        x.powf(4.0)
    }

    /// `GMAX(ref A, ref B)`
    pub fn gmax(a: f64, b: f64) -> f64 {
        crate::dotnet::math_max(a, b)
    }

    /// `GMIN(ref A, ref B)`
    pub fn gmin(a: f64, b: f64) -> f64 {
        if a < b {
            a
        } else {
            b
        }
    }

    /// `IMOD(A, B)` (floating point, so it is always 0 apart from rounding)
    pub fn imod(a: f64, b: f64) -> f64 {
        a - ((a / b) * b)
    }

    /// `sign(x)`
    pub fn sign(x: f64) -> f64 {
        if x < 0.0 {
            -1.0
        } else {
            1.0
        }
    }

    /// `adjust_lon(x)`: wraps a longitude into -PI..PI.
    pub fn adjust_lon(mut x: f64) -> f64 {
        // C# (long) casts: truncate, out-of-range gives long.MinValue
        fn to_long(v: f64) -> i64 {
            if v.is_nan() || v >= 9.223372036854775807e18 || v < -9.223372036854775808e18 {
                i64::MIN
            } else {
                v as i64
            }
        }
        let mut count: i64 = 0;
        loop {
            if x.abs() <= PI {
                break;
            } else if to_long((x / MATH_PI).abs()) < 2 {
                x -= Self::sign(x) * TWO_PI;
            } else if (to_long((x / TWO_PI).abs()) as f64) < PRJ_MAXLONG {
                x -= (to_long(x / TWO_PI) as f64) * TWO_PI;
            } else if (to_long((x / (PRJ_MAXLONG * TWO_PI)).abs()) as f64) < PRJ_MAXLONG {
                x -= (to_long(x / (PRJ_MAXLONG * TWO_PI)) as f64) * (TWO_PI * PRJ_MAXLONG);
            } else if (to_long((x / (DBLLONG * TWO_PI)).abs()) as f64) < PRJ_MAXLONG {
                x -= (to_long(x / (DBLLONG * TWO_PI)) as f64) * (TWO_PI * DBLLONG);
            } else {
                x -= Self::sign(x) * TWO_PI;
            }
            count += 1;
            if count as f64 > MAX_VAL {
                break;
            }
        }
        x
    }

    /// `msfnz(eccent, sinphi, cosphi)`
    pub fn msfnz(eccent: f64, sinphi: f64, cosphi: f64) -> f64 {
        let con = eccent * sinphi;
        cosphi / (1.0 - con * con).sqrt()
    }

    /// `qsfnz(eccent, sinphi)`
    pub fn qsfnz(eccent: f64, sinphi: f64) -> f64 {
        if eccent > 1.0e-7 {
            let con = eccent * sinphi;
            (1.0 - eccent * eccent) * (sinphi / (1.0 - con * con) - (0.5 / eccent) * ((1.0 - con) / (1.0 + con)).ln())
        } else {
            2.0 * sinphi
        }
    }

    /// `sincos(val, out sin_val, out cos_val)`
    pub fn sincos(val: f64) -> (f64, f64) {
        (val.sin(), val.cos())
    }

    /// `tsfnz(eccent, phi, sinphi)`
    pub fn tsfnz(eccent: f64, phi: f64, sinphi: f64) -> f64 {
        let mut con = eccent * sinphi;
        let com = 0.5 * eccent;
        con = ((1.0 - con) / (1.0 + con)).powf(com);
        (0.5 * (HALF_PI - phi)).tan() / con
    }

    /// `phi1z(eccent, qs, out flag)`
    pub fn phi1z(eccent: f64, qs: f64) -> Result<(f64, i64), Exception> {
        let flag = 0;
        let mut phi = Self::asinz(0.5 * qs);
        if eccent < EPSLN {
            return Ok((phi, flag));
        }
        let eccnts = eccent * eccent;
        for _ in 1..=25 {
            let (sinpi, cospi) = Self::sincos(phi);
            let con = eccent * sinpi;
            let com = 1.0 - con * con;
            let dphi = 0.5 * com * com / cospi
                * (qs / (1.0 - eccnts) - sinpi / com + 0.5 / eccent * ((1.0 - con) / (1.0 + con)).ln());
            phi += dphi;
            if dphi.abs() <= 1e-7 {
                return Ok((phi, flag));
            }
        }
        Err(Exception::Argument("Convergence error.".into()))
    }

    /// `asinz(con)`: asin with the argument clamped to -1..1.
    pub fn asinz(mut con: f64) -> f64 {
        if con.abs() > 1.0 {
            if con > 1.0 {
                con = 1.0;
            } else {
                con = -1.0;
            }
        }
        con.asin()
    }

    /// `phi2z(eccent, ts, out flag)`
    pub fn phi2z(eccent: f64, ts: f64) -> Result<(f64, i64), Exception> {
        let flag = 0;
        let eccnth = 0.5 * eccent;
        let mut chi = HALF_PI - 2.0 * ts.atan();
        for _ in 0..=15 {
            let sinpi = chi.sin();
            let con = eccent * sinpi;
            let dphi = HALF_PI - 2.0 * (ts * ((1.0 - con) / (1.0 + con)).powf(eccnth)).atan() - chi;
            chi += dphi;
            if dphi.abs() <= 0.0000000001 {
                return Ok((chi, flag));
            }
        }
        Err(Exception::Argument("Convergence error - phi2z-conv".into()))
    }

    /// `e0fn(x)`
    pub fn e0fn(x: f64) -> f64 {
        1.0 - 0.25 * x * (1.0 + x / 16.0 * (3.0 + 1.25 * x))
    }

    /// `e1fn(x)`
    pub fn e1fn(x: f64) -> f64 {
        0.375 * x * (1.0 + 0.25 * x * (1.0 + 0.46875 * x))
    }

    /// `e2fn(x)`
    pub fn e2fn(x: f64) -> f64 {
        0.05859375 * x * x * (1.0 + 0.75 * x)
    }

    /// `e3fn(x)`
    pub fn e3fn(x: f64) -> f64 {
        x * x * x * (35.0 / 3072.0)
    }

    /// `e4fn(x)`
    pub fn e4fn(x: f64) -> f64 {
        let con = 1.0 + x;
        let com = 1.0 - x;
        (con.powf(con) * com.powf(com)).sqrt()
    }

    /// `mlfn(e0, e1, e2, e3, phi)`: distance along a meridian from the equator.
    pub fn mlfn_series(e0: f64, e1: f64, e2: f64, e3: f64, phi: f64) -> f64 {
        e0 * phi - e1 * (2.0 * phi).sin() + e2 * (4.0 * phi).sin() - e3 * (6.0 * phi).sin()
    }

    /// `mlfn(phi, sphi, cphi)`: meridian distance from the equator to `phi`.
    pub fn mlfn(&self, phi: f64, mut sphi: f64, mut cphi: f64) -> f64 {
        cphi *= sphi;
        sphi *= sphi;
        self.en0 * phi - cphi * (self.en1 + sphi * (self.en2 + sphi * (self.en3 + sphi * self.en4)))
    }

    /// `inv_mlfn(arg)`: latitude from a meridian distance, to 1e-11 radians.
    pub fn inv_mlfn(&self, arg: f64) -> Result<f64, Exception> {
        const MLFN_TOL: f64 = 1E-11;
        const MAXIMUM_ITERATIONS: i32 = 20;
        let k = 1.0 / (1.0 - self.es);
        let mut phi = arg;
        let mut i = MAXIMUM_ITERATIONS;
        loop {
            // rarely goes over 5 iterations
            i -= 1;
            if i < 0 {
                return Err(Exception::InvalidOperation("No convergence".into()));
            }
            let s = phi.sin();
            let mut t = 1.0 - self.es * s * s;
            t = (self.mlfn(phi, s, phi.cos()) - arg) * (t * t.sqrt()) * k;
            phi -= t;
            if t.abs() < MLFN_TOL {
                return Ok(phi);
            }
        }
    }

    /// `CalcUtmZone(lon)`
    pub fn calc_utm_zone(lon: f64) -> i64 {
        crate::dotnet::cast_i64(((lon + 180.0) / 6.0) + 1.0)
    }

    /// `LongitudeToRadians(x, edge)`
    pub fn longitude_to_radians(x: f64, edge: bool) -> Result<f64, Exception> {
        if if edge { x >= -180.0 && x <= 180.0 } else { x > -180.0 && x < 180.0 } {
            return Ok(degrees2radians(x));
        }
        Err(Exception::ArgumentOutOfRange(format!(
            "{} not a valid longitude in degrees.\r\nParameter name: x",
            crate::dotnet::double_to_string_culture(x, &crate::dotnet::INVARIANT_CULTURE)
        )))
    }

    /// `LatitudeToRadians(y, edge)`
    pub fn latitude_to_radians(y: f64, edge: bool) -> Result<f64, Exception> {
        if if edge { y >= -90.0 && y <= 90.0 } else { y > -90.0 && y < 90.0 } {
            return Ok(degrees2radians(y));
        }
        Err(Exception::ArgumentOutOfRange(format!(
            "{} not a valid latitude in degrees.\r\nParameter name: y",
            crate::dotnet::double_to_string_culture(y, &crate::dotnet::INVARIANT_CULTURE)
        )))
    }
}

/// `FlatteningFactor(equatorialRadius, polarRadius)`
fn flattening_factor(equatorial_radius: f64, polar_radius: f64) -> f64 {
    (equatorial_radius - polar_radius) / equatorial_radius
}

/// `EccentricySquared(equatorialRadius, polarRadius)`: es = 2f - f^2.
fn eccentricy_squared(equatorial_radius: f64, polar_radius: f64) -> f64 {
    let f = flattening_factor(equatorial_radius, polar_radius);
    2.0 * f - f * f
}

/// The abstract members of `MapProjection` and the `IMathTransform` operations built on them.
pub trait MapProjectionImpl {
    fn map(&self) -> &MapProjection;

    /// `protected abstract double[] RadiansToMeters(double[] lonlat)`
    fn radians_to_meters(&self, lonlat: &[f64]) -> Result<Vec<f64>, Exception>;

    /// `protected abstract double[] MetersToRadians(double[] p)`
    fn meters_to_radians(&self, p: &[f64]) -> Result<Vec<f64>, Exception>;

    /// `MetersToDegrees(double[] p)`
    fn meters_to_degrees(&self, p: &[f64]) -> Result<Vec<f64>, Exception> {
        let m = self.map();
        let tmp: Vec<f64> = if p.len() == 2 {
            vec![p[0] * m.meters_per_unit - m.false_easting, p[1] * m.meters_per_unit - m.false_northing]
        } else {
            vec![
                p[0] * m.meters_per_unit - m.false_easting,
                p[1] * m.meters_per_unit - m.false_northing,
                p[2] * m.meters_per_unit,
            ]
        };

        let mut res = self.meters_to_radians(&tmp)?;

        res[0] = radians2degrees(res[0]);
        res[1] = radians2degrees(res[1]);
        if m.dim_target() == 3 {
            res[2] = radians2degrees(res[2]);
        }

        Ok(res)
    }

    /// `DegreesToMeters(double[] lonlat)`: a third ordinate goes through the degree to radian
    /// conversion too, as in the C#.
    fn degrees_to_meters(&self, lonlat: &[f64]) -> Result<Vec<f64>, Exception> {
        let m = self.map();
        // Convert to radians
        let tmp: Vec<f64> = if lonlat.len() == 2 {
            vec![degrees2radians(lonlat[0]), degrees2radians(lonlat[1])]
        } else {
            vec![degrees2radians(lonlat[0]), degrees2radians(lonlat[1]), degrees2radians(lonlat[2])]
        };

        // Convert to meters
        let mut res = self.radians_to_meters(&tmp)?;

        // Add false easting and northing, convert to unit
        res[0] = (res[0] + m.false_easting) / m.meters_per_unit;
        res[1] = (res[1] + m.false_northing) / m.meters_per_unit;
        if res.len() == 3 {
            res[2] /= m.meters_per_unit;
        }

        Ok(res)
    }

    /// `Transform(double[] cp)`: forward or inverse depending on `IsInverse`.
    fn map_transform(&self, cp: &[f64]) -> Result<Vec<f64>, Exception> {
        if cp.len() < 2 {
            return Err(Exception::IndexOutOfRange("Index was outside the bounds of the array.".into()));
        }
        if !self.map().is_inverse() {
            self.degrees_to_meters(cp)
        } else {
            self.meters_to_degrees(cp)
        }
    }
}

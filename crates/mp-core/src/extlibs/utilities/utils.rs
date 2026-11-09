//! Port of `ExtLibs/Utilities/Utils.cs`
//!
//! C math wrappers used by code translated from ArduPilot/pymavlink. The `float` helpers keep
//! single-precision arithmetic where the C# does (x64 JIT SSE: each `float` operation rounds
//! to single).

use super::math::MathHelper;
use super::matrix3::Matrix3;
use super::vector3::Vector3;
use crate::dotnet::{ascii_get_string, cast_i32, Exception};

/// `public class Utils`
#[derive(Debug, Clone, Copy, Default)]
pub struct Utils;

/// `Single.Epsilon`
const SINGLE_EPSILON: f32 = 1.401298E-45;

impl Utils {
    pub const FT2M: f64 = 1.0 / 3.2808399;
    pub const KTS2FPS: f64 = 1.68780986;
    pub const PI: f32 = std::f64::consts::PI as f32;
    pub const TRUE: bool = true;
    pub const FALSE: bool = false;

    /// scaling factor from 1e-7 degrees to meters at equater
    pub const LOCATION_SCALING_FACTOR: f32 = 0.011131884502145034;
    /// inverse of LOCATION_SCALING_FACTOR
    pub const LOCATION_SCALING_FACTOR_INV: f32 = 89.83204953368922;

    /// Single precision conversions
    pub const DEG_TO_RAD: f32 = 0.017453292519943295769236907684886;
    pub const RAD_TO_DEG: f32 = 57.295779513082320876798154814105;

    /// `memset(array, start, size)`: `array.Initialize()`, which does nothing for a `byte[]`.
    pub fn memset(_array: &mut [u8], _start: i32, _size: i32) {}

    /// `str(byte[])`: ASCII up to the first NUL.
    pub fn str(input: &[u8]) -> String {
        let st = ascii_get_string(input);
        match st.find('\0') {
            Some(pos) => st[..pos].to_string(),
            None => st,
        }
    }

    /// `is_equal(float, float)`: the difference is a float, then widened.
    pub fn is_equal(f_val1: f32, f_val2: f32) -> bool {
        Self::fabsf((f_val1 - f_val2) as f64) < SINGLE_EPSILON
    }

    /// `is_zero(float)`
    pub fn is_zero(f_val1: f32) -> bool {
        Self::fabsf(f_val1 as f64) < SINGLE_EPSILON
    }

    /// `labs(double)`: `(Int32)Math.Abs(val)`.
    pub fn labs(val: f64) -> i32 {
        cast_i32(val.abs())
    }

    /// `fabsf(double)`
    pub fn fabsf(val: f64) -> f32 {
        val.abs() as f32
    }

    /// `sinf(double)`
    pub fn sinf(val: f64) -> f32 {
        val.sin() as f32
    }

    /// `cosf(double)`
    pub fn cosf(val: f64) -> f32 {
        val.cos() as f32
    }

    pub fn sin(val: f64) -> f64 {
        val.sin()
    }

    pub fn cos(val: f64) -> f64 {
        val.cos()
    }

    pub fn acos(val: f64) -> f64 {
        val.acos()
    }

    /// `acosf(float)`
    pub fn acosf(val: f32) -> f32 {
        (val as f64).acos() as f32
    }

    /// `asinf(float)`
    pub fn asinf(val: f32) -> f32 {
        (val as f64).asin() as f32
    }

    pub fn asin(val: f64) -> f64 {
        val.asin()
    }

    pub fn atan2(val: f64, val2: f64) -> f64 {
        val.atan2(val2)
    }

    /// `free(object)`: assigns null to its own parameter, so it does nothing.
    pub fn free<T>(_inp: T) {}

    pub fn radians(val: f64) -> f64 {
        val * MathHelper::DEG2RAD
    }

    pub fn degrees(val: f64) -> f64 {
        val * MathHelper::RAD2DEG
    }

    pub fn sqrt(val: f64) -> f64 {
        val.sqrt()
    }

    /// `sqrtf(float)`
    pub fn sqrtf(val: f32) -> f32 {
        (val as f64).sqrt() as f32
    }

    pub fn abs(val: f64) -> f64 {
        val.abs()
    }

    pub fn isnan(val: f32) -> bool {
        val.is_nan()
    }

    /// `constrain_float(amt, low, high)`: NaN gives the midpoint.
    pub fn constrain_float(amt: f32, low: f32, high: f32) -> f32 {
        // the check for NaN as a float prevents propogation of
        // floating point errors through any function that uses
        // constrain_float(). The normal float semantics already handle -Inf
        // and +Inf
        if Self::isnan(amt) {
            return (low + high) * 0.5;
        }
        if amt < low {
            low
        } else if amt > high {
            high
        } else {
            amt
        }
    }

    /// `roundf(float)`: `Math.Round(double)` (ties to even).
    pub fn roundf(input: f32) -> f32 {
        (input as f64).round_ties_even() as f32
    }

    /// `safe_asin(float)`: NaN gives 0; out-of-range input clamps to +-PI/2.
    pub fn safe_asin(v: f32) -> f32 {
        if Self::isnan(v) {
            return 0.0;
        }
        if v >= 1.0 {
            return Self::PI / 2.0;
        }
        if v <= -1.0 {
            return -Self::PI / 2.0;
        }
        Self::asinf(v)
    }

    /// `safe_sqrt(float)`: a NaN result (negative input) gives 0.
    pub fn safe_sqrt(v: f32) -> f32 {
        let ret = Self::sqrtf(v);
        if Self::isnan(ret) {
            return 0.0;
        }
        ret
    }

    /// `sq(double)`
    pub fn sq(inp: f64) -> f64 {
        inp * inp
    }

    /// `sq(float)`
    pub fn sqf(inp: f32) -> f32 {
        inp * inp
    }

    /// `inverse3x3(float[] m, float[] invOut)`: false when the matrix is singular. Index errors
    /// (arrays shorter than 9) throw as in C#.
    pub fn inverse3x3(m: &[f32], inv_out: &mut [f32]) -> Result<bool, Exception> {
        if m.len() < 9 {
            return Err(crate::dotnet::index_out_of_range());
        }
        let mut inv = [0f32; 9];
        // computes the inverse of a matrix m
        let det = m[0] * (m[4] * m[8] - m[7] * m[5]) - m[1] * (m[3] * m[8] - m[5] * m[6]) + m[2] * (m[3] * m[7] - m[4] * m[6]);
        if Self::is_zero(det) || det.is_infinite() {
            return Ok(false);
        }

        let invdet = 1.0 / det;

        inv[0] = (m[4] * m[8] - m[7] * m[5]) * invdet;
        inv[1] = (m[2] * m[7] - m[1] * m[8]) * invdet;
        inv[2] = (m[1] * m[5] - m[2] * m[4]) * invdet;
        inv[3] = (m[5] * m[6] - m[3] * m[8]) * invdet;
        inv[4] = (m[0] * m[8] - m[2] * m[6]) * invdet;
        inv[5] = (m[3] * m[2] - m[0] * m[5]) * invdet;
        inv[6] = (m[3] * m[7] - m[6] * m[4]) * invdet;
        inv[7] = (m[6] * m[1] - m[0] * m[7]) * invdet;
        inv[8] = (m[0] * m[4] - m[3] * m[1]) * invdet;

        for i in 0..9 {
            *inv_out.get_mut(i).ok_or_else(crate::dotnet::index_out_of_range)? = inv[i];
        }

        Ok(true)
    }

    /// `inverse4x4(float[] m, float[] invOut)` (gluInvertMatrix): false when singular.
    pub fn inverse4x4(m: &[f32], inv_out: &mut [f32]) -> Result<bool, Exception> {
        if m.len() < 16 {
            return Err(crate::dotnet::index_out_of_range());
        }
        let mut inv = [0f32; 16];

        inv[0] = m[5] * m[10] * m[15] - m[5] * m[11] * m[14] - m[9] * m[6] * m[15] + m[9] * m[7] * m[14] + m[13] * m[6] * m[11]
            - m[13] * m[7] * m[10];

        inv[4] = -m[4] * m[10] * m[15] + m[4] * m[11] * m[14] + m[8] * m[6] * m[15] - m[8] * m[7] * m[14] - m[12] * m[6] * m[11]
            + m[12] * m[7] * m[10];

        inv[8] = m[4] * m[9] * m[15] - m[4] * m[11] * m[13] - m[8] * m[5] * m[15] + m[8] * m[7] * m[13] + m[12] * m[5] * m[11]
            - m[12] * m[7] * m[9];

        inv[12] = -m[4] * m[9] * m[14] + m[4] * m[10] * m[13] + m[8] * m[5] * m[14] - m[8] * m[6] * m[13] - m[12] * m[5] * m[10]
            + m[12] * m[6] * m[9];

        inv[1] = -m[1] * m[10] * m[15] + m[1] * m[11] * m[14] + m[9] * m[2] * m[15] - m[9] * m[3] * m[14] - m[13] * m[2] * m[11]
            + m[13] * m[3] * m[10];

        inv[5] = m[0] * m[10] * m[15] - m[0] * m[11] * m[14] - m[8] * m[2] * m[15] + m[8] * m[3] * m[14] + m[12] * m[2] * m[11]
            - m[12] * m[3] * m[10];

        inv[9] = -m[0] * m[9] * m[15] + m[0] * m[11] * m[13] + m[8] * m[1] * m[15] - m[8] * m[3] * m[13] - m[12] * m[1] * m[11]
            + m[12] * m[3] * m[9];

        inv[13] = m[0] * m[9] * m[14] - m[0] * m[10] * m[13] - m[8] * m[1] * m[14] + m[8] * m[2] * m[13] + m[12] * m[1] * m[10]
            - m[12] * m[2] * m[9];

        inv[2] = m[1] * m[6] * m[15] - m[1] * m[7] * m[14] - m[5] * m[2] * m[15] + m[5] * m[3] * m[14] + m[13] * m[2] * m[7]
            - m[13] * m[3] * m[6];

        inv[6] = -m[0] * m[6] * m[15] + m[0] * m[7] * m[14] + m[4] * m[2] * m[15] - m[4] * m[3] * m[14] - m[12] * m[2] * m[7]
            + m[12] * m[3] * m[6];

        inv[10] = m[0] * m[5] * m[15] - m[0] * m[7] * m[13] - m[4] * m[1] * m[15] + m[4] * m[3] * m[13] + m[12] * m[1] * m[7]
            - m[12] * m[3] * m[5];

        inv[14] = -m[0] * m[5] * m[14] + m[0] * m[6] * m[13] + m[4] * m[1] * m[14] - m[4] * m[2] * m[13] - m[12] * m[1] * m[6]
            + m[12] * m[2] * m[5];

        inv[3] = -m[1] * m[6] * m[11] + m[1] * m[7] * m[10] + m[5] * m[2] * m[11] - m[5] * m[3] * m[10] - m[9] * m[2] * m[7]
            + m[9] * m[3] * m[6];

        inv[7] = m[0] * m[6] * m[11] - m[0] * m[7] * m[10] - m[4] * m[2] * m[11] + m[4] * m[3] * m[10] + m[8] * m[2] * m[7]
            - m[8] * m[3] * m[6];

        inv[11] = -m[0] * m[5] * m[11] + m[0] * m[7] * m[9] + m[4] * m[1] * m[11] - m[4] * m[3] * m[9] - m[8] * m[1] * m[7]
            + m[8] * m[3] * m[5];

        inv[15] = m[0] * m[5] * m[10] - m[0] * m[6] * m[9] - m[4] * m[1] * m[10] + m[4] * m[2] * m[9] + m[8] * m[1] * m[6]
            - m[8] * m[2] * m[5];

        let mut det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

        if Self::is_zero(det) || det.is_infinite() {
            return Ok(false);
        }

        det = 1.0 / det;

        for i in 0..16 {
            *inv_out.get_mut(i).ok_or_else(crate::dotnet::index_out_of_range)? = inv[i] * det;
        }
        Ok(true)
    }

    /// `inverse(float[] x, float[] y, ushort dim)`: 3x3 and 4x4 use the closed forms; any
    /// other size goes through `alglib.rmatrixinverse` in double precision and always returns
    /// true. alglib is a vendored library; `rmatrix_inverse` below is an LU inversion with
    /// partial pivoting that, like alglib, zero-fills a singular matrix. Its last-bit results
    /// can differ from alglib's blocked algorithm.
    pub fn inverse(x: &[f32], y: &mut [f32], dim: u16) -> Result<bool, Exception> {
        match dim {
            3 => Self::inverse3x3(x, y),
            4 => Self::inverse4x4(x, y),
            _ => {
                let n = dim as usize;
                let mut input = vec![0f64; n * n];

                for (a, f) in x.iter().enumerate() {
                    if n == 0 {
                        return Err(Exception::DivideByZero("Attempted to divide by zero.".into()));
                    }
                    if a / n >= n {
                        return Err(crate::dotnet::index_out_of_range());
                    }
                    input[(a / n) * n + a % n] = *f as f64;
                }

                rmatrix_inverse(&mut input, n);

                for (a, f) in input.iter().enumerate() {
                    *y.get_mut(a).ok_or_else(crate::dotnet::index_out_of_range)? = *f as f32;
                }

                Ok(true)
            }
        }
    }

    /// `range(int no)`: 0..no
    pub fn range(no: i32) -> Result<Vec<i32>, Exception> {
        if no < 0 {
            return Err(Exception::Overflow("Arithmetic operation resulted in an overflow.".into()));
        }
        Ok((0..no).collect())
    }

    pub fn fabs(val: f64) -> f64 {
        val.abs()
    }

    pub fn tan(val: f64) -> f64 {
        val.tan()
    }

    /// `len(object[])` / `len<T>(List<T>)`
    pub fn len<T>(data: &[T]) -> i32 {
        data.len() as i32
    }

    /// `EarthRatesToBodyRates(...)`: earth-frame angular rates to body rates (degrees).
    pub fn earth_rates_to_body_rates(
        roll: f64,
        pitch: f64,
        _yaw: f64,
        roll_rate: f64,
        pitch_rate: f64,
        yaw_rate: f64,
    ) -> (f64, f64, f64) {
        //convert the angular velocities from earth frame to
        //body frame. Thanks to James Goppert for the formula

        //all inputs and outputs are in degrees

        //returns a tuple, (p,q,r)

        let phi = Self::radians(roll);
        let theta = Self::radians(pitch);
        let phi_dot = Self::radians(roll_rate);
        let theta_dot = Self::radians(pitch_rate);
        let psi_dot = Self::radians(yaw_rate);

        let p = phi_dot - psi_dot * Self::sin(theta);
        let q = Self::cos(phi) * theta_dot + Self::sin(phi) * psi_dot * Self::cos(theta);
        let r = Self::cos(phi) * psi_dot * Self::cos(theta) - Self::sin(phi) * theta_dot;

        (Self::degrees(p), Self::degrees(q), Self::degrees(r))
    }

    /// `EarthRatesToBodyRatesRyan(...)` (thanks to Ryan Beall)
    pub fn earth_rates_to_body_rates_ryan(roll: f64, pitch: f64, yaw: f64, x: f64, y: f64, z: f64) -> (f64, f64, f64) {
        let phi = Self::radians(roll);
        let theta = Self::radians(pitch);
        let psi = Self::radians((360.0 - yaw) * 1.0);
        let po = Self::radians(x);
        let qo = Self::radians(y);
        let ro = Self::radians(-z);

        let (sin, cos) = (Self::sin, Self::cos);

        let p = po * cos(psi) * cos(theta) - ro * sin(theta) + qo * cos(theta) * sin(psi);

        let q = qo * (cos(phi) * cos(psi) + sin(phi) * sin(psi) * sin(theta))
            - po * (cos(phi) * sin(psi) - cos(psi) * sin(phi) * sin(theta))
            + ro * cos(theta) * sin(phi);

        let r = po * (sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta))
            - qo * (cos(psi) * sin(phi) - cos(phi) * sin(psi) * sin(theta))
            + ro * cos(phi) * cos(theta);

        (Self::degrees(p), Self::degrees(q), Self::degrees(r))
    }

    /// `EarthRatesToBodyRatesMine(...)`: roll and pitch are unused, as in the C#.
    pub fn earth_rates_to_body_rates_mine(
        _roll: f64,
        _pitch: f64,
        yaw: f64,
        roll_rate: f64,
        pitch_rate: f64,
        yaw_rate: f64,
    ) -> (f64, f64, f64) {
        let psi = Self::radians(yaw);
        let po = Self::radians(pitch_rate);
        let ro = Self::radians(yaw_rate);
        let qo = Self::radians(roll_rate);

        let q = po * Self::cos(psi) + qo * Self::sin(psi);

        let p = po * Self::sin(psi) + qo * Self::cos(psi);

        let r = ro;

        (Self::degrees(p), Self::degrees(q), Self::degrees(r))
    }

    /// `OGLtoBCBF(phi, theta, psi, x, y, z)`: X-Plane OpenGL-frame accelerations to body frame.
    pub fn ogl_to_bcbf(phi: f64, theta: f64, psi: f64, x: f64, y: f64, z: f64) -> (f64, f64, f64) {
        // First we shall convert from this East Up South frame, to a more conventional NED (North East Down) frame.
        let x_ned = x * -1.0;
        let y_ned = y * 1.0;
        let z_ned = z * -1.0;

        // Next calculate cos & sin of angles for use in the transformation matrix.
        let cr = phi.cos();
        let cp = theta.cos();
        let cy = psi.cos();
        let sr = phi.sin();
        let sp = theta.sin();
        let sy = psi.sin();

        // CpCy, CpSy, -Sp					| local_ax
        // SrSpCy-CrSy, SrSpSy+CrCy, SrCp	| local_ay
        // CrSpCy+SrSy, CrSpSy-SrCy, CrCp	| local_az
        let x = (x_ned * cp * cy) + (y_ned * cp * sy) - (z_ned * sp);
        let y = (x_ned * ((sr * sp * cy) - (cr * sy))) + (y_ned * ((sr * sp * sy) + (cr * cy))) + (z_ned * sr * cp);
        let z = (x_ned * ((cr * sp * cy) + (sr * sy))) + (y_ned * ((cr * sp * sy) - (sr * cy))) + (z_ned * cr * cp);

        (x, y, z)
    }

    /// `FLIGHTtoBCBF(ref x, ref y, ref z, alpha, beta)`: single precision throughout.
    #[allow(non_snake_case)]
    pub fn FLIGHTtoBCBF(x: &mut f32, y: &mut f32, z: &mut f32, alpha: f32, beta: f32) {
        let ca = (alpha as f64).cos() as f32;
        let cb = (beta as f64).cos() as f32;
        let sa = (alpha as f64).sin() as f32;
        let sb = (beta as f64).sin() as f32;

        let x_plane = (*x * ca * cb) - (*z * sa * cb) - (*y * sb);
        let y_plane = (*z * sa * sb) - (*x * ca * sb) - (*y * cb);
        let z_plane = (*x * sa) + (*z * ca);

        *x = x_plane;
        *y = y_plane;
        *z = z_plane;
    }

    /// `BodyRatesToEarthRates(Matrix3 dcm, Vector3 gyro)`: body to earth angular rates
    /// (radians/s).
    pub fn body_rates_to_earth_rates(dcm: &Matrix3, gyro: &Vector3) -> Vector3 {
        let p = gyro.x;
        let q = gyro.y;
        let r = gyro.z;

        let (phi, mut theta, _psi) = dcm.to_euler();

        let phi_dot = p + Self::tan(theta) * (q * Self::sin(phi) + r * Self::cos(phi));
        let theta_dot = q * Self::cos(phi) - r * Self::sin(phi);
        if Self::fabs(Self::cos(theta)) < 1.0e-20 {
            theta += 1.0e-10;
        }
        let psi_dot = (q * Self::sin(phi) + r * Self::cos(phi)) / Self::cos(theta);

        Vector3::new(phi_dot, theta_dot, psi_dot)
    }

    /// `gps_newpos(lat, lon, bearing, distance, ref latitude, ref longitude)` (an instance
    /// method in C#): extrapolate a position, returned as (latitude, longitude).
    pub fn gps_newpos(&self, lat: f64, lon: f64, bearing: f64, distance: f64) -> (f64, f64) {
        let radius_of_earth = 6378100.0; //# in meters

        let lat1 = Self::radians(lat);
        let lon1 = Self::radians(lon);
        let brng = Self::radians(bearing);
        let dr = distance / radius_of_earth;

        let lat2 = Self::asin(Self::sin(lat1) * Self::cos(dr) + Self::cos(lat1) * Self::sin(dr) * Self::cos(brng));
        let lon2 = lon1 + Self::atan2(Self::sin(brng) * Self::sin(dr) * Self::cos(lat1), Self::cos(dr) - Self::sin(lat1) * Self::sin(lat2));

        (Self::degrees(lat2), Self::degrees(lon2))
    }
}

/// In-place inverse of the row-major `n` x `n` matrix `a` (LU with partial pivoting, then
/// inverting the factors), standing in for `alglib.rmatrixinverse`. A singular matrix is
/// filled with zeros, as alglib does when it reports info = -3.
fn rmatrix_inverse(a: &mut [f64], n: usize) {
    if n == 0 {
        return;
    }
    let mut lu = a.to_vec();
    let mut piv: Vec<usize> = (0..n).collect();
    for k in 0..n {
        let mut p = k;
        let mut maxv = lu[k * n + k].abs();
        for i in k + 1..n {
            let v = lu[i * n + k].abs();
            if v > maxv {
                maxv = v;
                p = i;
            }
        }
        if maxv == 0.0 || !maxv.is_finite() {
            a.iter_mut().for_each(|v| *v = 0.0);
            return;
        }
        if p != k {
            for j in 0..n {
                lu.swap(k * n + j, p * n + j);
            }
            piv.swap(k, p);
        }
        for i in k + 1..n {
            lu[i * n + k] /= lu[k * n + k];
            let l = lu[i * n + k];
            for j in k + 1..n {
                lu[i * n + j] -= l * lu[k * n + j];
            }
        }
    }
    // Solve A X = I column by column: P A = L U.
    let mut inv = vec![0f64; n * n];
    for col in 0..n {
        let mut b: Vec<f64> = (0..n).map(|i| if piv[i] == col { 1.0 } else { 0.0 }).collect();
        for i in 0..n {
            for j in 0..i {
                b[i] -= lu[i * n + j] * b[j];
            }
        }
        for i in (0..n).rev() {
            for j in i + 1..n {
                b[i] -= lu[i * n + j] * b[j];
            }
            b[i] /= lu[i * n + i];
        }
        for i in 0..n {
            inv[i * n + col] = b[i];
        }
    }
    a.copy_from_slice(&inv);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn helpers() {
        assert_eq!(Utils::str(b"abc\0def"), "abc");
        assert_eq!(Utils::constrain_float(f32::NAN, 1.0, 3.0), 2.0);
        assert_eq!(Utils::safe_sqrt(-1.0), 0.0);
        assert_eq!(Utils::roundf(2.5), 2.0);
        let m = [2.0f32, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 8.0];
        let mut out = [0f32; 9];
        assert!(Utils::inverse3x3(&m, &mut out).unwrap());
        assert_eq!(out[0], 0.5);
        let m2 = [2.0f32, 1.0, 1.0, 1.0];
        let mut out2 = [0f32; 4];
        assert!(Utils::inverse(&m2, &mut out2, 2).unwrap());
        assert!((out2[0] - 1.0).abs() < 1e-6 && (out2[1] + 1.0).abs() < 1e-6 && (out2[3] - 2.0).abs() < 1e-6);
    }
}

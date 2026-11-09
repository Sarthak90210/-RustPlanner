//! Port of `ExtLibs/Utilities/Quaternion.cs`

use super::matrix3::Matrix3;
use super::vector3::Vector3;

/// `public class Quaternion`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quaternion {
    pub q1: f64,
    pub q2: f64,
    pub q3: f64,
    pub q4: f64,
}

impl Default for Quaternion {
    /// `new Quaternion()`: no rotation.
    fn default() -> Self {
        Quaternion { q1: 1.0, q2: 0.0, q3: 0.0, q4: 0.0 }
    }
}

impl Quaternion {
    /// `new Quaternion()`: no rotation.
    pub fn identity() -> Self {
        Self::default()
    }

    /// `new Quaternion(q1, q2, q3, q4)`: not normalized.
    pub fn new(q1: f64, q2: f64, q3: f64, q4: f64) -> Self {
        Quaternion { q1, q2, q3, q4 }
    }

    /// `from_rotation_matrix(Matrix3 m)` (Martin John Baker's method).
    pub fn from_rotation_matrix(m: &Matrix3) -> Quaternion {
        let m00 = m.a.x;
        let m11 = m.b.y;
        let m22 = m.c.z;
        let m10 = m.b.x;
        let m01 = m.a.y;
        let m20 = m.c.x;
        let m02 = m.a.z;
        let m21 = m.c.y;
        let m12 = m.b.z;

        let tr = m00 + m11 + m22;

        if tr > 0.0 {
            let s = (tr + 1.0).sqrt() * 2.0;
            Quaternion::new(0.25 * s, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s)
        } else if (m00 > m11) && (m00 > m22) {
            let s = (1.0 + m00 - m11 - m22).sqrt() * 2.0;
            Quaternion::new((m21 - m12) / s, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s)
        } else if m11 > m22 {
            let s = (1.0 + m11 - m00 - m22).sqrt() * 2.0;
            Quaternion::new((m02 - m20) / s, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s)
        } else {
            let s = (1.0 + m22 - m00 - m11).sqrt() * 2.0;
            Quaternion::new((m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s)
        }
    }

    /// `from_euler(roll, pitch, yaw)` (radians)
    pub fn from_euler(roll: f64, pitch: f64, yaw: f64) -> Quaternion {
        let cr2 = (roll * 0.5).cos();
        let cp2 = (pitch * 0.5).cos();
        let cy2 = (yaw * 0.5).cos();
        let sr2 = (roll * 0.5).sin();
        let sp2 = (pitch * 0.5).sin();
        let sy2 = (yaw * 0.5).sin();

        Quaternion::new(
            cr2 * cp2 * cy2 + sr2 * sp2 * sy2,
            sr2 * cp2 * cy2 - cr2 * sp2 * sy2,
            cr2 * sp2 * cy2 + sr2 * cp2 * sy2,
            cr2 * cp2 * sy2 - sr2 * sp2 * cy2,
        )
    }

    /// `from_euler312(roll, pitch, yaw)`: 3-1-2 order (gimbal joints).
    pub fn from_euler312(roll: f64, pitch: f64, yaw: f64) -> Quaternion {
        let mut m = Matrix3::new();
        m.from_euler312(roll, pitch, yaw);

        Self::from_rotation_matrix(&m)
    }

    /// `from_axis_angle(Vector3 axis, double theta)`: the axis must be a unit vector.
    pub fn from_axis_angle_theta(axis: &Vector3, theta: f64) -> Quaternion {
        if theta == 0.0 {
            return Quaternion::default();
        }

        let st2 = (theta / 2.0).sin();

        Quaternion::new((theta / 2.0).cos(), axis.x * st2, axis.y * st2, axis.z * st2)
    }

    /// `from_axis_angle(Vector3 v)`: the length of `v` is the angle in radians.
    pub fn from_axis_angle(v: &Vector3) -> Quaternion {
        let theta = v.length();
        if theta == 0.0 {
            return Quaternion::default();
        }

        Self::from_axis_angle_theta(&(*v / theta), theta)
    }

    /// `is_nan()`
    pub fn is_nan(&self) -> bool {
        self.q1.is_nan() || self.q2.is_nan() || self.q3.is_nan() || self.q4.is_nan()
    }

    /// `rotation_matrix()`
    pub fn rotation_matrix(&self) -> Matrix3 {
        let (q1, q2, q3, q4) = (self.q1, self.q2, self.q3, self.q4);
        let q3q3 = q3 * q3;
        let q3q4 = q3 * q4;
        let q2q2 = q2 * q2;
        let q2q3 = q2 * q3;
        let q2q4 = q2 * q4;
        let q1q2 = q1 * q2;
        let q1q3 = q1 * q3;
        let q1q4 = q1 * q4;
        let q4q4 = q4 * q4;

        let mut m = Matrix3::new();
        m.a.x = 1.0 - 2.0 * (q3q3 + q4q4);
        m.a.y = 2.0 * (q2q3 - q1q4);
        m.a.z = 2.0 * (q2q4 + q1q3);
        m.b.x = 2.0 * (q2q3 + q1q4);
        m.b.y = 1.0 - 2.0 * (q2q2 + q4q4);
        m.b.z = 2.0 * (q3q4 - q1q2);
        m.c.x = 2.0 * (q2q4 - q1q3);
        m.c.y = 2.0 * (q3q4 + q1q2);
        m.c.z = 1.0 - 2.0 * (q2q2 + q3q3);

        m
    }

    /// `rotation_matrix_norm()`: the matrix of the normalized quaternion (this is unchanged).
    pub fn rotation_matrix_norm(&self) -> Matrix3 {
        let (q1, q2, q3, q4) = (self.q1, self.q2, self.q3, self.q4);
        let q1q1 = q1 * q1;
        let q1q2 = q1 * q2;
        let q1q3 = q1 * q3;
        let q1q4 = q1 * q4;
        let q2q2 = q2 * q2;
        let q2q3 = q2 * q3;
        let q2q4 = q2 * q4;
        let q3q3 = q3 * q3;
        let q3q4 = q3 * q4;
        let q4q4 = q4 * q4;
        let invs = 1.0 / (q1q1 + q2q2 + q3q3 + q4q4);

        let mut m = Matrix3::new();
        m.a.x = (q2q2 - q3q3 - q4q4 + q1q1) * invs;
        m.a.y = 2.0 * (q2q3 - q1q4) * invs;
        m.a.z = 2.0 * (q2q4 + q1q3) * invs;
        m.b.x = 2.0 * (q2q3 + q1q4) * invs;
        m.b.y = (-q2q2 + q3q3 - q4q4 + q1q1) * invs;
        m.b.z = 2.0 * (q3q4 - q1q2) * invs;
        m.c.x = 2.0 * (q2q4 - q1q3) * invs;
        m.c.y = 2.0 * (q3q4 + q1q2) * invs;
        m.c.z = (-q2q2 - q3q3 + q4q4 + q1q1) * invs;

        m
    }

    /// `body_to_earth(Vector3 v)`
    pub fn body_to_earth(&self, v: &Vector3) -> Vector3 {
        self.rotation_matrix() * *v
    }

    /// `earth_to_body(Vector3 v)`
    pub fn earth_to_body(&self, v: &Vector3) -> Vector3 {
        self.conjugate().body_to_earth(v)
    }

    /// `to_axis_angle()`: the axis, scaled to the rotation angle in radians.
    pub fn to_axis_angle(&self) -> Vector3 {
        let mut v = Vector3::new(self.q2, self.q3, self.q4);
        let l = v.length();
        if l != 0.0 {
            v /= l; // normalize
            v *= Self::wrap_pi(2.0 * l.atan2(self.q1));
        }
        v
    }

    /// `wrap_PI(radian)`: -PI..PI
    fn wrap_pi(radian: f64) -> f64 {
        let mut res = Self::wrap_2pi(radian);
        if res > std::f64::consts::PI {
            res -= std::f64::consts::PI * 2.0;
        }
        res
    }

    /// `wrap_2PI(radian)`: 0..2PI
    fn wrap_2pi(radian: f64) -> f64 {
        let mut res = radian % (std::f64::consts::PI * 2.0);
        if res < 0.0 {
            res += std::f64::consts::PI * 2.0;
        }
        res
    }

    /// `sq(n)`
    fn sq(n: f64) -> f64 {
        n * n
    }

    /// `get_euler_roll()`
    pub fn get_euler_roll(&self) -> f64 {
        (2.0 * (self.q1 * self.q2 + self.q3 * self.q4)).atan2(1.0 - 2.0 * (self.q2 * self.q2 + self.q3 * self.q3))
    }

    /// `get_euler_pitch()`
    pub fn get_euler_pitch(&self) -> f64 {
        (2.0 * (self.q1 * self.q3 - self.q4 * self.q2)).asin()
    }

    /// `get_euler_yaw()`
    pub fn get_euler_yaw(&self) -> f64 {
        (2.0 * (self.q1 * self.q4 + self.q2 * self.q3)).atan2(1.0 - 2.0 * (self.q3 * self.q3 + self.q4 * self.q4))
    }

    /// `to_euler(out roll, out pitch, out yaw)`
    pub fn to_euler(&self) -> (f64, f64, f64) {
        (self.get_euler_roll(), self.get_euler_pitch(), self.get_euler_yaw())
    }

    /// `length()`
    pub fn length(&self) -> f64 {
        (Self::sq(self.q1) + Self::sq(self.q2) + Self::sq(self.q3) + Self::sq(self.q4)).sqrt()
    }

    /// `conjugate()`: the inverse for unit quaternions.
    pub fn conjugate(&self) -> Quaternion {
        Quaternion::new(self.q1, -self.q2, -self.q3, -self.q4)
    }

    /// `normalize()`
    pub fn normalize(&mut self) {
        let quat_mag = self.length();
        if quat_mag != 0.0 {
            let quat_mag_inv = 1.0 / quat_mag;
            self.q1 *= quat_mag_inv;
            self.q2 *= quat_mag_inv;
            self.q3 *= quat_mag_inv;
            self.q4 *= quat_mag_inv;
        }
    }
}

impl std::ops::Mul for Quaternion {
    type Output = Quaternion;
    /// `operator *(Quaternion w, Quaternion v)`: applies v, then w.
    fn mul(self, v: Quaternion) -> Quaternion {
        let w = self;
        Quaternion::new(
            w.q1 * v.q1 - w.q2 * v.q2 - w.q3 * v.q3 - w.q4 * v.q4,
            w.q1 * v.q2 + w.q2 * v.q1 + w.q3 * v.q4 - w.q4 * v.q3,
            w.q1 * v.q3 - w.q2 * v.q4 + w.q3 * v.q1 + w.q4 * v.q2,
            w.q1 * v.q4 + w.q2 * v.q3 - w.q3 * v.q2 + w.q4 * v.q1,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn euler_and_matrix_agree() {
        let q = Quaternion::from_euler(0.1, 0.2, 0.3);
        let (r, p, y) = q.to_euler();
        assert!((r - 0.1).abs() < 1e-12 && (p - 0.2).abs() < 1e-12 && (y - 0.3).abs() < 1e-12);
        let q2 = Quaternion::from_rotation_matrix(&q.rotation_matrix());
        assert!((q2.q1 - q.q1).abs() < 1e-12 && (q2.q4 - q.q4).abs() < 1e-12);
        let aa = Quaternion::from_axis_angle(&Vector3::new(0.0, 0.0, 0.5)).to_axis_angle();
        assert!((aa.z - 0.5).abs() < 1e-12);
    }
}

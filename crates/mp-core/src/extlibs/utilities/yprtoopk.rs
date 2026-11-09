//! Port of `ExtLibs/Utilities/YPRtoOPK.cs`
//!
//! Yaw/pitch/roll to photogrammetry omega/phi/kappa (Pix4D's method; see the references in
//! the C#).

use super::math::MathHelper;
use super::matrix3::Matrix3;
use super::pointlatlngalt::PointLatLngAlt;
use super::rtcm3::rtcm3;
use super::vector3::Vector3;

/// `public class YPRtoOPK`
pub struct YPRtoOPK;

impl YPRtoOPK {
    /// `Convert(roll, pitch, yaw, plla)`: the tuple named (phi, omega, kappa) in C#, holding
    /// (asin(CEB[0,2]), atan2(-CEB[1,2], CEB[2,2]), atan2(-CEB[0,1], CEB[0,0])) in degrees.
    #[allow(non_snake_case)]
    pub fn Convert(roll: f64, pitch: f64, yaw: f64, plla: &PointLatLngAlt) -> (f64, f64, f64) {
        let mut ypr_matrix = Matrix3::new();

        ypr_matrix.from_euler(roll * MathHelper::DEG2RAD, pitch * MathHelper::DEG2RAD, yaw * MathHelper::DEG2RAD);

        let c_bb = Matrix3::from_floats(0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0);

        let delta = 1e-7;

        let mut p1 = [0f64; 3];
        let mut p2 = [0f64; 3];
        rtcm3::pos2ecef(&[(plla.Lat + delta) * MathHelper::DEG2RAD, plla.Lng * MathHelper::DEG2RAD, plla.Alt], &mut p1);
        rtcm3::pos2ecef(&[(plla.Lat - delta) * MathHelper::DEG2RAD, plla.Lng * MathHelper::DEG2RAD, plla.Alt], &mut p2);
        let mut xnp = Vector3::new(p1[0], p1[1], p1[2]) - Vector3::new(p2[0], p2[1], p2[2]);
        xnp.normalize();

        let znp = Vector3::new(0.0, 0.0, -1.0);
        let ynp = znp.copy() % xnp.copy();

        let c_en = Matrix3::from_rows(xnp, ynp, znp);

        let c_eb = c_en * ypr_matrix * c_bb;

        (
            c_eb.a.z.asin() * MathHelper::RAD2DEG,
            (-c_eb.b.z).atan2(c_eb.c.z) * MathHelper::RAD2DEG,
            (-c_eb.a.y).atan2(c_eb.a.x) * MathHelper::RAD2DEG,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn level_north_facing_camera() {
        // at 0,0 the ECEF north vector xnp is (0,0,1), opposite the C#'s fixed znp (0,0,-1):
        // ynp is zero and CEB[0,2] = xnp . CBb column 2 = -1, so phi = asin(-1)
        let (phi, omega, kappa) = YPRtoOPK::Convert(0.0, 0.0, 0.0, &PointLatLngAlt::new_alt(0.0, 0.0, 0.0));
        assert!((phi + 90.0).abs() < 1e-6 && kappa.is_finite() && omega.is_finite(), "{phi} {omega} {kappa}");
    }
}

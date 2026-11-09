//! Port of `ExtLibs/Utilities/Kalman3D.cs`
//!
//! Philip R. Braica's position/velocity/acceleration Kalman filter (CPOL). It uses
//! `Matrix.MakeIdentity`, whose bug leaves the first diagonal element 0 (see matrix.rs),
//! exactly as the C# does.

use super::matrix::Matrix;
use crate::dotnet::Exception;

/// `public class Kalman3D`
#[derive(Debug, Clone, PartialEq)]
pub struct Kalman3D {
    /// State.
    m_x: Matrix,
    /// Covariance.
    m_p: Matrix,
    /// Minimal covariance.
    m_q: Matrix,
    /// Minimal innovative covariance, keeps filter from locking in to a solution.
    m_r: f64,
    last_gain: f64,
}

impl Kalman3D {
    pub fn new() -> Result<Self, Exception> {
        Ok(Kalman3D { m_x: Matrix::new(1, 3)?, m_p: Matrix::new(3, 3)?, m_q: Matrix::new(3, 3)?, m_r: 0.0, last_gain: 0.0 })
    }

    fn d(m: &Matrix, i: usize) -> f64 {
        m.Data.as_ref().and_then(|d| d.get(i)).copied().unwrap_or(0.0)
    }

    fn set_d(m: &mut Matrix, i: usize, v: f64) -> Result<(), Exception> {
        let d = m
            .Data
            .as_mut()
            .ok_or_else(|| Exception::NullReference("Object reference not set to an instance of an object.".into()))?;
        *d.get_mut(i).ok_or_else(|| Exception::IndexOutOfRange("Index was outside the bounds of the array.".into()))? = v;
        Ok(())
    }

    /// `Value`: the last updated value.
    #[allow(non_snake_case)]
    pub fn Value(&self) -> f64 {
        Self::d(&self.m_x, 0)
    }

    /// `Value` setter: for a sudden absolute measurement.
    #[allow(non_snake_case)]
    pub fn set_Value(&mut self, value: f64) -> Result<(), Exception> {
        Self::set_d(&mut self.m_x, 0, value)
    }

    /// `Velocity`
    #[allow(non_snake_case)]
    pub fn Velocity(&self) -> f64 {
        Self::d(&self.m_x, 1)
    }

    /// `Acceleration`
    #[allow(non_snake_case)]
    pub fn Acceleration(&self) -> f64 {
        Self::d(&self.m_x, 2)
    }

    /// `LastGain`: the determinant of the last gain matrix.
    #[allow(non_snake_case)]
    pub fn LastGain(&self) -> f64 {
        self.last_gain
    }

    /// `Variance()`: last positional variance.
    #[allow(non_snake_case)]
    pub fn Variance(&self) -> f64 {
        Self::d(&self.m_p, 0)
    }

    /// `Predicition(dt)`: X = F*X + H*U, position only.
    #[allow(non_snake_case)]
    pub fn Predicition(&self, dt: f64) -> f64 {
        Self::d(&self.m_x, 0) + (dt * Self::d(&self.m_x, 1)) + (dt * dt * Self::d(&self.m_x, 2))
    }

    fn f(dt: f64) -> Result<Matrix, Exception> {
        Matrix::with_data(3, 3, vec![1.0, dt, dt * dt, 0.0, 1.0, dt, 0.0, 0.0, 1.0])
    }

    /// `Variance(dt)`: predicted position covariance, P = F*P*F^T + Q.
    #[allow(non_snake_case)]
    pub fn VarianceAt(&self, dt: f64) -> Result<f64, Exception> {
        let f = Self::f(dt)?;
        let mut p_predicted = Matrix::MultiplyABAT(&f, &self.m_p)?;
        p_predicted.Add(&self.m_q)?;

        Ok(Self::d(&p_predicted, 0))
    }

    /// `Reset(qx, qv, qa, r, pd, ix)`
    #[allow(non_snake_case)]
    pub fn Reset(&mut self, qx: f64, qv: f64, qa: f64, r: f64, pd: f64, ix: f64) -> Result<(), Exception> {
        let q = [qx * qx, qv * qx, qa * qx, qx * qv, qv * qv, qa * qv, qx * qa, qv * qa, qa * qa];
        for (i, v) in q.iter().enumerate() {
            Self::set_d(&mut self.m_q, i, *v)?;
        }

        self.m_r = r;

        // Diagonal to pd, others to zero.
        let n = self.m_p.Data.as_ref().map(|d| d.len()).unwrap_or(0);
        for i in 0..n {
            Self::set_d(&mut self.m_p, i, 0.0)?;
        }
        Self::set_d(&mut self.m_p, 8, pd)?;
        Self::set_d(&mut self.m_p, 4, pd)?;
        Self::set_d(&mut self.m_p, 0, pd)?;

        // Initial position, velocity and acceleration.
        Self::set_d(&mut self.m_x, 0, ix)?;
        Self::set_d(&mut self.m_x, 1, 0.0)?;
        Self::set_d(&mut self.m_x, 2, 0.0)
    }

    /// `Update(mx, dt)`: position only; velocity and acceleration are synthesised.
    #[allow(non_snake_case)]
    pub fn Update(&mut self, mx: f64, dt: f64) -> Result<f64, Exception> {
        let mv = if dt != 0.0 { (mx - Self::d(&self.m_x, 0)) / dt } else { 0.0 };
        let ma = if dt != 0.0 { (mv - Self::d(&self.m_x, 1)) / dt } else { 0.0 };
        self.Update4(mx, mv, ma, dt)
    }

    /// `Update(mx, mv, dt)`: position and velocity; acceleration is synthesised.
    #[allow(non_snake_case)]
    pub fn Update3(&mut self, mx: f64, mv: f64, dt: f64) -> Result<f64, Exception> {
        let ma = if dt != 0.0 { (mv - Self::d(&self.m_x, 1)) / dt } else { 0.0 };
        self.Update4(mx, mv, ma, dt)
    }

    /// `Update(mx, mv, ma, dt)`: predict to now, then update; returns the new position.
    #[allow(non_snake_case)]
    pub fn Update4(&mut self, mx: f64, mv: f64, ma: f64, dt: f64) -> Result<f64, Exception> {
        // X = F*X + H*U
        let f = Self::f(dt)?;
        let h = Matrix::MakeIdentity(3)?;
        let ht = Matrix::MakeIdentity(3)?; // h is identity, so ht is also identity.

        // U = {0,0};
        self.m_x = Matrix::Multiply(&f, &self.m_x)?;

        // P = F*P*F^T + Q
        self.m_p = Matrix::MultiplyABAT(&f, &self.m_p)?;
        self.m_p.Add(&self.m_q)?;

        // Y = M – H*X
        let y = Matrix::with_data(
            1,
            3,
            vec![mx - Self::d(&self.m_x, 0), mv - Self::d(&self.m_x, 1), ma - Self::d(&self.m_x, 2)],
        )?;

        // S = H*P*H^T + R
        let mut s = Matrix::MultiplyABAT(&h, &self.m_p)?;

        // R is added across the diagnal, reducing by 10% for velocity and accel.,
        // which is often good for motion, and allows us
        // to work off of a single m_r value.
        for i in [0usize, 4, 8] {
            let v = Self::d(&s, i) + self.m_r;
            Self::set_d(&mut s, i, v)?;
        }

        // K = P * H^T *S^-1
        let tmp = Matrix::Multiply(&self.m_p, &ht)?;
        let sinv = Matrix::Invert(&s)?;
        let mut k = Matrix::new(3, 3)?; // inited to zero.

        if let Some(sinv) = sinv {
            k = Matrix::Multiply(&tmp, &sinv)?;
        }
        // Measurements are unstable so make the gain always be between 0 and 1.
        if let Some(kd) = k.Data.as_mut() {
            for g in kd.iter_mut() {
                let gain = *g;
                *g = if gain < 0.0 {
                    0.0
                } else if gain > 0.25 {
                    0.25
                } else {
                    gain
                };
            }
        }

        self.last_gain = k.Determinant()?;

        // X = X + K*Y
        let ky = Matrix::Multiply(&k, &y)?;
        self.m_x.Add(&ky)?;

        // P = (I – K * H) * P
        let mut kh = Matrix::Multiply(&k, &h)?;
        let mut id = Matrix::MakeIdentity(3)?;
        kh.MultiplyScalar(-1.0)?;
        id.Add(&kh)?;
        id.MultiplyInPlace(&self.m_p)?;
        self.m_p.Set(&id)?;

        // Return latest estimate.
        Ok(Self::d(&self.m_x, 0))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tracks_a_constant() {
        let mut k = Kalman3D::new().unwrap();
        k.Reset(0.1, 0.1, 0.1, 1.0, 1.0, 5.0).unwrap();
        assert_eq!(k.Predicition(1.0), 5.0);
        assert_eq!(k.Variance(), 1.0);
        let v = k.Update(10.0, 0.1).unwrap();
        assert!(v.is_finite());
        assert_eq!(k.Value(), v);
    }
}

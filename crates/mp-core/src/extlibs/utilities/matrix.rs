//! Port of `ExtLibs/Utilities/Matrix.cs`
//!
//! Philip R. Braica's small matrix class (CPOL), kept with its bugs, which `Kalman3D`
//! depends on:
//! - `SetIdentity`/`MakeIdentity` start the diagonal counter at 0, so `Data[0]` stays 0 and
//!   only the later diagonal entries are 1; `IsIdentity` has the same counter, so it is false
//!   for a true identity.
//! - the 3x3 `Invert` has the brackets as `det * a - b`, not `det * (a - b)`.
//! - `Resize` to the current size keeps the old `Data` (even one assigned with a different
//!   length), and a matrix created as 0x0 or with the default constructor has null `Data`.
//!
//! `Data` is `Option<Vec<f64>>` for the null case; out-of-range or null accesses return the
//! .NET exception.

use crate::dotnet::Exception;

fn null_ref() -> Exception {
    Exception::NullReference("Object reference not set to an instance of an object.".into())
}

fn oob() -> Exception {
    Exception::IndexOutOfRange("Index was outside the bounds of the array.".into())
}

/// `public class Matrix`
#[allow(non_snake_case)]
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Matrix {
    /// Columns
    m_c: i32,
    /// Rows
    m_r: i32,
    /// `public double[] Data { get; set; }`
    pub Data: Option<Vec<f64>>,
}

impl Matrix {
    /// `MakeIdentity(rank)` (see the module note: `Data[0]` stays 0)
    #[allow(non_snake_case)]
    pub fn MakeIdentity(rank: i32) -> Result<Matrix, Exception> {
        let mut m = Matrix::new(rank, rank)?;
        m.SetIdentity()?;
        Ok(m)
    }

    /// `new Matrix()`: null data.
    pub fn empty() -> Self {
        Matrix { m_c: 0, m_r: 0, Data: None }
    }

    /// `new Matrix(cols, rows)`
    pub fn new(cols: i32, rows: i32) -> Result<Self, Exception> {
        let mut m = Self::empty();
        m.Resize(cols, rows)?;
        Ok(m)
    }

    /// `new Matrix(cols, rows) { Data = data }`
    pub fn with_data(cols: i32, rows: i32, data: Vec<f64>) -> Result<Self, Exception> {
        let mut m = Self::new(cols, rows)?;
        m.Data = Some(data);
        Ok(m)
    }

    /// `new Matrix(Matrix m)`: copy constructor.
    pub fn copy_of(m: &Matrix) -> Result<Self, Exception> {
        let mut rv = Self::empty();
        rv.Set(m)?;
        Ok(rv)
    }

    fn data(&self) -> Result<&Vec<f64>, Exception> {
        self.Data.as_ref().ok_or_else(null_ref)
    }

    fn data_mut(&mut self) -> Result<&mut Vec<f64>, Exception> {
        self.Data.as_mut().ok_or_else(null_ref)
    }

    /// `Set(Matrix m)`: resize to m and copy its data element by element.
    #[allow(non_snake_case)]
    pub fn Set(&mut self, m: &Matrix) -> Result<(), Exception> {
        self.Resize(m.Columns(), m.Rows())?;
        let src = m.data()?.clone();
        let dst = self.data_mut()?;
        for (i, v) in src.iter().enumerate() {
            *dst.get_mut(i).ok_or_else(oob)? = *v;
        }
        Ok(())
    }

    /// `Columns`
    #[allow(non_snake_case)]
    pub fn Columns(&self) -> i32 {
        self.m_c
    }

    #[allow(non_snake_case)]
    pub fn set_Columns(&mut self, value: i32) -> Result<(), Exception> {
        self.Resize(value, self.m_r)
    }

    /// `Rows`
    #[allow(non_snake_case)]
    pub fn Rows(&self) -> i32 {
        self.m_r
    }

    #[allow(non_snake_case)]
    pub fn set_Rows(&mut self, value: i32) -> Result<(), Exception> {
        self.Resize(self.m_c, value)
    }

    /// `Resize(cols, rows)`: a no-op when the size is unchanged.
    #[allow(non_snake_case)]
    pub fn Resize(&mut self, cols: i32, rows: i32) -> Result<(), Exception> {
        if (self.m_c == cols) && (self.m_r == rows) {
            return Ok(());
        }
        self.m_c = cols;
        self.m_r = rows;
        let n = cols.wrapping_mul(rows);
        if n < 0 {
            return Err(Exception::Overflow("Arithmetic operation resulted in an overflow.".into()));
        }
        self.Data = Some(vec![0.0; n as usize]);
        self.Zero()
    }

    /// `Clone()`
    #[allow(non_snake_case)]
    pub fn Clone(&self) -> Result<Matrix, Exception> {
        let mut m = Matrix::empty();
        m.Resize(self.Columns(), self.Rows())?;
        let src = self.data()?.clone();
        let dst = m.data_mut()?;
        for (i, v) in src.iter().enumerate() {
            *dst.get_mut(i).ok_or_else(oob)? = *v;
        }
        Ok(m)
    }

    /// `Get(x, y)`: `Data[x + y * m_c]`.
    #[allow(non_snake_case)]
    pub fn Get(&self, x: i32, y: i32) -> Result<f64, Exception> {
        let idx = x.wrapping_add(y.wrapping_mul(self.m_c));
        if idx < 0 {
            return Err(oob());
        }
        self.data()?.get(idx as usize).copied().ok_or_else(oob)
    }

    /// `Trace(index)`: `Get(index, index)`.
    #[allow(non_snake_case)]
    pub fn Trace(&self, index: i32) -> Result<f64, Exception> {
        self.Get(index, index)
    }

    /// `Set(x, y, v)`
    #[allow(non_snake_case)]
    pub fn SetAt(&mut self, x: i32, y: i32, v: f64) -> Result<(), Exception> {
        let idx = x.wrapping_add(y.wrapping_mul(self.m_c));
        if idx < 0 {
            return Err(oob());
        }
        *self.data_mut()?.get_mut(idx as usize).ok_or_else(oob)? = v;
        Ok(())
    }

    /// `Multiply(double scalar)` in place.
    #[allow(non_snake_case)]
    pub fn MultiplyScalar(&mut self, scalar: f64) -> Result<(), Exception> {
        for d in self.data_mut()?.iter_mut() {
            *d *= scalar;
        }
        Ok(())
    }

    /// `Multiply(Matrix m, double scalar)`
    #[allow(non_snake_case)]
    pub fn MultiplyByScalar(m: &Matrix, scalar: f64) -> Result<Matrix, Exception> {
        let mut rv = m.Clone()?;
        rv.MultiplyScalar(scalar)?;
        Ok(rv)
    }

    /// `Multiply(Matrix a, Matrix b)`: a * b over the shared inner dimension.
    #[allow(non_snake_case)]
    pub fn Multiply(a: &Matrix, b: &Matrix) -> Result<Matrix, Exception> {
        let mut rv = Matrix::new(b.Columns(), a.Rows())?;
        let min = if a.Columns() < b.Rows() { a.Columns() } else { b.Rows() };
        for i in 0..a.Rows() {
            for j in 0..b.Columns() {
                let mut s = 0.0;
                for k in 0..min {
                    let av = a.Get(k, i)?;
                    let bv = b.Get(j, k)?;
                    s += av * bv;
                }
                rv.SetAt(j, i, s)?;
            }
        }
        Ok(rv)
    }

    /// `Multiply(Matrix b)` in place: this * b.
    #[allow(non_snake_case)]
    pub fn MultiplyInPlace(&mut self, b: &Matrix) -> Result<(), Exception> {
        let tmp = Matrix::Multiply(self, b)?;
        self.Set(&tmp)
    }

    /// `MultiplyABAT(a, b)`: a * b * a^T.
    #[allow(non_snake_case)]
    pub fn MultiplyABAT(a: &Matrix, b: &Matrix) -> Result<Matrix, Exception> {
        let mut rv = Matrix::Multiply(a, b)?;
        let t = Matrix::TransposeOf(a)?;
        rv.MultiplyInPlace(&t)?;
        Ok(rv)
    }

    /// `Add(Matrix a, double scalar)`
    #[allow(non_snake_case)]
    pub fn AddScalarTo(a: &Matrix, scalar: f64) -> Result<Matrix, Exception> {
        let mut rv = Matrix::copy_of(a)?;
        rv.AddScalar(scalar)?;
        Ok(rv)
    }

    /// `Add(double scalar)` in place.
    #[allow(non_snake_case)]
    pub fn AddScalar(&mut self, scalar: f64) -> Result<(), Exception> {
        for d in self.data_mut()?.iter_mut() {
            *d += scalar;
        }
        Ok(())
    }

    /// `Add(Matrix a, Matrix b)`
    #[allow(non_snake_case)]
    pub fn AddMatrices(a: &Matrix, b: &Matrix) -> Result<Matrix, Exception> {
        let mut rv = Matrix::copy_of(a)?;
        rv.Add(b)?;
        Ok(rv)
    }

    /// `Add(Matrix a)` in place: element by element over this matrix's length.
    #[allow(non_snake_case)]
    pub fn Add(&mut self, a: &Matrix) -> Result<(), Exception> {
        let src = a.data()?.clone();
        for (i, d) in self.data_mut()?.iter_mut().enumerate() {
            *d += *src.get(i).ok_or_else(oob)?;
        }
        Ok(())
    }

    /// `Subtract(Matrix a, double scalar)`
    #[allow(non_snake_case)]
    pub fn SubtractScalarFrom(a: &Matrix, scalar: f64) -> Result<Matrix, Exception> {
        let mut rv = Matrix::copy_of(a)?;
        rv.SubtractScalar(scalar)?;
        Ok(rv)
    }

    /// `Subtract(double scalar)` in place.
    #[allow(non_snake_case)]
    pub fn SubtractScalar(&mut self, scalar: f64) -> Result<(), Exception> {
        for d in self.data_mut()?.iter_mut() {
            *d -= scalar;
        }
        Ok(())
    }

    /// `Subtract(Matrix a, Matrix b)`
    #[allow(non_snake_case)]
    pub fn SubtractMatrices(a: &Matrix, b: &Matrix) -> Result<Matrix, Exception> {
        let mut rv = Matrix::copy_of(a)?;
        rv.Subtract(b)?;
        Ok(rv)
    }

    /// `Subtract(Matrix a)` in place.
    #[allow(non_snake_case)]
    pub fn Subtract(&mut self, a: &Matrix) -> Result<(), Exception> {
        let src = a.data()?.clone();
        for (i, d) in self.data_mut()?.iter_mut().enumerate() {
            *d -= *src.get(i).ok_or_else(oob)?;
        }
        Ok(())
    }

    /// `Transpose(Matrix m)`
    #[allow(non_snake_case)]
    pub fn TransposeOf(m: &Matrix) -> Result<Matrix, Exception> {
        let mut rv = Matrix::new(m.m_r, m.m_c)?;
        for i in 0..m.m_c {
            for j in 0..m.m_r {
                rv.SetAt(j, i, m.Get(i, j)?)?;
            }
        }
        Ok(rv)
    }

    /// `Transpose()` in place.
    #[allow(non_snake_case)]
    pub fn Transpose(&mut self) -> Result<(), Exception> {
        let rv = Matrix::TransposeOf(self)?;
        self.Set(&rv)
    }

    /// `IsIdentity()` (with the counter bug: false for a true identity)
    #[allow(non_snake_case)]
    pub fn IsIdentity(&self) -> Result<bool, Exception> {
        if self.m_c != self.m_r {
            return Ok(false);
        }
        let check = self.m_c + 1;
        let mut j = 0;
        for d in self.data()? {
            if j == check {
                j = 0;
                if *d != 1.0 {
                    return Ok(false);
                }
            } else if *d != 0.0 {
                return Ok(false);
            }
            j += 1;
        }
        Ok(true)
    }

    /// `SetIdentity()` (with the counter bug: `Data[0]` is set to 0)
    #[allow(non_snake_case)]
    pub fn SetIdentity(&mut self) -> Result<(), Exception> {
        if self.m_c != self.m_r {
            return Ok(());
        }
        let check = self.m_c + 1;
        let mut j = 0;
        for d in self.data_mut()?.iter_mut() {
            *d = if j == check { 1.0 } else { 0.0 };
            j = if j == check { 1 } else { j + 1 };
        }
        Ok(())
    }

    /// `Zero()`
    #[allow(non_snake_case)]
    pub fn Zero(&mut self) -> Result<(), Exception> {
        for d in self.data_mut()?.iter_mut() {
            *d = 0.0;
        }
        Ok(())
    }

    /// `Determinant`: 1x1, 2x2 and 3x3 only, otherwise 0.
    #[allow(non_snake_case)]
    pub fn Determinant(&self) -> Result<f64, Exception> {
        if self.m_c != self.m_r {
            return Ok(0.0);
        }

        if self.m_c == 0 {
            return Ok(0.0);
        }
        let d = self.data()?;
        let at = |i: usize| d.get(i).copied().ok_or_else(oob);
        if self.m_c == 1 {
            return at(0);
        }
        if self.m_c == 2 {
            return Ok((at(0)? * at(3)?) - (at(1)? * at(2)?));
        }
        if self.m_c == 3 {
            return Ok((at(0)? * ((at(8)? * at(4)?) - (at(7)? * at(5)?))) - (at(3)? * ((at(8)? * at(1)?) - (at(7)? * at(2)?)))
                + (at(6)? * ((at(5)? * at(1)?) - (at(4)? * at(2)?))));
        }

        // only supporting 1x1, 2x2 and 3x3
        Ok(0.0)
    }

    /// `Invert(Matrix m)`: `null` for a non-square or singular matrix. The 3x3 entries keep
    /// the C#'s bracket bug.
    #[allow(non_snake_case)]
    pub fn Invert(m: &Matrix) -> Result<Option<Matrix>, Exception> {
        if m.m_c != m.m_r {
            return Ok(None);
        }
        let mut det = m.Determinant()?;
        if det == 0.0 {
            return Ok(None);
        }

        let mut rv = Matrix::copy_of(m)?;
        let md = m.data()?.clone();
        let at = |i: usize| md.get(i).copied().ok_or_else(oob);
        {
            let r = rv.data_mut()?;
            if m.m_c == 1 {
                let v = 1.0 / *r.first().ok_or_else(oob)?;
                r[0] = v;
            }
        }
        det = 1.0 / det;
        let r = rv.data_mut()?;
        let mut put = |i: usize, v: f64| -> Result<(), Exception> {
            *r.get_mut(i).ok_or_else(oob)? = v;
            Ok(())
        };
        if m.m_c == 2 {
            put(0, det * at(3)?)?;
            put(3, det * at(0)?)?;
            put(1, -det * at(2)?)?;
            put(2, -det * at(1)?)?;
        }
        if m.m_c == 3 {
            put(0, det * (at(8)? * at(4)?) - (at(7)? * at(5)?))?;
            put(1, -det * (at(8)? * at(1)?) - (at(7)? * at(2)?))?;
            put(2, det * (at(5)? * at(1)?) - (at(4)? * at(2)?))?;

            put(3, -det * (at(8)? * at(3)?) - (at(6)? * at(5)?))?;
            put(4, det * (at(8)? * at(0)?) - (at(6)? * at(2)?))?;
            put(5, -det * (at(5)? * at(0)?) - (at(3)? * at(2)?))?;

            put(6, det * (at(7)? * at(3)?) - (at(6)? * at(4)?))?;
            put(7, -det * (at(7)? * at(0)?) - (at(6)? * at(2)?))?;
            put(8, det * (at(4)? * at(0)?) - (at(3)? * at(1)?))?;
        }
        Ok(Some(rv))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity_bug_and_basics() {
        let id = Matrix::MakeIdentity(3).unwrap();
        assert_eq!(id.Data.as_ref().unwrap(), &vec![0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]);
        let mut real = Matrix::new(3, 3).unwrap();
        real.Data = Some(vec![1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]);
        assert!(!real.IsIdentity().unwrap());
        assert_eq!(real.Determinant().unwrap(), 1.0);
        let a = Matrix::with_data(2, 2, vec![1.0, 2.0, 3.0, 4.0]).unwrap();
        let p = Matrix::Multiply(&a, &a).unwrap();
        assert_eq!(p.Data.unwrap(), vec![7.0, 10.0, 15.0, 22.0]);
        assert!(Matrix::new(0, 0).unwrap().Data.is_none());
        assert!(Matrix::Invert(&Matrix::with_data(2, 2, vec![1.0, 2.0, 2.0, 4.0]).unwrap()).unwrap().is_none());
    }
}

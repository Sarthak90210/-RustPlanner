//! Port of `ExtLibs/Core/Geometry/SizeE.cs`
//!
//! `IXmlSerializable.ReadXml`/`WriteXml` read and write the element's text content, so they
//! take and return that text here (`r.ReadElementContentAsString()` / `w.WriteString`).

use crate::dotnet::convert::to_double_str;
use crate::dotnet::drawing::{Size, SizeF};
use crate::dotnet::{cast_i32, double_to_string, Exception};
use crate::extlibs::core::utils::stringutils::{list_get, StringUtils};

/// `public class SizeE : IXmlSerializable`
#[derive(Debug, Clone, Default, PartialEq)]
pub struct SizeE {
    m_width: f64,
    m_height: f64,
}

impl SizeE {
    /// `new SizeE(double width, double height)`
    pub fn new(width: f64, height: f64) -> Self {
        SizeE { m_width: width, m_height: height }
    }

    /// `new SizeE(PointE pt)`
    pub fn from_point(pt: &super::pointe::PointE) -> Self {
        SizeE::new(pt.x(), pt.y())
    }

    /// `Width`
    pub fn width(&self) -> f64 {
        self.m_width
    }

    /// `Width` setter
    pub fn set_width(&mut self, v: f64) {
        self.m_width = v;
    }

    /// `Height`
    pub fn height(&self) -> f64 {
        self.m_height
    }

    /// `Height` setter
    pub fn set_height(&mut self, v: f64) {
        self.m_height = v;
    }

    /// `IsEmpty`: either dimension zero.
    pub fn is_empty(&self) -> bool {
        self.m_height == 0.0 || self.m_width == 0.0
    }

    /// `operator +(SizeE c)`
    pub fn plus(&self) -> SizeE {
        SizeE::new(self.width(), self.height())
    }

    /// `operator *(SizeE a, float scalar)`
    pub fn mul_f32(&self, scalar: f32) -> SizeE {
        SizeE::new(self.width() * scalar as f64, self.height() * scalar as f64)
    }

    /// `operator /(SizeE a, float scalar)`
    pub fn div_f32(&self, scalar: f32) -> SizeE {
        SizeE::new(self.width() / scalar as f64, self.height() / scalar as f64)
    }

    /// `implicit operator SizeE(SizeF)`
    pub fn from_size_f(s: SizeF) -> SizeE {
        SizeE::new(s.Width as f64, s.Height as f64)
    }

    /// `explicit operator SizeF(SizeE)`
    pub fn to_size_f(&self) -> SizeF {
        SizeF { Width: self.m_width as f32, Height: self.m_height as f32 }
    }

    /// `implicit operator SizeE(Size)`
    pub fn from_size(s: Size) -> SizeE {
        SizeE::new(s.Width as f64, s.Height as f64)
    }

    /// `explicit operator Size(SizeE)`
    pub fn to_size(&self) -> Size {
        Size { Width: cast_i32(self.m_width), Height: cast_i32(self.m_height) }
    }

    /// `ReadXml(XmlReader r)` given the element content.
    pub fn read_xml(&mut self, content: &str) -> Result<(), Exception> {
        self.deserialize(Some(content))
    }

    /// `WriteXml(XmlWriter w)`: the text written.
    pub fn write_xml(&self) -> String {
        self.serialize()
    }

    /// `Serialize()`: current-culture numbers (a `,` decimal separator makes this ambiguous).
    pub fn serialize(&self) -> String {
        format!("{},{}", double_to_string(self.m_width), double_to_string(self.m_height))
    }

    /// `Deserialize(string str)`
    pub fn deserialize(&mut self, str: Option<&str>) -> Result<(), Exception> {
        let bits = StringUtils::split_to_list(str, ",")?;
        self.m_width = to_double_str(Some(list_get(&bits, 0)?))?;
        self.m_height = to_double_str(Some(list_get(&bits, 1)?))?;
        Ok(())
    }

    /// `Inflate(double amount)`
    pub fn inflate(&self, amount: f64) -> SizeE {
        self.inflate_xy(amount, amount)
    }

    /// `Inflate(SizeE amount)`
    pub fn inflate_size(&self, amount: &SizeE) -> SizeE {
        self.inflate_xy(amount.width(), amount.height())
    }

    /// `Inflate(double X, double Y)`: grows by twice each amount.
    pub fn inflate_xy(&self, x: f64, y: f64) -> SizeE {
        SizeE::new(self.m_width + 2.0 * x, self.m_height + 2.0 * y)
    }

    /// `SetMaxDim(double maxSize)`
    pub fn set_max_dim(&mut self, max_size: f64) {
        if self.m_height > self.m_width {
            self.m_width = max_size * self.m_width / self.m_height;
            self.m_height = max_size;
        } else {
            self.m_height = max_size * self.m_height / self.m_width;
            self.m_width = max_size;
        }
    }

    /// `SetMinDim(double minSize)`
    pub fn set_min_dim(&mut self, min_size: f64) {
        if self.m_height > self.m_width {
            self.m_height = min_size * self.m_height / self.m_width;
            self.m_width = min_size;
        } else {
            self.m_width = min_size * self.m_width / self.m_height;
            self.m_height = min_size;
        }
    }
}

impl std::ops::Neg for &SizeE {
    type Output = SizeE;
    /// `operator -(SizeE c)`
    fn neg(self) -> SizeE {
        SizeE::new(-self.width(), -self.height())
    }
}

impl std::ops::Add for &SizeE {
    type Output = SizeE;
    /// `operator +(SizeE a, SizeE b)`
    fn add(self, b: &SizeE) -> SizeE {
        SizeE::new(self.width() + b.width(), self.height() + b.height())
    }
}

impl std::ops::Sub for &SizeE {
    type Output = SizeE;
    /// `operator -(SizeE a, SizeE b)`
    fn sub(self, b: &SizeE) -> SizeE {
        SizeE::new(self.width() - b.width(), self.height() - b.height())
    }
}

impl std::ops::Mul for &SizeE {
    type Output = SizeE;
    /// `operator *(SizeE a, SizeE b)`
    fn mul(self, b: &SizeE) -> SizeE {
        SizeE::new(self.width() * b.width(), self.height() * b.height())
    }
}

impl std::ops::Mul<f64> for &SizeE {
    type Output = SizeE;
    /// `operator *(SizeE a, double scalar)`
    fn mul(self, scalar: f64) -> SizeE {
        SizeE::new(self.width() * scalar, self.height() * scalar)
    }
}

impl std::ops::Div for &SizeE {
    type Output = SizeE;
    /// `operator /(SizeE a, SizeE b)`
    fn div(self, b: &SizeE) -> SizeE {
        SizeE::new(self.width() / b.width(), self.height() / b.height())
    }
}

impl std::ops::Div<f64> for &SizeE {
    type Output = SizeE;
    /// `operator /(SizeE a, double scalar)`
    fn div(self, scalar: f64) -> SizeE {
        SizeE::new(self.width() / scalar, self.height() / scalar)
    }
}

/// `internal class SizeEExpandableConverter : ExpandableObjectConverter`
pub struct SizeEExpandableConverter;

impl SizeEExpandableConverter {
    /// `CanConvertTo(context, typeof(string))`
    pub fn can_convert_to_string() -> bool {
        true
    }

    /// `CanConvertFrom(context, typeof(string))`
    pub fn can_convert_from_string() -> bool {
        true
    }

    /// `ConvertTo(context, culture, value, typeof(string))`: `"W, H"` in the current culture
    /// (not the `culture` argument); a null value converts to "" as `TypeConverter` does.
    pub fn convert_to_string(value: Option<&SizeE>) -> String {
        match value {
            Some(size) => format!("{}, {}", double_to_string(size.width()), double_to_string(size.height())),
            None => String::new(),
        }
    }

    /// `ConvertFrom(context, culture, value)` for a string value (None for a null value): any
    /// parse failure becomes the comma message; text without a comma falls to the base
    /// converter, which cannot convert it.
    pub fn convert_from(value: Option<&str>) -> Result<SizeE, Exception> {
        if let Some(str) = value {
            if let Some(commapos) = str.find(',') {
                let parse = || -> Result<SizeE, Exception> {
                    let width = to_double_str(Some(str[..commapos].trim()))?;
                    let height = to_double_str(Some(str[commapos + 1..].trim()))?;
                    Ok(SizeE::new(width, height))
                };
                return parse().map_err(|_| Exception::Argument("Please enter a width and height seperated by a comma.".into()));
            }
            return Err(Exception::NotSupported("SizeEExpandableConverter cannot convert from System.String.".into()));
        }
        Err(Exception::NotSupported("SizeEExpandableConverter cannot convert from (null).".into()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn size_ops() {
        let s = SizeE::new(4.0, 2.0);
        assert_eq!(s.inflate(1.0), SizeE::new(6.0, 4.0));
        assert_eq!(&s * 2.0, SizeE::new(8.0, 4.0));
        assert_eq!(-&s, SizeE::new(-4.0, -2.0));
        assert!(SizeE::new(0.0, 3.0).is_empty());
        let mut m = SizeE::new(4.0, 2.0);
        m.set_max_dim(10.0);
        assert_eq!(m, SizeE::new(10.0, 5.0));
        let mut n = SizeE::new(4.0, 2.0);
        n.set_min_dim(1.0);
        assert_eq!(n, SizeE::new(2.0, 1.0));
        assert_eq!(SizeE::new(2.9, -2.9).to_size(), Size { Width: 2, Height: -2 });
        let mut d = SizeE::default();
        assert!(d.deserialize(Some("3")).is_err());
        assert_eq!(d.width(), 3.0);
    }

    #[test]
    fn converter() {
        assert_eq!(SizeEExpandableConverter::convert_to_string(None), "");
        assert_eq!(SizeEExpandableConverter::convert_from(Some(" 3 , 4 ")).unwrap(), SizeE::new(3.0, 4.0));
        assert_eq!(
            SizeEExpandableConverter::convert_from(Some("a,4")).unwrap_err().message(),
            "Please enter a width and height seperated by a comma."
        );
        assert!(matches!(SizeEExpandableConverter::convert_from(Some("34")), Err(Exception::NotSupported(_))));
    }
}

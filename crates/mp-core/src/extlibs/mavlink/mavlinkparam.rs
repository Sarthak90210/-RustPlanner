//! Port of `ExtLibs/Mavlink/MAVLinkParam.cs`

use super::mavlink::MAV_PARAM_TYPE;
use crate::dotnet::{self, Exception};

/// `MAVLink.MAVLinkParam`
#[derive(Debug, Clone, PartialEq)]
pub struct MAVLinkParam {
    /// Parameter name
    pub name: String,
    /// Over the wire storage format
    pub r#type: MAV_PARAM_TYPE,
    /// Default value of the parameter; `None` when not available.
    pub default_value: Option<f64>,
    _typeap: MAV_PARAM_TYPE,
    _data: [u8; 4],
}

impl MAVLinkParam {
    /// `MAVLinkParam(string name, double value, MAV_PARAM_TYPE wiretype, double? _default_value = null)`:
    /// used as a generic input to type the input data.
    pub fn new(name: &str, value: f64, wiretype: MAV_PARAM_TYPE, default_value: Option<f64>) -> Self {
        let mut p = MAVLinkParam { name: name.to_string(), r#type: wiretype, default_value: None, _typeap: MAV_PARAM_TYPE(0), _data: [0; 4] };
        p.set_value(value);
        p.default_value = default_value;
        p
    }

    /// `MAVLinkParam(string name, byte[] inputwire, MAV_PARAM_TYPE wiretype, MAV_PARAM_TYPE typeap, double? _default_value = null)`:
    /// used to set ArduPilot params.
    pub fn from_wire(name: &str, inputwire: &[u8], wiretype: MAV_PARAM_TYPE, typeap: MAV_PARAM_TYPE, default_value: Option<f64>) -> Self {
        let mut data = [0u8; 4];
        data.copy_from_slice(&inputwire[..4]);
        MAVLinkParam { name: name.to_string(), r#type: wiretype, default_value, _typeap: typeap, _data: data }
    }

    /// `TypeAP`: the ArduPilot storage type when known, otherwise the wire type.
    pub fn type_ap(&self) -> MAV_PARAM_TYPE {
        if self._typeap.0 != 0 {
            return self._typeap;
        }
        self.r#type
    }

    pub fn set_type_ap(&mut self, value: MAV_PARAM_TYPE) {
        self._typeap = value;
    }

    fn uint8_value(&self) -> u8 {
        self._data[0]
    }
    fn int8_value(&self) -> i8 {
        self._data[0] as i8
    }
    fn uint16_value(&self) -> u16 {
        u16::from_le_bytes([self._data[0], self._data[1]])
    }
    fn int16_value(&self) -> i16 {
        i16::from_le_bytes([self._data[0], self._data[1]])
    }
    fn uint32_value(&self) -> u32 {
        u32::from_le_bytes(self._data)
    }
    fn int32_value(&self) -> i32 {
        i32::from_le_bytes(self._data)
    }

    pub fn float_value(&self) -> f32 {
        f32::from_le_bytes(self._data)
    }

    pub fn data(&self) -> &[u8; 4] {
        &self._data
    }

    /// The `data` setter: the bytes resized to four.
    pub fn set_data(&mut self, value: &[u8]) {
        let mut d = [0u8; 4];
        let n = value.len().min(4);
        d[..n].copy_from_slice(&value[..n]);
        self._data = d;
    }

    /// `Value` getter (`GetValue()`); a wire type outside the 8/16/32-bit and REAL32 set throws.
    pub fn value(&self) -> Result<f64, Exception> {
        self.get_value()
    }

    pub fn get_value(&self) -> Result<f64, Exception> {
        Ok(match self.r#type {
            MAV_PARAM_TYPE::UINT8 => self.uint8_value() as f64,
            MAV_PARAM_TYPE::INT8 => self.int8_value() as f64,
            MAV_PARAM_TYPE::UINT16 => self.uint16_value() as f64,
            MAV_PARAM_TYPE::INT16 => self.int16_value() as f64,
            MAV_PARAM_TYPE::UINT32 => self.uint32_value() as f64,
            MAV_PARAM_TYPE::INT32 => self.int32_value() as f64,
            // (double)0.8f is 0.800000011920929; round to the float's 7 significant digits.
            MAV_PARAM_TYPE::REAL32 => Self::round_to_significant_digits(self.float_value() as f64, 7),
            _ => return Err(Exception::Format("invalid type".into())),
        })
    }

    fn round_to_significant_digits(d: f64, digits: i32) -> f64 {
        if d == 0.0 {
            return 0.0;
        }
        let scale = 10f64.powf(d.abs().log10().floor() + 1.0);
        scale * dotnet::math_round(d / scale, digits)
    }

    /// `Value` setter (`SetValue(double)`); other wire types leave the data unchanged.
    pub fn set_value(&mut self, input: f64) {
        match self.r#type {
            MAV_PARAM_TYPE::UINT8 => self.set_data(&[dotnet::cast_u8(input)]),
            MAV_PARAM_TYPE::INT8 => self.set_data(&[dotnet::cast_i8(input) as u8]),
            MAV_PARAM_TYPE::UINT16 => self.set_data(&dotnet::cast_u16(input).to_le_bytes()),
            MAV_PARAM_TYPE::INT16 => self.set_data(&dotnet::cast_i16(input).to_le_bytes()),
            MAV_PARAM_TYPE::UINT32 => self.set_data(&dotnet::cast_u32(input).to_le_bytes()),
            MAV_PARAM_TYPE::INT32 => self.set_data(&dotnet::cast_i32(input).to_le_bytes()),
            MAV_PARAM_TYPE::REAL32 => self.set_data(&(input as f32).to_le_bytes()),
            _ => {}
        }
    }

    /// `(byte)param` and the other explicit conversions: `(T)param.Value`.
    pub fn as_u8(&self) -> Result<u8, Exception> {
        Ok(dotnet::cast_u8(self.value()?))
    }
    pub fn as_i8(&self) -> Result<i8, Exception> {
        Ok(dotnet::cast_i8(self.value()?))
    }
    pub fn as_i16(&self) -> Result<i16, Exception> {
        Ok(dotnet::cast_i16(self.value()?))
    }
    pub fn as_u16(&self) -> Result<u16, Exception> {
        Ok(dotnet::cast_u16(self.value()?))
    }
    pub fn as_i32(&self) -> Result<i32, Exception> {
        Ok(dotnet::cast_i32(self.value()?))
    }
    pub fn as_u32(&self) -> Result<u32, Exception> {
        Ok(dotnet::cast_u32(self.value()?))
    }
    pub fn as_f32(&self) -> Result<f32, Exception> {
        Ok(self.value()? as f32)
    }
    pub fn as_f64(&self) -> Result<f64, Exception> {
        self.value()
    }

    /// `ToString()`: REAL32 formats the float, everything else the double.
    pub fn to_cs_string(&self) -> Result<String, Exception> {
        if self.r#type == MAV_PARAM_TYPE::REAL32 {
            return Ok(dotnet::float_to_string(self.as_f32()?));
        }
        Ok(dotnet::double_to_string(self.value()?))
    }

    /// `default_value_to_string()`
    pub fn default_value_to_string(&self) -> String {
        match self.default_value {
            None => "NaN".into(),
            Some(d) if self.r#type == MAV_PARAM_TYPE::REAL32 => dotnet::float_to_string(d as f32),
            Some(d) => dotnet::double_to_string(d),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn real32_rounds_to_seven_digits() {
        let p = MAVLinkParam::new("X", 0.8, MAV_PARAM_TYPE::REAL32, None);
        assert_eq!(p.float_value(), 0.8f32);
        assert_eq!(p.value().unwrap(), 0.8);
        assert_eq!(p.to_cs_string().unwrap(), "0.8");
    }

    #[test]
    fn integer_types_wrap_like_csharp_casts() {
        let p = MAVLinkParam::new("X", 300.0, MAV_PARAM_TYPE::UINT8, Some(1.0));
        assert_eq!(p.data(), &[44, 0, 0, 0]);
        assert_eq!(p.value().unwrap(), 44.0);
        assert_eq!(p.default_value_to_string(), "1");
        let p = MAVLinkParam::new("X", -1.0, MAV_PARAM_TYPE::INT16, None);
        assert_eq!(p.value().unwrap(), -1.0);
        assert_eq!(p.default_value_to_string(), "NaN");
        let p = MAVLinkParam::new("X", 1.0, MAV_PARAM_TYPE::REAL64, None);
        assert!(matches!(p.value(), Err(Exception::Format(_))));
    }

    #[test]
    fn type_ap_falls_back_to_wire_type() {
        let p = MAVLinkParam::from_wire("X", &[1, 0, 0, 0], MAV_PARAM_TYPE::REAL32, MAV_PARAM_TYPE(0), None);
        assert_eq!(p.type_ap(), MAV_PARAM_TYPE::REAL32);
        let p = MAVLinkParam::from_wire("X", &[1, 0, 0, 0], MAV_PARAM_TYPE::REAL32, MAV_PARAM_TYPE::INT8, None);
        assert_eq!(p.type_ap(), MAV_PARAM_TYPE::INT8);
    }
}

//! Port of `ExtLibs/Mavlink/component_information/Parameter.cs`
//!
//! QuickType classes for the COMP_METADATA_TYPE_PARAMETER JSON schema, with the Newtonsoft
//! settings and converters of the C# file expressed as serde attributes and impls: properties
//! marked `NullValueHandling.Ignore` are skipped when null, `parameters` is always written,
//! and the union, type and scope converters accept and emit the same tokens.

use serde::de::{self, Deserializer};
use serde::ser::Serializer;
use serde::{Deserialize, Serialize};
use serde_json::Value;

/// Schema for COMP_METADATA_TYPE_PARAMETER
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Parameter {
    #[serde(rename = "parameters", default, deserialize_with = "nullable_union")]
    pub parameters: Option<ParametersUnion>,

    /// Scope to which this metadata applies. Firmware: Any vehicle running this same vehicles
    /// firmware type. VehicleGroup: Any vehicle running this same firmware and this vehicles
    /// group type (Fixed Wing, Multi-Rotor, VTOL, Rover). VehicleType: Any vehicle match this
    /// vehicles firmware type and specific vehicle type. Vehicle: Only applies to this specific
    /// vehicle.
    #[serde(rename = "scope", default, skip_serializing_if = "Option::is_none")]
    pub scope: Option<Scope>,

    /// Unique id for this metadata. Same as ```COMPONENT_INFORMATION. comp_metadata_uid```.
    #[serde(rename = "uid", default, skip_serializing_if = "Option::is_none")]
    pub uid: Option<i64>,

    /// Version number for the format of this file.
    #[serde(rename = "version", default, skip_serializing_if = "Option::is_none")]
    pub version: Option<i64>,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct ParametersClass {
    /// User readable name for a 'type' of parameter. For example 'Developer', 'System', or
    /// 'Advanced'.
    #[serde(rename = "category", default, skip_serializing_if = "Option::is_none")]
    pub category: Option<String>,

    /// Number of decimal places to show for user facing display.
    #[serde(rename = "decimalPlaces", default, skip_serializing_if = "Option::is_none")]
    pub decimal_places: Option<i64>,

    /// Default value for parameter.
    #[serde(rename = "default", default, skip_serializing_if = "Option::is_none")]
    pub default: Option<f64>,

    /// User readable name for a group of parameters which are commonly modified together. For
    /// example a GCS can shows params in a hierarchical display based on group
    #[serde(rename = "group", default, skip_serializing_if = "Option::is_none")]
    pub group: Option<String>,

    /// Increment to use for user facing UI which increments a value
    #[serde(rename = "increment", default, skip_serializing_if = "Option::is_none")]
    pub increment: Option<f64>,

    /// Long user facing documentation of how the parameters works.
    #[serde(rename = "longDesc", default, skip_serializing_if = "Option::is_none")]
    pub long_desc: Option<String>,

    /// Maximum valid value
    #[serde(rename = "max", default, skip_serializing_if = "Option::is_none")]
    pub max: Option<f64>,

    /// Minimum valid value
    #[serde(rename = "min", default, skip_serializing_if = "Option::is_none")]
    pub min: Option<f64>,

    /// Parameter Name.
    #[serde(rename = "name", default, skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,

    /// true: Vehicle must be rebooted if this value is changed
    #[serde(rename = "rebootRequired", default, skip_serializing_if = "Option::is_none")]
    pub reboot_required: Option<bool>,

    /// Short user facing description/name for parameter. Used in UI intead of internal parameter
    /// name.
    #[serde(rename = "shortDesc", default, skip_serializing_if = "Option::is_none")]
    pub short_desc: Option<String>,

    /// Parameter type.
    #[serde(rename = "type", default, skip_serializing_if = "Option::is_none")]
    pub r#type: Option<TypeEnum>,

    /// Units for parameter value.
    #[serde(rename = "units", default, skip_serializing_if = "Option::is_none")]
    pub units: Option<String>,

    /// Array of values and textual descriptions for use by GCS ui.
    #[serde(rename = "values", default, skip_serializing_if = "Option::is_none")]
    pub values: Option<Vec<Value>>,

    /// true: value is volatile. Should not be included in creation of a CRC over param values
    /// for example.
    #[serde(rename = "volatile", default, skip_serializing_if = "Option::is_none")]
    pub volatile: Option<bool>,
}

/// Parameter type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TypeEnum {
    Float,
    Int16,
    Int32,
    Int8,
    Uint16,
    Uint32,
    Uint8,
}

/// Scope to which this metadata applies.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Scope {
    Firmware,
    Vehicle,
    VehicleGroup,
    VehicleType,
}

/// `partial struct ParametersUnion`: one of the members is set, or none (`IsNull`).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ParametersUnion {
    pub anything_array: Option<Vec<Value>>,
    pub bool: Option<bool>,
    pub double: Option<f64>,
    pub integer: Option<i64>,
    pub parameters_class: Option<ParametersClass>,
    pub string: Option<String>,
}

impl ParametersUnion {
    pub fn is_null(&self) -> bool {
        self.anything_array.is_none()
            && self.bool.is_none()
            && self.parameters_class.is_none()
            && self.double.is_none()
            && self.integer.is_none()
            && self.string.is_none()
    }
}

impl From<Vec<Value>> for ParametersUnion {
    fn from(v: Vec<Value>) -> Self {
        ParametersUnion { anything_array: Some(v), ..Default::default() }
    }
}
impl From<bool> for ParametersUnion {
    fn from(v: bool) -> Self {
        ParametersUnion { bool: Some(v), ..Default::default() }
    }
}
impl From<f64> for ParametersUnion {
    fn from(v: f64) -> Self {
        ParametersUnion { double: Some(v), ..Default::default() }
    }
}
impl From<i64> for ParametersUnion {
    fn from(v: i64) -> Self {
        ParametersUnion { integer: Some(v), ..Default::default() }
    }
}
impl From<ParametersClass> for ParametersUnion {
    fn from(v: ParametersClass) -> Self {
        ParametersUnion { parameters_class: Some(v), ..Default::default() }
    }
}
impl From<String> for ParametersUnion {
    fn from(v: String) -> Self {
        ParametersUnion { string: Some(v), ..Default::default() }
    }
}

/// `ParametersUnionConverter.ReadJson`: dispatch on the JSON token.
fn union_from_value(v: Value) -> Result<ParametersUnion, String> {
    Ok(match v {
        Value::Null => ParametersUnion::default(),
        Value::Number(n) => {
            if n.is_f64() {
                ParametersUnion::from(n.as_f64().unwrap())
            } else if let Some(i) = n.as_i64() {
                ParametersUnion::from(i)
            } else {
                // A JSON integer beyond long: Deserialize<long> overflows.
                return Err(format!("Value was either too large or too small for an Int64: {n}"));
            }
        }
        Value::Bool(b) => ParametersUnion::from(b),
        Value::String(s) => ParametersUnion::from(s),
        Value::Object(_) => ParametersUnion::from(serde_json::from_value::<ParametersClass>(v).map_err(|e| e.to_string())?),
        Value::Array(a) => ParametersUnion::from(a),
    })
}

impl<'de> Deserialize<'de> for ParametersUnion {
    fn deserialize<D: Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        union_from_value(Value::deserialize(d)?).map_err(de::Error::custom)
    }
}

/// `ParametersUnion?` with the converter: a JSON null still becomes an (empty) union value.
fn nullable_union<'de, D: Deserializer<'de>>(d: D) -> Result<Option<ParametersUnion>, D::Error> {
    union_from_value(Value::deserialize(d)?).map(Some).map_err(de::Error::custom)
}

impl Serialize for ParametersUnion {
    /// `ParametersUnionConverter.WriteJson`: null, then integer, double, bool, string, array, object.
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        if self.is_null() {
            return s.serialize_none();
        }
        if let Some(i) = self.integer {
            return s.serialize_i64(i);
        }
        if let Some(d) = self.double {
            return s.serialize_f64(d);
        }
        if let Some(b) = self.bool {
            return s.serialize_bool(b);
        }
        if let Some(st) = &self.string {
            return s.serialize_str(st);
        }
        if let Some(a) = &self.anything_array {
            return a.serialize(s);
        }
        if let Some(c) = &self.parameters_class {
            return c.serialize(s);
        }
        Err(serde::ser::Error::custom("Cannot marshal type ParametersUnion"))
    }
}

impl<'de> Deserialize<'de> for TypeEnum {
    /// `TypeEnumConverter.ReadJson`
    fn deserialize<D: Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        let value = String::deserialize(d)?;
        Ok(match value.as_str() {
            "Float" => TypeEnum::Float,
            "Int16" => TypeEnum::Int16,
            "Int32" => TypeEnum::Int32,
            "Int8" => TypeEnum::Int8,
            "Uint16" => TypeEnum::Uint16,
            "Uint32" => TypeEnum::Uint32,
            "Uint8" => TypeEnum::Uint8,
            _ => return Err(de::Error::custom("Cannot unmarshal type TypeEnum")),
        })
    }
}

impl Serialize for TypeEnum {
    /// `TypeEnumConverter.WriteJson`
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        s.serialize_str(match self {
            TypeEnum::Float => "Float",
            TypeEnum::Int16 => "Int16",
            TypeEnum::Int32 => "Int32",
            TypeEnum::Int8 => "Int8",
            TypeEnum::Uint16 => "Uint16",
            TypeEnum::Uint32 => "Uint32",
            TypeEnum::Uint8 => "Uint8",
        })
    }
}

impl<'de> Deserialize<'de> for Scope {
    /// `ScopeConverter.ReadJson`
    fn deserialize<D: Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        let value = String::deserialize(d)?;
        Ok(match value.as_str() {
            "Firmware" => Scope::Firmware,
            "Vehicle" => Scope::Vehicle,
            "VehicleGroup" => Scope::VehicleGroup,
            "VehicleType" => Scope::VehicleType,
            _ => return Err(de::Error::custom("Cannot unmarshal type Scope")),
        })
    }
}

impl Serialize for Scope {
    /// `ScopeConverter.WriteJson`
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        s.serialize_str(match self {
            Scope::Firmware => "Firmware",
            Scope::Vehicle => "Vehicle",
            Scope::VehicleGroup => "VehicleGroup",
            Scope::VehicleType => "VehicleType",
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_metadata_and_union_tokens() {
        let p: Parameter = serde_json::from_str(
            r#"{"version":1,"uid":7,"scope":"Firmware","parameters":{"name":"RATE","type":"Float","default":0.5,"values":[{"value":0,"description":"off"}],"rebootRequired":true}}"#,
        )
        .unwrap();
        assert_eq!(p.scope, Some(Scope::Firmware));
        let c = p.parameters.as_ref().unwrap().parameters_class.as_ref().unwrap();
        assert_eq!(c.r#type, Some(TypeEnum::Float));
        assert_eq!(c.reboot_required, Some(true));
        let p: Parameter = serde_json::from_str(r#"{"parameters":null}"#).unwrap();
        assert!(p.parameters.unwrap().is_null(), "a null token still yields an (empty) union");
        let p: Parameter = serde_json::from_str(r#"{}"#).unwrap();
        assert!(p.parameters.is_none());
        let p: Parameter = serde_json::from_str(r#"{"parameters":3}"#).unwrap();
        assert_eq!(p.parameters.unwrap().integer, Some(3));
        let p: Parameter = serde_json::from_str(r#"{"parameters":3.0}"#).unwrap();
        assert_eq!(p.parameters.unwrap().double, Some(3.0));
        assert!(serde_json::from_str::<Parameter>(r#"{"scope":"Fleet"}"#).is_err());
    }

    #[test]
    fn writes_like_newtonsoft_settings() {
        let p = Parameter { version: Some(1), ..Default::default() };
        assert_eq!(serde_json::to_string(&p).unwrap(), r#"{"parameters":null,"version":1}"#);
        let p = Parameter { parameters: Some(ParametersUnion::from(2i64)), ..Default::default() };
        assert_eq!(serde_json::to_string(&p).unwrap(), r#"{"parameters":2}"#);
    }
}

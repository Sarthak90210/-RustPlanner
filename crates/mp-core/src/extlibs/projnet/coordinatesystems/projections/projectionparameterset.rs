//! Port of `ExtLibs/ProjNet/CoordinateSystems/Projections/ProjectionParameterSet.cs`

use crate::dotnet::collections::Dictionary;
use crate::dotnet::Exception;

/// `GeoAPI.CoordinateSystems.ProjectionParameter`: a named projection parameter. (GeoAPI is a
/// NuGet package, not part of the C# tree; this is the class ProjNet uses from it.)
#[derive(Debug, Clone, PartialEq)]
pub struct ProjectionParameter {
    pub name: String,
    pub value: f64,
}

impl ProjectionParameter {
    pub fn new(name: &str, value: f64) -> Self {
        ProjectionParameter { name: name.to_string(), value }
    }
}

/// `public class ProjectionParameterSet : Dictionary<string, double>`: keys are the
/// lower-cased parameter names; the original spelling and order are kept beside them.
#[derive(Debug, Clone, Default)]
pub struct ProjectionParameterSet {
    values: Dictionary<String, f64>,
    original_names: Dictionary<String, String>,
    original_index: Dictionary<i32, String>,
}

impl ProjectionParameterSet {
    /// `new ProjectionParameterSet(parameters)`: a repeated name throws, as `Dictionary.Add` does.
    pub fn new<'a>(parameters: impl IntoIterator<Item = &'a ProjectionParameter>) -> Result<Self, Exception> {
        let mut s = Self::default();
        for pp in parameters {
            let key = pp.name.to_lowercase();
            if s.original_names.contains_key(&key) {
                return Err(Exception::Argument("An item with the same key has already been added.".into()));
            }
            s.original_names.insert(key.clone(), pp.name.clone());
            let idx = s.original_index.len() as i32;
            s.original_index.insert(idx, key.clone());
            s.values.insert(key, pp.value);
        }
        Ok(s)
    }

    /// `Count`
    pub fn count(&self) -> usize {
        self.values.len()
    }

    /// `ToProjectionParameter()`
    pub fn to_projection_parameter(&self) -> Vec<ProjectionParameter> {
        self.original_index
            .iter()
            .map(|(_, key)| {
                ProjectionParameter::new(
                    self.original_names.get(key).map(String::as_str).unwrap_or(""),
                    self.values.get(key).copied().unwrap_or(0.0),
                )
            })
            .collect()
    }

    /// `GetParameterValue(parameterName, params alternateNames)`: throws `ArgumentException`
    /// when neither the name nor any alternate is defined.
    pub fn get_parameter_value(&self, parameter_name: &str, alternate_names: &[&str]) -> Result<f64, Exception> {
        let name = parameter_name.to_lowercase();
        match self.values.get(&name) {
            Some(v) => Ok(*v),
            None => {
                for alternate_name in alternate_names {
                    if let Some(res) = self.values.get(&alternate_name.to_lowercase()) {
                        return Ok(*res);
                    }
                }

                let mut sb = format!("Missing projection parameter '{parameter_name}'");
                if !alternate_names.is_empty() {
                    sb += &format!("\nIt is also not defined as '{}'", alternate_names[0]);
                    for a in &alternate_names[1..] {
                        sb += &format!(", '{a}'");
                    }
                    sb.push('.');
                }
                Err(Exception::Argument(format!("{sb}\r\nParameter name: parameterName")))
            }
        }
    }

    /// `GetOptionalParameterValue(name, value, params alternateNames)`
    pub fn get_optional_parameter_value(&self, name: &str, value: f64, alternate_names: &[&str]) -> f64 {
        let name = name.to_lowercase();
        match self.values.get(&name) {
            Some(v) => *v,
            None => {
                for alternate_name in alternate_names {
                    if let Some(res) = self.values.get(&alternate_name.to_lowercase()) {
                        return *res;
                    }
                }
                value
            }
        }
    }

    /// `Find(name)`: the parameter, or `null`.
    pub fn find(&self, name: &str) -> Option<ProjectionParameter> {
        let name = name.to_lowercase();
        let v = self.values.get(&name)?;
        Some(ProjectionParameter::new(self.original_names.get(&name).map(String::as_str).unwrap_or(""), *v))
    }

    /// `GetAtIndex(index)`
    pub fn get_at_index(&self, index: i32) -> Result<ProjectionParameter, Exception> {
        if index < 0 || index as usize >= self.count() {
            return Err(Exception::ArgumentOutOfRange("Specified argument was out of the range of valid values.\r\nParameter name: index".into()));
        }
        let name = self
            .original_index
            .get(&index)
            .ok_or_else(|| Exception::KeyNotFound("The given key was not present in the dictionary.".into()))?;
        Ok(ProjectionParameter::new(
            self.original_names.get(name).map(String::as_str).unwrap_or(""),
            self.values.get(name).copied().unwrap_or(0.0),
        ))
    }

    /// `SetParameterValue(name, value)`
    pub fn set_parameter_value(&mut self, name: &str, value: f64) {
        let key = name.to_lowercase();
        if !self.values.contains_key(&key) {
            let idx = self.original_index.len() as i32;
            self.original_index.insert(idx, key.clone());
            self.original_names.insert(key.clone(), name.to_string());
            self.values.insert(key, value);
        } else {
            self.values.remove(&key);
            self.values.insert(key, value);
        }
    }
}

impl PartialEq for ProjectionParameterSet {
    /// `Equals(ProjectionParameterSet other)`: same count and the same value for every key.
    fn eq(&self, other: &Self) -> bool {
        if other.count() != self.count() {
            return false;
        }
        for (k, v) in self.values.iter() {
            match other.values.get(k) {
                Some(ov) if ov == v => {}
                _ => return false,
            }
        }
        true
    }
}

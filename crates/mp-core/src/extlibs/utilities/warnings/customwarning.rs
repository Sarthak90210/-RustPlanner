//! Port of `ExtLibs/Utilities/Warnings/CustomWarning.cs`
//!
//! A warning on one property of `defaultsrc` (the vehicle's `CurrentState`). The C# finds
//! the property by reflection; here `defaultsrc` is a `PropertySource` that lists its public
//! properties and returns their values.

#![allow(non_snake_case)]

use std::sync::{Arc, RwLock};

use crate::dotnet::{current_culture, format_fixed_optional, DateTime, Exception};

/// A public property's value, as `PropertyInfo.GetValue` boxes it.
#[derive(Debug, Clone, PartialEq)]
pub enum PropertyValue {
    Number(f64),
    Bool(bool),
    Text(String),
    /// a reference-type value that is not null
    Object,
    Null,
}

/// `defaultsrc.GetType().GetProperties()` with `GetValue`.
pub trait PropertySource: Send + Sync {
    /// The public property names, in reflection order.
    fn property_names(&self) -> Vec<String>;
    /// The current value of a property, `None` if there is no such property.
    fn property_value(&self, name: &str) -> Option<PropertyValue>;
}

/// `public static object defaultsrc`
static DEFAULTSRC: RwLock<Option<Arc<dyn PropertySource>>> = RwLock::new(None);

fn argument_null(param: &str) -> Exception {
    Exception::ArgumentNull(format!("Value cannot be null.\r\nParameter name: {param}"))
}

crate::cs_enum! {
    /// `public enum Conditional`
    pub Conditional: i32 {
        NONE = 0,
        LT = 1,
        LTEQ = 2,
        EQ = 3,
        GT = 4,
        GTEQ = 5,
        NEQ = 6,
    }
}

crate::cs_enum! {
    /// `public enum WarningColors`
    pub WarningColors: i32 {
        NoColor = 0,
        Red = 1,
        OrangeRed = 2,
        Maroon = 3,
        Yellow = 4,
        Gold = 5,
        Goldenrod = 6,
        LawnGreen = 7,
        Green = 8,
        DarkGreen = 9,
    }
}

crate::cs_enum! {
    /// `public enum WarningType`: speak/text or quick view colouring items
    pub WarningType: i32 {
        SpeakAndText = 0,
        Coloring = 1,
    }
}

/// `public class CustomWarning`
#[derive(Debug, Clone)]
pub struct CustomWarning {
    /// `System.Reflection.PropertyInfo Item`: the property name once found
    Item: Option<String>,
    pub Child: Option<Box<CustomWarning>>,
    _name: String,
    /// Warning on this number based on ConditionType
    pub Warning: f64,
    /// used to track last time something was said
    lastrepeat: DateTime,
    /// identify the type of warning (SpeakAndText or Coloring)
    pub type_: WarningType,
    /// this color is used when warning fired
    pub color: Option<String>,
    /// in seconds
    pub RepeatTime: i32,
    /// how we are checking Warning
    pub ConditionType: Conditional,
    _text: Option<String>,
}

impl Default for CustomWarning {
    fn default() -> Self {
        Self::new()
    }
}

impl CustomWarning {
    /// `defaultsrc` getter
    pub fn defaultsrc() -> Option<Arc<dyn PropertySource>> {
        DEFAULTSRC.read().unwrap().clone()
    }

    /// `defaultsrc` setter
    pub fn set_defaultsrc(src: Option<Arc<dyn PropertySource>>) {
        *DEFAULTSRC.write().unwrap() = src;
    }

    pub fn new() -> Self {
        CustomWarning {
            Item: None,
            Child: None,
            _name: String::new(),
            Warning: 0.0,
            lastrepeat: DateTime::MIN_VALUE,
            type_: WarningType::SpeakAndText,
            color: None,
            RepeatTime: 10,
            ConditionType: Conditional::NONE,
            _text: Some("WARNING: {name} is {value}".into()),
        }
    }

    /// `GetOptions()`: the properties of `defaultsrc` whose value is not null.
    pub fn GetOptions(&self) -> Result<Vec<String>, Exception> {
        let src = Self::defaultsrc().ok_or_else(|| argument_null("src"))?;
        let mut answer = Vec::new();
        for field in src.property_names() {
            match src.property_value(&field) {
                None | Some(PropertyValue::Null) => continue,
                Some(_) => answer.push(field),
            }
        }
        Ok(answer)
    }

    /// `Name`
    pub fn Name(&self) -> &str {
        &self._name
    }

    /// `Name` setter: with a `defaultsrc` the property is looked up, and an unknown name
    /// throws MissingFieldException (after the name is stored).
    pub fn set_Name(&mut self, value: &str) -> Result<(), Exception> {
        if self._name == value {
            return Ok(());
        }
        self._name = value.to_string();
        if Self::defaultsrc().is_some() {
            self.SetField(value)?;
        }
        Ok(())
    }

    /// `Text` (null when set to null)
    pub fn Text(&self) -> Option<&str> {
        self._text.as_deref()
    }

    pub fn set_Text(&mut self, value: Option<String>) {
        self._text = value;
    }

    /// `SayText()`: `Text` with {warning}, {value} and {name} replaced.
    pub fn SayText(&self) -> Result<String, Exception> {
        let text = self._text.as_deref().ok_or_else(|| Exception::NullReference("Object reference not set to an instance of an object.".into()))?;
        let c = current_culture();
        let t = text.replace("{warning}", &format_fixed_optional(self.Warning, 1, 0, 2, c));
        let t = t.replace("{value}", &format_fixed_optional(self.GetValue()?, 1, 0, 2, c));
        let name = self.Item.as_deref().ok_or_else(|| Exception::NullReference("Object reference not set to an instance of an object.".into()))?;
        Ok(t.replace("{name}", name))
    }

    /// `GetValue`: `(double)Convert.ChangeType(value, typeof(double))`.
    pub fn GetValue(&self) -> Result<f64, Exception> {
        let src = Self::defaultsrc().ok_or_else(|| argument_null("src"))?;
        let item = self.Item.as_deref().ok_or_else(|| argument_null("Item"))?;
        match src.property_value(item) {
            Some(PropertyValue::Number(v)) => Ok(v),
            Some(PropertyValue::Bool(b)) => Ok(if b { 1.0 } else { 0.0 }),
            Some(PropertyValue::Text(s)) => crate::dotnet::double_parse(&s),
            // Convert.ChangeType(null, double) throws InvalidCastException
            Some(PropertyValue::Null) | None => Err(Exception::InvalidCast("Null object cannot be converted to a value type.".into())),
            Some(PropertyValue::Object) => Err(Exception::InvalidCast("Object must implement IConvertible.".into())),
        }
    }

    /// `CheckValue(userepeattime = true)`: true on a match; `RepeatTime` seconds pass
    /// before the same warning matches again.
    pub fn CheckValue(&mut self, userepeattime: bool) -> Result<bool, Exception> {
        if userepeattime && DateTime::now() < self.lastrepeat.add_seconds(self.RepeatTime as f64)? {
            return Ok(false);
        }
        let mut condition = false;
        match self.ConditionType {
            Conditional::EQ => condition = self.GetValue()? == self.Warning,
            Conditional::GT => condition = self.GetValue()? > self.Warning,
            Conditional::GTEQ => condition = self.GetValue()? >= self.Warning,
            Conditional::LT => condition = self.GetValue()? < self.Warning,
            Conditional::LTEQ => condition = self.GetValue()? <= self.Warning,
            Conditional::NEQ => condition = self.GetValue()? != self.Warning,
            _ => {}
        }
        if condition && userepeattime {
            self.lastrepeat = DateTime::now();
        }
        Ok(condition)
    }

    /// `SetField(name)`
    pub fn SetField(&mut self, name: &str) -> Result<(), Exception> {
        let src = Self::defaultsrc().ok_or_else(|| argument_null("src"))?;
        if name.is_empty() {
            return Ok(());
        }
        if src.property_names().iter().any(|f| f == name) {
            self.Item = Some(name.to_string());
            return self.set_Name(name);
        }
        Err(Exception::Custom("MissingFieldException", "No such name".into()))
    }

    /// The XML element `XmlSerializer` writes for this object (fields, then properties in
    /// declaration order; null members are left out).
    pub(crate) fn to_xml(&self, out: &mut String, tag: &str, indent: usize) {
        let pad = "  ".repeat(indent);
        let esc = |s: &str| s.replace('&', "&amp;").replace('<', "&lt;").replace('>', "&gt;");
        out.push_str(&format!("{pad}<{tag}>\r\n"));
        if let Some(c) = &self.Child {
            c.to_xml(out, "Child", indent + 1);
        }
        let inner = "  ".repeat(indent + 1);
        out.push_str(&format!("{inner}<Name>{}</Name>\r\n", esc(&self._name)));
        out.push_str(&format!("{inner}<Warning>{}</Warning>\r\n", xml_double(self.Warning)));
        out.push_str(&format!("{inner}<type>{}</type>\r\n", self.type_));
        if let Some(c) = &self.color {
            out.push_str(&format!("{inner}<color>{}</color>\r\n", esc(c)));
        }
        out.push_str(&format!("{inner}<RepeatTime>{}</RepeatTime>\r\n", self.RepeatTime));
        out.push_str(&format!("{inner}<ConditionType>{}</ConditionType>\r\n", self.ConditionType));
        if let Some(t) = &self._text {
            out.push_str(&format!("{inner}<Text>{}</Text>\r\n", esc(t)));
        }
        out.push_str(&format!("{pad}</{tag}>\r\n"));
    }

    /// `XmlSerializer.Deserialize` of one element: members are set in document order,
    /// through their setters.
    pub(crate) fn from_xml(node: roxmltree::Node) -> Result<CustomWarning, Exception> {
        let mut w = CustomWarning::new();
        let bad = |what: &str| Exception::InvalidOperation(format!("There is an error in XML document: {what}"));
        for c in node.children().filter(|c| c.is_element()) {
            let text = c.text().unwrap_or("");
            match c.tag_name().name() {
                "Child" => w.Child = Some(Box::new(Self::from_xml(c)?)),
                "Name" => w.set_Name(text)?,
                "Warning" => w.Warning = text.trim().parse().map_err(|_| bad("Warning"))?,
                "type" => w.type_ = WarningType::parse(text).ok_or_else(|| bad("type"))?,
                "color" => w.color = Some(text.to_string()),
                "RepeatTime" => w.RepeatTime = text.trim().parse().map_err(|_| bad("RepeatTime"))?,
                "ConditionType" => w.ConditionType = Conditional::parse(text).ok_or_else(|| bad("ConditionType"))?,
                "Text" => w._text = Some(text.to_string()),
                _ => {}
            }
        }
        Ok(w)
    }
}

/// `XmlConvert.ToString(double)`: round-trip "R", with INF / -INF / NaN.
fn xml_double(v: f64) -> String {
    if v.is_nan() {
        return "NaN".into();
    }
    if v.is_infinite() {
        return if v > 0.0 { "INF".into() } else { "-INF".into() };
    }
    crate::dotnet::double_to_string_culture(v, &crate::dotnet::INVARIANT_CULTURE)
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    pub(crate) struct Src;
    impl PropertySource for Src {
        fn property_names(&self) -> Vec<String> {
            vec!["alt".into(), "armed".into(), "mode".into()]
        }
        fn property_value(&self, name: &str) -> Option<PropertyValue> {
            match name {
                "alt" => Some(PropertyValue::Number(12.345)),
                "armed" => Some(PropertyValue::Bool(true)),
                "mode" => Some(PropertyValue::Text("Auto".into())),
                _ => None,
            }
        }
    }

    #[test]
    fn check_and_say() {
        CustomWarning::set_defaultsrc(Some(Arc::new(Src)));
        let mut w = CustomWarning::new();
        w.set_Name("alt").unwrap();
        w.ConditionType = Conditional::GT;
        w.Warning = 10.0;
        assert_eq!(w.SayText().unwrap(), "WARNING: alt is 12.35");
        assert!(w.CheckValue(true).unwrap());
        assert!(!w.CheckValue(true).unwrap(), "repeat time holds it back");
        let mut b = CustomWarning::new();
        assert!(b.set_Name("nope").is_err());
        assert_eq!(b.Name(), "nope");
        b.set_Name("armed").unwrap();
        assert_eq!(b.GetValue().unwrap(), 1.0);
    }
}

//! Port of `ExtLibs/Core/CommonObjects/IdNameColorItem.cs`
//!
//! The interfaces are traits. The abstract base classes hold only fields and a few
//! non-virtual helpers, so each is a struct implementing its interfaces; a C# subclass embeds
//! one and forwards to it. C# `null` strings are `None`.

use crate::dotnet::convert::string_compare_to;
use crate::dotnet::drawing::Color;
use crate::dotnet::Exception;
use crate::extlibs::core::utils::colorutils::ColorTranslator;

fn nre() -> Exception {
    Exception::NullReference("Object reference not set to an instance of an object.".into())
}

/// `public interface IIdItem`
pub trait IIdItem {
    /// `string Id { get; }`
    fn id(&self) -> Option<String>;
    /// `string Id { set; }`
    fn set_id(&mut self, value: Option<String>);
}

/// `public interface INameItem`
pub trait INameItem {
    /// `string Name { get; }`
    fn name(&self) -> Option<String>;
    /// `string Name { set; }`
    fn set_name(&mut self, value: Option<String>);
}

/// `public interface IDisplayNameItem`
pub trait IDisplayNameItem {
    /// `string DisplayName { get; }`
    fn display_name(&self) -> Option<String>;
    /// `string DisplayName { set; }`
    fn set_display_name(&mut self, value: Option<String>);
}

/// `public interface IIdNameItem : IIdItem, INameItem`
pub trait IIdNameItem: IIdItem + INameItem {}

/// `public interface IIdNameColorItem : IIdNameItem`
pub trait IIdNameColorItem: IIdNameItem {
    /// `Color Color { get; }`
    fn color(&self) -> Color;
    /// `Color Color { set; }`
    fn set_color(&mut self, value: Color);
}

/// `public abstract class AIdItem : IIdItem`
#[allow(non_snake_case)]
#[derive(Debug, Clone, Default, PartialEq)]
pub struct AIdItem {
    pub m_Id: Option<String>,
}

impl AIdItem {
    /// `static int CompareIds(IIdItem a, IIdItem b)`: `a.Id.CompareTo(b.Id)`, so a null
    /// `a.Id` throws and a null `b.Id` sorts first.
    pub fn compare_ids(a: &dyn IIdItem, b: &dyn IIdItem) -> Result<i32, Exception> {
        let a_id = a.id().ok_or_else(nre)?;
        Ok(string_compare_to(&a_id, b.id().as_deref()))
    }

    /// `MatchId(IIdItem item)`
    pub fn match_id(&mut self, item: &dyn IIdItem) {
        self.m_Id = item.id();
    }
}

impl IIdItem for AIdItem {
    fn id(&self) -> Option<String> {
        self.m_Id.clone()
    }
    fn set_id(&mut self, value: Option<String>) {
        self.m_Id = value;
    }
}

/// `public abstract class AIdNameItem : AIdItem, IIdNameItem`
#[allow(non_snake_case)]
#[derive(Debug, Clone, Default, PartialEq)]
pub struct AIdNameItem {
    pub m_Id: Option<String>,
    pub m_Name: Option<String>,
}

impl AIdNameItem {
    /// `static int CompareNames(IIdNameItem a, IIdNameItem b)`
    pub fn compare_names(a: &dyn IIdNameItem, b: &dyn IIdNameItem) -> Result<i32, Exception> {
        match a.name() {
            None => {
                if b.name().is_some() {
                    Ok(1)
                } else {
                    AIdItem::compare_ids(a, b)
                }
            }
            Some(an) => Ok(string_compare_to(&an, b.name().as_deref())),
        }
    }

    /// `NameOrId`
    pub fn name_or_id(&self) -> Option<String> {
        if self.m_Name.is_none() {
            return self.m_Id.clone();
        }
        self.m_Name.clone()
    }

    /// `protected virtual void OnIdChange(string oldId)`
    pub fn on_id_change(&mut self, _old_id: Option<&str>) {}

    /// `MatchId(IIdItem item)` (inherited)
    pub fn match_id(&mut self, item: &dyn IIdItem) {
        self.m_Id = item.id();
    }

    /// `MatchIdName(IIdNameItem item)`
    pub fn match_id_name(&mut self, item: &dyn IIdNameItem) {
        self.m_Id = item.id();
        self.m_Name = item.name();
    }
}

impl IIdItem for AIdNameItem {
    fn id(&self) -> Option<String> {
        self.m_Id.clone()
    }
    fn set_id(&mut self, value: Option<String>) {
        self.m_Id = value;
    }
}

impl INameItem for AIdNameItem {
    fn name(&self) -> Option<String> {
        self.m_Name.clone()
    }
    fn set_name(&mut self, value: Option<String>) {
        self.m_Name = value;
    }
}

impl IIdNameItem for AIdNameItem {}

/// `public abstract class AIdNameColorItem : AIdNameItem, IIdNameColorItem`
#[allow(non_snake_case)]
#[derive(Debug, Clone, Default, PartialEq)]
pub struct AIdNameColorItem {
    pub m_Id: Option<String>,
    pub m_Name: Option<String>,
    pub m_Color: Color,
}

impl AIdNameColorItem {
    /// `AIdNameColorItem(string id, string name, Color color)`
    pub fn new(id: Option<String>, name: Option<String>, color: Color) -> Self {
        AIdNameColorItem { m_Id: id, m_Name: name, m_Color: color }
    }

    /// `NameOrId` (inherited)
    pub fn name_or_id(&self) -> Option<String> {
        if self.m_Name.is_none() {
            return self.m_Id.clone();
        }
        self.m_Name.clone()
    }

    /// `Color_XML` getter: `#RRGGBB`.
    pub fn color_xml(&self) -> String {
        ColorTranslator::to_html(&self.m_Color)
    }

    /// `Color_XML` setter
    pub fn set_color_xml(&mut self, value: Option<&str>) -> Result<(), Exception> {
        let c = ColorTranslator::from_html(value)?;
        self.set_color(c);
        Ok(())
    }

    /// `MatchIdNameColor(IIdNameColorItem item)`
    pub fn match_id_name_color(&mut self, item: &dyn IIdNameColorItem) {
        self.m_Id = item.id();
        self.m_Name = item.name();
        self.m_Color = item.color();
    }
}

impl IIdItem for AIdNameColorItem {
    fn id(&self) -> Option<String> {
        self.m_Id.clone()
    }
    fn set_id(&mut self, value: Option<String>) {
        self.m_Id = value;
    }
}

impl INameItem for AIdNameColorItem {
    fn name(&self) -> Option<String> {
        self.m_Name.clone()
    }
    fn set_name(&mut self, value: Option<String>) {
        self.m_Name = value;
    }
}

impl IIdNameItem for AIdNameColorItem {}

impl IIdNameColorItem for AIdNameColorItem {
    fn color(&self) -> Color {
        self.m_Color.clone()
    }
    fn set_color(&mut self, value: Color) {
        self.m_Color = value;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn compare_and_match() {
        let a = AIdNameItem { m_Id: Some("1".into()), m_Name: None };
        let b = AIdNameItem { m_Id: Some("2".into()), m_Name: Some("x".into()) };
        assert_eq!(AIdNameItem::compare_names(&a, &b).unwrap(), 1);
        assert_eq!(AIdNameItem::compare_names(&b, &a).unwrap(), 1);
        let c = AIdNameItem { m_Id: None, m_Name: None };
        assert!(AIdNameItem::compare_names(&c, &a).is_err());
        assert_eq!(AIdNameItem::compare_names(&a, &c).unwrap(), 1);
        assert_eq!(a.name_or_id().as_deref(), Some("1"));
        let mut item = AIdNameColorItem::new(Some("i".into()), Some("n".into()), Color::from_rgb(1, 2, 255).unwrap());
        assert_eq!(item.color_xml(), "#0102FF");
        item.set_color_xml(Some("#fff")).unwrap();
        assert_eq!(item.color().to_argb(), -1);
        assert!(item.set_color_xml(Some("#zzz")).is_err());
        let mut m = AIdNameColorItem::default();
        m.match_id_name_color(&item);
        assert_eq!(m, item);
    }
}

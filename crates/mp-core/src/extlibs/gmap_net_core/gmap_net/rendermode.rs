//! Port of `ExtLibs/GMap.NET.Core/GMap.NET/RenderMode.cs`

crate::cs_enum! {
    /// `public enum RenderMode`: types of map rendering
    pub RenderMode: i32 {
        /// gdi+ should work anywhere on Windows Forms
        GDI_PLUS = 0,
        /// only on Windows Presentation Foundation
        WPF = 1,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn names() {
        assert_eq!(RenderMode::GDI_PLUS.to_string(), "GDI_PLUS");
        assert_eq!(RenderMode::get_values(), vec![RenderMode::GDI_PLUS, RenderMode::WPF]);
    }
}

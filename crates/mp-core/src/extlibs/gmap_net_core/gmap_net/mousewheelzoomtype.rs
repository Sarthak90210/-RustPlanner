//! Port of `ExtLibs/GMap.NET.Core/GMap.NET/MouseWheelZoomType.cs`

crate::cs_enum! {
    /// `public enum MouseWheelZoomType`: map zooming type
    pub MouseWheelZoomType: i32 {
        /// zooms map to current mouse position and makes it map center
        MousePositionAndCenter = 0,
        /// zooms to current mouse position, but doesn't make it map center, google/bing style ;}
        MousePositionWithoutCenter = 1,
        /// zooms map to current view center
        ViewCenter = 2,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn names() {
        assert_eq!(MouseWheelZoomType::ViewCenter.to_string(), "ViewCenter");
        assert_eq!(MouseWheelZoomType::parse("MousePositionWithoutCenter"), Some(MouseWheelZoomType(1)));
    }
}

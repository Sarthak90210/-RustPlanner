//! Port of `ExtLibs/GMap.NET.Core/GMap.NET/PureImageCache.cs`

#![allow(non_snake_case)]

use super::gpoint::GPoint;
use crate::dotnet::{DateTime, Exception};
use crate::extlibs::gmap_net_core::gmap_net_internals::pureimage::PureImage;

/// `public interface PureImageCache`: pure abstraction for image cache
pub trait PureImageCache: Send + Sync {
    /// puts image to db
    fn PutImageToCache(&self, tile: &[u8], type_: i32, pos: GPoint, zoom: i32) -> Result<bool, Exception>;

    /// gets image from db
    fn GetImageFromCache(&self, type_: i32, pos: GPoint, zoom: i32) -> Result<Option<Box<dyn PureImage>>, Exception>;

    /// delete old tiles beyond a supplied date; `type_` is the provider dbid or `None` for
    /// all providers. Returns the number of deleted tiles.
    fn DeleteOlderThan(&self, date: DateTime, type_: Option<i32>) -> Result<i32, Exception>;

    /// check if image exist on db
    fn CheckImageFromCache(&self, type_: i32, pos: GPoint, zoom: i32) -> Result<bool, Exception>;
}

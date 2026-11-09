//! `ExtLibs/GeoUtility/Transformation`: `internal partial class GeoUtility.Transformation`.
//!
//! Each C# file adds methods to the one partial class; here each file is a submodule with
//! its own `impl Transformation` block. Not ported yet: `WGSGK`, `GKPOD`, `WGSIMAP`,
//! `WGSPixel`, `MapDimension`, `MapInternal`, `MapMove` and `Conversion` (Gauss-Krueger and
//! MapService), which nothing in Mission Planner calls.

pub mod definition;
pub mod mgrutm;
pub mod podwgs;
pub mod utmmgr;
pub mod utmwgs;
pub mod wgspod;
pub mod wgsutm;

/// `internal partial class Transformation`
#[derive(Debug, Clone, Copy, Default)]
pub struct Transformation;

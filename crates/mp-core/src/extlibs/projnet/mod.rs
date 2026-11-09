//! `ExtLibs/ProjNet` (`ProjNET.csproj`): coordinate system transformations. Ported so far:
//! the WGS84 to UTM path that `PointLatLngAlt` and `utmpos` use (transverse Mercator,
//! `MapProjection`, `ProjectionParameterSet`, the WGS84 ellipsoid and `WGS84_UTM`). The WKT
//! reader and the other projections are not ported yet.

pub mod coordinatesystems;

//! Port of `ExtLibs/GMap.NET.Core/GMap.NET.Projections/PlateCarreeProjectionPergo.cs`
//!
//! The whole `PlateCarreeProjectionPergo` class sits inside `#if OLD_PERGO`, and
//! `GMap.NET.Core.csproj` never defines `OLD_PERGO` (its DefineConstants are
//! TRACE;DEBUG;MONO;NETSTANDARD2_0 / RELEASE variants), so the file compiles to an empty
//! `GMap.NET.Projections` namespace. This module is likewise empty.

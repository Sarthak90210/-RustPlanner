//! `ExtLibs/`: the libraries Mission Planner builds alongside the main project. Each child
//! module is one C# project, laid out file-for-file (see `port/MANIFEST.md`).

pub mod ardupilot;
pub mod comms;
pub mod core;
pub mod geoutility;
pub mod gmap_net_core;
pub mod interfaces;
pub mod maps;
pub mod mavlink;
pub mod projnet;
pub mod utilities;

#![allow(deprecated)]
//! Mission Planner core, ported from the C# `ExtLibs` (Comms, ArduPilot, Utilities).
//!
//! Platform independent: no UI and no OS-specific code outside of `comms` serial support.

pub mod comms;
pub mod dataflash;
pub mod devid;
pub mod firmware;
pub mod ftp;
pub mod fwupload;
pub mod geo;
pub mod georef;
pub mod link;
pub mod mavcmd;
pub mod mission;
pub mod param_meta;
pub mod parampck;
pub mod paramfile;
pub mod sik;
pub mod sim;
pub mod state;
pub mod survey;
pub mod tlog;

pub use mavlink;

/// Local time zone offset from UTC, in seconds.
pub fn local_utc_offset_secs() -> i64 {
    chrono::Local::now().offset().local_minus_utc() as i64
}

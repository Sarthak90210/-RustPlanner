//! Port of `ExtLibs/Utilities/locationwp.cs`
//!
//! The conversions to and from `MissionFile.Item` are in missionfile.rs, beside `Item`.

use std::any::Any;
use std::sync::Arc;

use crate::dotnet::cast_i32;
use crate::extlibs::mavlink::{
    mavlink_mission_item_int_t, mavlink_mission_item_t, mavlink_set_position_target_global_int_t, CsEnum, MAV_CMD,
};

/// `public struct Locationwp`: a mission item as ArduPilot stores it.
#[derive(Clone, Default)]
pub struct Locationwp {
    /// `private ushort _seq` (crate-visible so other ported files can build items with
    /// struct-update syntax; the C# sets it only through the conversions)
    pub(crate) seq: u16,
    pub frame: u8,
    /// `public object Tag`
    pub tag: Option<Arc<dyn Any + Send + Sync>>,

    /// command id
    pub id: u16,
    /// param 1
    pub p1: f32,
    /// param 2
    pub p2: f32,
    /// param 3
    pub p3: f32,
    /// param 4
    pub p4: f32,
    /// Latitude
    pub lat: f64,
    /// Longitude
    pub lng: f64,
    /// Altitude
    pub alt: f32,
}

impl std::fmt::Debug for Locationwp {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Locationwp")
            .field("_seq", &self.seq)
            .field("frame", &self.frame)
            .field("Tag", &self.tag.as_ref().map(|_| "object"))
            .field("id", &self.id)
            .field("p1", &self.p1)
            .field("p2", &self.p2)
            .field("p3", &self.p3)
            .field("p4", &self.p4)
            .field("lat", &self.lat)
            .field("lng", &self.lng)
            .field("alt", &self.alt)
            .finish()
    }
}

impl PartialEq for Locationwp {
    /// `ValueType.Equals`: the struct holds a reference (`Tag`), so .NET compares field by
    /// field through the boxed `Equals`: `Single`/`Double.Equals` treat NaN as equal to NaN,
    /// and `Tag` compares by reference.
    fn eq(&self, o: &Self) -> bool {
        fn f32eq(a: f32, b: f32) -> bool {
            a == b || (a.is_nan() && b.is_nan())
        }
        fn f64eq(a: f64, b: f64) -> bool {
            a == b || (a.is_nan() && b.is_nan())
        }
        let tag_eq = match (&self.tag, &o.tag) {
            (None, None) => true,
            (Some(a), Some(b)) => Arc::ptr_eq(a, b),
            _ => false,
        };
        self.seq == o.seq
            && self.frame == o.frame
            && tag_eq
            && self.id == o.id
            && f32eq(self.p1, o.p1)
            && f32eq(self.p2, o.p2)
            && f32eq(self.p3, o.p3)
            && f32eq(self.p4, o.p4)
            && f64eq(self.lat, o.lat)
            && f64eq(self.lng, o.lng)
            && f32eq(self.alt, o.alt)
    }
}

impl Locationwp {
    /// `Set(lat, lng, alt, id)`: updates this item (frame 3, relative altitude) and returns a
    /// copy of it.
    pub fn set(&mut self, lat: f64, lng: f64, alt: f64, id: u16) -> Locationwp {
        self.lat = lat;
        self.lng = lng;
        self.alt = alt as f32;
        self.id = id;
        self.frame = 3;

        self.clone()
    }

    /// `_seq` (private in C#; the MissionFile conversions read it)
    #[allow(dead_code)]
    pub(crate) fn seq(&self) -> u16 {
        self.seq
    }

    #[allow(dead_code)]
    pub(crate) fn set_seq(&mut self, v: u16) {
        self.seq = v;
    }

    /// `isLocationCommand(id)`: whether the MAV_CMD member with this value carries
    /// `[hasLocation]`. An undefined value has no member and is not a location command.
    /// (The C# `catch { return true; }` cannot trigger: `Enum.Parse` of a number never throws.)
    pub fn is_location_command(id: u16) -> bool {
        let cmd = MAV_CMD::from_i64(id as i64);
        let Some(name) = cmd.get_name() else {
            return false;
        };
        MAV_CMD::entries().iter().any(|e| e.name == name && e.has_location)
    }

    /// `static object Convert(Locationwp cmd, bool isint)` for the int variant:
    /// `implicit operator mavlink_mission_item_int_t(Locationwp)`
    pub fn to_mission_item_int(&self) -> mavlink_mission_item_int_t {
        let mut x = self.lat;
        let mut y = self.lng;
        if Self::is_location_command(self.id) {
            x *= 1.0e7;
            y *= 1.0e7;
        }
        mavlink_mission_item_int_t {
            command: self.id,
            param1: self.p1,
            param2: self.p2,
            param3: self.p3,
            param4: self.p4,
            x: cast_i32(x),
            y: cast_i32(y),
            z: self.alt,
            seq: self.seq,
            frame: self.frame,
            ..Default::default()
        }
    }

    /// `implicit operator mavlink_mission_item_t(Locationwp)`
    pub fn to_mission_item(&self) -> mavlink_mission_item_t {
        mavlink_mission_item_t {
            command: self.id,
            param1: self.p1,
            param2: self.p2,
            param3: self.p3,
            param4: self.p4,
            x: self.lat as f32,
            y: self.lng as f32,
            z: self.alt,
            seq: self.seq,
            frame: self.frame,
            ..Default::default()
        }
    }
}

impl From<&Locationwp> for mavlink_mission_item_t {
    fn from(input: &Locationwp) -> Self {
        input.to_mission_item()
    }
}

impl From<&Locationwp> for mavlink_mission_item_int_t {
    fn from(input: &Locationwp) -> Self {
        input.to_mission_item_int()
    }
}

impl From<mavlink_set_position_target_global_int_t> for Locationwp {
    /// `implicit operator Locationwp(mavlink_set_position_target_global_int_t input)`
    fn from(input: mavlink_set_position_target_global_int_t) -> Self {
        Locationwp {
            id: MAV_CMD::WAYPOINT.0 as u8 as u16,
            p1: 0.0,
            p2: 0.0,
            p3: 0.0,
            p4: 0.0,
            lat: input.lat_int as f64 / 1e7,
            lng: input.lon_int as f64 / 1e7,
            alt: input.alt,
            seq: 0,
            frame: input.coordinate_frame,
            tag: None,
        }
    }
}

impl From<mavlink_mission_item_t> for Locationwp {
    /// `implicit operator Locationwp(mavlink_mission_item_t input)`
    fn from(input: mavlink_mission_item_t) -> Self {
        Locationwp {
            id: input.command,
            p1: input.param1,
            p2: input.param2,
            p3: input.param3,
            p4: input.param4,
            lat: input.x as f64,
            lng: input.y as f64,
            alt: input.z,
            seq: input.seq,
            frame: input.frame,
            tag: None,
        }
    }
}

impl From<mavlink_mission_item_int_t> for Locationwp {
    /// `implicit operator Locationwp(mavlink_mission_item_int_t input)`
    fn from(input: mavlink_mission_item_int_t) -> Self {
        let mut x = input.x as f64;
        let mut y = input.y as f64;
        if Locationwp::is_location_command(input.command) {
            x *= 1.0e-7;
            y *= 1.0e-7;
        }
        Locationwp {
            id: input.command,
            p1: input.param1,
            p2: input.param2,
            p3: input.param3,
            p4: input.param4,
            lat: x,
            lng: y,
            alt: input.z,
            seq: input.seq,
            frame: input.frame,
            tag: None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn location_commands() {
        assert!(Locationwp::is_location_command(16)); // NAV_WAYPOINT
        assert!(!Locationwp::is_location_command(178)); // DO_CHANGE_SPEED
        assert!(!Locationwp::is_location_command(65000)); // undefined
    }

    #[test]
    fn int_round_trip_scales_location_commands() {
        let mut wp = Locationwp::default();
        wp.set(-12.5, 100.25, 100.0, 16);
        let mi = wp.to_mission_item_int();
        // (int) truncates toward zero
        assert_eq!((mi.x, mi.y, mi.frame), (-125000000, 1002500000, 3));
        let back: Locationwp = mi.into();
        assert!((back.lat - -12.5).abs() < 1e-9);
        let mut speed = Locationwp::default();
        speed.set(5.0, 6.0, 0.0, 178);
        assert_eq!(speed.to_mission_item_int().x, 5);
    }
}

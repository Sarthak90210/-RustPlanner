//! Port of `ExtLibs/Utilities/VehicleLocation.cs`
//!
//! (namespace `MissionPlanner.GeoRef`)

use super::singlelocation::SingleLocation;

/// `public class VehicleLocation : SingleLocation`: adds nothing to its base.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct VehicleLocation {
    pub base: SingleLocation,
}

impl std::ops::Deref for VehicleLocation {
    type Target = SingleLocation;
    fn deref(&self) -> &SingleLocation {
        &self.base
    }
}

impl std::ops::DerefMut for VehicleLocation {
    fn deref_mut(&mut self) -> &mut SingleLocation {
        &mut self.base
    }
}

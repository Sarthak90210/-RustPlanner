//! Port of `ExtLibs/Utilities/Spline2.cs`
//!
//! (namespace `MissionPlanner.Controls.Waypoints`)
//!
//! ArduCopter's spline waypoint navigation (AC_WPNav), used to draw spline mission legs.
//! `Spline2 : Utils` only inherits static helpers. The float/double mix of the C# is kept:
//! `float` locals and fields round to single precision at each assignment, and the `f`
//! literals widen to double where the other operand is a double.

#![allow(non_camel_case_types)]

use std::sync::atomic::{AtomicU32, Ordering};

use super::math::MathHelper;
use super::pointlatlngalt::PointLatLngAlt;
use super::utils::Utils;
use super::vector3::Vector3;
use crate::dotnet::cast_i32;

crate::cs_enum! {
    /// spline segment end types enum
    pub spline_segment_end_type: i32 {
        SEGMENT_END_STOP = 0,
        SEGMENT_END_STRAIGHT = 1,
        SEGMENT_END_SPLINE = 2,
    }
}

crate::cs_enum! {
    /// segment types, either straight or spine
    pub SegmentType: i32 {
        SEGMENT_STRAIGHT = 0,
        SEGMENT_SPLINE = 1,
    }
}

/// `public struct wpnav_flags`
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct wpnav_flags {
    /// true if we have reached the destination
    pub reached_destination: bool,
    /// true if we should ignore the waypoint radius and consider the waypoint complete once
    /// the intermediate target has reached the waypoint
    pub fast_waypoint: bool,
    /// active segment is either straight or spline
    pub segment_type: SegmentType,
}

/// `public static float _wp_accel_cms` (1 m/s/s)
static WP_ACCEL_CMS: AtomicU32 = AtomicU32::new(0x42C8_0000); // 100.0f
/// `public static float _wp_speed_cms` (6 m/s)
static WP_SPEED_CMS: AtomicU32 = AtomicU32::new(0x4416_0000); // 600.0f

const LATLON_TO_CM: f32 = 1.113195;

/// `public class Spline2 : Utils`
#[derive(Debug, Clone)]
pub struct Spline2 {
    pub target_pos: Vector3,
    pub target_vel: Vector3,
    pub _flags: wpnav_flags,
    /// current spline time between origin and destination
    _spline_time: f32,
    /// the target velocity vector at the origin of the spline segment
    pub _spline_origin_vel: Vector3,
    /// the target velocity vector at the destination point of the spline segment
    _spline_destination_vel: Vector3,
    /// array describing spline path between origin and destination (null entries until set)
    _hermite_spline_solution: [Option<Vector3>; 4],
    _spline_vel_scaler: f32,
    /// vehicle should begin to slow down once it is within this distance from the destination
    _slow_down_dist: f32,
    /// heading according to yaw
    _yaw: f32,
    _wp_last_update: u32,
    /// null until `set_wp_origin_and_destination`
    pub _origin: Option<Vector3>,
    pub _destination: Option<Vector3>,
    scale_long_down: f64,
    scale_long_up: f64,
    home_location: PointLatLngAlt,
    /// `_track_length`
    pub _track_length: f64,
}

impl Spline2 {
    /// `_wp_accel_cms`
    pub fn wp_accel_cms() -> f32 {
        f32::from_bits(WP_ACCEL_CMS.load(Ordering::Relaxed))
    }

    pub fn set_wp_accel_cms(v: f32) {
        WP_ACCEL_CMS.store(v.to_bits(), Ordering::Relaxed);
    }

    /// `_wp_speed_cms`
    pub fn wp_speed_cms() -> f32 {
        f32::from_bits(WP_SPEED_CMS.load(Ordering::Relaxed))
    }

    pub fn set_wp_speed_cms(v: f32) {
        WP_SPEED_CMS.store(v.to_bits(), Ordering::Relaxed);
    }

    /// `new Spline2(PointLatLngAlt homeLocation)`
    pub fn new(home_location: PointLatLngAlt) -> Self {
        Spline2 {
            target_pos: Vector3::default(),
            target_vel: Vector3::default(),
            _flags: wpnav_flags::default(),
            _spline_time: 0.0,
            _spline_origin_vel: Vector3::default(),
            _spline_destination_vel: Vector3::default(),
            _hermite_spline_solution: [None; 4],
            _spline_vel_scaler: 0.0,
            _slow_down_dist: 0.0,
            _yaw: 0.0,
            _wp_last_update: 0,
            _origin: None,
            _destination: None,
            scale_long_down: 0.0,
            scale_long_up: 0.0,
            home_location,
            _track_length: 0.0,
        }
    }

    /// `pv_location_to_vector(loc)`: lat/lon to a position vector (cm from home).
    pub fn pv_location_to_vector(&mut self, loc: &PointLatLngAlt) -> Vector3 {
        let home = &self.home_location;

        self.scale_long_down = Self::longitude_scale(home) as f64;
        self.scale_long_up = 1.0f32 as f64 / self.scale_long_down;

        let l = LATLON_TO_CM as f64;
        Vector3::new(
            (loc.Lat * 1.0e7f32 as f64 - home.Lat * 1.0e7f32 as f64) * l,
            (loc.Lng * 1.0e7f32 as f64 - home.Lng * 1.0e7f32 as f64) * l * self.scale_long_down,
            loc.Alt * 100.0,
        )
    }

    /// `pv_vector_to_location(loc)`: the int lat/lon times `1.0e-7f` is float arithmetic.
    pub fn pv_vector_to_location(&self, loc: &Vector3) -> PointLatLngAlt {
        let lat = (self.pv_get_lat(loc) as f32 * 1.0e-7f32) as f64;
        let lng = (self.pv_get_lon(loc) as f32 * 1.0e-7f32) as f64;
        PointLatLngAlt::new_tagged(lat, lng, loc.z / 100.0, "")
    }

    /// `pv_get_lat(pos_vec)`
    fn pv_get_lat(&self, pos_vec: &Vector3) -> i32 {
        let home = &self.home_location;
        cast_i32(home.Lat * 1.0e7f32 as f64).wrapping_add(cast_i32(pos_vec.x / LATLON_TO_CM as f64))
    }

    /// `pv_get_lon(pos_vec)`
    fn pv_get_lon(&self, pos_vec: &Vector3) -> i32 {
        let home = &self.home_location;
        cast_i32(home.Lng * 1.0e7f32 as f64).wrapping_add(cast_i32(pos_vec.y / LATLON_TO_CM as f64 * self.scale_long_up))
    }

    /// `longitude_scale(loc)`
    fn longitude_scale(loc: &PointLatLngAlt) -> f32 {
        (loc.Lat * MathHelper::DEG2RAD).cos() as f32
    }

    /// `set_wp_origin_and_destination(origin, destination)`
    pub fn set_wp_origin_and_destination(&mut self, origin: Vector3, destination: Vector3) {
        // store origin and destination locations
        self._origin = Some(origin);
        self._destination = Some(destination);
        let pos_delta = destination - origin;

        self._track_length = pos_delta.length(); // get track length

        self._flags.reached_destination = false;
        self._flags.fast_waypoint = false; // default waypoint back to slow
        self._flags.segment_type = SegmentType::SEGMENT_STRAIGHT;
    }

    /// `set_spline_destination(destination, stopped_at_start, seg_end_type, next_destination)`:
    /// the origin is the previous destination once reached, else the stored origin.
    pub fn set_spline_destination(
        &mut self,
        destination: Vector3,
        stopped_at_start: bool,
        seg_end_type: spline_segment_end_type,
        next_destination: Vector3,
    ) -> Result<(), crate::dotnet::Exception> {
        let origin = if self._flags.reached_destination { self._destination } else { self._origin };
        // null origin: the vector arithmetic below throws NullReferenceException in C#
        let origin = origin.ok_or_else(|| {
            crate::dotnet::Exception::NullReference("Object reference not set to an instance of an object.".into())
        })?;

        self.set_spline_origin_and_destination(origin, destination, stopped_at_start, seg_end_type, next_destination)
    }

    fn set_spline_origin_and_destination(
        &mut self,
        origin: Vector3,
        destination: Vector3,
        stopped_at_start: bool,
        seg_end_type: spline_segment_end_type,
        next_destination: Vector3,
    ) -> Result<(), crate::dotnet::Exception> {
        let nre = || crate::dotnet::Exception::NullReference("Object reference not set to an instance of an object.".into());

        // mission is "active" if wpnav has been called recently and vehicle reached the previous waypoint
        let prev_segment_exists = self._flags.reached_destination;

        // calculate spline velocity at origin
        if stopped_at_start || !prev_segment_exists {
            // if vehicle is stopped at the origin, set origin velocity to 0.1 * distance vector from origin to destination
            self._spline_origin_vel = (destination - origin) * (0.1f32 as f64);
            self._spline_time = 0.0;
            self._spline_vel_scaler = 0.0;
        } else if self._flags.segment_type == SegmentType::SEGMENT_STRAIGHT {
            // previous segment is straight, vehicle is moving so vehicle should fly straight through the origin
            // before beginning it's spline path to the next waypoint.
            self._spline_origin_vel = self._destination.ok_or_else(nre)? - self._origin.ok_or_else(nre)?;
            self._spline_time = 0.0;
            self._spline_vel_scaler = 0.0;
        } else {
            // previous segment is splined, vehicle will fly through origin
            self._spline_origin_vel = self._spline_destination_vel;
            if self._spline_time > 1.0 && self._spline_time < 1.1 {
                // To-Do: remove hard coded 1.1f
                self._spline_time -= 1.0;
            } else {
                self._spline_time = 0.0;
            }
            self._spline_vel_scaler = 0.0;
        }

        // calculate spline velocity at destination
        match seg_end_type {
            spline_segment_end_type::SEGMENT_END_STOP => {
                // if vehicle stops at the destination set destination velocity to 0.1 * distance vector from origin to destination
                self._spline_destination_vel = (destination - origin) * (0.1f32 as f64);
                self._flags.fast_waypoint = false;
            }
            spline_segment_end_type::SEGMENT_END_STRAIGHT => {
                // if next segment is straight, vehicle's final velocity should face along the next segment's position
                self._spline_destination_vel = next_destination - destination;
                self._flags.fast_waypoint = true;
            }
            spline_segment_end_type::SEGMENT_END_SPLINE => {
                // if next segment is splined, vehicle's final velocity should face parallel to the line from the origin to the next destination
                self._spline_destination_vel = next_destination - origin;
                self._flags.fast_waypoint = true;
            }
            _ => {}
        }

        // code below ensures we don't get too much overshoot when the next segment is short
        let vel_len = (self._spline_origin_vel + self._spline_destination_vel).length() as f32;
        let pos_len = (destination - origin).length() as f32 * 4.0f32;
        if vel_len > pos_len {
            // if total start+stop velocity is more than twice position difference
            // use a scaled down start and stop velocityscale the  start and stop velocities down
            let vel_scaling = pos_len / vel_len;
            // update spline calculator
            self.update_spline_solution(
                origin,
                destination,
                self._spline_origin_vel * vel_scaling as f64,
                self._spline_destination_vel * vel_scaling as f64,
            );
        } else {
            // update spline calculator
            self.update_spline_solution(origin, destination, self._spline_origin_vel, self._spline_destination_vel);
        }

        // store origin and destination locations
        self._origin = Some(origin);
        self._destination = Some(destination);

        // calculate slow down distance
        self.calc_slow_down_distance(Self::wp_speed_cms(), Self::wp_accel_cms());

        // initialise intermediate point to the origin
        self._flags.reached_destination = false;
        self._flags.segment_type = SegmentType::SEGMENT_SPLINE;
        Ok(())
    }

    /// `calc_slow_down_distance(speed_cms, accel_cmss)`: v^2 / (4a).
    fn calc_slow_down_distance(&mut self, speed_cms: f32, accel_cmss: f32) {
        // protect against divide by zero
        if accel_cmss <= 0.0 {
            self._slow_down_dist = 0.0;
            return;
        }
        self._slow_down_dist = speed_cms * speed_cms / (4.0f32 * accel_cmss);
    }

    /// `update_spline()`: the clock is fixed at 0 in the C# (`now = 0`), so `now -
    /// _wp_last_update` wraps as a uint.
    pub fn update_spline(&mut self) -> Result<(), crate::dotnet::Exception> {
        // exit immediately if this is not a spline segment
        if self._flags.segment_type != SegmentType::SEGMENT_SPLINE {
            return Ok(());
        }

        // calculate dt
        let now: u32 = 0; //hal.scheduler->millis();
        let mut dt = now.wrapping_sub(self._wp_last_update) as f32 / 1000.0f32;

        // reset step back to 0 if 0.1 seconds has passed and we completed the last full cycle
        if dt > 0.095f32 {
            // double check dt is reasonable
            if dt >= 1.0f32 {
                dt = 0.0;
            }
            // capture time since last iteration
            self._wp_last_update = now;

            // advance the target if necessary
            self.advance_spline_target_along_track(dt)?;
        }
        Ok(())
    }

    fn update_spline_solution(&mut self, origin: Vector3, dest: Vector3, origin_vel: Vector3, dest_vel: Vector3) {
        self._hermite_spline_solution[0] = Some(origin);
        self._hermite_spline_solution[1] = Some(origin_vel);
        self._hermite_spline_solution[2] = Some(-origin * 3.0 - origin_vel * 2.0 + dest * 3.0 - dest_vel);
        self._hermite_spline_solution[3] = Some(origin * 2.0 + origin_vel - dest * 2.0 + dest_vel);
    }

    /// `advance_spline_target_along_track(dt)`
    pub fn advance_spline_target_along_track(&mut self, dt: f32) -> Result<(), crate::dotnet::Exception> {
        if !self._flags.reached_destination {
            // update target position and velocity from spline calculator
            let (pos, vel) = self.calc_spline_pos_vel(self._spline_time)?;
            self.target_pos = pos;
            self.target_vel = vel;

            let destination = self._destination.ok_or_else(|| {
                crate::dotnet::Exception::NullReference("Object reference not set to an instance of an object.".into())
            })?;

            // update velocity
            let spline_dist_to_wp = (destination - self.target_pos).length() as f32;

            let accel = Self::wp_accel_cms();
            let speed = Self::wp_speed_cms();
            // if within the stopping distance from destination, set target velocity to sqrt of distance * 2 * acceleration
            if !self._flags.fast_waypoint && spline_dist_to_wp < self._slow_down_dist {
                self._spline_vel_scaler = ((spline_dist_to_wp * 2.0f32 * accel) as f64).sqrt() as f32;
            } else if self._spline_vel_scaler < speed {
                // increase velocity using acceleration
                self._spline_vel_scaler += accel * dt;
            }

            // constrain target velocity
            if self._spline_vel_scaler > speed {
                self._spline_vel_scaler = speed;
            }

            // scale the spline_time by the velocity we've calculated vs the velocity that came out of the spline calculator
            let spline_time_scale = (self._spline_vel_scaler as f64 / self.target_vel.length()) as f32;

            // update the yaw
            self._yaw = Self::radians_to_centi_degrees(Utils::atan2(self.target_vel.y, self.target_vel.x));

            // advance spline time to next step
            self._spline_time += spline_time_scale * dt;

            // we will reach the next waypoint in the next step so set reached_destination flag
            if self._spline_time >= 1.0f32 {
                self._flags.reached_destination = true;
            }
        }
        Ok(())
    }

    /// `RadiansToCentiDegrees(p)`: despite the name, degrees (`p * rad2deg`).
    fn radians_to_centi_degrees(p: f64) -> f32 {
        (p * MathHelper::RAD2DEG) as f32
    }

    /// `calc_spline_pos_vel(spline_time, ref position, ref velocity)`
    fn calc_spline_pos_vel(&self, spline_time: f32) -> Result<(Vector3, Vector3), crate::dotnet::Exception> {
        let nre = || crate::dotnet::Exception::NullReference("Object reference not set to an instance of an object.".into());
        let h0 = self._hermite_spline_solution[0].ok_or_else(nre)?;
        let h1 = self._hermite_spline_solution[1].ok_or_else(nre)?;
        let h2 = self._hermite_spline_solution[2].ok_or_else(nre)?;
        let h3 = self._hermite_spline_solution[3].ok_or_else(nre)?;

        let spline_time_sqrd = spline_time * spline_time;
        let spline_time_cubed = spline_time_sqrd * spline_time;

        let position = h0 + h1 * spline_time as f64 + h2 * spline_time_sqrd as f64 + h3 * spline_time_cubed as f64;

        let velocity = h1 + h2 * 2.0 * spline_time as f64 + h3 * 3.0 * spline_time_sqrd as f64;
        Ok((position, velocity))
    }

    /// `_yaw` (private in C#; for inspection)
    pub fn yaw(&self) -> f32 {
        self._yaw
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn straight_spline_reaches_destination() {
        let mut s = Spline2::new(PointLatLngAlt::new_alt(-35.0, 149.0, 0.0));
        let a = s.pv_location_to_vector(&PointLatLngAlt::new_alt(-35.0, 149.0, 10.0));
        let b = s.pv_location_to_vector(&PointLatLngAlt::new_alt(-35.001, 149.0, 10.0));
        s.set_wp_origin_and_destination(a, a);
        s.set_spline_destination(b, true, spline_segment_end_type::SEGMENT_END_STOP, b).unwrap();
        for _ in 0..1000 {
            s.advance_spline_target_along_track(0.1).unwrap();
            if s._flags.reached_destination {
                break;
            }
        }
        assert!(s._flags.reached_destination);
        let back = s.pv_vector_to_location(&b);
        assert!((back.Lat - -35.001).abs() < 1e-5, "{back}");
        assert_eq!(Spline2::wp_speed_cms(), 600.0);
    }
}

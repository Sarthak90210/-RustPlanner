//! Per-sensor proximity retention, based on ExtLibs/ArduPilot/Proximity.cs.
use std::time::{Duration, Instant};
use mavlink::dialects::ardupilotmega::{MavFrame, MavMessage};

#[derive(Debug, Clone)]
pub struct Reading {
    pub orientation: u8,
    pub angle_deg: f32,
    pub width_deg: f32,
    pub distance_m: f32,
    pub obstacle_array: bool,
    expires: Instant,
}

#[derive(Debug, Default)]
pub struct Proximity {
    samples: Vec<Sample>,
}

#[derive(Debug)]
struct Sample {
    sys: u8,
    comp: u8,
    sensor_id: u8,
    orientation: u8,
    // Scalar directionState.data leaves Angle at its default zero. Custom
    // samples use the unnormalized angle, matching the reference replacement key.
    angle_key: f32,
    reading: Reading,
}

impl Proximity {
    pub fn update(&mut self, sys: u8, comp: u8, msg: &MavMessage, yaw_deg: f32, now: Instant) {
        // Each reference MAVState subscribes to its own sysid/compid. Within that
        // instance, replacing a sample appends it in arrival order.
        self.samples.retain(|s| s.reading.expires >= now);
        match msg {
            MavMessage::DISTANCE_SENSOR(d) => {
                let orientation = d.orientation as u8;
                // Reference renders the eight yaw rotations and CUSTOM (default
                // Angle=0, Size=45). Other rotations are not horizontal sectors.
                if (orientation > 7 && orientation != 100) || d.current_distance <= d.min_distance || d.current_distance >= d.max_distance { return; }
                self.samples.retain(|s| !(s.sys == sys && s.comp == comp && s.sensor_id == d.id && s.orientation == orientation));
                self.samples.push(Sample { sys, comp, sensor_id: d.id, orientation, angle_key: 0.0, reading: Reading {
                    orientation, angle_deg: if orientation == 100 { 0.0 } else { orientation as f32 * 45.0 }, width_deg: 45.0,
                    distance_m: d.current_distance as f32 / 100.0, obstacle_array: false,
                    expires: now + Duration::from_secs(3),
                }});
            }
            MavMessage::OBSTACLE_DISTANCE(d) => {
                // Preserve reference precedence and GLOBAL yaw convention.
                let step = if d.increment == 0 { d.increment_f } else { d.increment as f32 };
                let start = d.angle_offset + if d.frame == MavFrame::MAV_FRAME_GLOBAL { yaw_deg } else { 0.0 };
                if !step.is_finite() || !start.is_finite() { return; }
                for (i, &distance) in d.distances.iter().enumerate() {
                    if distance == u16::MAX || distance < d.min_distance || distance > d.max_distance { continue; }
                    let angle = start + step * i as f32;
                    let sensor_id = d.sensor_type as u8;
                    // Add(id, angle, ...) removes all matching id/angle samples,
                    // even a scalar sample whose default Angle happens to be zero.
                    self.samples.retain(|s| !(s.sys == sys && s.comp == comp && s.sensor_id == sensor_id && s.angle_key == angle));
                    self.samples.push(Sample { sys, comp, sensor_id, orientation: 100, angle_key: angle, reading: Reading {
                        orientation: 100, angle_deg: angle, width_deg: step,
                        distance_m: distance as f32 / 100.0, obstacle_array: true,
                        expires: now + Duration::from_millis(200),
                    }});
                }
            }
            _ => {}
        }
    }

    pub fn readings(&self, sys: u8, comp: u8, now: Instant) -> Vec<Reading> {
        self.samples.iter().filter(|s| s.sys == sys && s.comp == comp && s.reading.expires >= now).map(|s| s.reading.clone()).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavlink::dialects::ardupilotmega::*;
    fn distance(id: u8, orientation: MavSensorOrientation, cm: u16) -> MavMessage {
        MavMessage::DISTANCE_SENSOR(DISTANCE_SENSOR_DATA { id, orientation, min_distance: 10, max_distance: 1000, current_distance: cm, ..Default::default() })
    }
    fn scan(offset: f32, width: f32, values: &[(usize, u16)]) -> MavMessage {
        let mut d = OBSTACLE_DISTANCE_DATA { frame: MavFrame::MAV_FRAME_BODY_FRD, angle_offset: offset, increment_f: width, min_distance: 10, max_distance: 1000, ..Default::default() };
        d.distances.fill(u16::MAX);
        for &(index, cm) in values { d.distances[index] = cm; }
        MavMessage::OBSTACLE_DISTANCE(d)
    }

    #[test]
    fn moving_scan_origin_keeps_previous_angles_until_individual_expiry() {
        let now = Instant::now();
        let mut p = Proximity::default();
        p.update(1, 1, &scan(10.0, 5.0, &[(0, 100)]), 0.0, now);
        p.update(1, 1, &scan(20.0, 5.0, &[(0, 200)]), 0.0, now + Duration::from_millis(100));
        assert_eq!(p.readings(1, 1, now + Duration::from_millis(100)).iter().map(|r| r.angle_deg).collect::<Vec<_>>(), [10.0, 20.0]);
        let active = p.readings(1, 1, now + Duration::from_millis(201));
        assert_eq!((active.len(), active[0].angle_deg, active[0].distance_m), (1, 20.0, 2.0));
    }

    #[test]
    fn changed_increment_replaces_equal_angles_and_unknown_bins_preserve_history() {
        let now = Instant::now();
        let mut p = Proximity::default();
        p.update(1, 1, &scan(0.0, 5.0, &[(0, 100), (1, 200)]), 0.0, now);
        p.update(1, 1, &scan(0.0, 10.0, &[(0, 300), (1, 400)]), 0.0, now + Duration::from_millis(100));
        p.update(1, 1, &scan(0.0, 10.0, &[]), 0.0, now + Duration::from_millis(150));
        let r = p.readings(1, 1, now + Duration::from_millis(150));
        assert_eq!(r.iter().map(|r| (r.angle_deg, r.width_deg, r.distance_m)).collect::<Vec<_>>(), [(5.0, 5.0, 2.0), (0.0, 10.0, 3.0), (10.0, 10.0, 4.0)]);
        assert_eq!(p.readings(1, 1, now + Duration::from_millis(201)).len(), 2);
        assert!(p.readings(1, 1, now + Duration::from_millis(301)).is_empty());
    }

    #[test]
    fn scalar_and_array_keys_match_reference_shared_sensor_list() {
        let now = Instant::now();
        let mut p = Proximity::default();
        p.update(1, 1, &distance(0, MavSensorOrientation::MAV_SENSOR_ROTATION_YAW_90, 100), 0.0, now);
        p.update(1, 1, &scan(0.0, 5.0, &[(0, 200)]), 0.0, now);
        // Scalar Angle defaults to zero, so array Add(id=0,angle=0) replaces it.
        assert_eq!(p.readings(1, 1, now).len(), 1);
        assert!(p.readings(1, 1, now)[0].obstacle_array);
        // Scalar Add compares Orientation, so it coexists with custom Orientation=100.
        p.update(1, 1, &distance(0, MavSensorOrientation::MAV_SENSOR_ROTATION_YAW_90, 150), 0.0, now);
        assert_eq!(p.readings(1, 1, now).len(), 2);
        p.update(1, 1, &scan(0.0, 5.0, &[(0, 250)]), 0.0, now);
        assert_eq!(p.readings(1, 1, now).len(), 1);
        assert_eq!(p.readings(1, 1, now)[0].distance_m, 2.5);
    }

    #[test]
    fn raw_angles_are_not_normalized_and_zero_or_negative_steps_match_reference() {
        let now = Instant::now();
        let mut p = Proximity::default();
        p.update(1, 1, &scan(360.0, 0.0, &[(0, 100), (1, 200)]), 0.0, now);
        p.update(1, 1, &scan(0.0, -5.0, &[(0, 300), (1, 400)]), 0.0, now);
        let r = p.readings(1, 1, now);
        assert_eq!(r.iter().map(|r| r.angle_deg).collect::<Vec<_>>(), [360.0, 0.0, -5.0]);
        assert_eq!(r[0].distance_m, 2.0);
    }

    #[test]
    fn scalar_custom_orientation_uses_reference_default_angle_and_sector_size() {
        let now = Instant::now();
        let mut p = Proximity::default();
        p.update(1, 1, &distance(9, MavSensorOrientation::MAV_SENSOR_ROTATION_CUSTOM, 300), 0.0, now);
        let r = p.readings(1, 1, now);
        assert_eq!(r.len(), 1);
        assert_eq!((r[0].orientation, r[0].angle_deg, r[0].width_deg, r[0].distance_m), (100, 0.0, 45.0, 3.0));
        assert!(!r[0].obstacle_array);
    }

    #[test]
    fn changing_global_yaw_retains_old_angular_samples() {
        let now = Instant::now();
        let mut p = Proximity::default();
        let mut msg = scan(0.0, 5.0, &[(0, 100)]);
        if let MavMessage::OBSTACLE_DISTANCE(d) = &mut msg { d.frame = MavFrame::MAV_FRAME_GLOBAL; }
        p.update(1, 1, &msg, 90.0, now);
        p.update(1, 1, &msg, 135.0, now + Duration::from_millis(100));
        assert_eq!(p.readings(1, 1, now + Duration::from_millis(150)).iter().map(|r| r.angle_deg).collect::<Vec<_>>(), [90.0, 135.0]);
    }
    #[test]
    fn sensor_instances_are_isolated_by_vehicle_and_component() {
        let now = Instant::now();
        let mut p = Proximity::default();
        p.update(1, 1, &distance(0, MavSensorOrientation::MAV_SENSOR_ROTATION_NONE, 100), 0.0, now);
        p.update(1, 1, &distance(1, MavSensorOrientation::MAV_SENSOR_ROTATION_YAW_90, 200), 0.0, now);
        p.update(1, 2, &distance(0, MavSensorOrientation::MAV_SENSOR_ROTATION_NONE, 300), 0.0, now);
        p.update(2, 1, &distance(0, MavSensorOrientation::MAV_SENSOR_ROTATION_NONE, 400), 0.0, now);
        assert_eq!(p.readings(1, 1, now).len(), 2);
        assert_eq!(p.readings(1, 1, now)[1].distance_m, 2.0);
        assert_eq!(p.readings(1, 2, now)[0].distance_m, 3.0);
        assert_eq!(p.readings(2, 1, now).len(), 1);
        p.update(1, 1, &distance(0, MavSensorOrientation::MAV_SENSOR_ROTATION_NONE, 150), 0.0, now + Duration::from_secs(2));
        let active = p.readings(1, 1, now + Duration::from_millis(3001));
        assert_eq!(active.len(), 1);
        assert_eq!(active[0].distance_m, 1.5);
    }
    #[test]
    fn vertical_and_out_of_range_distance_sensors_are_not_horizontal_obstacles() {
        let now = Instant::now();
        let mut p = Proximity::default();
        for msg in [distance(0, MavSensorOrientation::MAV_SENSOR_ROTATION_PITCH_270, 100), distance(1, MavSensorOrientation::MAV_SENSOR_ROTATION_NONE, 10), distance(2, MavSensorOrientation::MAV_SENSOR_ROTATION_NONE, 1000)] {
            p.update(1, 1, &msg, 0.0, now);
        }
        assert!(p.readings(1, 1, now).is_empty());
    }
    #[test]
    fn obstacle_array_angles_sentinels_and_short_expiry_match_reference() {
        let now = Instant::now();
        let mut p = Proximity::default();
        let mut d = OBSTACLE_DISTANCE_DATA { frame: MavFrame::MAV_FRAME_GLOBAL, increment: 0, increment_f: 5.5, angle_offset: -10.0, min_distance: 10, max_distance: 1000, ..Default::default() };
        d.distances.fill(u16::MAX);
        d.distances[0] = 100;
        d.distances[1] = 200;
        d.distances[2] = 1001;
        d.distances[3] = 9;
        p.update(1, 1, &MavMessage::OBSTACLE_DISTANCE(d), 90.0, now);
        let r = p.readings(1, 1, now);
        assert_eq!(r.len(), 2);
        assert_eq!((r[0].angle_deg, r[1].angle_deg, r[0].width_deg), (80.0, 85.5, 5.5));
        assert_eq!(p.readings(1, 1, now + Duration::from_millis(200)).len(), 2);
        assert!(p.readings(1, 1, now + Duration::from_millis(201)).is_empty());
    }
}

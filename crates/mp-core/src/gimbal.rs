//! GimbalManagerProtocol and CameraProtocol (ExtLibs/ArduPilot/Mavlink): the MAVLink gimbal
//! manager and camera state a vehicle reports, and the commands GimbalVideoControl sends.
//! Commands are returned as (MAV_CMD, params) for the link's COMMAND_LONG queue; `None` is the
//! original's `Task.FromResult(false)` when the capability is missing.

use std::collections::BTreeMap;

use mavlink::dialects::ardupilotmega::MavMessage;

use crate::geo::PointLatLngAlt;

pub mod cap {
    pub const HAS_RETRACT: u32 = 1;
    pub const HAS_NEUTRAL: u32 = 2;
    pub const HAS_PITCH_AXIS: u32 = 32;
    pub const HAS_YAW_AXIS: u32 = 256;
    pub const HAS_YAW_FOLLOW: u32 = 512;
    pub const HAS_YAW_LOCK: u32 = 1024;
    pub const CAN_POINT_LOCATION_LOCAL: u32 = 65536;
    pub const CAN_POINT_LOCATION_GLOBAL: u32 = 131072;
}

/// GIMBAL_MANAGER_FLAGS / GIMBAL_DEVICE_FLAGS
pub mod flags {
    pub const RETRACT: u32 = 1;
    pub const NEUTRAL: u32 = 2;
    pub const YAW_LOCK: u32 = 16;
    pub const YAW_IN_VEHICLE_FRAME: u32 = 32;
    pub const YAW_IN_EARTH_FRAME: u32 = 64;
}

/// CAMERA_CAP_FLAGS
pub mod camcap {
    pub const CAPTURE_VIDEO: u32 = 1;
    pub const CAPTURE_IMAGE: u32 = 2;
    pub const HAS_MODES: u32 = 4;
    pub const CAN_CAPTURE_IMAGE_IN_VIDEO_MODE: u32 = 8;
    pub const CAN_CAPTURE_VIDEO_IN_IMAGE_MODE: u32 = 16;
    pub const HAS_BASIC_ZOOM: u32 = 64;
    pub const HAS_BASIC_FOCUS: u32 = 128;
    pub const HAS_TRACKING_POINT: u32 = 512;
    pub const HAS_TRACKING_RECTANGLE: u32 = 1024;
}

pub const CMD_DO_SET_ROI_LOCATION: u16 = 195;
pub const CMD_SET_MESSAGE_INTERVAL: u16 = 511;
pub const CMD_REQUEST_MESSAGE: u16 = 512;
pub const CMD_REQUEST_CAMERA_INFORMATION: u16 = 521;
pub const CMD_SET_CAMERA_ZOOM: u16 = 531;
pub const CMD_DO_GIMBAL_MANAGER_PITCHYAW: u16 = 1000;
pub const CMD_IMAGE_START_CAPTURE: u16 = 2000;
pub const CMD_CAMERA_TRACK_POINT: u16 = 2004;
pub const CMD_CAMERA_TRACK_RECTANGLE: u16 = 2005;
pub const CMD_VIDEO_START_CAPTURE: u16 = 2500;
pub const CMD_VIDEO_STOP_CAPTURE: u16 = 2501;

pub const MSG_CAMERA_INFORMATION: f32 = 259.0;
pub const MSG_CAMERA_SETTINGS: f32 = 260.0;
pub const MSG_CAMERA_CAPTURE_STATUS: f32 = 262.0;
pub const MSG_VIDEO_STREAM_INFORMATION: f32 = 269.0;
pub const MSG_CAMERA_FOV_STATUS: f32 = 271.0;
pub const MSG_CAMERA_TRACKING_IMAGE_STATUS: f32 = 275.0;
pub const MSG_GIMBAL_MANAGER_INFORMATION: f32 = 280.0;

/// CAMERA_ZOOM_TYPE.ZOOM_TYPE_CONTINUOUS
pub const ZOOM_TYPE_CONTINUOUS: f32 = 1.0;

pub type Cmd = (u16, [f32; 7]);

/// `MissionPlanner.Utilities.Quaternion` (q1 = w), the file-by-file port of Quaternion.cs.
pub use crate::extlibs::utilities::quaternion::Quaternion;
use crate::extlibs::utilities::vector3::Vector3;

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn wrap_180(mut a: f64) -> f64 {
    while a > 180.0 {
        a -= 360.0;
    }
    while a < -180.0 {
        a += 360.0;
    }
    a
}

/// GimbalManagerProtocol: GIMBAL_MANAGER_INFORMATION / _STATUS and GIMBAL_DEVICE_ATTITUDE_STATUS
/// by gimbal device id; id 0 holds the lowest-numbered gimbal.
#[derive(Debug, Default, Clone)]
pub struct GimbalManager {
    pub cap_flags: BTreeMap<u8, u32>,
    pub status_flags: BTreeMap<u8, u32>,
    pub attitude: BTreeMap<u8, [f32; 4]>,
    /// gimbal_device_id each id-0 alias came from
    alias: [Option<u8>; 3],
}

impl GimbalManager {
    fn store<T: Copy>(map: &mut BTreeMap<u8, T>, alias: &mut Option<u8>, id: u8, v: T) {
        map.insert(id, v);
        if alias.is_none_or(|a| id <= a) {
            map.insert(0, v);
            *alias = Some(id);
        }
    }

    /// MessagesHandler (every packet on the interface).
    pub fn on_message(&mut self, msg: &MavMessage) {
        match msg {
            MavMessage::GIMBAL_MANAGER_INFORMATION(m) => Self::store(&mut self.cap_flags, &mut self.alias[0], m.gimbal_device_id, m.cap_flags.bits()),
            MavMessage::GIMBAL_MANAGER_STATUS(m) => Self::store(&mut self.status_flags, &mut self.alias[1], m.gimbal_device_id, m.flags.bits()),
            MavMessage::GIMBAL_DEVICE_ATTITUDE_STATUS(m) => Self::store(&mut self.attitude, &mut self.alias[2], m.gimbal_device_id, m.q),
            _ => {}
        }
    }

    pub fn has_capability(&self, f: u32, id: u8) -> bool {
        self.cap_flags.get(&id).is_some_and(|c| c & f != 0)
    }
    fn has_status(&self, f: u32, id: u8) -> bool {
        self.status_flags.get(&id).is_some_and(|c| c & f != 0)
    }
    pub fn yaw_in_vehicle_frame(&self, id: u8) -> bool {
        let earth = self.has_status(flags::YAW_IN_EARTH_FRAME, id);
        let vehicle = self.has_status(flags::YAW_IN_VEHICLE_FRAME, id);
        if !earth && !vehicle {
            return !self.has_status(flags::YAW_LOCK, id);
        }
        vehicle
    }
    /// GetAttitude: earth-frame attitude (vehicle yaw added when the gimbal reports body yaw).
    pub fn attitude(&self, id: u8, vehicle_yaw_deg: f64) -> Option<Quaternion> {
        let q = self.attitude.get(&id)?;
        let q = Quaternion::new(q[0] as f64, q[1] as f64, q[2] as f64, q[3] as f64);
        Some(if self.yaw_in_vehicle_frame(id) { Quaternion::from_euler(0.0, 0.0, vehicle_yaw_deg.to_radians()) * q } else { q })
    }

    /// Discover: REQUEST_MESSAGE GIMBAL_MANAGER_INFORMATION (broadcast)
    pub fn discover() -> Cmd {
        (CMD_REQUEST_MESSAGE, [MSG_GIMBAL_MANAGER_INFORMATION, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    }
    pub fn retract(&self) -> Option<Cmd> {
        self.has_capability(cap::HAS_RETRACT, 0).then_some((CMD_DO_GIMBAL_MANAGER_PITCHYAW, [f32::NAN, f32::NAN, f32::NAN, f32::NAN, flags::RETRACT as f32, 0.0, 0.0]))
    }
    pub fn neutral(&self) -> Option<Cmd> {
        self.has_capability(cap::HAS_NEUTRAL, 0).then_some((CMD_DO_GIMBAL_MANAGER_PITCHYAW, [f32::NAN, f32::NAN, f32::NAN, f32::NAN, flags::NEUTRAL as f32, 0.0, 0.0]))
    }
    pub fn set_angles(&self, pitch: f64, yaw: f64, yaw_lock: bool, id: u8) -> Option<Cmd> {
        if !self.has_capability(cap::CAN_POINT_LOCATION_LOCAL, 0)
            || (pitch != 0.0 && !self.has_capability(cap::HAS_PITCH_AXIS, 0))
            || (yaw != 0.0 && !self.has_capability(cap::HAS_YAW_AXIS, 0))
            || (yaw != 0.0 && yaw_lock && !self.has_capability(cap::HAS_YAW_LOCK, 0))
            || (yaw != 0.0 && !yaw_lock && !self.has_capability(cap::HAS_YAW_FOLLOW, 0))
        {
            return None;
        }
        let f = if yaw_lock { flags::YAW_LOCK as f32 } else { 0.0 };
        Some((CMD_DO_GIMBAL_MANAGER_PITCHYAW, [wrap_180(pitch) as f32, wrap_180(yaw) as f32, f32::NAN, f32::NAN, f, 0.0, id as f32]))
    }
    /// SetAttitudeAsync: an earth-frame attitude as pitch/yaw angles.
    pub fn set_attitude(&self, q: &Quaternion, yaw_lock: bool, vehicle_yaw_deg: f64, id: u8) -> Option<Cmd> {
        let pitch = q.get_euler_pitch().to_degrees();
        let mut yaw = q.get_euler_yaw().to_degrees();
        if !yaw_lock {
            yaw -= vehicle_yaw_deg;
        }
        self.set_angles(pitch, yaw, yaw_lock, id)
    }
    pub fn set_rates(&self, pitch_rate: f32, yaw_rate: f32, yaw_in_earth_frame: bool, id: u8) -> Option<Cmd> {
        if (pitch_rate != 0.0 && !self.has_capability(cap::HAS_PITCH_AXIS, 0))
            || (yaw_rate != 0.0 && !self.has_capability(cap::HAS_YAW_AXIS, 0))
            || (yaw_rate != 0.0 && yaw_in_earth_frame && !self.has_capability(cap::HAS_YAW_LOCK, 0))
            || (yaw_rate != 0.0 && !yaw_in_earth_frame && !self.has_capability(cap::HAS_YAW_FOLLOW, 0))
        {
            return None;
        }
        let f = if yaw_in_earth_frame { flags::YAW_LOCK as f32 } else { 0.0 };
        Some((CMD_DO_GIMBAL_MANAGER_PITCHYAW, [f32::NAN, f32::NAN, pitch_rate, yaw_rate, f, 0.0, id as f32]))
    }
    /// SetROILocationAsync: COMMAND_INT DO_SET_ROI_LOCATION (frame GLOBAL from GimbalVideoControl)
    pub fn roi_location(&self, lat: f64, lng: f64, alt: f64) -> Option<CommandInt> {
        self.has_capability(cap::CAN_POINT_LOCATION_GLOBAL, 0).then_some(CommandInt {
            command: CMD_DO_SET_ROI_LOCATION,
            frame: 0,
            params: [0.0; 4],
            x: (lat * 1e7) as i32,
            y: (lng * 1e7) as i32,
            z: alt as f32,
        })
    }
}

/// COMMAND_INT fields (doCommandInt).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CommandInt {
    pub command: u16,
    pub frame: u8,
    pub params: [f32; 4],
    pub x: i32,
    pub y: i32,
    pub z: f32,
}

/// VIDEO_STREAM_INFORMATION as CameraProtocol.VideoStreams keeps it.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct VideoStream {
    pub sysid: u8,
    pub compid: u8,
    pub stream_id: u8,
    /// VIDEO_STREAM_TYPE: RTSP 0, RTPUDP 1, TCP_MPEG 2, MPEG_TS 3
    pub kind: u8,
    /// VIDEO_STREAM_ENCODING: H265 2
    pub encoding: u8,
    pub uri: String,
    pub name: String,
}

/// CameraProtocol.GStreamerPipeline
pub fn gstreamer_pipeline(stream: &VideoStream) -> String {
    let uri = stream.uri.as_str();
    if let Some(p) = uri.strip_prefix("gst://") {
        return p.to_string();
    }
    let mut port = 0i64;
    if stream.kind == 1 || stream.kind == 3 {
        port = uri.parse().unwrap_or_else(|_| {
            // a colon followed by digits
            uri.match_indices(':')
                .find_map(|(i, _)| {
                    let d: String = uri[i + 1..].chars().take_while(char::is_ascii_digit).collect();
                    (!d.is_empty()).then(|| d.parse().unwrap_or(0))
                })
                .unwrap_or(0)
        });
        if !(1..=65535).contains(&port) {
            return String::new();
        }
    }
    let strip_scheme = |u: &str| u.find("://").map_or(u.to_string(), |i| u[i + 3..].to_string());
    match stream.kind {
        0 => format!("rtspsrc location=rtsp://{} latency=41 udp-reconnect=1 timeout=0 do-retransmission=false ! application/x-rtp ! decodebin3 ! queue leaky=2 ! videoconvert ! video/x-raw,format=BGRA ! appsink name=outsink sync=false", strip_scheme(uri)),
        1 => {
            let enc = if stream.encoding == 2 { "H265" } else { "H264" };
            format!("udpsrc port={port} buffer-size=90000 ! application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string){enc} ! decodebin3 ! queue max-size-buffers=1 leaky=2 ! videoconvert ! video/x-raw,format=BGRA ! appsink name=outsink sync=false")
        }
        2 => {
            let rest = strip_scheme(uri);
            let host: String = rest.chars().take_while(|c| *c != ':' && *c != '/').collect();
            let digits: String = rest.get(host.len() + 1..).unwrap_or("").chars().take_while(char::is_ascii_digit).collect();
            if host.is_empty() || !rest[host.len()..].starts_with(':') || digits.is_empty() {
                return String::new();
            }
            format!("tcpclientsrc host={host} port={digits} ! decodebin ! queue max-size-buffers=1 leaky=2 ! videoconvert ! video/x-raw,format=BGRA ! appsink name=outsink sync=false")
        }
        3 => format!("udpsrc port={port} buffer-size=90000 ! tsparse ! tsdemux ! decodebin ! queue max-size-buffers=1 leaky=2 ! videoconvert ! video/x-raw,format=BGRA ! appsink name=outsink sync=false"),
        _ => String::new(),
    }
}

/// CAMERA_FOV_STATUS
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct FovStatus {
    pub camera: (i32, i32, i32),
    pub image: (i32, i32, i32),
    pub q: [f32; 4],
    pub hfov: f32,
    pub vfov: f32,
}

/// CAMERA_TRACKING_IMAGE_STATUS
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct TrackingStatus {
    pub point: (f32, f32),
    pub radius: f32,
    pub rec_top: (f32, f32),
    pub rec_bottom: (f32, f32),
    /// CAMERA_TRACKING_STATUS_FLAGS: ACTIVE 1
    pub status: u8,
    /// CAMERA_TRACKING_MODE: POINT 1, RECTANGLE 2
    pub mode: u8,
    /// CAMERA_TRACKING_TARGET_DATA: RENDERED 2, IN_STATUS 4
    pub target_data: u8,
}

/// CameraProtocol for the selected vehicle's autopilot component.
#[derive(Debug, Clone)]
pub struct Camera {
    pub have_information: bool,
    pub flags: u32,
    pub cam_definition_uri: String,
    pub mode_id: u8,
    pub fov: FovStatus,
    pub tracking: TrackingStatus,
    pub use_fov_status: bool,
    pub hfov_setting: f32,
    pub vfov_setting: f32,
    image_sequence: i32,
}

impl Default for Camera {
    fn default() -> Self {
        Self {
            have_information: false,
            flags: 0,
            cam_definition_uri: String::new(),
            mode_id: 0,
            fov: FovStatus::default(),
            tracking: TrackingStatus::default(),
            use_fov_status: true,
            hfov_setting: f32::NAN,
            vfov_setting: f32::NAN,
            image_sequence: 1,
        }
    }
}

fn chars(b: &[u8]) -> String {
    let end = b.iter().position(|c| *c == 0).unwrap_or(b.len());
    String::from_utf8_lossy(&b[..end]).into_owned()
}

impl Camera {
    /// ParseMessages for this camera's (sysid, compid); a VIDEO_STREAM_INFORMATION is returned
    /// for the shared VideoStreams table.
    pub fn on_message(&mut self, msg: &MavMessage, sysid: u8, compid: u8) -> Option<VideoStream> {
        match msg {
            MavMessage::CAMERA_INFORMATION(m) => {
                self.have_information = true;
                self.flags = m.flags.bits();
                self.cam_definition_uri = chars(&m.cam_definition_uri[..]);
            }
            MavMessage::CAMERA_SETTINGS(m) => self.mode_id = m.mode_id as u8,
            MavMessage::VIDEO_STREAM_INFORMATION(m) => {
                return Some(VideoStream { sysid, compid, stream_id: m.stream_id, kind: m.mavtype as u8, encoding: m.encoding as u8, uri: chars(&m.uri[..]), name: chars(&m.name[..]) });
            }
            MavMessage::CAMERA_FOV_STATUS(m) => {
                self.fov = FovStatus { camera: (m.lat_camera, m.lon_camera, m.alt_camera), image: (m.lat_image, m.lon_image, m.alt_image), q: m.q, hfov: m.hfov, vfov: m.vfov };
            }
            MavMessage::CAMERA_TRACKING_IMAGE_STATUS(m) => {
                self.tracking = TrackingStatus {
                    point: (m.point_x, m.point_y),
                    radius: m.radius,
                    rec_top: (m.rec_top_x, m.rec_top_y),
                    rec_bottom: (m.rec_bottom_x, m.rec_bottom_y),
                    status: m.tracking_status.bits(),
                    mode: m.tracking_mode as u8,
                    target_data: m.target_data.bits(),
                };
            }
            _ => {}
        }
        None
    }

    fn has(&self, f: u32) -> bool {
        self.flags & f != 0
    }
    pub fn has_modes(&self) -> bool {
        self.has(camcap::HAS_MODES)
    }
    pub fn can_capture_video(&self) -> bool {
        if !self.has(camcap::CAPTURE_VIDEO) {
            return false;
        }
        match self.mode_id {
            _ if !self.has_modes() => true,
            1 => true,
            0 | 2 => self.has(camcap::CAN_CAPTURE_VIDEO_IN_IMAGE_MODE),
            _ => false,
        }
    }
    pub fn can_capture_image(&self) -> bool {
        if !self.has(camcap::CAPTURE_IMAGE) {
            return false;
        }
        match self.mode_id {
            _ if !self.has_modes() => true,
            0 | 2 => true,
            1 => self.has(camcap::CAN_CAPTURE_IMAGE_IN_VIDEO_MODE),
            _ => false,
        }
    }
    /// HFOV: the camera's CAMERA_FOV_STATUS value unless UseFOVStatus is off. The original's
    /// `hfov == float.NaN` test is never true, so a camera that sent nothing reads 0.
    pub fn hfov(&self) -> f32 {
        if self.use_fov_status { self.fov.hfov } else { self.hfov_setting }
    }
    pub fn vfov(&self) -> f32 {
        if self.use_fov_status { self.fov.vfov } else { self.vfov_setting }
    }

    /// StartID / RequestCameraInformationAsync: CAMERA_INFORMATION, then VIDEO_STREAM_INFORMATION.
    pub fn request_information() -> [Cmd; 2] {
        [
            (CMD_REQUEST_MESSAGE, [MSG_CAMERA_INFORMATION, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            (CMD_REQUEST_MESSAGE, [MSG_VIDEO_STREAM_INFORMATION, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        ]
    }
    /// REQUEST_CAMERA_INFORMATION, sent when the REQUEST_MESSAGE form is refused.
    pub fn request_information_legacy() -> Cmd {
        (CMD_REQUEST_CAMERA_INFORMATION, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    }
    /// RequestMessageIntervals(ratehz)
    pub fn message_intervals(&self, ratehz: i32) -> Vec<Cmd> {
        if ratehz < 0 {
            return Vec::new();
        }
        let us = if ratehz > 0 { (1e6 / ratehz as f64) as f32 } else { -1.0 };
        let mut out = Self::request_information().to_vec();
        out.push((CMD_SET_MESSAGE_INTERVAL, [MSG_CAMERA_FOV_STATUS, us, 0.0, 0.0, 0.0, 0.0, 0.0]));
        if self.has_modes() || self.has(camcap::HAS_BASIC_ZOOM) || self.has(camcap::HAS_BASIC_FOCUS) {
            out.push((CMD_SET_MESSAGE_INTERVAL, [MSG_CAMERA_SETTINGS, us, 0.0, 0.0, 0.0, 0.0, 0.0]));
        }
        if self.has(camcap::CAPTURE_VIDEO) || self.has(camcap::CAPTURE_IMAGE) {
            out.push((CMD_SET_MESSAGE_INTERVAL, [MSG_CAMERA_CAPTURE_STATUS, us, 0.0, 0.0, 0.0, 0.0, 0.0]));
        }
        out
    }
    pub fn tracking_interval(ratehz: i32) -> Cmd {
        (CMD_SET_MESSAGE_INTERVAL, [MSG_CAMERA_TRACKING_IMAGE_STATUS, (1e6 / ratehz as f64) as f32, 0.0, 0.0, 0.0, 0.0, 0.0])
    }
    pub fn take_picture(&mut self) -> Cmd {
        let seq = self.image_sequence;
        self.image_sequence += 1;
        (CMD_IMAGE_START_CAPTURE, [0.0, 0.0, 1.0, seq as f32, 0.0, 0.0, 0.0])
    }
    pub fn start_recording() -> Cmd {
        (CMD_VIDEO_START_CAPTURE, [0.0, f32::NAN, f32::NAN, f32::NAN, 0.0, 0.0, f32::NAN])
    }
    pub fn stop_recording() -> Cmd {
        (CMD_VIDEO_STOP_CAPTURE, [0.0, f32::NAN, f32::NAN, f32::NAN, 0.0, 0.0, f32::NAN])
    }
    pub fn zoom(level: f32, zoom_type: f32) -> Cmd {
        (CMD_SET_CAMERA_ZOOM, [zoom_type, level, 0.0, 0.0, 0.0, 0.0, 0.0])
    }
    /// SetTrackingPointAsync: image coordinates -1..1 become 0..1.
    pub fn track_point(&self, x: f32, y: f32) -> Option<Cmd> {
        self.has(camcap::HAS_TRACKING_POINT).then_some((CMD_CAMERA_TRACK_POINT, [(x + 1.0) / 2.0, (y + 1.0) / 2.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
    }
    pub fn track_rectangle(&self, x1: f32, y1: f32, x2: f32, y2: f32) -> Option<Cmd> {
        if !self.has(camcap::HAS_TRACKING_RECTANGLE) {
            return None;
        }
        let (a, b) = ((x1 + 1.0) / 2.0, (x2 + 1.0) / 2.0);
        let (c, d) = ((y1 + 1.0) / 2.0, (y2 + 1.0) / 2.0);
        Some((CMD_CAMERA_TRACK_RECTANGLE, [a.min(b), c.min(d), a.max(b), c.max(d), 0.0, 0.0, 0.0]))
    }

    /// CalculateImagePointLocation: where image point (x, y) in -1..1 meets the ground.
    pub fn image_point_location(&self, x: f64, y: f64) -> Option<PointLatLngAlt> {
        let f = &self.fov;
        let image = PointLatLngAlt::new(f.image.0 as f64 * 1e-7, f.image.1 as f64 * 1e-7, f.image.2 as f64 * 1e-3);
        if x == 0.0 && y == 0.0 {
            return Some(image);
        }
        let cam = PointLatLngAlt::new(f.camera.0 as f64 * 1e-7, f.camera.1 as f64 * 1e-7, f.camera.2 as f64 * 1e-3);
        let height = cam.alt - image.alt;
        if height < 0.0 {
            return None;
        }
        let dist = cam.distance(&image);
        let down = (height.atan2(dist) + y / 2.0 * self.vfov() as f64 * std::f64::consts::PI / 180.0).max(0.0001);
        let out = (height * down.cos() / down.sin()).min(1e5);
        let side_angle = x / 2.0 * self.hfov() as f64 * std::f64::consts::PI / 180.0;
        let side = (out * out + height * height).sqrt() * side_angle.tan();
        let bearing = cam.bearing(&image);
        let mut pos = cam.newpos(bearing, out).newpos(bearing + 90.0, side);
        pos.alt = image.alt;
        Some(pos)
    }

    fn image_vector_camera_frame(&self, x: f64, y: f64) -> [f64; 3] {
        let mut v = [1.0, 0.0, 0.0];
        if x != 0.0 && y != 0.0 {
            let (h, w) = ((self.hfov() as f64).to_radians(), (self.vfov() as f64).to_radians());
            v[1] = (x * h / 2.0).tan();
            v[2] = (y * w / 2.0).tan();
            let n = dot(v, v).sqrt();
            v = v.map(|c| c / n);
        }
        v
    }

    /// CalculateImagePointRotation: the rotation taking the image centre to (x, y).
    pub fn image_point_rotation(&self, x: f64, y: f64) -> Quaternion {
        let v1 = self.image_vector_camera_frame(0.0, 0.0);
        let v2 = self.image_vector_camera_frame(x, y);
        if v1 == v2.map(|c| -c) {
            return Quaternion::from_axis_angle_theta(&Vector3::new(0.0, 0.0, 1.0), std::f64::consts::PI);
        }
        let axis = cross(v1, v2);
        let len = dot(axis, axis).sqrt();
        if len == 0.0 {
            return Quaternion::default();
        }
        let [x, y, z] = axis.map(|c| c / len);
        Quaternion::from_axis_angle_theta(&Vector3::new(x, y, z), dot(v1, v2).clamp(-1.0, 1.0).acos())
    }
}

/// The gimbal video control's mouse position: control pixels to image coordinates -1..1,
/// with the letterbox of a zoomed (aspect-kept) picture removed (getMousePosition).
pub fn image_position(x: f64, y: f64, box_w: f64, box_h: f64, img_w: f64, img_h: f64) -> Option<(f64, f64)> {
    if img_w <= 0.0 || img_h <= 0.0 {
        return None;
    }
    let iw = box_w.min(box_h * img_w / img_h);
    let ih = box_h.min(box_w * img_h / img_w);
    let (mut x, mut y) = (x, y);
    // the original works in integer pixels: `x *= Width / imageWidth` is an integer factor
    let (iw, ih) = (iw.floor(), ih.floor());
    if iw < box_w {
        x = (x - ((box_w - iw) / 2.0).floor()) * (box_w / iw).floor();
        x = x.clamp(0.0, iw);
    }
    if ih < box_h {
        y = (y - ((box_h - ih) / 2.0).floor()) * (box_h / ih).floor();
        y = y.clamp(0.0, ih);
    }
    Some((2.0 * x / iw - 1.0, 2.0 * y / ih - 1.0))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn capability_checks_gate_gimbal_commands() {
        let mut g = GimbalManager::default();
        assert_eq!(g.retract(), None, "no GIMBAL_MANAGER_INFORMATION yet");
        g.cap_flags.insert(0, cap::HAS_RETRACT | cap::HAS_PITCH_AXIS | cap::CAN_POINT_LOCATION_LOCAL);
        let (c, p) = g.retract().unwrap();
        assert_eq!(c, 1000);
        assert!(p[0].is_nan() && p[3].is_nan());
        assert_eq!(p[4], 1.0);
        assert_eq!(g.neutral(), None);
        // Point Down: pitch only
        let (_, p) = g.set_angles(-90.0, 0.0, false, 0).unwrap();
        assert_eq!((p[0], p[1], p[4]), (-90.0, 0.0, 0.0));
        // a yaw needs the yaw axis
        assert_eq!(g.set_angles(-10.0, 30.0, false, 0), None);
        assert_eq!(g.set_rates(5.0, 0.0, true, 0).unwrap().1[2..5], [5.0, 0.0, 16.0]);
        assert_eq!(g.set_rates(0.0, 5.0, true, 0), None);
        assert_eq!(g.set_angles(190.0, 0.0, false, 0).unwrap().1[0], -170.0, "wrap_180");
    }

    #[test]
    fn lowest_gimbal_is_alias_zero() {
        let mut g = GimbalManager::default();
        let mut m = BTreeMap::new();
        GimbalManager::store(&mut m, &mut g.alias[0], 2, 7u32);
        GimbalManager::store(&mut m, &mut g.alias[0], 1, 9u32);
        GimbalManager::store(&mut m, &mut g.alias[0], 3, 11u32);
        assert_eq!((m[&0], m[&1], m[&2], m[&3]), (9, 9, 7, 11));
    }

    #[test]
    fn yaw_frame_follows_status_flags() {
        let mut g = GimbalManager::default();
        assert!(g.yaw_in_vehicle_frame(0), "no flags and no yaw lock: vehicle frame");
        g.status_flags.insert(0, flags::YAW_LOCK);
        assert!(!g.yaw_in_vehicle_frame(0));
        g.status_flags.insert(0, flags::YAW_LOCK | flags::YAW_IN_VEHICLE_FRAME);
        assert!(g.yaw_in_vehicle_frame(0));
        // body attitude straight ahead plus vehicle heading 90
        g.status_flags.insert(0, 0);
        g.attitude.insert(0, [1.0, 0.0, 0.0, 0.0]);
        let q = g.attitude(0, 90.0).unwrap();
        assert!((q.get_euler_yaw().to_degrees() - 90.0).abs() < 1e-6);
    }

    #[test]
    fn pipelines_match_gstreamerpipeline() {
        let s = |kind, uri: &str| VideoStream { kind, uri: uri.into(), ..Default::default() };
        assert_eq!(gstreamer_pipeline(&s(0, "gst://videotestsrc ! appsink name=outsink")), "videotestsrc ! appsink name=outsink");
        assert!(gstreamer_pipeline(&s(0, "http://10.0.0.1:8554/live")).starts_with("rtspsrc location=rtsp://10.0.0.1:8554/live latency=41"));
        assert!(gstreamer_pipeline(&s(1, "5600")).starts_with("udpsrc port=5600 buffer-size=90000 ! application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string)H264"));
        assert!(gstreamer_pipeline(&VideoStream { encoding: 2, ..s(1, "udp://0.0.0.0:5601") }).contains("port=5601") );
        assert!(gstreamer_pipeline(&VideoStream { encoding: 2, ..s(1, "udp://0.0.0.0:5601") }).contains("H265"));
        assert_eq!(gstreamer_pipeline(&s(1, "nothing")), "");
        assert!(gstreamer_pipeline(&s(2, "tcp://192.168.0.10:5000")).starts_with("tcpclientsrc host=192.168.0.10 port=5000 ! decodebin"));
        assert_eq!(gstreamer_pipeline(&s(2, "nohost")), "");
        assert!(gstreamer_pipeline(&s(3, "5600")).contains("tsparse ! tsdemux"));
        assert_eq!(gstreamer_pipeline(&s(9, "x")), "");
    }

    #[test]
    fn camera_commands_and_capabilities() {
        let mut c = Camera::default();
        assert!(!c.can_capture_image() && !c.can_capture_video());
        c.flags = camcap::CAPTURE_IMAGE | camcap::CAPTURE_VIDEO | camcap::HAS_MODES;
        c.mode_id = 1;
        assert!(c.can_capture_video() && !c.can_capture_image());
        c.mode_id = 0;
        assert!(!c.can_capture_video() && c.can_capture_image());
        assert_eq!(c.take_picture().1[3], 1.0);
        assert_eq!(c.take_picture().1[3], 2.0, "sequence prevents double triggers");
        assert_eq!(c.track_point(0.0, 0.0), None);
        c.flags |= camcap::HAS_TRACKING_POINT | camcap::HAS_TRACKING_RECTANGLE;
        assert_eq!(c.track_point(0.0, -1.0).unwrap().1[..2], [0.5, 0.0]);
        assert_eq!(c.track_rectangle(1.0, 1.0, -1.0, 0.0).unwrap().1[..4], [0.0, 0.5, 1.0, 1.0]);
        assert_eq!(c.message_intervals(4).len(), 5);
        assert_eq!(c.message_intervals(-1).len(), 0);
    }

    #[test]
    fn image_geometry() {
        // 4:3 picture pillarboxed in a 200x120 box: the 20 px bars clamp to the picture edges
        assert_eq!(image_position(100.0, 60.0, 200.0, 120.0, 640.0, 480.0), Some((0.0, 0.0)));
        let (x, _) = image_position(0.0, 60.0, 200.0, 120.0, 640.0, 480.0).unwrap();
        assert_eq!(x, -1.0);
        let mut c = Camera::default();
        c.fov = FovStatus { camera: (0, 0, 100_000), image: (8_983, 0, 0), q: [1.0, 0.0, 0.0, 0.0], hfov: 40.0, vfov: 30.0 };
        assert_eq!(c.image_point_location(0.0, 0.0).unwrap().lat, 8_983.0 * 1e-7);
        // lower in the picture is closer to the camera
        let near = c.image_point_location(0.0, 0.5).unwrap();
        assert!(near.lat < 8_983.0 * 1e-7 && near.lat > 0.0);
        let right = c.image_point_location(0.5, 0.0).unwrap();
        assert!(right.lng > 0.0, "right of a north-looking camera is east");
        let q = c.image_point_rotation(0.5, 0.5);
        assert!(q.get_euler_yaw() > 0.0 && q.get_euler_pitch() < 0.0);
        assert_eq!(c.image_point_rotation(0.0, 0.0), Quaternion::default());
    }
}

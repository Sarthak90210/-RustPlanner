//! Port of `ExtLibs/Utilities/DFLog.cs`
//!
//! Read a log and extract its messages. The C# `DFItem` is a struct whose property getters
//! read (and write) its parent `DFLog`; here the parent is passed to those methods. The
//! `_dfLogBuffer` back reference is the `instance_source` argument of `DFItem::instance`.

#![allow(non_snake_case, non_camel_case_types)]

use std::collections::HashMap;
use std::sync::Mutex;

use super::binarylog::LogObject;
use super::rtcm3::StaticUtils;
use crate::dotnet::collections::Dictionary;
use crate::dotnet::{cast_i64, int_parse, long_parse, DateTime, DateTimeKind, Exception};

/// `public struct Label`
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Label {
    pub Id: i32,
    pub Format: String,
    pub FieldNames: Vec<String>,
    pub Length: i32,
    pub Name: String,
}

/// What `DFItem.instance` reads from `parent._dfLogBuffer`: `InstanceType` keys and the
/// `FMTU` unit strings.
pub trait InstanceSource {
    fn instance_type_contains(&self, typeno: i32) -> bool;
    fn fmtu_units(&self, typeno: i32) -> Option<String>;
}

/// `DFItem.instanceCache`: static, keyed by the message type's hash code.
static INSTANCE_CACHE: Mutex<Option<HashMap<String, usize>>> = Mutex::new(None);

fn key_not_found() -> Exception {
    Exception::KeyNotFound("The given key was not present in the dictionary.".into())
}

/// `public struct DFItem`
#[derive(Debug, Clone, Default)]
pub struct DFItem {
    _items: Option<Vec<Option<String>>>,
    _timems: f64,
    pub lineno: i32,
    _raw: Vec<LogObject>,
    _instance: Option<String>,
}

impl DFItem {
    /// `msgtype`: `raw[0].ToString()`, or "" for an empty item.
    pub fn msgtype(&self) -> String {
        match self._raw.first() {
            Some(a) => a.ToString().unwrap_or_default(),
            None => String::new(),
        }
    }

    /// `time`: always a time relative to the gps start time; sets `parent.lasttime`.
    pub fn time(&mut self, parent: &mut DFLog) -> Result<DateTime, Exception> {
        let timems = self.timems(parent)?;
        let time = parent.gpsstarttime.add_ticks(cast_i64((timems - parent.msoffset as f64) * 10000.0))?;
        parent.lasttime = time;
        Ok(time)
    }

    /// `instance`: the value of the message's instance (`#`) column, or "".
    pub fn instance(&mut self, parent: &DFLog, buffer: Option<&dyn InstanceSource>) -> Result<String, Exception> {
        // double access save
        if let Some(i) = &self._instance {
            return Ok(i.clone());
        }
        let msgtype = self.msgtype();
        // field offset save
        if let Some(&col) = INSTANCE_CACHE.lock().unwrap().get_or_insert_with(HashMap::new).get(&msgtype) {
            let v = self._raw.get(col).ok_or_else(crate::dotnet::index_out_of_range)?;
            return v.ToString().ok_or_else(null_reference);
        }
        let typeno = parent.logformat.get(&msgtype).ok_or_else(key_not_found)?.Id;
        let buffer = buffer.ok_or_else(null_reference)?;
        if !buffer.instance_type_contains(typeno) {
            return Ok(String::new());
        }
        let unittypes = buffer.fmtu_units(typeno).ok_or_else(key_not_found)?;
        let colinst = unittypes.find('#').map(|i| unittypes[..i].chars().count() as i32).unwrap_or(-1) + 1;
        INSTANCE_CACHE.lock().unwrap().get_or_insert_with(HashMap::new).insert(msgtype, colinst as usize);
        let v = self._raw.get(colinst as usize).ok_or_else(crate::dotnet::index_out_of_range)?;
        let s = v.ToString().ok_or_else(null_reference)?;
        self._instance = Some(s.clone());
        Ok(s)
    }

    /// `items`: numbers in the invariant culture, `byte[]` as ASCII without NULs, anything
    /// else through `ToString()` (`null` stays null).
    pub fn items(&mut self) -> &Vec<Option<String>> {
        if self._items.is_none() {
            let items = self
                ._raw
                .iter()
                .map(|a| {
                    if a.IsNumber() {
                        Some(a.to_string_invariant())
                    } else if let LogObject::Bytes(b) = a {
                        Some(super::binarylog::ascii_get_string(b).trim_matches('\0').to_string())
                    } else {
                        a.ToString()
                    }
                })
                .collect();
            self._items = Some(items);
        }
        self._items.as_ref().unwrap()
    }

    /// `timems`: from TimeMS, TimeUS / 1000 or T, cached once non-zero.
    pub fn timems(&mut self, parent: &DFLog) -> Result<f64, Exception> {
        if self._timems != 0.0 {
            return Ok(self._timems);
        }
        let msgtype = self.msgtype();
        if parent.logformat.contains_key(&msgtype) {
            let parse = |raw: &[LogObject], index: i32| -> Result<i64, Exception> {
                let v = raw.get(index as usize).ok_or_else(crate::dotnet::index_out_of_range)?;
                long_parse(&v.ToString().ok_or_else(null_reference)?)
            };
            let index = parent.FindMessageOffset(Some(msgtype.as_str()), Some("TimeMS"))?;
            if index >= 0 {
                self._timems = parse(&self._raw, index)? as f64;
                return Ok(self._timems);
            }
            let index = parent.FindMessageOffset(Some(msgtype.as_str()), Some("TimeUS"))?;
            if index >= 0 {
                self._timems = parse(&self._raw, index)? as f64 / 1000.0;
                return Ok(self._timems);
            }
            let index = parent.FindMessageOffset(Some(msgtype.as_str()), Some("T"))?;
            if index >= 0 {
                self._timems = parse(&self._raw, index)? as f64;
                return Ok(self._timems);
            }
        }
        Ok(self._timems)
    }

    /// `raw`
    pub fn raw(&self) -> &[LogObject] {
        &self._raw
    }

    /// `raw = value`: also drops the cached `items`.
    pub fn set_raw(&mut self, value: Vec<LogObject>) {
        self._raw = value;
        self._items = None;
    }

    /// `new DFItem(_parent, _answer, lineno)`: a GPS message seen before any GPS time sets
    /// the parent's start time and its board-time offset.
    pub fn new(parent: &mut DFLog, answer: Option<Vec<LogObject>>, lineno: i32) -> Result<DFItem, Exception> {
        let mut item = DFItem { lineno, ..Default::default() };
        let Some(answer) = answer else {
            item.set_raw(Vec::new());
            return Ok(item);
        };
        item.set_raw(answer);
        if !item._raw.is_empty() {
            // check this is a gps message and we dont have the current gpsstarttime
            let msgtype = item.msgtype();
            if parent.gpsstarttime == DateTime::MIN_VALUE && msgtype.starts_with("GPS") && parent.logformat.contains_key(&msgtype) {
                let _indextimems = parent.FindMessageOffset(Some(msgtype.as_str()), Some("T"))?;
                let joined = item.items().iter().map(|s| s.clone().unwrap_or_default()).collect::<Vec<_>>().join(",");
                let time = parent.GetTimeGPS(&joined)?;
                if time != DateTime::MIN_VALUE {
                    parent.gpsstarttime = time;
                    parent.lasttime = parent.gpsstarttime;
                    let items = item.items().clone();
                    let first = items.first().cloned().flatten();
                    let indextimems = parent.FindMessageOffset(first.as_deref(), Some("T"))?;
                    if indextimems != -1 {
                        match items.get(indextimems as usize).map(|s| int_parse(s.as_deref().unwrap_or(""))) {
                            Some(Ok(v)) => parent.msoffset = v as i64,
                            _ => parent.gpsstarttime = DateTime::MIN_VALUE,
                        }
                    }
                    let indextimeus = parent.FindMessageOffset(first.as_deref(), Some("TimeUS"))?;
                    if indextimeus != -1 {
                        match items.get(indextimeus as usize).map(|s| long_parse(s.as_deref().unwrap_or(""))) {
                            Some(Ok(v)) => parent.msoffset = v / 1000,
                            _ => parent.gpsstarttime = DateTime::MIN_VALUE,
                        }
                    }
                }
            }
        }
        Ok(item)
    }

    /// `this[string item]`: the field's text, `None` when the field is unknown or past the end.
    pub fn get(&mut self, parent: &DFLog, item: &str) -> Result<Option<String>, Exception> {
        let index = parent.FindMessageOffset(Some(self.msgtype().as_str()), Some(item))?;
        let items = self.items();
        if index == -1 || index as usize >= items.len() {
            return Ok(None);
        }
        Ok(items[index as usize].clone())
    }

    /// `ToDictionary()`: `a.IsNumber()` tests the int counter, so every field takes the raw
    /// value cast to `IConvertible` (a `byte[]` field throws InvalidCastException).
    pub fn ToDictionary(&self, parent: &DFLog) -> Result<Dictionary<String, LogObject>, Exception> {
        let mut ans = Dictionary::new();
        let mut a = 1usize;
        let label = parent.logformat.get(&self.msgtype()).ok_or_else(key_not_found)?;
        for fieldName in &label.FieldNames {
            let v = self._raw.get(a).ok_or_else(crate::dotnet::index_out_of_range)?;
            if !v.is_convertible() && *v != LogObject::Null {
                return Err(Exception::InvalidCast("Unable to cast object of type 'System.Byte[]' to type 'System.IConvertible'.".into()));
            }
            ans.insert(fieldName.clone(), v.clone());
            a += 1;
        }
        Ok(ans)
    }

    /// `GetRaw(item)`
    pub fn GetRaw(&self, parent: &DFLog, item: &str) -> Result<Option<&LogObject>, Exception> {
        let index = parent.FindMessageOffset(Some(self.msgtype().as_str()), Some(item))?;
        if index == -1 {
            return Ok(None);
        }
        self._raw.get(index as usize).map(Some).ok_or_else(crate::dotnet::index_out_of_range)
    }
}

fn null_reference() -> Exception {
    Exception::NullReference("Object reference not set to an instance of an object.".into())
}

crate::cs_enum! {
    /// `public enum Log_Event : byte`
    /// https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Logger/AP_Logger.h
    pub Log_Event: u8 {
        AP_STATE = 7,
        INIT_SIMPLE_BEARING = 9,
        ARMED = 10,
        DISARMED = 11,
        AUTO_ARMED = 15,
        LAND_COMPLETE_MAYBE = 17,
        LAND_COMPLETE = 18,
        NOT_LANDED = 28,
        LOST_GPS = 19,
        FLIP_START = 21,
        FLIP_END = 22,
        SET_HOME = 25,
        SET_SIMPLE_ON = 26,
        SET_SIMPLE_OFF = 27,
        SET_SUPERSIMPLE_ON = 29,
        AUTOTUNE_INITIALISED = 30,
        AUTOTUNE_OFF = 31,
        AUTOTUNE_RESTART = 32,
        AUTOTUNE_SUCCESS = 33,
        AUTOTUNE_FAILED = 34,
        AUTOTUNE_REACHED_LIMIT = 35,
        AUTOTUNE_PILOT_TESTING = 36,
        AUTOTUNE_SAVEDGAINS = 37,
        SAVE_TRIM = 38,
        SAVEWP_ADD_WP = 39,
        FENCE_ENABLE = 41,
        FENCE_DISABLE = 42,
        ACRO_TRAINER_OFF = 43,
        ACRO_TRAINER_LEVELING = 44,
        ACRO_TRAINER_LIMITED = 45,
        GRIPPER_GRAB = 46,
        GRIPPER_RELEASE = 47,
        PARACHUTE_DISABLED = 49,
        PARACHUTE_ENABLED = 50,
        PARACHUTE_RELEASED = 51,
        LANDING_GEAR_DEPLOYED = 52,
        LANDING_GEAR_RETRACTED = 53,
        MOTORS_EMERGENCY_STOPPED = 54,
        MOTORS_EMERGENCY_STOP_CLEARED = 55,
        MOTORS_INTERLOCK_DISABLED = 56,
        MOTORS_INTERLOCK_ENABLED = 57,
        ROTOR_RUNUP_COMPLETE = 58,
        ROTOR_SPEED_BELOW_CRITICAL = 59,
        EKF_ALT_RESET = 60,
        LAND_CANCELLED_BY_PILOT = 61,
        EKF_YAW_RESET = 62,
        AVOIDANCE_ADSB_ENABLE = 63,
        AVOIDANCE_ADSB_DISABLE = 64,
        AVOIDANCE_PROXIMITY_ENABLE = 65,
        AVOIDANCE_PROXIMITY_DISABLE = 66,
        GPS_PRIMARY_CHANGED = 67,
        WINCH_RELAXED = 68,
        WINCH_LENGTH_CONTROL = 69,
        WINCH_RATE_CONTROL = 70,
        ZIGZAG_STORE_A = 71,
        ZIGZAG_STORE_B = 72,
        LAND_REPO_ACTIVE = 73,
        STANDBY_ENABLE = 74,
        STANDBY_DISABLE = 75,
        FENCE_FLOOR_ENABLE = 80,
        FENCE_FLOOR_DISABLE = 81,
        EK3_SOURCES_SET_TO_PRIMARY = 85,
        EK3_SOURCES_SET_TO_SECONDARY = 86,
        EK3_SOURCES_SET_TO_TERTIARY = 87,
        AIRSPEED_PRIMARY_CHANGED = 90,
        SURFACED = 163,
        NOT_SURFACED = 164,
        BOTTOMED = 165,
        NOT_BOTTOMED = 166,
    }
}

crate::cs_enum! {
    /// `public enum LogErrorSubsystem : byte`
    pub LogErrorSubsystem: u8 {
        MAIN = 1,
        RADIO = 2,
        COMPASS = 3,
        OPTFLOW = 4,
        FAILSAFE_RADIO = 5,
        FAILSAFE_BATT = 6,
        FAILSAFE_GPS = 7,
        FAILSAFE_GCS = 8,
        FAILSAFE_FENCE = 9,
        FLIGHT_MODE = 10,
        GPS = 11,
        CRASH_CHECK = 12,
        FLIP = 13,
        AUTOTUNE = 14,
        PARACHUTES = 15,
        EKFCHECK = 16,
        FAILSAFE_EKFINAV = 17,
        BARO = 18,
        CPU = 19,
        FAILSAFE_ADSB = 20,
        TERRAIN = 21,
        NAVIGATION = 22,
        FAILSAFE_TERRAIN = 23,
        EKF_PRIMARY = 24,
        THRUST_LOSS_CHECK = 25,
        FAILSAFE_SENSORS = 26,
        FAILSAFE_LEAK = 27,
        PILOT_INPUT = 28,
        FAILSAFE_VIBE = 29,
        INTERNAL_ERROR = 30,
        FAILSAFE_DEADRECKON = 31,
    }
}

crate::cs_enum! {
    /// `public enum LogErrorCode : byte` (bizarrely this enumeration has lots of duplicate
    /// values, offering very little in the way of typesafety)
    pub LogErrorCode: u8 {
        ERROR_RESOLVED = 0,
        FAILED_TO_INITIALISE = 1,
        UNHEALTHY = 4,
        RADIO_LATE_FRAME = 2,
        FAILSAFE_RESOLVED = 0,
        FAILSAFE_OCCURRED = 1,
        MAIN_INS_DELAY = 1,
        CRASH_CHECK_CRASH = 1,
        CRASH_CHECK_LOSS_OF_CONTROL = 2,
        FLIP_ABANDONED = 2,
        MISSING_TERRAIN_DATA = 2,
        FAILED_TO_SET_DESTINATION = 2,
        RESTARTED_RTL = 3,
        FAILED_CIRCLE_INIT = 4,
        DEST_OUTSIDE_FENCE = 5,
        RTL_MISSING_RNGFND = 6,
        INTERNAL_ERRORS_DETECTED = 1,
        PARACHUTE_TOO_LOW = 2,
        PARACHUTE_LANDED = 3,
        EKFCHECK_BAD_VARIANCE = 2,
        EKFCHECK_VARIANCE_CLEARED = 0,
        BARO_GLITCH = 2,
        BAD_DEPTH = 3,
        GPS_GLITCH = 2,
    }
}

/// `public class DFLog`
#[derive(Debug, Clone)]
pub struct DFLog {
    pub logformat: Dictionary<String, Label>,
    /// current gps time
    gpstime: DateTime,
    /// last time of message
    pub(crate) lasttime: DateTime,
    /// first valid gpstime
    pub gpsstarttime: DateTime,
    pub(crate) msoffset: i64,
    /// `msgoffsetcache`: filled by lookups, so it sits behind a `RefCell`
    msgoffsetcache: std::cell::RefCell<HashMap<(String, String), i32>>,
}

impl Default for DFLog {
    fn default() -> Self {
        Self::new()
    }
}

impl DFLog {
    /// `new DFLog(dfLogBuffer)`: the buffer reference is passed to `DFItem::instance`.
    pub fn new() -> Self {
        DFLog {
            logformat: Dictionary::new(),
            gpstime: DateTime::MIN_VALUE,
            lasttime: DateTime::MIN_VALUE,
            gpsstarttime: DateTime::MIN_VALUE,
            msoffset: 0,
            msgoffsetcache: Default::default(),
        }
    }

    /// `Clear()`
    pub fn Clear(&mut self) {
        self.logformat.clear();
        // current gps time
        self.gpstime = DateTime::MIN_VALUE;
        // last time of message
        self.lasttime = DateTime::MIN_VALUE;
        // first valid gpstime
        self.gpsstarttime = DateTime::MIN_VALUE;
    }

    /// `GetFirstGpsTime(fn)`: the time of the first GPS line with a 3D lock.
    pub fn GetFirstGpsTime(&mut self, fn_: &str) -> Result<DateTime, Exception> {
        for line in crate::dotnet::io::read_lines(fn_)? {
            if line.starts_with("FMT") {
                self.FMTLine(&line);
            } else if line.starts_with("GPS") {
                let answer = self.GetTimeGPS(&line)?;
                if answer != DateTime::MIN_VALUE {
                    return Ok(answer);
                }
            }
        }
        Ok(DateTime::MIN_VALUE)
    }

    /// `ReadLog(string fn)`
    pub fn ReadLog_file(&mut self, fn_: &str) -> Result<Vec<DFItem>, Exception> {
        let bytes = std::fs::read(fn_).map_err(|e| crate::dotnet::io::file_error(fn_, &e))?;
        Ok(self.ReadLog(&bytes))
    }

    /// `ReadLog(Stream fn)`: every line as an item; a line that throws is left out.
    pub fn ReadLog(&mut self, fn_: &[u8]) -> Vec<DFItem> {
        self.Clear();
        let mut answer = Vec::new();
        // current gps time
        self.gpstime = DateTime::MIN_VALUE;
        // last time of message
        self.lasttime = DateTime::MIN_VALUE;
        // first valid gpstime
        self.gpsstarttime = DateTime::MIN_VALUE;
        let mut lineno = 0;
        self.msoffset = 0;
        log::info!("loading log");
        for line in crate::dotnet::io::split_lines(&crate::dotnet::io::decode_detect(fn_)) {
            lineno += 1;
            if let Ok(newitem) = self.GetDFItemFromLine(&line, lineno) {
                answer.push(newitem);
            }
        }
        log::info!("loaded log");
        answer
    }

    /// `GetDFItemFromLine(line, lineno)`: FMT lines are learnt; ERR gains its subsystem name
    /// and EV its event name.
    pub fn GetDFItemFromLine(&mut self, line: &str, lineno: i32) -> Result<DFItem, Exception> {
        let mut items: Vec<LogObject> = line.trim().split([',', ':']).filter(|s| !s.is_empty()).map(|s| LogObject::String(s.to_string())).collect();

        if line.starts_with("FMT") {
            self.FMTLine(line);
        } else if line.starts_with("GPS") {
        } else if line.starts_with("ERR") {
            items.push(LogObject::Null);
            items.push(LogObject::Null);
            let r = (|| -> Result<(), Exception> {
                let index = self.FindMessageOffset(Some("ERR"), Some("Subsys"))?;
                if index == -1 {
                    return Err(Exception::ArgumentNull("Value cannot be null.".into()));
                }
                let index2 = self.FindMessageOffset(Some("ERR"), Some("ECode"))?;
                if index2 == -1 {
                    return Err(Exception::ArgumentNull("Value cannot be null.".into()));
                }
                let v = int_parse(&items.get(index as usize).ok_or_else(crate::dotnet::index_out_of_range)?.ToString().ok_or_else(null_reference)?)?;
                let n = items.len();
                items[n - 2] = LogObject::String(LogErrorSubsystem(v as u8).to_string());
                Ok(())
            })();
            let _ = r;
        } else if line.starts_with("EV") {
            items.push(LogObject::Null);
            let r = (|| -> Result<(), Exception> {
                let index = self.FindMessageOffset(Some("EV"), Some("Id"))?;
                if index == -1 {
                    return Err(Exception::ArgumentNull("Value cannot be null.".into()));
                }
                let v = int_parse(&items.get(index as usize).ok_or_else(crate::dotnet::index_out_of_range)?.ToString().ok_or_else(null_reference)?)?;
                let n = items.len();
                items[n - 1] = LogObject::String(Log_Event(v as u8).to_string());
                Ok(())
            })();
            let _ = r;
        } else if line.starts_with("MAG") {
        }

        DFItem::new(self, Some(items), lineno)
    }

    /// `FMTLine(strLine)`: learn a format line; anything malformed is ignored.
    pub fn FMTLine(&mut self, strLine: &str) {
        if !strLine.starts_with("FMT") {
            return;
        }
        let strLine = strLine.replace(", ", ",").replace(": ", ":");
        let items: Vec<&str> = strLine.trim().split([',', ':']).collect();
        let r = (|| -> Result<Label, Exception> {
            if items.len() < 5 {
                // new string[items.Length - 5]
                return Err(Exception::Overflow("Arithmetic operation resulted in an overflow.".into()));
            }
            let names: Vec<String> = items[5..].iter().map(|s| s.to_string()).collect();
            Ok(Label {
                Name: items[3].to_string(),
                Id: int_parse(items[1])?,
                Format: items[4].to_string(),
                Length: int_parse(items[2])?,
                FieldNames: names,
            })
        })();
        let Ok(lbl) = r else { return };
        self.logformat.insert(lbl.Name.clone(), lbl);
        if !self.logformat.contains_key(&"FMT".to_string()) {
            // mod for custom logformat that hides the FMT key
            self.logformat.insert(
                "FMT".into(),
                Label {
                    Name: "FMT".into(),
                    Id: 0x80,
                    Format: "BBnNZ".into(),
                    Length: 59,
                    FieldNames: ["Type", "Length", "Name", "Format", "Columns"].iter().map(|s| s.to_string()).collect(),
                },
            );
        }
    }

    /// `GetTimeGPS(gpsline)`: the GPS time of a line with a 3D lock or better, else MinValue.
    pub fn GetTimeGPS(&self, gpsline: &str) -> Result<DateTime, Exception> {
        if gpsline.starts_with("GPS") && !self.logformat.is_empty() {
            let strLine = gpsline.replace(", ", ",").replace(": ", ":");
            let items: Vec<&str> = strLine.split([',', ':']).collect();
            // check its a valid lock
            let indexstatus = self.FindMessageOffset(Some("GPS"), Some("Status"))?;
            if indexstatus != -1 {
                let s = items.get(indexstatus as usize).ok_or_else(crate::dotnet::index_out_of_range)?.trim();
                // 3d lock or better
                if s == "0" || s == "1" || s == "2" {
                    return Ok(DateTime::MIN_VALUE);
                }
            }
            // get time since start of week
            let mut indextimems = self.FindMessageOffset(Some("GPS"), Some("TimeMS"))?;
            if indextimems == -1 {
                indextimems = self.FindMessageOffset(Some("GPS"), Some("GMS"))?;
            }
            // get week number
            let mut indexweek = self.FindMessageOffset(Some("GPS"), Some("Week"))?;
            if indexweek == -1 {
                indexweek = self.FindMessageOffset(Some("GPS"), Some("GWk"))?;
            }
            if indextimems == -1 || indexweek == -1 {
                return Ok(DateTime::MIN_VALUE);
            }
            let r = (|| -> Result<DateTime, Exception> {
                let week = int_parse(items.get(indexweek as usize).ok_or_else(crate::dotnet::index_out_of_range)?)?;
                let sec = long_parse(items.get(indextimems as usize).ok_or_else(crate::dotnet::index_out_of_range)?)? as f64 / 1000.0;
                if week > 5000 || week < 0 || sec > (60 * 60 * 24 * 7) as f64 || sec < 0.0 {
                    return Ok(DateTime::MIN_VALUE);
                }
                Self::gpsTimeToTime(week, sec)
            })();
            return Ok(r.unwrap_or(DateTime::MIN_VALUE));
        }
        Ok(DateTime::MIN_VALUE)
    }

    /// `gpsTimeToTime(week, sec)`: local time, with this month's GPS leap seconds.
    pub fn gpsTimeToTime(week: i32, sec: f64) -> Result<DateTime, Exception> {
        let (y, mo, ..) = DateTime::now().parts();
        let leap = StaticUtils::LeapSecondsGPS(y, mo as i32);
        let basetime = DateTime::new(1980, 1, 6, 0, 0, 0, DateTimeKind::Utc)?;
        let basetime = basetime.add_days(week.wrapping_mul(7) as f64)?;
        let basetime = basetime.add_seconds(sec - leap as f64)?;
        Ok(basetime.to_local_time())
    }

    /// `FindMessageOffset(linetype, find)`: the field's index in the item (the type is at 0),
    /// or -1. A type known only in upper case throws KeyNotFoundException, as the C# looks
    /// it up as given.
    pub fn FindMessageOffset(&self, linetype: Option<&str>, find: Option<&str>) -> Result<i32, Exception> {
        let (Some(linetype), Some(find)) = (linetype, find) else { return Ok(-1) };
        let key = (linetype.to_string(), find.to_string());
        if let Some(&v) = self.msgoffsetcache.borrow().get(&key) {
            return Ok(v);
        }
        if self.logformat.contains_key(&crate::dotnet::to_upper(linetype)) {
            let lbl = self.logformat.get(&linetype.to_string()).ok_or_else(key_not_found)?;
            let Some(ans) = lbl.FieldNames.iter().position(|f| f == find) else { return Ok(-1) };
            // + type
            let v = ans as i32 + 1;
            self.msgoffsetcache.borrow_mut().insert(key, v);
            return Ok(v);
        }
        Ok(-1)
    }

    /// `GetLineNoFromTime(logdata, p1)`: the first GPS/GPS2/POS line at or after `p1`;
    /// `long.MaxValue` if every one is before it, 0 if there are none.
    pub fn GetLineNoFromTime(logdata: &mut super::dflogbuffer::DFLogBuffer, p1: DateTime) -> Result<i64, Exception> {
        let mut last = DateTime::MAX_VALUE;
        let mut it = logdata.GetEnumeratorType(&["GPS", "GPS2", "POS"]);
        while let Some(dfItem) = it.next(logdata) {
            let mut dfItem = dfItem?;
            // always forwards
            let t = dfItem.time(&mut logdata.dflog)?;
            if t >= p1 {
                return Ok(dfItem.lineno as i64);
            }
            last = t;
        }
        if last != DateTime::MAX_VALUE {
            return Ok(i64::MAX);
        }
        Ok(0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn text_lines_fmt_err_ev() {
        let mut d = DFLog::new();
        d.FMTLine("FMT, 128, 89, FMT, BBnNZ, Type,Length,Name,Format,Columns");
        d.FMTLine("FMT, 13, 15, ERR, QBB, TimeUS,Subsys,ECode");
        d.FMTLine("FMT, 14, 12, EV, QB, TimeUS,Id");
        assert_eq!(d.FindMessageOffset(Some("ERR"), Some("Subsys")).unwrap(), 2);
        let mut e = d.GetDFItemFromLine("ERR, 1000, 3, 1", 1).unwrap();
        assert_eq!(e.items().last().cloned().flatten(), None, "ECode name is not filled");
        let it = e.items().clone();
        assert_eq!(it[it.len() - 2].as_deref(), Some("COMPASS"));
        let mut ev = d.GetDFItemFromLine("EV, 2000, 10", 2).unwrap();
        assert_eq!(ev.items().last().cloned().flatten().as_deref(), Some("ARMED"));
        assert_eq!(ev.timems(&d).unwrap(), 2.0);
        assert_eq!(ev.get(&d, "Id").unwrap().as_deref(), Some(" 10"));
        // an unknown subsystem byte keeps its number
        let mut e = d.GetDFItemFromLine("ERR, 1000, 300, 1", 3).unwrap();
        let it = e.items().clone();
        assert_eq!(it[it.len() - 2].as_deref(), Some("44"));
    }

    #[test]
    fn gps_time_needs_3d_lock() {
        let mut d = DFLog::new();
        d.FMTLine("FMT, 130, 45, GPS, BIHBcLLeeEefI, Status,TimeMS,Week,NSats,HDop,Lat,Lng,RelAlt,Alt,Spd,GCrs,VZ,T");
        assert_eq!(d.GetTimeGPS("GPS, 2, 130040903, 1769, 10").unwrap(), DateTime::MIN_VALUE);
        assert_ne!(d.GetTimeGPS("GPS, 3, 130040903, 1769, 10").unwrap(), DateTime::MIN_VALUE);
        // lower-case type known only upper-case: KeyNotFound
        assert!(d.FindMessageOffset(Some("gps"), Some("Status")).is_err());
    }
}

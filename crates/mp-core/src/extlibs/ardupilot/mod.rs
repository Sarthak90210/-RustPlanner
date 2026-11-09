//! `ExtLibs/ArduPilot` (`MissionPlanner.ArduPilot.csproj`): the vehicle and link layer.

pub mod ap_product;
pub mod deviceinfo;
pub mod fence;
pub mod fencecircle;
pub mod fencepolygon;
pub mod fencereturn;
pub mod firmwares;
pub mod frame;
pub mod hal_ins_i2c;
pub mod hal_ins_spi;
pub mod linux;
pub mod mavlink;
pub mod missionpck;
pub mod motor_frame_class;
pub mod motor_frame_type;
pub mod parampck;
pub mod polygontools;
pub mod px4_custom_main_mode;
pub mod px4_custom_sub_mode_auto;
pub mod retrytimeout;
pub mod serial;

pub use self::deviceinfo::DeviceInfo;
pub use self::firmwares::Firmwares;
pub use self::frame::Frame;
pub use self::px4_custom_main_mode::PX4_CUSTOM_MAIN_MODE;
pub use self::px4_custom_sub_mode_auto::PX4_CUSTOM_SUB_MODE_AUTO;
pub use self::linux::Linux;
pub use self::retrytimeout::RetryTimeout;
pub use self::serial::Serial;

//! `ExtLibs/Mavlink` (`MAVLink.csproj`).
//!
//! In C# every file here adds to `public partial class MAVLink`, apart from the static
//! `MavlinkUtil`. Each file is a submodule and the module re-exports them all, so
//! `MAVLink.X` in C# is `extlibs::mavlink::X` in Rust.

#[allow(clippy::module_inception)]
pub mod mavlink;
pub mod component_information;
pub mod mavlinkcrc;
pub mod mavlinkheader;
pub mod mavlinkmessage;
pub mod mavlinkparam;
pub mod mavlinkparamlist;
pub mod mavlinkparse;
pub mod mavlinkutil;

pub use self::mavlink::*;
pub use self::mavlinkcrc::MavlinkCRC;
pub use self::mavlinkheader::*;
pub use self::mavlinkmessage::MAVLinkMessage;
pub use self::mavlinkparam::MAVLinkParam;
pub use self::mavlinkparamlist::MAVLinkParamList;
pub use self::mavlinkparse::{get_unit, MavlinkParse};
pub use self::mavlinkutil::{get_message_info, MarshalStruct, MavlinkUtil};

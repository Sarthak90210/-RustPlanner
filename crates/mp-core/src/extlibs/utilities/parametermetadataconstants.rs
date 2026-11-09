//! Port of `ExtLibs/Utilities/ParameterMetaDataConstants.cs`

/// `public sealed class ParameterMetaDataConstants`
pub struct ParameterMetaDataConstants;

#[allow(non_upper_case_globals)]
impl ParameterMetaDataConstants {
    // Markers

    pub const ParamDelimeter: &'static str = "@";
    pub const PathDelimeter: &'static str = ",";
    pub const Param: &'static str = "Param";
    pub const Group: &'static str = "Group";
    pub const Path: &'static str = "Path";

    pub const NestedGroup: &'static str = r"AP_NESTEDGROUPINFO\((.+),.+\)";

    // Meta Keys

    pub const DisplayName: &'static str = "DisplayName";
    pub const Description: &'static str = "Description";
    pub const Units: &'static str = "Units";
    pub const Range: &'static str = "Range";
    pub const Values: &'static str = "Values";
    pub const Increment: &'static str = "Increment";
    pub const User: &'static str = "User";
    pub const RebootRequired: &'static str = "RebootRequired";
    pub const Bitmask: &'static str = "Bitmask";
    pub const ReadOnly: &'static str = "ReadOnly";

    // Meta Values

    pub const Advanced: &'static str = "Advanced";
    pub const Standard: &'static str = "Standard";
}

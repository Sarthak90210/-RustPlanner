//! Port of `ExtLibs/GMap.NET.Core/GMap.NET/AccessMode.cs`

crate::cs_enum! {
    /// `public enum AccessMode`: tile access mode
    pub AccessMode: i32 {
        /// access only server
        ServerOnly = 0,
        /// access first server and caches localy
        ServerAndCache = 1,
        /// access only cache
        CacheOnly = 2,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn names_and_values() {
        assert_eq!(AccessMode::default(), AccessMode::ServerOnly);
        assert_eq!(AccessMode::CacheOnly.to_string(), "CacheOnly");
        assert_eq!(AccessMode::parse("ServerAndCache"), Some(AccessMode(1)));
        assert_eq!(AccessMode(7).to_string(), "7");
    }
}

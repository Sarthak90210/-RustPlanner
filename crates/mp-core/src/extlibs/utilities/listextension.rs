//! Port of `ExtLibs/Utilities/ListExtension.cs`

use crate::dotnet::collections::{introspective_sort_named, list_capacity_for_count};

/// The `Comparison<string>` lambda's `ToString()`.
const COMPARER_NAME: &str = "System.Comparison`1[System.String]";
use crate::dotnet::culture_compare;

/// `public static class ListExtension`
pub struct ListExtension;

impl ListExtension {
    /// `SortENABLE(this List<string> list)`: names ending in "ENABLE" first, each group in
    /// culture order; `List.Sort` (introsort, unstable) with errors swallowed. `capacity` is
    /// the list's backing array length (`list_capacity_for_count` for a list built by `Add`).
    #[allow(non_snake_case)]
    pub fn SortENABLE(list: &mut [Option<String>], capacity: usize) {
        let _ = introspective_sort_named(list, capacity, COMPARER_NAME, |a: &Option<String>, b: &Option<String>| {
            Ok(sorter(a.as_deref(), b.as_deref()))
        });
    }

    /// `SortENABLE` on a list built by `Add` (capacity from its count).
    pub fn sort_enable(list: &mut Vec<String>) {
        let cap = list_capacity_for_count(list.len());
        let _ = introspective_sort_named(list, cap, COMPARER_NAME, |a: &String, b: &String| {
            Ok(sorter(Some(a.as_str()), Some(b.as_str())))
        });
    }
}

/// `string.CompareTo(string)`: current-culture comparison as -1/0/1.
fn compare_to(a: &str, b: &str) -> i32 {
    culture_compare(a, b) as i32
}

/// `sorter(a, b)`
fn sorter(a: Option<&str>, b: Option<&str>) -> i32 {
    let (Some(a), Some(b)) = (a, b) else {
        return 0;
    };
    if a.ends_with("ENABLE") && b.ends_with("ENABLE") {
        return compare_to(a, b);
    }
    if a.ends_with("ENABLE") {
        return -1;
    }
    if b.ends_with("ENABLE") {
        return 1;
    }
    compare_to(a, b)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn enable_first() {
        let mut l: Vec<String> = ["RTL_ALT", "FENCE_ENABLE", "ARMING_CHECK", "AHRS_ENABLE"].iter().map(|s| s.to_string()).collect();
        ListExtension::sort_enable(&mut l);
        assert_eq!(l, vec!["AHRS_ENABLE", "FENCE_ENABLE", "ARMING_CHECK", "RTL_ALT"]);
    }
}

//! Port of `ExtLibs/Utilities/CollectionExtensions.cs`

/// `public static class CollectionExtensions`: `ForEach` over any enumerable.
pub struct CollectionExtensions;

impl CollectionExtensions {
    /// `ForEach(this IEnumerable enumerable, Action<object> action)` and the generic
    /// overloads: the action on each element, in enumeration order.
    pub fn for_each<I: IntoIterator, F: FnMut(I::Item)>(enumerable: I, action: F) {
        enumerable.into_iter().for_each(action);
    }
}

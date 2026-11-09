//! `System.Collections.Generic.Dictionary<TKey, TValue>` with the .NET enumeration order:
//! entries enumerate in slot order; a removed slot goes on a free list and the next insertion
//! reuses the most recently freed slot, so it enumerates where the removed entry was.

use std::collections::HashMap;
use std::hash::Hash;

#[derive(Debug, Clone)]
pub struct Dictionary<K, V> {
    entries: Vec<Option<(K, V)>>,
    free: Vec<usize>,
    index: HashMap<K, usize>,
}

impl<K: Eq + Hash + Clone, V> Default for Dictionary<K, V> {
    fn default() -> Self {
        Self::new()
    }
}

impl<K: Eq + Hash + Clone, V> Dictionary<K, V> {
    pub fn new() -> Self {
        Dictionary { entries: vec![], free: vec![], index: HashMap::new() }
    }

    pub fn len(&self) -> usize {
        self.index.len()
    }

    pub fn is_empty(&self) -> bool {
        self.index.is_empty()
    }

    /// `TryGetValue`
    pub fn get(&self, key: &K) -> Option<&V> {
        self.index.get(key).and_then(|&i| self.entries[i].as_ref()).map(|(_, v)| v)
    }

    pub fn get_mut(&mut self, key: &K) -> Option<&mut V> {
        let i = *self.index.get(key)?;
        self.entries[i].as_mut().map(|(_, v)| v)
    }

    /// `ContainsKey`
    pub fn contains_key(&self, key: &K) -> bool {
        self.index.contains_key(key)
    }

    /// `this[key] = value`: replaces in place, or adds in the next free slot.
    pub fn insert(&mut self, key: K, value: V) {
        if let Some(&i) = self.index.get(&key) {
            self.entries[i] = Some((key, value));
            return;
        }
        let slot = match self.free.pop() {
            Some(s) => {
                self.entries[s] = Some((key.clone(), value));
                s
            }
            None => {
                self.entries.push(Some((key.clone(), value)));
                self.entries.len() - 1
            }
        };
        self.index.insert(key, slot);
    }

    /// `Remove(key)`
    pub fn remove(&mut self, key: &K) -> bool {
        match self.index.remove(key) {
            Some(i) => {
                self.entries[i] = None;
                self.free.push(i);
                true
            }
            None => false,
        }
    }

    /// `Clear()`
    pub fn clear(&mut self) {
        self.entries.clear();
        self.free.clear();
        self.index.clear();
    }

    /// Enumeration order.
    pub fn iter(&self) -> impl Iterator<Item = (&K, &V)> {
        self.entries.iter().filter_map(|e| e.as_ref().map(|(k, v)| (k, v)))
    }

    /// `Keys`
    pub fn keys(&self) -> impl Iterator<Item = &K> {
        self.iter().map(|(k, _)| k)
    }

    /// `Values`
    pub fn values(&self) -> impl Iterator<Item = &V> {
        self.iter().map(|(_, v)| v)
    }
}

/// `List<T>` capacity after `count` calls to `Add` on a new list: 0, then 4, doubling.
pub fn list_capacity_for_count(count: usize) -> usize {
    if count == 0 {
        return 0;
    }
    let mut cap = 4;
    while cap < count {
        cap *= 2;
    }
    cap
}

/// `List<T>.Sort(Comparison<T>)` / `Array.Sort(T[], Comparison<T>)` on .NET Framework 4.5+:
/// `ArraySortHelper<T>.IntrospectiveSort`. The result order for elements that compare equal
/// is the one .NET produces (the sort is not stable). `backing_len` is the length of the
/// array being sorted (a `List`'s capacity, see `list_capacity_for_count`), which sets the
/// depth limit before heapsort takes over.
///
/// The sort runs over a buffer of `backing_len` elements, the List's backing array: slots past
/// the count hold `default(T)`. An inconsistent comparer can make the partition scan read
/// those slots (and swap them in), and past the array end .NET's IndexOutOfRangeException
/// becomes the "inconsistent results" ArgumentException. Any other comparer exception is
/// reported as InvalidOperationException. Either way the elements stay partly sorted.
pub fn introspective_sort<T, F>(keys: &mut [T], backing_len: usize, comparer: F) -> Result<(), super::Exception>
where
    T: Default + Clone,
    F: FnMut(&T, &T) -> Result<i32, super::Exception>,
{
    introspective_sort_named(keys, backing_len, "System.Comparison`1[System.Object]", comparer)
}

/// `introspective_sort` with the comparer's `ToString()` (e.g. "System.Comparison`1[System.String]")
/// for the bad-comparer message.
pub fn introspective_sort_named<T, F>(keys: &mut [T], backing_len: usize, comparer_name: &str, mut comparer: F) -> Result<(), super::Exception>
where
    T: Default + Clone,
    F: FnMut(&T, &T) -> Result<i32, super::Exception>,
{
    if keys.len() < 2 {
        return Ok(());
    }
    let len = keys.len();
    let mut buf: Vec<T> = keys.to_vec();
    buf.resize(backing_len.max(len), T::default());
    let depth = 2 * floor_log2(buf.len());
    let hi = len as isize - 1;
    let r = intro_sort(&mut buf, 0, hi, depth, &mut comparer);
    keys.clone_from_slice(&buf[..len]);
    r.map_err(|e| match e {
        Exception::Argument(m) if m == BOGUS_COMPARER => bogus_comparer(comparer_name),
        Exception::IndexOutOfRange(_) => bogus_comparer(comparer_name),
        // the comparer's own exception becomes the InnerException, which this enum cannot carry
        _ => Exception::InvalidOperation("Failed to compare two elements in the array.".into()),
    })
}

use super::Exception;

/// Internal marker for an index that ran off the backing array.
const BOGUS_COMPARER: &str = "\u{0}bogus comparer";

/// `Arg_BogusIComparer`
fn bogus_comparer(name: &str) -> Exception {
    Exception::Argument(format!(
        "Unable to sort because the IComparer.Compare() method returns inconsistent results. Either a value does not compare equal to itself, or one value repeatedly compared to another value yields different results. IComparer: '{name}'."
    ))
}

/// `IntrospectiveSortUtilities.FloorLog2`
fn floor_log2(mut n: usize) -> i32 {
    let mut result = 0;
    while n >= 1 {
        result += 1;
        n /= 2;
    }
    result
}

const INTROSORT_SIZE_THRESHOLD: isize = 16;

fn swap_if_greater<T, C: FnMut(&T, &T) -> Result<i32, super::Exception>>(keys: &mut [T], comparer: &mut C, a: isize, b: isize) -> Result<(), super::Exception> {
    if a != b && comparer(&keys[a as usize], &keys[b as usize])? > 0 {
        keys.swap(a as usize, b as usize);
    }
    Ok(())
}

fn intro_sort<T, C: FnMut(&T, &T) -> Result<i32, super::Exception>>(keys: &mut [T], lo: isize, mut hi: isize, mut depth_limit: i32, comparer: &mut C) -> Result<(), super::Exception> {
    while hi > lo {
        let partition_size = hi - lo + 1;
        if partition_size <= INTROSORT_SIZE_THRESHOLD {
            if partition_size == 1 {
                return Ok(());
            }
            if partition_size == 2 {
                return swap_if_greater(keys, comparer, lo, hi);
            }
            if partition_size == 3 {
                swap_if_greater(keys, comparer, lo, hi - 1)?;
                swap_if_greater(keys, comparer, lo, hi)?;
                return swap_if_greater(keys, comparer, hi - 1, hi);
            }
            return insertion_sort(keys, lo, hi, comparer);
        }
        if depth_limit == 0 {
            return heapsort(keys, lo, hi, comparer);
        }
        depth_limit -= 1;
        let p = pick_pivot_and_partition(keys, lo, hi, comparer)?;
        intro_sort(keys, p + 1, hi, depth_limit, comparer)?;
        hi = p - 1;
    }
    Ok(())
}

fn pick_pivot_and_partition<T, C: FnMut(&T, &T) -> Result<i32, super::Exception>>(keys: &mut [T], lo: isize, hi: isize, comparer: &mut C) -> Result<isize, super::Exception> {
    let middle = lo + ((hi - lo) >> 1);
    swap_if_greater(keys, comparer, lo, middle)?;
    swap_if_greater(keys, comparer, lo, hi)?;
    swap_if_greater(keys, comparer, middle, hi)?;
    // the pivot stays at hi - 1 during the partition, so it is compared in place
    if middle != hi - 1 {
        keys.swap(middle as usize, (hi - 1) as usize);
    }
    let pivot = (hi - 1) as usize;
    let mut left = lo;
    let mut right = hi - 1;
    let off_end = || Exception::Argument(BOGUS_COMPARER.into());
    while left < right {
        loop {
            left += 1;
            if left as usize >= keys.len() {
                return Err(off_end());
            }
            if comparer(&keys[left as usize], &keys[pivot])? >= 0 {
                break;
            }
        }
        loop {
            right -= 1;
            if right < 0 {
                return Err(off_end());
            }
            if comparer(&keys[pivot], &keys[right as usize])? >= 0 {
                break;
            }
        }
        if left >= right {
            break;
        }
        keys.swap(left as usize, right as usize);
    }
    if left != hi - 1 {
        keys.swap(left as usize, (hi - 1) as usize);
    }
    Ok(left)
}

fn heapsort<T, C: FnMut(&T, &T) -> Result<i32, super::Exception>>(keys: &mut [T], lo: isize, hi: isize, comparer: &mut C) -> Result<(), super::Exception> {
    let n = hi - lo + 1;
    let mut i = n / 2;
    while i >= 1 {
        down_heap(keys, i, n, lo, comparer)?;
        i -= 1;
    }
    let mut i = n;
    while i > 1 {
        if lo != lo + i - 1 {
            keys.swap(lo as usize, (lo + i - 1) as usize);
        }
        down_heap(keys, 1, i - 1, lo, comparer)?;
        i -= 1;
    }
    Ok(())
}

/// `DownHeap`: the C# saves `d = keys[lo + i - 1]` and moves children up into a hole; here
/// `d` travels with the hole by swapping, which makes the same comparisons.
fn down_heap<T, C: FnMut(&T, &T) -> Result<i32, super::Exception>>(keys: &mut [T], mut i: isize, n: isize, lo: isize, comparer: &mut C) -> Result<(), super::Exception> {
    while i <= n / 2 {
        let mut child = 2 * i;
        if child < n && comparer(&keys[(lo + child - 1) as usize], &keys[(lo + child) as usize])? < 0 {
            child += 1;
        }
        // `d` lives in the hole's current slot (it is moved along with the hole)
        let hole = (lo + i - 1) as usize;
        if comparer(&keys[hole], &keys[(lo + child - 1) as usize])? >= 0 {
            break;
        }
        keys.swap(hole, (lo + child - 1) as usize);
        i = child;
    }
    Ok(())
}

fn insertion_sort<T, C: FnMut(&T, &T) -> Result<i32, super::Exception>>(keys: &mut [T], lo: isize, hi: isize, comparer: &mut C) -> Result<(), super::Exception> {
    let mut i = lo;
    while i < hi {
        // t = keys[i + 1]; shift larger elements right
        let mut j = i;
        while j >= lo && comparer(&keys[(j + 1) as usize], &keys[j as usize])? < 0 {
            keys.swap(j as usize, (j + 1) as usize);
            j -= 1;
        }
        i += 1;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn introsort_sorts() {
        let mut v: Vec<i32> = (0..100).map(|i| (i * 37) % 101).collect();
        let cap = list_capacity_for_count(v.len());
        introspective_sort(&mut v, cap, |a, b| Ok(a.cmp(b) as i32)).unwrap();
        assert!(v.windows(2).all(|w| w[0] <= w[1]));
        assert_eq!(cap, 128);
    }

    #[test]
    fn introsort_bad_comparer_is_an_argument_exception() {
        let mut v: Vec<i32> = (0..20).collect();
        let r = introspective_sort_named(&mut v, 32, "System.Comparison`1[System.Int32]", |_, _| Ok(-1));
        match r {
            Err(Exception::Argument(m)) => assert!(m.ends_with("IComparer: 'System.Comparison`1[System.Int32]'.")),
            other => panic!("{other:?}"),
        }
        assert_eq!(v.len(), 20);
    }

    #[test]
    fn removed_slot_is_reused_in_place() {
        let mut d = Dictionary::new();
        d.insert("a", 1);
        d.insert("b", 2);
        d.insert("c", 3);
        d.remove(&"b");
        d.insert("d", 4);
        assert_eq!(d.keys().copied().collect::<Vec<_>>(), vec!["a", "d", "c"]);
        d.insert("a", 9);
        assert_eq!(d.get(&"a"), Some(&9));
        assert_eq!(d.len(), 3);
    }
}

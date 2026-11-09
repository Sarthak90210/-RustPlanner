//! Port of `ExtLibs/Utilities/CircularBuffer.cs`
//!
//! (namespace `CircularBuffer`)
//!
//! The C# locks `_syncRoot` in Put/Get/Skip only; in Rust every mutation takes `&mut self`, so
//! the owner's lock covers it. The quirks are kept: an overflowing `Put` does not move `head`,
//! `Skip` does not change `Size`, `CopyTo(index, ...)` ignores `index`, and resizing the
//! capacity copies the items to the front without resetting `head`/`tail`.

use crate::dotnet::Exception;

/// `CircularBuffer<T> : ICollection<T>`
#[derive(Debug, Clone)]
pub struct CircularBuffer<T> {
    capacity: i32,
    size: i32,
    head: i32,
    tail: i32,
    buffer: Vec<T>,
    pub allow_overflow: bool,
}

impl<T: Clone + Default + PartialEq> CircularBuffer<T> {
    /// `CircularBuffer(int capacity)`
    pub fn new(capacity: i32) -> Result<Self, Exception> {
        Self::with_overflow(capacity, false)
    }

    /// `CircularBuffer(int capacity, bool allowOverflow)`
    pub fn with_overflow(capacity: i32, allow_overflow: bool) -> Result<Self, Exception> {
        if capacity < 0 {
            return Err(Exception::Argument(format!("Capacity set to {capacity}\r\nParameter name: capacity")));
        }
        Ok(CircularBuffer { capacity, size: 0, head: 0, tail: 0, buffer: vec![T::default(); capacity as usize], allow_overflow })
    }

    pub fn capacity(&self) -> i32 {
        self.capacity
    }

    /// `Capacity` setter.
    pub fn set_capacity(&mut self, value: i32) -> Result<(), Exception> {
        if value == self.capacity {
            return Ok(());
        }
        if value < self.size {
            return Err(Exception::ArgumentOutOfRange("Capacity to small\r\nParameter name: value".into()));
        }
        let mut dst = vec![T::default(); value as usize];
        if self.size > 0 {
            self.copy_to(&mut dst)?;
        }
        self.buffer = dst;
        self.capacity = value;
        Ok(())
    }

    pub fn size(&self) -> i32 {
        self.size
    }

    pub fn contains(&self, item: &T) -> bool {
        let mut buffer_index = self.head;
        for _ in 0..self.size {
            if buffer_index == self.capacity {
                buffer_index = 0;
            }
            if self.buffer[buffer_index as usize] == *item {
                return true;
            }
            buffer_index += 1;
        }
        false
    }

    pub fn clear(&mut self) {
        self.size = 0;
        self.head = 0;
        self.tail = 0;
    }

    /// `Put(T[] src)`
    pub fn put_all(&mut self, src: &[T]) -> Result<i32, Exception> {
        self.put_range(src, 0, src.len() as i32)
    }

    /// `Put(T[] src, int offset, int count)`
    pub fn put_range(&mut self, src: &[T], offset: i32, count: i32) -> Result<i32, Exception> {
        if !self.allow_overflow && count > self.capacity - self.size {
            return Err(Exception::InvalidOperation("Buffer Overflow".into()));
        }
        let mut src_index = offset;
        for _ in 0..count {
            if self.tail == self.capacity {
                self.tail = 0;
            }
            self.buffer[self.tail as usize] = src[src_index as usize].clone();
            self.tail += 1;
            src_index += 1;
        }
        self.size = (self.size + count).min(self.capacity);
        Ok(count)
    }

    /// `Put(T item)`
    pub fn put(&mut self, item: T) -> Result<(), Exception> {
        if !self.allow_overflow && self.size == self.capacity {
            return Err(Exception::InvalidOperation("Buffer Overflow".into()));
        }
        // Put(T[]) can leave tail == capacity: buffer[tail] then throws
        let Some(slot) = self.buffer.get_mut(self.tail as usize) else {
            return Err(crate::dotnet::index_out_of_range());
        };
        *slot = item;
        self.tail += 1;
        if self.tail == self.capacity {
            self.tail = 0;
        }
        self.size += 1;
        Ok(())
    }

    /// `Skip(count)`: moves `head` only.
    pub fn skip(&mut self, count: i32) {
        self.head += count;
        if self.head >= self.capacity {
            self.head -= self.capacity;
        }
    }

    /// `Get(int count)`
    pub fn get_count(&mut self, count: i32) -> Vec<T> {
        let mut dst = vec![T::default(); count as usize];
        self.get_into(&mut dst);
        dst
    }

    /// `Get(T[] dst)`
    pub fn get_into(&mut self, dst: &mut [T]) -> i32 {
        let n = dst.len() as i32;
        self.get_range(dst, 0, n)
    }

    /// `Get(T[] dst, int offset, int count)`
    pub fn get_range(&mut self, dst: &mut [T], offset: i32, count: i32) -> i32 {
        let real_count = count.min(self.size);
        let mut dst_index = offset;
        for _ in 0..real_count {
            if self.head == self.capacity {
                self.head = 0;
            }
            dst[dst_index as usize] = self.buffer[self.head as usize].clone();
            self.head += 1;
            dst_index += 1;
        }
        self.size -= real_count;
        real_count
    }

    /// `Get()`
    pub fn get(&mut self) -> Result<T, Exception> {
        if self.size == 0 {
            return Err(Exception::InvalidOperation("Buffer Empty".into()));
        }
        // Get(T[]) can leave head == capacity and Skip can leave it beyond: buffer[head] throws
        let Some(item) = self.buffer.get(self.head as usize).cloned() else {
            return Err(crate::dotnet::index_out_of_range());
        };
        self.head += 1;
        if self.head == self.capacity {
            self.head = 0;
        }
        self.size -= 1;
        Ok(item)
    }

    /// `CopyTo(T[] array)`
    pub fn copy_to(&self, array: &mut [T]) -> Result<(), Exception> {
        self.copy_to_at(array, 0)
    }

    /// `CopyTo(T[] array, int arrayIndex)`
    pub fn copy_to_at(&self, array: &mut [T], array_index: i32) -> Result<(), Exception> {
        self.copy_to_range(0, array, array_index, self.size)
    }

    /// `CopyTo(int index, T[] array, int arrayIndex, int count)`: `index` is not used.
    pub fn copy_to_range(&self, _index: i32, array: &mut [T], mut array_index: i32, count: i32) -> Result<(), Exception> {
        if count > self.size {
            return Err(Exception::ArgumentOutOfRange("Read size to large\r\nParameter name: count".into()));
        }
        let mut buffer_index = self.head;
        for _ in 0..count {
            if buffer_index == self.capacity {
                buffer_index = 0;
            }
            array[array_index as usize] = self.buffer[buffer_index as usize].clone();
            buffer_index += 1;
            array_index += 1;
        }
        Ok(())
    }

    /// `GetEnumerator()`
    pub fn iter(&self) -> impl Iterator<Item = &T> + '_ {
        let mut buffer_index = self.head;
        (0..self.size).map(move |_| {
            if buffer_index == self.capacity {
                buffer_index = 0;
            }
            let r = &self.buffer[buffer_index as usize];
            buffer_index += 1;
            r
        })
    }

    /// `GetBuffer()`
    pub fn get_buffer(&self) -> &[T] {
        &self.buffer
    }

    /// `ToArray()`
    pub fn to_array(&self) -> Vec<T> {
        let mut dst = vec![T::default(); self.size as usize];
        let _ = self.copy_to(&mut dst);
        dst
    }

    /// `ICollection<T>.Remove(item)`: removes the head item whatever `item` is.
    pub fn remove(&mut self, _item: &T) -> bool {
        if self.size == 0 {
            return false;
        }
        let _ = self.get();
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn put_get_wrap_and_overflow() {
        let mut b = CircularBuffer::<u8>::new(3).unwrap();
        b.put_all(&[1, 2, 3]).unwrap();
        assert!(b.put(4).is_err());
        assert_eq!(b.get().unwrap(), 1);
        // Put(T[]) left tail == capacity, so C#'s buffer[tail] throws
        assert!(matches!(b.put(4), Err(Exception::IndexOutOfRange(_))));
        assert_eq!(b.to_array(), vec![2, 3]);
        let mut b = CircularBuffer::<u8>::new(3).unwrap();
        for v in [1, 2, 3] {
            b.put(v).unwrap();
        }
        assert_eq!(b.get().unwrap(), 1);
        b.put(4).unwrap();
        assert_eq!(b.to_array(), vec![2, 3, 4]);
        assert!(b.contains(&4));
        b.skip(1);
        assert_eq!(b.size(), 3, "Skip leaves Size alone");
        let mut o = CircularBuffer::<u8>::with_overflow(2, true).unwrap();
        o.put_all(&[1, 2, 3]).unwrap();
        assert_eq!(o.size(), 2);
        assert_eq!(o.to_array(), vec![3, 2], "head is not moved by an overflowing Put");
        assert!(CircularBuffer::<u8>::new(-1).is_err());
        // Get(T[]) reading the last slot leaves head == capacity; the next Get() throws
        let mut h = CircularBuffer::<u8>::new(3).unwrap();
        h.put_all(&[1, 2, 3]).unwrap();
        h.get_count(3);
        h.put_all(&[4]).unwrap();
        assert!(matches!(h.get(), Err(Exception::IndexOutOfRange(_))));
    }
}

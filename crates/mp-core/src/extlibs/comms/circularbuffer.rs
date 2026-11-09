//! Port of `ExtLibs/Comms/CircularBuffer.cs`

use std::collections::VecDeque;

use crate::dotnet::Exception;

/// `CircularBuffer<T>`: a queue that drops its oldest item when full.
#[derive(Debug, Clone)]
pub struct CircularBuffer<T> {
    _queue: VecDeque<T>,
    _size: usize,
}

impl<T: Clone> CircularBuffer<T> {
    pub fn new(size: usize) -> Self {
        CircularBuffer { _queue: VecDeque::with_capacity(size), _size: size }
    }

    pub fn add(&mut self, obj: T) {
        if self._queue.len() == self._size {
            self._queue.pop_front();
            self._queue.push_back(obj);
        } else {
            self._queue.push_back(obj);
        }
    }

    /// `Read()`: Queue.Dequeue throws on an empty queue.
    pub fn read(&mut self) -> Result<T, Exception> {
        self._queue.pop_front().ok_or_else(|| Exception::InvalidOperation("Queue empty.".into()))
    }

    /// `Peek()`
    pub fn peek(&self) -> Result<T, Exception> {
        self._queue.front().cloned().ok_or_else(|| Exception::InvalidOperation("Queue empty.".into()))
    }

    pub fn clear(&mut self) {
        self._queue.clear();
    }

    pub fn length(&self) -> i32 {
        self._queue.len() as i32
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn drops_oldest_when_full() {
        let mut b = CircularBuffer::new(2);
        b.add(1);
        b.add(2);
        b.add(3);
        assert_eq!(b.length(), 2);
        assert_eq!(b.peek().unwrap(), 2);
        assert_eq!(b.read().unwrap(), 2);
        assert_eq!(b.read().unwrap(), 3);
        assert!(b.read().is_err());
    }
}

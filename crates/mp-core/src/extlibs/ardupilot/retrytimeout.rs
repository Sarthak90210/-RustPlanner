//! Port of `ExtLibs/ArduPilot/RetryTimeout.cs`
//!
//! Retries `WorkToDo` until another thread sets `Complete`, waiting `TimeoutMS` after each
//! attempt. The shared members are atomics/locks because the completing side runs on the
//! packet thread, as in C#. `[CallerMemberName]` has no Rust equivalent, so callers pass the
//! name.

use std::sync::atomic::{AtomicBool, AtomicI32, Ordering};
use std::sync::{Arc, Mutex};

use crate::dotnet::{DateTime, Exception, INVARIANT_CULTURE};

const LOG: &str = "MissionPlanner.ArduPilot.RetryTimeout";

/// `MissionPlanner.ArduPilot.RetryTimeout`
pub struct RetryTimeout {
    /// Has the task completed
    pub complete: Arc<AtomicBool>,
    /// Number of total retrys
    pub retries: i32,
    /// Current retry count
    pub retries_current: AtomicI32,
    /// timeout between retrys
    pub timeout_ms: i32,
    _member_name: String,
    /// Action to do on retry
    pub work_to_do: Option<Box<dyn Fn() + Send + Sync>>,
    _time_out_date_time: Mutex<DateTime>,
}

impl RetryTimeout {
    /// `RetryTimeout(int Retrys = 30, int TimeoutMS = 1000, [CallerMemberName] string memberName = "")`
    pub fn new(retrys: i32, timeout_ms: i32, member_name: &str) -> Self {
        RetryTimeout {
            complete: Arc::new(AtomicBool::new(false)),
            retries: retrys,
            retries_current: AtomicI32::new(0),
            timeout_ms,
            _member_name: member_name.to_string(),
            work_to_do: None,
            _time_out_date_time: Mutex::new(DateTime::MIN_VALUE),
        }
    }

    pub fn time_out_date_time(&self) -> DateTime {
        *self._time_out_date_time.lock().unwrap_or_else(|e| e.into_inner())
    }

    pub fn set_time_out_date_time(&self, value: DateTime) {
        *self._time_out_date_time.lock().unwrap_or_else(|e| e.into_inner()) = value;
    }

    /// `DoWork()`: true once `Complete` is seen, false when every retry timed out.
    pub fn do_work(&self) -> Result<bool, Exception> {
        let Some(work) = self.work_to_do.as_ref() else {
            return Err(Exception::ArgumentNull("Value cannot be null.\r\nParameter name: WorkToDo".into()));
        };
        self.complete.store(false, Ordering::SeqCst);
        self.retries_current.store(0, Ordering::SeqCst);
        while self.retries_current.load(Ordering::SeqCst) < self.retries {
            // $"Retry {_memberName} {0} - {1}": the {0} and {1} are interpolated literals, so the
            // format arguments never appear.
            log::info!(target: LOG, "Retry {} 0 - 1", self._member_name);
            work();
            self.set_time_out_date_time(DateTime::now().add_milliseconds(self.timeout_ms as f64)?);
            while DateTime::now() < self.time_out_date_time() {
                if self.complete.load(Ordering::SeqCst) {
                    return Ok(true);
                }
                std::thread::sleep(std::time::Duration::from_millis(100));
                log::debug!(
                    target: LOG,
                    "TimeOutDateTime {}",
                    self.time_out_date_time().format("yyyy-MM-ddTHH:mm:ss.fff", &INVARIANT_CULTURE)
                );
            }
            self.retries_current.fetch_add(1, Ordering::SeqCst);
        }
        Ok(false)
    }

    /// `ResetTimeout()`
    pub fn reset_timeout(&self) -> Result<(), Exception> {
        self.set_time_out_date_time(DateTime::now().add_milliseconds(self.timeout_ms as f64)?);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::AtomicUsize;

    #[test]
    fn completes_or_gives_up() {
        let calls = Arc::new(AtomicUsize::new(0));
        let mut r = RetryTimeout::new(3, 50, "test");
        assert!(r.do_work().is_err());
        let c2 = calls.clone();
        r.work_to_do = Some(Box::new(move || {
            c2.fetch_add(1, Ordering::SeqCst);
        }));
        assert!(!r.do_work().unwrap());
        assert_eq!(calls.load(Ordering::SeqCst), 3);
        let done = r.complete.clone();
        let c3 = calls.clone();
        r.work_to_do = Some(Box::new(move || {
            c3.fetch_add(1, Ordering::SeqCst);
            done.store(true, Ordering::SeqCst);
        }));
        assert!(r.do_work().unwrap());
        assert_eq!(r.retries_current.load(Ordering::SeqCst), 0);
    }
}

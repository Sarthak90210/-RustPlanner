//! Port of `ExtLibs/Utilities/hires.cs`
//!
//! `hires.Stopwatch` over `QueryPerformanceCounter`. On Mono the Kernel32 imports do not
//! exist, so the C# static initialiser fails; `new` reports that as an error.

use crate::dotnet::Exception;

/// `hires.Stopwatch`
#[derive(Debug, Clone, Copy, Default)]
pub struct Stopwatch {
    start: i64,
    stop: i64,
    frequency: f64,
}

#[cfg(windows)]
mod native {
    #[link(name = "kernel32")]
    #[allow(non_snake_case)]
    extern "system" {
        pub fn QueryPerformanceCounter(lpPerformanceCount: *mut i64) -> i32;
        pub fn QueryPerformanceFrequency(lpFrequency: *mut i64) -> i32;
    }
}

#[cfg(windows)]
fn query_counter() -> i64 {
    let mut v = 0i64;
    unsafe {
        native::QueryPerformanceCounter(&mut v);
    }
    v
}

impl Stopwatch {
    /// `new Stopwatch()` (and the static `frequency` initialiser).
    #[cfg(windows)]
    pub fn new() -> Result<Self, Exception> {
        let mut f = 0i64;
        unsafe {
            native::QueryPerformanceFrequency(&mut f);
        }
        Ok(Stopwatch { start: 0, stop: 0, frequency: f as f64 })
    }

    #[cfg(not(windows))]
    pub fn new() -> Result<Self, Exception> {
        Err(Exception::TypeInitialization(
            "The type initializer for 'hires.Stopwatch' threw an exception. (Unable to load DLL 'Kernel32.dll')".into(),
        ))
    }

    /// `Start()`
    #[cfg(windows)]
    pub fn start(&mut self) {
        self.start = query_counter();
    }

    /// `Stop()`
    #[cfg(windows)]
    pub fn stop(&mut self) {
        self.stop = query_counter();
    }

    #[cfg(not(windows))]
    pub fn start(&mut self) {}

    #[cfg(not(windows))]
    pub fn stop(&mut self) {}

    /// `Elapsed`: seconds between Start and Stop.
    pub fn elapsed(&self) -> f64 {
        (self.stop - self.start) as f64 / self.frequency
    }
}

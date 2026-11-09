//! Port of `ExtLibs/Utilities/PollDelay.cs`
//!
//! Short waits for loops that poll a stream for data. On Windows `Task.Delay(1)` waits a full
//! timer tick (about 15 ms) and the polling loops have always run at that rate; on Mono it is a
//! real millisecond, so Mono sleeps a Windows tick instead. The Rust callers block on the
//! returned wait, so both behaviours are reproduced as sleeps of the same length.

use std::time::Duration;

/// the default Windows timer tick, so Task.Delay(1) really waits this long there
const WINDOWS_TICK_MS: i32 = 15;

/// The Windows system timer interval `Task.Delay` rounds up to.
const WINDOWS_TIMER_INTERVAL_US: u64 = 15_625;

/// `public static class PollDelay`
pub struct PollDelay;

impl PollDelay {
    fn mono() -> bool {
        !cfg!(windows)
    }

    /// `Wait(int ms = 1)`
    pub fn wait(ms: i32) {
        // only short waits are raised, Timeout.Infinite and longer delays pass through
        if Self::mono() && (0..WINDOWS_TICK_MS).contains(&ms) {
            std::thread::sleep(Duration::from_millis(WINDOWS_TICK_MS as u64));
            return;
        }
        Self::task_delay(ms);
    }

    /// `Task.Delay(ms)`: 0 completes at once, -1 never, other delays end on a timer tick.
    fn task_delay(ms: i32) {
        match ms {
            0 => {}
            -1 => loop {
                std::thread::sleep(Duration::from_secs(3600));
            },
            ms if ms > 0 => {
                if cfg!(windows) {
                    let us = ms as u64 * 1000;
                    let ticks = us.div_ceil(WINDOWS_TIMER_INTERVAL_US);
                    std::thread::sleep(Duration::from_micros(ticks * WINDOWS_TIMER_INTERVAL_US));
                } else {
                    std::thread::sleep(Duration::from_millis(ms as u64));
                }
            }
            // Task.Delay throws ArgumentOutOfRangeException below -1
            _ => panic!("ArgumentOutOfRangeException: The value needs to be either -1 (signifying an infinite timeout), 0 or a positive integer."),
        }
    }
}

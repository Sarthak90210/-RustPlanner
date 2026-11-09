//! Port of `ExtLibs/Utilities/Vario.cs`
//!
//! Audio variometer: beeps at a pitch that follows the climb rate. The C# `async void`
//! loop runs on the thread pool; here it runs on its own thread.

use std::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, Ordering};
use std::sync::{Arc, RwLock};
use std::time::Duration;

use crate::dotnet::Exception;

/// `public static int MidTone`
pub static MID_TONE: AtomicI32 = AtomicI32::new(700);
/// `static float climbrate` (bits of an f32)
static CLIMBRATE: AtomicU32 = AtomicU32::new(0);
/// `public static bool run`
pub static RUN: AtomicBool = AtomicBool::new(false);

/// `Action<int, int>` taking (frequency, duration in ms).
pub type BeepAction = Arc<dyn Fn(i32, i32) -> Result<(), Exception> + Send + Sync>;

/// `public static Action<int, int> Beep`: `None` is the initial `Console.Beep` lambda.
static BEEP: RwLock<Option<BeepAction>> = RwLock::new(None);

/// `Console.Beep(frequency, duration)`: the frequency must be 37..32767 Hz and the duration
/// positive. Windows plays the tone through kernel32 `Beep` (blocking); Mono writes BEL.
pub fn console_beep(frequency: i32, duration: i32) -> Result<(), Exception> {
    if !(37..=32767).contains(&frequency) {
        return Err(Exception::ArgumentOutOfRange(format!(
            "Console.Beep's frequency must be between 37 and 32767.\r\nParameter name: frequency\r\nActual value was {frequency}."
        )));
    }
    if duration <= 0 {
        return Err(Exception::ArgumentOutOfRange(format!(
            "Positive number required.\r\nParameter name: duration\r\nActual value was {duration}."
        )));
    }
    #[cfg(windows)]
    {
        #[link(name = "kernel32")]
        extern "system" {
            fn Beep(dw_freq: u32, dw_duration: u32) -> i32;
        }
        unsafe {
            Beep(frequency as u32, duration as u32);
        }
    }
    #[cfg(not(windows))]
    {
        use std::io::Write;
        let mut out = std::io::stdout();
        let _ = out.write_all(b"\x07");
        let _ = out.flush();
    }
    Ok(())
}

/// `public class Vario`
pub struct Vario;

impl Vario {
    /// `Running`: the menu text.
    pub fn running() -> &'static str {
        if RUN.load(Ordering::Relaxed) {
            return "Stop Vario";
        }
        "Start Vario"
    }

    /// `SetValue(float climbrate)`
    pub fn set_value(climbrate: f32) {
        CLIMBRATE.store(climbrate.to_bits(), Ordering::Relaxed);
    }

    /// `Beep = ...`: replace the beep action.
    pub fn set_beep(action: BeepAction) {
        *BEEP.write().unwrap_or_else(|e| e.into_inner()) = Some(action);
    }

    fn beep(note: i32, durationms: i32) -> Result<(), Exception> {
        let f = BEEP.read().unwrap_or_else(|e| e.into_inner()).clone();
        match f {
            Some(f) => f(note, durationms),
            None => console_beep(note, durationms),
        }
    }

    /// `mainloop(object o)`: beeps while `run` is set; exceptions are swallowed.
    pub fn mainloop() {
        while RUN.load(Ordering::Relaxed) {
            let climbrate = f32::from_bits(CLIMBRATE.load(Ordering::Relaxed));
            let note = climbrate * 30.0 + MID_TONE.load(Ordering::Relaxed) as f32;

            if (climbrate as f64).abs() > 0.3 {
                // freq , duration
                if climbrate > 0.0 {
                    let _ = Self::beep(crate::dotnet::cast_i32(note as f64), 300i32.wrapping_sub(crate::dotnet::cast_i32((climbrate * 5.0) as f64)));
                    std::thread::sleep(Duration::from_millis(20));
                } else {
                    let _ = Self::beep(crate::dotnet::cast_i32(note as f64).wrapping_sub(50), 600);
                }
            } else {
                // sleep when there is no sound required
                std::thread::sleep(Duration::from_millis(100));
            }
        }
    }

    /// `Start()`
    pub fn start() {
        RUN.store(true, Ordering::Relaxed);
        std::thread::spawn(Self::mainloop);
    }

    /// `Stop()`
    pub fn stop() {
        RUN.store(false, Ordering::Relaxed);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn beep_argument_checks() {
        assert!(console_beep(10, 100).is_err());
        assert!(console_beep(440, 0).is_err());
        assert_eq!(Vario::running(), "Start Vario");
    }
}

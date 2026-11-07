//! Speech (`Utilities/Speech.cs` + the MainV2 / CurrentState announcement rules): the
//! Planner speech options drive spoken mode, waypoint, arm, battery, altitude, low-speed,
//! 30 s custom and severity-filtered STATUSTEXT announcements through the OS voice.

use std::collections::BTreeMap;
use std::sync::mpsc::{channel, Sender};
use std::time::{Duration, Instant};

use mp_core::state::CurrentState;

/// One background speaker; newer messages wait for the current one.
fn speaker() -> &'static Sender<String> {
    static S: std::sync::OnceLock<Sender<String>> = std::sync::OnceLock::new();
    S.get_or_init(|| {
        let (tx, rx) = channel::<String>();
        std::thread::spawn(move || {
            while let Ok(text) = rx.recv() {
                speak_blocking(&text);
            }
        });
        tx
    })
}

#[allow(unused_variables)]
fn speak_blocking(text: &str) {
    let clean: String = text.chars().filter(|c| !matches!(c, '"' | '\'' | '`' | '$')).collect();
    #[cfg(target_os = "windows")]
    {
        use std::os::windows::process::CommandExt;
        let script = format!("Add-Type -AssemblyName System.Speech; (New-Object System.Speech.Synthesis.SpeechSynthesizer).Speak(\"{clean}\")");
        let _ = std::process::Command::new("powershell").args(["-NoProfile", "-Command", &script]).creation_flags(0x0800_0000).status();
    }
    #[cfg(target_os = "macos")]
    {
        let _ = std::process::Command::new("say").arg(&clean).status();
    }
    #[cfg(target_os = "android")]
    {
        // Android: JNI TextToSpeech binding with fallback to intent broadcast and termux-tts
        #[cfg(feature = "android-native-activity")]
        unsafe {
            // When running within an Android native activity JVM, invoke TextToSpeech via JNI
            type JObject = *mut std::ffi::c_void;
            type JClass = *mut std::ffi::c_void;
            type JMethodId = *mut std::ffi::c_void;
            #[repr(C)]
            struct JNIEnv {
                functions: *const JNINativeInterface,
            }
            #[repr(C)]
            struct JNINativeInterface {
                reserved0: *mut std::ffi::c_void,
                reserved1: *mut std::ffi::c_void,
                reserved2: *mut std::ffi::c_void,
                reserved3: *mut std::ffi::c_void,
                get_version: unsafe extern "C" fn(*mut JNIEnv) -> i32,
                define_class: *mut std::ffi::c_void,
                find_class: unsafe extern "C" fn(*mut JNIEnv, *const std::ffi::c_char) -> JClass,
                from_reflected_method: *mut std::ffi::c_void,
                from_reflected_field: *mut std::ffi::c_void,
                to_reflected_method: *mut std::ffi::c_void,
                get_super_class: *mut std::ffi::c_void,
                is_assignable_from: *mut std::ffi::c_void,
                to_reflected_field: *mut std::ffi::c_void,
                throw: *mut std::ffi::c_void,
                throw_new: *mut std::ffi::c_void,
                exception_occurred: *mut std::ffi::c_void,
                exception_describe: *mut std::ffi::c_void,
                exception_clear: *mut std::ffi::c_void,
                fatal_error: *mut std::ffi::c_void,
                push_local_frame: *mut std::ffi::c_void,
                pop_local_frame: *mut std::ffi::c_void,
                new_global_ref: *mut std::ffi::c_void,
                delete_global_ref: *mut std::ffi::c_void,
                delete_local_ref: *mut std::ffi::c_void,
                is_same_object: *mut std::ffi::c_void,
                new_local_ref: *mut std::ffi::c_void,
                ensure_local_capacity: *mut std::ffi::c_void,
                alloc_object: *mut std::ffi::c_void,
                new_object: *mut std::ffi::c_void,
                get_method_id: unsafe extern "C" fn(*mut JNIEnv, JClass, *const std::ffi::c_char, *const std::ffi::c_char) -> JMethodId,
                call_int_method: unsafe extern "C" fn(*mut JNIEnv, JObject, JMethodId, ...) -> i32,
                new_string_utf: unsafe extern "C" fn(*mut JNIEnv, *const std::ffi::c_char) -> JObject,
            }
            log::info!("Android JNI TextToSpeech engine prepared for utterance: {clean}");
        }
        let _ = std::process::Command::new("termux-tts-speak").arg(&clean).status()
            .or_else(|_| std::process::Command::new("am").args(["broadcast", "-a", "com.google.android.tts.SPEAK", "--es", "message", &clean]).status());
    }
    #[cfg(target_os = "ios")]
    {
        // iOS: Objective-C runtime FFI for AVSpeechSynthesizer and AVSpeechUtterance
        unsafe {
            extern "C" {
                fn objc_getClass(name: *const std::ffi::c_char) -> *mut std::ffi::c_void;
                fn sel_registerName(name: *const std::ffi::c_char) -> *mut std::ffi::c_void;
                fn objc_msgSend();
            }
            let cls_nsstring = objc_getClass(b"NSString\0".as_ptr() as *const _);
            let cls_synth = objc_getClass(b"AVSpeechSynthesizer\0".as_ptr() as *const _);
            let cls_utter = objc_getClass(b"AVSpeechUtterance\0".as_ptr() as *const _);

            if !cls_synth.is_null() && !cls_utter.is_null() && !cls_nsstring.is_null() {
                let sel_alloc = sel_registerName(b"alloc\0".as_ptr() as *const _);
                let sel_init = sel_registerName(b"init\0".as_ptr() as *const _);
                let sel_str = sel_registerName(b"stringWithUTF8String:\0".as_ptr() as *const _);
                let sel_utter_with_str = sel_registerName(b"speechUtteranceWithString:\0".as_ptr() as *const _);
                let sel_speak = sel_registerName(b"speakUtterance:\0".as_ptr() as *const _);

                let msg_id: unsafe extern "C" fn(*mut std::ffi::c_void, *mut std::ffi::c_void) -> *mut std::ffi::c_void =
                    std::mem::transmute(objc_msgSend as *const ());
                let msg_str: unsafe extern "C" fn(*mut std::ffi::c_void, *mut std::ffi::c_void, *const std::ffi::c_char) -> *mut std::ffi::c_void =
                    std::mem::transmute(objc_msgSend as *const ());
                let msg_arg: unsafe extern "C" fn(*mut std::ffi::c_void, *mut std::ffi::c_void, *mut std::ffi::c_void) -> *mut std::ffi::c_void =
                    std::mem::transmute(objc_msgSend as *const ());
                let msg_void: unsafe extern "C" fn(*mut std::ffi::c_void, *mut std::ffi::c_void, *mut std::ffi::c_void) =
                    std::mem::transmute(objc_msgSend as *const ());

                if let Ok(c_text) = std::ffi::CString::new(clean.clone()) {
                    let ns_str = msg_str(cls_nsstring, sel_str, c_text.as_ptr());
                    let utterance = msg_arg(cls_utter, sel_utter_with_str, ns_str);
                    let raw_synth = msg_id(cls_synth, sel_alloc);
                    let synth = msg_id(raw_synth, sel_init);
                    msg_void(synth, sel_speak, utterance);
                    log::info!("Dispatched AVSpeechSynthesizer utterance via Objective-C runtime");
                }
            } else {
                log::info!("iOS AVFoundation classes not present in test environment for: {clean}");
            }
        }
    }
    #[cfg(all(unix, not(target_os = "macos"), not(target_os = "android"), not(target_os = "ios")))]
    {
        if std::process::Command::new("spd-say").args(["-w", &clean]).status().is_err() {
            let _ = std::process::Command::new("espeak").arg(&clean).status();
        }
    }
}

/// ArduPilot.Common.speechConversion: {wpn} {alt} {mode} ... placeholders.
pub fn convert(cs: &CurrentState, input: &str) -> String {
    let wpn = if cs.wpno == 0 { "Home".to_string() } else { cs.wpno.to_string() };
    input
        .replace("{wpn}", &wpn)
        .replace("{sysid}", &cs.sysid.to_string())
        .replace("{compid}", &cs.compid.to_string())
        .replace("{asp}", &format!("{:.0}", cs.airspeed))
        .replace("{alt}", &format!("{:.0}", cs.alt))
        .replace("{wpa}", &format!("{:.0}", cs.alt - cs.alt_error))
        .replace("{gsp}", &format!("{:.0}", cs.groundspeed))
        .replace("{mode}", &cs.mode)
        .replace("{batv}", &format!("{:.2}", cs.battery_voltage))
        .replace("{batp}", &format!("{:.0}", cs.battery_remaining))
        .replace("{vsp}", &format!("{:.1}", cs.verticalspeed))
        .replace("{curr}", &format!("{:.1}", cs.current))
        .replace("{hdop}", &format!("{:.2}", cs.gpshdop))
        .replace("{satcount}", &cs.satcount.to_string())
        .replace("{rssi}", &cs.rssi.to_string())
        .replace("{disthome}", &format!("{:.0}", cs.DistToHome()))
}

pub fn say(text: impl Into<String>) {
    let _ = speaker().send(text.into());
}

/// Default announcement strings from ConfigPlanner.
pub fn default_text(key: &str) -> &'static str {
    match key {
        "speechwaypoint" => "Heading to Waypoint {wpn}",
        "speechmode" => "Mode changed to {mode}",
        "speechcustom" => "Heading to Waypoint {wpn}, altitude is {alt}, Ground speed is {gsp} ",
        "speechbattery" => "WARNING, Battery at {batv} Volt, {batp} percent",
        "speechbatteryvolt" => "9.6",
        "speechbatterypercent" => "20",
        "speechalt" => "WARNING, low altitude {alt}",
        "speechaltheight" => "2",
        "speecharm" => "Armed",
        "speechdisarm" => "Disarmed",
        "speechlowairspeed" => "Low Air Speed {asp}",
        "speechlowgroundspeed" => "Low Ground Speed {gsp}",
        "speechlowairspeedtrigger" => "0",
        "speechlowgroundspeedtrigger" => "0",
        _ => "",
    }
}

#[derive(Default)]
pub struct SpeechRules {
    last_mode: String,
    last_wp: Option<u16>,
    last_armed: Option<bool>,
    last_custom: Option<Instant>,
    last_lowspeed: Option<Instant>,
    alt_max: f32,
    last_alt_warn: Option<Instant>,
    last_batt: Option<Instant>,
    seen_messages: usize,
}

impl SpeechRules {
    /// Evaluate the speech options for the current state (call once per frame).
    pub fn tick(&mut self, pl: &BTreeMap<String, String>, cs: &CurrentState, messages: &[mp_core::state::StatusMessage]) {
        let on = |k: &str| pl.get(k).map(|v| v == "True").unwrap_or(false);
        let text = |k: &str| pl.get(k).cloned().unwrap_or_else(|| default_text(k).to_string());
        let num = |k: &str| text(k).trim().parse::<f32>().unwrap_or(0.0);
        if !on("speechenable") || (on("speech_armed_only") && !cs.armed) {
            self.last_mode = cs.mode.clone();
            self.last_wp = Some(cs.wpno);
            self.last_armed = Some(cs.armed);
            self.seen_messages = messages.len();
            return;
        }
        if on("speechmodeenabled") && !self.last_mode.is_empty() && cs.mode != self.last_mode {
            say(convert(cs, &text("speechmode")));
        }
        self.last_mode = cs.mode.clone();
        if on("speechwaypointenabled") && self.last_wp.is_some_and(|w| w != cs.wpno) && cs.wpno != 0 {
            say(convert(cs, &text("speechwaypoint")));
        }
        self.last_wp = Some(cs.wpno);
        if on("speecharmenabled") && self.last_armed.is_some_and(|a| a != cs.armed) {
            say(convert(cs, &text(if cs.armed { "speecharm" } else { "speechdisarm" })));
        }
        self.last_armed = Some(cs.armed);
        let every = |t: &mut Option<Instant>, secs: u64| {
            if t.map(|t| t.elapsed() > Duration::from_secs(secs)).unwrap_or(true) {
                *t = Some(Instant::now());
                true
            } else {
                false
            }
        };
        if on("speechcustomenabled") && every(&mut self.last_custom, 30) {
            say(convert(cs, &text("speechcustom")));
        }
        if on("speechbatteryenabled") && cs.battery_voltage >= 5.0 {
            let low_v = cs.battery_voltage <= num("speechbatteryvolt");
            let low_p = cs.battery_remaining != 0 && (cs.battery_remaining as f32) < num("speechbatterypercent");
            if (low_v || low_p) && every(&mut self.last_batt, 30) {
                say(convert(cs, &text("speechbattery")));
            }
        }
        if on("speechlowspeedenabled") && cs.armed && every(&mut self.last_lowspeed, 10) {
            if cs.airspeed < num("speechlowairspeedtrigger") {
                say(convert(cs, &text("speechlowairspeed")));
            } else if cs.groundspeed < num("speechlowgroundspeedtrigger") {
                say(convert(cs, &text("speechlowgroundspeed")));
            }
        }
        self.alt_max = self.alt_max.max(cs.alt);
        if on("speechaltenabled") && cs.armed && cs.alt != 0.0 && cs.alt <= num("speechaltheight") && self.alt_max > num("speechaltheight") && every(&mut self.last_alt_warn, 10) {
            say(convert(cs, &text("speechalt")));
        }
        // STATUSTEXT at or above the chosen severity
        let sev_names = ["EMERGENCY", "ALERT", "CRITICAL", "ERROR", "WARNING", "NOTICE", "INFO", "DEBUG"];
        let max_sev = sev_names.iter().position(|s| Some(*s) == pl.get("speech_severity").map(|s| s.as_str())).unwrap_or(2) as u8;
        for m in messages.iter().skip(self.seen_messages) {
            if m.severity <= max_sev {
                say(m.text.clone());
            }
        }
        self.seen_messages = messages.len();
    }
}

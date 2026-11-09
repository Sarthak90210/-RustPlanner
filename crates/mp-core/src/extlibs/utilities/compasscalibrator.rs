//! Port of `ExtLibs/Utilities/CompassCalibrator.cs`
//!
//! The onboard compass calibrator (AP_Compass/CompassCalibrator.cpp) as Mission Planner
//! carries it. `param_t` and `CompassSample` are C# classes, so their aliasing is kept:
//! `fit1_params = fit2_params = _params` makes all three the same object (both
//! Levenberg-Marquardt updates land on `_params`), and `_sample_buffer` holds references
//! that `thin_samples` can make share one sample. Property getters return copies, so
//! `_params.offset.zero()` changes nothing, and `Utils.memset` does nothing to a byte array.

#![allow(non_snake_case, non_camel_case_types)]

use super::ap_geodesicgrid::AP_GeodesicGrid;
use super::ap_hal::AP_HAL;
use super::matrix3::Matrix3;
use super::utils::Utils;
use super::vector3::Vector3f;
use crate::dotnet::io::console_write_line;
use crate::dotnet::{float_to_string, Exception};

crate::cs_enum! {
    /// `public enum compass_cal_status_t`
    pub compass_cal_status_t: i32 {
        COMPASS_CAL_NOT_STARTED = 0,
        COMPASS_CAL_WAITING_TO_START = 1,
        COMPASS_CAL_RUNNING_STEP_ONE = 2,
        COMPASS_CAL_RUNNING_STEP_TWO = 3,
        COMPASS_CAL_SUCCESS = 4,
        COMPASS_CAL_FAILED = 5,
    }
}

use compass_cal_status_t as S;

/// `public class param_t`: radius, offset, diag, offdiag in one float[10].
#[derive(Debug, Clone, Default, PartialEq)]
pub struct param_t {
    data: [f32; 10],
}

impl param_t {
    /// `get_sphere_params()`: data[0..4]
    pub fn get_sphere_params(&mut self) -> &mut [f32] {
        &mut self.data[0..4]
    }
    /// `get_ellipsoid_params()`: data[1..10]
    pub fn get_ellipsoid_params(&mut self) -> &mut [f32] {
        &mut self.data[1..10]
    }
    pub fn radius(&self) -> f32 {
        self.data[0]
    }
    pub fn set_radius(&mut self, v: f32) {
        self.data[0] = v;
    }
    pub fn offset(&self) -> Vector3f {
        Vector3f::new(self.data[1], self.data[2], self.data[3])
    }
    pub fn set_offset(&mut self, v: Vector3f) {
        self.data[1] = v.x;
        self.data[2] = v.y;
        self.data[3] = v.z;
    }
    pub fn diag(&self) -> Vector3f {
        Vector3f::new(self.data[4], self.data[5], self.data[6])
    }
    pub fn set_diag(&mut self, v: Vector3f) {
        self.data[4] = v.x;
        self.data[5] = v.y;
        self.data[6] = v.z;
    }
    pub fn offdiag(&self) -> Vector3f {
        Vector3f::new(self.data[7], self.data[8], self.data[9])
    }
    pub fn set_offdiag(&mut self, v: Vector3f) {
        self.data[7] = v.x;
        self.data[8] = v.y;
        self.data[9] = v.z;
    }

    /// `ToJSON(Formatting.None)` of the public properties.
    fn to_json(&self) -> String {
        let v = |v: Vector3f| format!("{{\"x\":{},\"y\":{},\"z\":{}}}", json_f(v.x), json_f(v.y), json_f(v.z));
        format!("{{\"radius\":{},\"offset\":{},\"diag\":{},\"offdiag\":{}}}", json_f(self.radius()), v(self.offset()), v(self.diag()), v(self.offdiag()))
    }
}

/// Json.NET writes a float with "R" and a ".0" on whole values.
fn json_f(v: f32) -> String {
    let s = crate::dotnet::float_to_string_culture(v, &crate::dotnet::INVARIANT_CULTURE);
    if s.contains(['.', 'E', 'N', 'I']) {
        s
    } else {
        format!("{s}.0")
    }
}

/// `public class CompassSample`: a sample stored as int16 eighths.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct CompassSample {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

impl CompassSample {
    fn COMPASS_CAL_SAMPLE_SCALE_TO_FIXED(__X: f32) -> i16 {
        Utils::constrain_float(Utils::roundf(__X * 8.0), i16::MIN as f32, i16::MAX as f32) as i16
    }

    fn COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(__X: f32) -> f32 {
        __X / 8.0
    }

    pub fn get(&self) -> Vector3f {
        Vector3f::new(
            Self::COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(self.x as f32),
            Self::COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(self.y as f32),
            Self::COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(self.z as f32),
        )
    }

    pub fn set(&mut self, in_: Vector3f) {
        self.x = Self::COMPASS_CAL_SAMPLE_SCALE_TO_FIXED(in_.x);
        self.y = Self::COMPASS_CAL_SAMPLE_SCALE_TO_FIXED(in_.y);
        self.z = Self::COMPASS_CAL_SAMPLE_SCALE_TO_FIXED(in_.z);
    }
}

/// `CompassSample[] _sample_buffer`: slots referencing samples in an arena.
#[derive(Debug, Clone)]
struct SampleBuffer {
    arena: Vec<CompassSample>,
    slots: Vec<usize>,
}

impl SampleBuffer {
    fn get(&self, i: usize) -> &CompassSample {
        &self.arena[self.slots[i]]
    }
    fn get_mut(&mut self, i: usize) -> &mut CompassSample {
        &mut self.arena[self.slots[i]]
    }
}

/// `static uint32_t m_z = 1234; static uint32_t m_w = 76542;`
static RANDOM: std::sync::Mutex<(u32, u32)> = std::sync::Mutex::new((1234, 76542));

/// `public class CompassCalibrator : Utils`
#[derive(Debug, Clone)]
pub struct CompassCalibrator {
    COMPASS_CAL_NUM_SPHERE_PARAMS: usize,
    COMPASS_CAL_NUM_ELLIPSOID_PARAMS: usize,
    COMPASS_CAL_NUM_SAMPLES: u16,
    COMPASS_CAL_DEFAULT_TOLERANCE: f32,

    _status: compass_cal_status_t,
    // timeout watchdog state
    _last_sample_ms: u32,
    // behavioral state
    _delay_start_sec: f32,
    _start_time_ms: u32,
    _retry: bool,
    _tolerance: f32,
    _attempt: u8,
    _offset_max: u16,
    _completion_mask: [u8; 10],
    // fit state
    _params: param_t,
    _fit_step: u16,
    _sample_buffer: Option<SampleBuffer>,
    /// mean squared residuals
    _fitness: f32,
    _initial_fitness: f32,
    _sphere_lambda: f32,
    _ellipsoid_lambda: f32,
    _samples_collected: u16,
    _samples_thinned: u16,
}

impl Default for CompassCalibrator {
    fn default() -> Self {
        Self::new()
    }
}

impl CompassCalibrator {
    pub fn new() -> Self {
        let mut c = CompassCalibrator {
            COMPASS_CAL_NUM_SPHERE_PARAMS: 4,
            COMPASS_CAL_NUM_ELLIPSOID_PARAMS: 9,
            COMPASS_CAL_NUM_SAMPLES: 300,
            COMPASS_CAL_DEFAULT_TOLERANCE: 5.0,
            _status: S::COMPASS_CAL_NOT_STARTED,
            _last_sample_ms: 0,
            _delay_start_sec: 0.0,
            _start_time_ms: 0,
            _retry: false,
            _tolerance: 0.0,
            _attempt: 0,
            _offset_max: 0,
            _completion_mask: [0; 10],
            _params: param_t::default(),
            _fit_step: 0,
            _sample_buffer: None,
            _fitness: 0.0,
            _initial_fitness: 0.0,
            _sphere_lambda: 0.0,
            _ellipsoid_lambda: 0.0,
            _samples_collected: 0,
            _samples_thinned: 0,
        };
        c._tolerance = c.COMPASS_CAL_DEFAULT_TOLERANCE;
        c._sample_buffer = None;
        c.clear();
        c
    }

    // ------------------------------------------------------------ PUBLIC INTERFACE

    pub fn clear(&mut self) {
        self.set_status(S::COMPASS_CAL_NOT_STARTED);
    }

    pub fn start(&mut self, retry: bool, delay: f32, offset_max: u16) {
        if self.running() {
            return;
        }
        self._offset_max = offset_max;
        self._attempt = 1;
        self._retry = retry;
        self._delay_start_sec = delay;
        self._start_time_ms = AP_HAL::millis();
        self.set_status(S::COMPASS_CAL_WAITING_TO_START);
    }

    /// `get_calibration(ref offsets, ref diagonals, ref offdiagonals)`: unchanged unless the
    /// calibration succeeded.
    pub fn get_calibration(&self, offsets: &mut Vector3f, diagonals: &mut Vector3f, offdiagonals: &mut Vector3f) {
        if self._status != S::COMPASS_CAL_SUCCESS {
            return;
        }
        *offsets = self._params.offset();
        *diagonals = self._params.diag();
        *offdiagonals = self._params.offdiag();
    }

    pub fn get_completion_percent(&self) -> f32 {
        // first sampling step is 1/3rd of the progress bar
        // never return more than 99% unless _status is COMPASS_CAL_SUCCESS
        match self._status {
            S::COMPASS_CAL_NOT_STARTED | S::COMPASS_CAL_WAITING_TO_START => 0.0,
            S::COMPASS_CAL_RUNNING_STEP_ONE => 33.3 * self._samples_collected as f32 / self.COMPASS_CAL_NUM_SAMPLES as f32,
            S::COMPASS_CAL_RUNNING_STEP_TWO => {
                33.3 + 65.7 * ((self._samples_collected as i32 - self._samples_thinned as i32) as f32 / (self.COMPASS_CAL_NUM_SAMPLES as i32 - self._samples_thinned as i32) as f32)
            }
            S::COMPASS_CAL_SUCCESS => 100.0,
            _ => 0.0,
        }
    }

    fn softiron(p: &param_t) -> Matrix3 {
        let (d, o) = (p.diag(), p.offdiag());
        Matrix3::from_floats(d.x, o.x, o.y, o.x, d.y, o.z, o.y, o.z, d.z)
    }

    /// `update_completion_mask(Vector3f v)`
    pub fn update_completion_mask_v(&mut self, v: Vector3f) {
        let softiron = Self::softiron(&self._params);
        // Matrix3 * Vector3 (double), assigned to a Vector3f, passed back as a Vector3
        let corrected = (softiron * (v + self._params.offset()).to_vector3()).to_vector3f();
        let section = AP_GeodesicGrid::section(corrected.to_vector3(), true);
        if section < 0 {
            return;
        }
        self._completion_mask[(section / 8) as usize] |= (1 << (section % 8)) as u8;
    }

    /// `update_completion_mask()`
    pub fn update_completion_mask(&mut self) {
        Utils::memset(&mut self._completion_mask, 0, 10);
        for i in 0..self._samples_collected as usize {
            let s = self._sample_buffer.as_ref().expect("sample buffer").get(i).get();
            self.update_completion_mask_v(s);
        }
    }

    pub fn get_completion_mask(&self) -> &[u8; 10] {
        &self._completion_mask
    }

    pub fn check_for_timeout(&mut self) -> bool {
        let tnow = AP_HAL::millis();
        if self.running() && tnow.wrapping_sub(self._last_sample_ms) > 1000 {
            self._retry = false;
            self.set_status(S::COMPASS_CAL_FAILED);
            return true;
        }
        false
    }

    pub fn new_sample(&mut self, sample: Vector3f) {
        self._last_sample_ms = AP_HAL::millis();
        if self._status == S::COMPASS_CAL_WAITING_TO_START {
            self.set_status(S::COMPASS_CAL_RUNNING_STEP_ONE);
        }
        if self.running() && self._samples_collected < self.COMPASS_CAL_NUM_SAMPLES && self.accept_sample(sample) {
            self.update_completion_mask_v(sample);
            let i = self._samples_collected as usize;
            self._sample_buffer.as_mut().expect("sample buffer").get_mut(i).set(sample);
            self._samples_collected += 1;
        }
    }

    /// `update(ref bool failure)`
    pub fn update(&mut self, failure: &mut bool) -> Result<(), Exception> {
        *failure = false;
        if !self.fitting() {
            return Ok(());
        }
        if self._status == S::COMPASS_CAL_RUNNING_STEP_ONE {
            if self._fit_step >= 10 {
                if Utils::is_equal(self._fitness, self._initial_fitness) || Utils::isnan(self._fitness) {
                    // if true, means that fitness is diverging instead of converging
                    self.set_status(S::COMPASS_CAL_FAILED);
                    *failure = true;
                }
                self.set_status(S::COMPASS_CAL_RUNNING_STEP_TWO);
            } else {
                if self._fit_step == 0 {
                    self.calc_initial_offset();
                }
                self.run_sphere_fit()?;
                self._fit_step += 1;
            }
        } else if self._status == S::COMPASS_CAL_RUNNING_STEP_TWO {
            if self._fit_step >= 35 {
                if self.fit_acceptable() {
                    self.set_status(S::COMPASS_CAL_SUCCESS);
                } else {
                    self.set_status(S::COMPASS_CAL_FAILED);
                    *failure = true;
                }
            } else if self._fit_step < 15 {
                self.run_sphere_fit()?;
                self._fit_step += 1;
            } else {
                self.run_ellipsoid_fit()?;
                self._fit_step += 1;
            }
        }
        Ok(())
    }

    pub fn status(&self) -> compass_cal_status_t {
        self._status
    }

    // ------------------------------------------------------------ PRIVATE METHODS

    fn running(&self) -> bool {
        self._status == S::COMPASS_CAL_RUNNING_STEP_ONE || self._status == S::COMPASS_CAL_RUNNING_STEP_TWO
    }

    fn fitting(&self) -> bool {
        self.running() && self._samples_collected == self.COMPASS_CAL_NUM_SAMPLES
    }

    fn initialize_fit(&mut self) {
        // initialize _fitness before starting a fit
        if self._samples_collected != 0 {
            self._fitness = self.calc_mean_squared_residuals(&self._params);
        } else {
            self._fitness = 1.0e30;
        }
        self._ellipsoid_lambda = 1.0;
        self._sphere_lambda = 1.0;
        self._initial_fitness = self._fitness;
        self._fit_step = 0;
    }

    fn reset_state(&mut self) {
        self._samples_collected = 0;
        self._samples_thinned = 0;
        self._params.set_radius(200.0);
        // _params.offset.zero(): zeroes the copy the getter returned
        let _ = self._params.offset();
        self._params.set_diag(Vector3f::new(1.0, 1.0, 1.0));
        // _params.offdiag.zero(): likewise
        let _ = self._params.offdiag();
        Utils::memset(&mut self._completion_mask, 0, 10);
        self.initialize_fit();
    }

    fn set_status(&mut self, status: compass_cal_status_t) -> bool {
        if status != S::COMPASS_CAL_NOT_STARTED && self._status == status {
            return true;
        }
        match status {
            S::COMPASS_CAL_NOT_STARTED => {
                self.reset_state();
                self._status = S::COMPASS_CAL_NOT_STARTED;
                self._sample_buffer = None;
                true
            }
            S::COMPASS_CAL_WAITING_TO_START => {
                self.reset_state();
                self._status = S::COMPASS_CAL_WAITING_TO_START;
                self.set_status(S::COMPASS_CAL_RUNNING_STEP_ONE);
                true
            }
            S::COMPASS_CAL_RUNNING_STEP_ONE => {
                if self._status != S::COMPASS_CAL_WAITING_TO_START {
                    return false;
                }
                if self._attempt == 1 && (AP_HAL::millis().wrapping_sub(self._start_time_ms)) as f32 * 1.0e-3 < self._delay_start_sec {
                    return false;
                }
                if self._sample_buffer.is_none() {
                    let n = self.COMPASS_CAL_NUM_SAMPLES as usize;
                    self._sample_buffer = Some(SampleBuffer { arena: vec![CompassSample::default(); n], slots: (0..n).collect() });
                }
                if self._sample_buffer.is_some() {
                    self.initialize_fit();
                    self._status = S::COMPASS_CAL_RUNNING_STEP_ONE;
                    return true;
                }
                false
            }
            S::COMPASS_CAL_RUNNING_STEP_TWO => {
                if self._status != S::COMPASS_CAL_RUNNING_STEP_ONE {
                    return false;
                }
                self.thin_samples();
                self.initialize_fit();
                self._status = S::COMPASS_CAL_RUNNING_STEP_TWO;
                true
            }
            S::COMPASS_CAL_SUCCESS => {
                if self._status != S::COMPASS_CAL_RUNNING_STEP_TWO {
                    return false;
                }
                self._sample_buffer = None;
                self._status = S::COMPASS_CAL_SUCCESS;
                true
            }
            S::COMPASS_CAL_FAILED => {
                if self._status == S::COMPASS_CAL_NOT_STARTED {
                    return false;
                }
                if self._retry && self.set_status(S::COMPASS_CAL_WAITING_TO_START) {
                    self._attempt = self._attempt.wrapping_add(1);
                    return true;
                }
                self._sample_buffer = None;
                self._status = S::COMPASS_CAL_FAILED;
                true
            }
            _ => false,
        }
    }

    fn fit_acceptable(&self) -> bool {
        let p = &self._params;
        let (o, d, od) = (p.offset(), p.diag(), p.offdiag());
        let om = self._offset_max as f32;
        if !Utils::isnan(self._fitness)
            && p.radius() > 150.0
            && p.radius() < 950.0 // Earth's magnetic field strength range: 250-850mG
            && Utils::fabsf(o.x as f64) < om
            && Utils::fabsf(o.y as f64) < om
            && Utils::fabsf(o.z as f64) < om
            && d.x > 0.2
            && d.x < 5.0
            && d.y > 0.2
            && d.y < 5.0
            && d.z > 0.2
            && d.z < 5.0
            && Utils::fabsf(od.x as f64) < 1.0 // absolute of sine/cosine output cannot be greater than 1
            && Utils::fabsf(od.y as f64) < 1.0
            && Utils::fabsf(od.z as f64) < 1.0
        {
            return self._fitness <= Utils::sqf(self._tolerance);
        }
        false
    }

    fn thin_samples(&mut self) {
        if self._sample_buffer.is_none() {
            return;
        }
        if self._samples_collected == 0 {
            return;
        }
        self._samples_thinned = 0;
        // shuffle the samples http://en.wikipedia.org/wiki/Fisher%E2%80%93Yates_shuffle
        // this is so that adjacent samples don't get sequentially eliminated
        let mut i = self._samples_collected - 1;
        while i >= 1 {
            let j = (Self::get_random16() as u32 % (i as u32 + 1)) as usize;
            let buf = self._sample_buffer.as_mut().unwrap();
            buf.slots.swap(i as usize, j);
            i -= 1;
        }
        let mut i: u16 = 0;
        while i < self._samples_collected {
            let s = *self._sample_buffer.as_ref().unwrap().get(i as usize);
            if !self.accept_sample(s.get()) {
                let buf = self._sample_buffer.as_mut().unwrap();
                buf.slots[i as usize] = buf.slots[(self._samples_collected - 1) as usize];
                self._samples_collected -= 1;
                self._samples_thinned += 1;
            }
            i += 1;
        }
        self.update_completion_mask();
    }

    /// `accept_sample(Vector3f sample)`: far enough from every collected sample (this
    /// includes the sample itself when it is already in the buffer).
    fn accept_sample(&self, sample: Vector3f) -> bool {
        let faces = (2 * self.COMPASS_CAL_NUM_SAMPLES as i32 - 4) as u16;
        let a = (4.0f32 * Utils::PI / (3.0 * faces as f32)) + Utils::PI / 3.0;
        let theta = 0.5f32 * Utils::acosf(Utils::cosf(a as f64) / (1.0 - Utils::cosf(a as f64)));
        let Some(buf) = self._sample_buffer.as_ref() else { return false };
        let min_distance = self._params.radius() * 2.0 * Utils::sinf((theta / 2.0) as f64);
        for i in 0..self._samples_collected as usize {
            let distance = (sample - buf.get(i).get()).length();
            if distance < min_distance {
                return false;
            }
        }
        true
    }

    fn calc_residual(sample: Vector3f, params: &param_t) -> f32 {
        let softiron = Self::softiron(params);
        (params.radius() as f64 - (softiron * (sample + params.offset()).to_vector3()).length()) as f32
    }

    fn calc_mean_squared_residuals(&self, params: &param_t) -> f32 {
        let Some(buf) = self._sample_buffer.as_ref() else { return 1.0e30 };
        if self._samples_collected == 0 {
            return 1.0e30;
        }
        let mut sum = 0.0f32;
        for i in 0..self._samples_collected as usize {
            let sample = buf.get(i).get();
            let resid = Self::calc_residual(sample, params);
            sum += Utils::sqf(resid);
        }
        sum /= self._samples_collected as f32;
        sum
    }

    fn calc_sphere_jacob(sample: Vector3f, params: &param_t, ret: &mut [f32]) {
        let (offset, diag, offdiag) = (params.offset(), params.diag(), params.offdiag());
        let softiron = Self::softiron(params);
        let A = (diag.x * (sample.x + offset.x)) + (offdiag.x * (sample.y + offset.y)) + (offdiag.y * (sample.z + offset.z));
        let B = (offdiag.x * (sample.x + offset.x)) + (diag.y * (sample.y + offset.y)) + (offdiag.z * (sample.z + offset.z));
        let C = (offdiag.y * (sample.x + offset.x)) + (offdiag.z * (sample.y + offset.y)) + (diag.z * (sample.z + offset.z));
        let length = (softiron * (sample + offset).to_vector3()).length() as f32;
        // 0: partial derivative (radius wrt fitness fn) fn operated on sample
        ret[0] = 1.0;
        // 1-3: partial derivative (offsets wrt fitness fn) fn operated on sample
        ret[1] = -1.0 * (((diag.x * A) + (offdiag.x * B) + (offdiag.y * C)) / length);
        ret[2] = -1.0 * (((offdiag.x * A) + (diag.y * B) + (offdiag.z * C)) / length);
        ret[3] = -1.0 * (((offdiag.y * A) + (offdiag.z * B) + (diag.z * C)) / length);
    }

    fn calc_initial_offset(&mut self) {
        // Set initial offset to the average value of the samples; offset.zero() acts on a
        // copy, so the sum starts from the current offset
        let _ = self._params.offset();
        for k in 0..self._samples_collected as usize {
            let s = self._sample_buffer.as_ref().expect("sample buffer").get(k).get();
            let o = self._params.offset() - s;
            self._params.set_offset(o);
        }
        let o = self._params.offset() / self._samples_collected as f64;
        self._params.set_offset(o);
    }

    /// One Levenberg-Marquardt step over `n` parameters starting at `data[first]`, with
    /// `fit1_params`, `fit2_params` and `_params` the same object.
    fn lm_step(&mut self, n: usize, first: usize, ellipsoid: bool) -> Result<(), Exception> {
        if self._sample_buffer.is_none() {
            return Ok(());
        }
        let lma_damping = 10.0f32;
        let mut fitness = self._fitness;
        let mut JTJ = vec![0f32; n * n];
        let mut JTJ2 = vec![0f32; n * n];
        let mut JTFI = vec![0f32; n];

        // Gauss Newton Part common for all kind of extensions including LM
        for k in 0..self._samples_collected as usize {
            let sample = self._sample_buffer.as_ref().unwrap().get(k).get();
            let mut jacob = vec![0f32; n];
            if ellipsoid {
                Self::calc_ellipsoid_jacob(sample, &self._params, &mut jacob);
            } else {
                Self::calc_sphere_jacob(sample, &self._params, &mut jacob);
            }
            for i in 0..n {
                // compute JTJ
                for j in 0..n {
                    JTJ[i * n + j] += jacob[i] * jacob[j];
                    JTJ2[i * n + j] += jacob[i] * jacob[j]; // a backup JTJ for LM
                }
                // compute JTFI
                JTFI[i] += jacob[i] * Self::calc_residual(sample, &self._params);
            }
        }

        // Levenberg-Marquardt part
        let lambda = if ellipsoid { self._ellipsoid_lambda } else { self._sphere_lambda };
        for i in 0..n {
            JTJ[i * n + i] += lambda;
            JTJ2[i * n + i] += lambda / lma_damping;
        }
        let x = JTJ.clone();
        if !Utils::inverse(&x, &mut JTJ, n as u16)? {
            return Ok(());
        }
        let x = JTJ2.clone();
        if !Utils::inverse(&x, &mut JTJ2, n as u16)? {
            return Ok(());
        }
        for row in 0..n {
            for col in 0..n {
                self._params.data[first + row] -= JTFI[col] * JTJ[row * n + col];
                self._params.data[first + row] -= JTFI[col] * JTJ2[row * n + col];
            }
        }
        let fit1 = self.calc_mean_squared_residuals(&self._params);
        let fit2 = self.calc_mean_squared_residuals(&self._params);
        console_write_line(&format!("{} {} {}", self._params.to_json(), float_to_string(fit1), float_to_string(fit2)));

        let lambda = if ellipsoid { &mut self._ellipsoid_lambda } else { &mut self._sphere_lambda };
        if fit1 > self._fitness && fit2 > self._fitness {
            *lambda *= lma_damping;
        } else if fit2 < self._fitness && fit2 < fit1 {
            *lambda /= lma_damping;
            fitness = fit2;
        } else if fit1 < self._fitness {
            fitness = fit1;
        }

        let accept = if ellipsoid { fitness < self._fitness } else { !Utils::isnan(fitness) && fitness < self._fitness };
        if accept {
            self._fitness = fitness;
            self.update_completion_mask();
        }
        Ok(())
    }

    fn run_sphere_fit(&mut self) -> Result<(), Exception> {
        self.lm_step(self.COMPASS_CAL_NUM_SPHERE_PARAMS, 0, false)
    }

    fn calc_ellipsoid_jacob(sample: Vector3f, params: &param_t, ret: &mut [f32]) {
        let (offset, diag, offdiag) = (params.offset(), params.diag(), params.offdiag());
        let softiron = Self::softiron(params);
        let A = (diag.x * (sample.x + offset.x)) + (offdiag.x * (sample.y + offset.y)) + (offdiag.y * (sample.z + offset.z));
        let B = (offdiag.x * (sample.x + offset.x)) + (diag.y * (sample.y + offset.y)) + (offdiag.z * (sample.z + offset.z));
        let C = (offdiag.y * (sample.x + offset.x)) + (offdiag.z * (sample.y + offset.y)) + (diag.z * (sample.z + offset.z));
        let length = (softiron * (sample + offset).to_vector3()).length() as f32;
        // 0-2: partial derivative (offset wrt fitness fn) fn operated on sample
        ret[0] = -1.0 * (((diag.x * A) + (offdiag.x * B) + (offdiag.y * C)) / length);
        ret[1] = -1.0 * (((offdiag.x * A) + (diag.y * B) + (offdiag.z * C)) / length);
        ret[2] = -1.0 * (((offdiag.y * A) + (offdiag.z * B) + (diag.z * C)) / length);
        // 3-5: partial derivative (diag offset wrt fitness fn) fn operated on sample
        ret[3] = -1.0 * ((sample.x + offset.x) * A) / length;
        ret[4] = -1.0 * ((sample.y + offset.y) * B) / length;
        ret[5] = -1.0 * ((sample.z + offset.z) * C) / length;
        // 6-8: partial derivative (off-diag offset wrt fitness fn) fn operated on sample
        ret[6] = -1.0 * (((sample.y + offset.y) * A) + ((sample.x + offset.x) * B)) / length;
        ret[7] = -1.0 * (((sample.z + offset.z) * A) + ((sample.x + offset.x) * C)) / length;
        ret[8] = -1.0 * (((sample.z + offset.z) * B) + ((sample.y + offset.y) * C)) / length;
    }

    fn run_ellipsoid_fit(&mut self) -> Result<(), Exception> {
        self.lm_step(self.COMPASS_CAL_NUM_ELLIPSOID_PARAMS, 1, true)
    }

    /// simple 16 bit random number generator
    fn get_random16() -> u16 {
        let mut r = RANDOM.lock().unwrap();
        r.0 = 36969u32.wrapping_mul(r.0 & 0xFFFF).wrapping_add(r.0 >> 16);
        r.1 = 18000u32.wrapping_mul(r.1 & 0xFFFF).wrapping_add(r.1 >> 16);
        ((r.0 << 16).wrapping_add(r.1) & 0xFFFF) as u16
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sample_fixed_point_and_status_flow() {
        let mut s = CompassSample::default();
        s.set(Vector3f::new(1.06, -2.5, 5000.0));
        assert_eq!((s.x, s.y, s.z), (8, -20, i16::MAX));
        assert_eq!(s.get().x, 1.0);

        let mut c = CompassCalibrator::new();
        assert_eq!(c.status(), S::COMPASS_CAL_NOT_STARTED);
        c.start(false, 0.0, 1000);
        assert_eq!(c.status(), S::COMPASS_CAL_RUNNING_STEP_ONE);
        // the first sample is always accepted; the same sample again is too close
        c.new_sample(Vector3f::new(100.0, 0.0, 0.0));
        c.new_sample(Vector3f::new(100.0, 0.0, 0.0));
        assert_eq!(c._samples_collected, 1);
        assert!(c.get_completion_percent() > 0.0);
        let mut failure = true;
        c.update(&mut failure).unwrap();
        assert!(!failure, "not fitting until 300 samples");
    }

    #[test]
    fn param_t_spans_share_the_array() {
        let mut p = param_t::default();
        p.get_ellipsoid_params()[0] = 3.0;
        assert_eq!(p.get_sphere_params()[1], 3.0);
        assert_eq!(p.offset().x, 3.0);
    }
}

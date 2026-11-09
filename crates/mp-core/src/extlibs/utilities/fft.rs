//! Port of `ExtLibs/Utilities/fft.cs`
//!
//! Gerald T. Beauregard's in-place complex FFT (MIT licence). The C# keeps its complex values
//! in a linked list of `FFTElement`s; here the list is a vector whose `next` is the following
//! index (the last element has none), walked in the same order.

#![allow(non_snake_case, non_camel_case_types)]

use crate::dotnet::{index_out_of_range, Exception};

/// `public class FFT2.datastate`
#[derive(Debug, Clone, Default, PartialEq)]
pub struct datastate {
    pub r#type: Option<String>,
    pub timedelta: f64,
    pub lasttime: f64,
    pub sample_rate: f64,
    pub datax: Vec<f64>,
    pub datay: Vec<f64>,
    pub dataz: Vec<f64>,

    pub avgx: Option<Vec<f64>>,
    pub avgy: Option<Vec<f64>>,
    pub avgz: Option<Vec<f64>>,
}

/// `class FFTElement`: one element of the linked list holding the input/output data.
#[derive(Debug, Clone, Default)]
struct FFTElement {
    /// Real component
    re: f64,
    /// Imaginary component
    im: f64,
    /// Next element in linked list (`null` for the last)
    next: Option<usize>,
    /// Target position post bit-reversal
    revTgt: u32,
}

/// `public class FFT2`
#[derive(Debug, Clone, Default)]
pub struct FFT2 {
    /// log2 of FFT size
    m_logN: u32,
    /// FFT size
    m_N: u32,
    /// Vector of linked list elements (`null` until `init`)
    m_X: Option<Vec<FFTElement>>,
}

fn null_reference() -> Exception {
    Exception::NullReference("Object reference not set to an instance of an object.".into())
}

fn at(v: &[f64], i: u32) -> Result<f64, Exception> {
    v.get(i as usize).copied().ok_or_else(index_out_of_range)
}

fn set(v: &mut [f64], i: u32, x: f64) -> Result<(), Exception> {
    *v.get_mut(i as usize).ok_or_else(index_out_of_range)? = x;
    Ok(())
}

impl FFT2 {
    /// `new FFT2()`
    pub fn new() -> Self {
        Self::default()
    }

    /// `init(uint logN)`: allocate the list for a 2^logN point FFT. `1 << (int)logN` uses
    /// the low five bits of the shift, as C# does.
    pub fn init(&mut self, logN: u32) {
        self.m_logN = logN;
        self.m_N = 1i32.wrapping_shl(self.m_logN) as u32;

        // Allocate elements for linked list of complex numbers.
        let mut x: Vec<FFTElement> = (0..self.m_N).map(|_| FFTElement::default()).collect();

        // Set up "next" pointers.
        let n = self.m_N as usize;
        for k in 0..n.saturating_sub(1) {
            x[k].next = Some(k + 1);
        }

        // Specify target for bit reversal re-ordering.
        for (k, e) in x.iter_mut().enumerate() {
            e.revTgt = Self::BitReverse(k as u32, logN);
        }
        self.m_X = Some(x);
    }

    /// `FreqTable(int samplecount, int samplerate)`: the frequency of each bin, worked out in
    /// int arithmetic (wrapping, truncating) before the widening to double.
    pub fn FreqTable(&self, samplecount: i32, samplerate: i32) -> Result<Vec<f64>, Exception> {
        let N = samplecount;
        let len = usize::try_from(N / 2).map_err(|_| Exception::Overflow("Arithmetic operation resulted in an overflow.".into()))?;

        // Vector with frequencies for each bin number. Used
        // in the graphing code (not in the analysis itself).
        let mut m_freq = vec![0.0; len];
        for (i, f) in m_freq.iter_mut().enumerate() {
            *f = ((i as i32).wrapping_mul(samplerate) / N) as f64;
        }

        Ok(m_freq)
    }

    /// `rin(Span<double> data, uint bins, bool outputLog = true)`: Hanning window, FFT of
    /// 2^bins points and the magnitude of the first N/2 bins, in dB when `outputLog`.
    pub fn rin(&mut self, data: &[f64], bins: u32, outputLog: bool) -> Result<Vec<f64>, Exception> {
        let SCALE = 20.0 / 10f64.ln();
        let N = data.len();
        let mut xRe = vec![0.0; N];
        let mut xIm = vec![0.0; N];
        let mut m_mag = vec![0.0; N / 2];
        let mut avg_sum = 0.0;

        self.init(bins); // 1024 = 1 << 10

        // Hanning analysis window
        let mut m_win = vec![0.0; N];
        for (i, w) in m_win.iter_mut().enumerate() {
            *w = (4.0 / N as f64) * 0.5 * (1.0 - (2.0 * std::f64::consts::PI * i as f64 / N as f64).cos());
        }

        // create Real and apply hanning window
        for a in 0..N {
            avg_sum += data[a];
            xRe[a] = m_win[a] * data[a];
            xIm[a] = 0.0;
        }

        let _avg = avg_sum / N as f64;

        // do fft
        self.run(&mut xRe, &mut xIm, false)?;

        // Convert from Decibel to Magnitude
        for i in 0..N / 2 {
            let re = xRe[i]; // get the Real FFT Number at position i
            let im = xIm[i]; // get the Imaginary FFT Number at position i

            m_mag[i] = (re * re + im * im).sqrt(); // Convert magnitude to decibels
            if outputLog {
                // double.Epsilon is the smallest subnormal, not the machine epsilon
                m_mag[i] = SCALE * (m_mag[i] + f64::from_bits(1)).ln();
            }
        }

        Ok(m_mag)
    }

    /// `run(double[] xRe, double[] xIm, bool inverse = false)`: in-place complex FFT of the
    /// first N values. Shorter arrays throw IndexOutOfRangeException; `init` must come first.
    pub fn run(&mut self, xRe: &mut [f64], xIm: &mut [f64], inverse: bool) -> Result<(), Exception> {
        let m_N = self.m_N;
        let m_logN = self.m_logN;
        let m_X = self.m_X.as_mut().ok_or_else(null_reference)?;

        let mut numFlies = m_N >> 1; // Number of butterflies per sub-FFT
        let mut span = m_N >> 1; // Width of the butterfly
        let mut spacing = m_N; // Distance between start of sub-FFTs
        let mut wIndexStep: u32 = 1; // Increment for twiddle table index

        // Copy data into linked complex number objects
        // If it's an IFFT, we divide by N while we're at it
        let mut x = if m_X.is_empty() { return Err(index_out_of_range()) } else { Some(0usize) };
        let mut k: u32 = 0;
        let scale = if inverse { 1.0 / m_N as f64 } else { 1.0 };
        while let Some(i) = x {
            m_X[i].re = scale * at(xRe, k)?;
            m_X[i].im = scale * at(xIm, k)?;
            x = m_X[i].next;
            k = k.wrapping_add(1);
        }

        // For each stage of the FFT
        for _stage in 0..m_logN {
            // Compute a multiplier factor for the "twiddle factors".
            let mut wAngleInc = wIndexStep as f64 * 2.0 * std::f64::consts::PI / m_N as f64;
            if !inverse {
                wAngleInc *= -1.0;
            }
            let wMulRe = wAngleInc.cos();
            let wMulIm = wAngleInc.sin();

            let mut start: u32 = 0;
            while start < m_N {
                let mut xTop = Some(start as usize);
                let mut xBot = Some(start.wrapping_add(span) as usize);
                if xBot.is_some_and(|b| b >= m_X.len()) {
                    return Err(index_out_of_range());
                }

                let mut wRe = 1.0;
                let mut wIm = 0.0;

                // For each butterfly in this stage
                for _flyCount in 0..numFlies {
                    let (t, b) = (xTop.ok_or_else(null_reference)?, xBot.ok_or_else(null_reference)?);
                    // Get the top & bottom values
                    let xTopRe = m_X[t].re;
                    let xTopIm = m_X[t].im;
                    let mut xBotRe = m_X[b].re;
                    let mut xBotIm = m_X[b].im;

                    // Top branch of butterfly has addition
                    m_X[t].re = xTopRe + xBotRe;
                    m_X[t].im = xTopIm + xBotIm;

                    // Bottom branch of butterly has subtraction,
                    // followed by multiplication by twiddle factor
                    xBotRe = xTopRe - xBotRe;
                    xBotIm = xTopIm - xBotIm;
                    m_X[b].re = xBotRe * wRe - xBotIm * wIm;
                    m_X[b].im = xBotRe * wIm + xBotIm * wRe;

                    // Advance butterfly to next top & bottom positions
                    xTop = m_X[t].next;
                    xBot = m_X[b].next;

                    // Update the twiddle factor, via complex multiply
                    // by unit vector with the appropriate angle
                    let tRe = wRe;
                    wRe = wRe * wMulRe - wIm * wMulIm;
                    wIm = tRe * wMulIm + wIm * wMulRe;
                }
                start = match start.checked_add(spacing) {
                    Some(s) => s,
                    None => break, // uint wrap ends the C# loop only by chance; spacing >= 1 here
                };
            }

            numFlies >>= 1; // Divide by 2 by right shift
            span >>= 1;
            spacing >>= 1;
            wIndexStep = wIndexStep.wrapping_shl(1); // Multiply by 2 by left shift
        }

        // The algorithm leaves the result in a scrambled order.
        // Unscramble while copying values from the complex
        // linked list elements back to the input/output vectors.
        let mut x = Some(0usize);
        while let Some(i) = x {
            let target = m_X[i].revTgt;
            set(xRe, target, m_X[i].re)?;
            set(xIm, target, m_X[i].im)?;
            x = m_X[i].next;
        }
        Ok(())
    }

    /// `BitReverse(uint x, uint numBits)`: e.g. 1101 bit-reversed is 1011.
    fn BitReverse(mut x: u32, numBits: u32) -> u32 {
        let mut y: u32 = 0;
        for _ in 0..numBits {
            y = y.wrapping_shl(1);
            y |= x & 0x0001;
            x = x.wrapping_shr(1);
        }
        y
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sine_peaks_in_its_bin_and_freq_table_truncates() {
        let n = 64usize;
        let data: Vec<f64> = (0..n).map(|i| (2.0 * std::f64::consts::PI * 8.0 * i as f64 / n as f64).sin()).collect();
        let mut f = FFT2::new();
        let mag = f.rin(&data, 6, false).unwrap();
        assert_eq!(mag.len(), 32);
        let peak = mag.iter().enumerate().max_by(|a, b| a.1.total_cmp(b.1)).unwrap().0;
        assert_eq!(peak, 8);
        // Hanning window scaled by 4/N: a unit sine reads 1.0 at its bin
        assert!((mag[8] - 1.0).abs() < 1e-9, "{}", mag[8]);
        assert_eq!(f.FreqTable(10, 7).unwrap(), vec![0.0, 0.0, 1.0, 2.0, 2.0]);
        // shorter input than the FFT size throws
        assert!(f.rin(&data[..10], 6, true).is_err());
        assert!(FFT2::new().run(&mut [0.0], &mut [0.0], false).is_err());
    }

    #[test]
    fn inverse_round_trips() {
        let mut f = FFT2::new();
        f.init(3);
        let mut re = vec![1.0, 2.0, 3.0, 4.0, 0.0, -1.0, 0.5, 2.0];
        let mut im = vec![0.0; 8];
        let orig = re.clone();
        f.run(&mut re, &mut im, false).unwrap();
        f.run(&mut re, &mut im, true).unwrap();
        for (a, b) in re.iter().zip(&orig) {
            assert!((a - b).abs() < 1e-12);
        }
    }
}

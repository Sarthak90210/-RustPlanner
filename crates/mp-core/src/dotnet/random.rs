//! `System.Random` (.NET Framework 4.x): Knuth's subtractive generator, seeded exactly as the
//! reference source does, so a given seed yields the same sequence as in C#. The `int`
//! arithmetic is unchecked in C# and wraps here.

use super::Exception;

/// `Environment.TickCount`: milliseconds on a monotonic clock, wrapped to `int`. The OS
/// boot-time origin is not available without another `windows` feature, so the origin here is
/// the first call; only differences between readings carry meaning in the ported code.
pub fn tick_count() -> i32 {
    static START: std::sync::OnceLock<std::time::Instant> = std::sync::OnceLock::new();
    let start = START.get_or_init(std::time::Instant::now);
    start.elapsed().as_millis() as u64 as u32 as i32
}

const MBIG: i32 = i32::MAX;
const MSEED: i32 = 161_803_398;

/// `public class Random`
#[derive(Debug, Clone)]
pub struct Random {
    inext: usize,
    inextp: usize,
    seed_array: [i32; 56],
}

impl Random {
    /// `new Random(int Seed)`
    pub fn new(seed: i32) -> Self {
        let mut seed_array = [0i32; 56];
        let subtraction = if seed == i32::MIN { i32::MAX } else { seed.abs() };
        let mut mj = MSEED.wrapping_sub(subtraction);
        seed_array[55] = mj;
        let mut mk = 1i32;
        for i in 1..55 {
            let ii = (21 * i) % 55;
            seed_array[ii] = mk;
            mk = mj.wrapping_sub(mk);
            if mk < 0 {
                mk = mk.wrapping_add(MBIG);
            }
            mj = seed_array[ii];
        }
        for _ in 1..5 {
            for i in 1..56 {
                seed_array[i] = seed_array[i].wrapping_sub(seed_array[1 + (i + 30) % 55]);
                if seed_array[i] < 0 {
                    seed_array[i] = seed_array[i].wrapping_add(MBIG);
                }
            }
        }
        Random { inext: 0, inextp: 21, seed_array }
    }

    /// `new Random()`: seeded with `Environment.TickCount`.
    pub fn new_default() -> Self {
        Random::new(tick_count())
    }

    fn internal_sample(&mut self) -> i32 {
        let mut loc_inext = self.inext + 1;
        if loc_inext >= 56 {
            loc_inext = 1;
        }
        let mut loc_inextp = self.inextp + 1;
        if loc_inextp >= 56 {
            loc_inextp = 1;
        }
        let mut ret_val = self.seed_array[loc_inext].wrapping_sub(self.seed_array[loc_inextp]);
        if ret_val == MBIG {
            ret_val -= 1;
        }
        if ret_val < 0 {
            ret_val = ret_val.wrapping_add(MBIG);
        }
        self.seed_array[loc_inext] = ret_val;
        self.inext = loc_inext;
        self.inextp = loc_inextp;
        ret_val
    }

    fn sample(&mut self) -> f64 {
        self.internal_sample() as f64 * (1.0 / MBIG as f64)
    }

    fn get_sample_for_large_range(&mut self) -> f64 {
        let mut result = self.internal_sample();
        let negative = self.internal_sample() % 2 == 0;
        if negative {
            result = -result;
        }
        let mut d = result as f64;
        d += (i32::MAX - 1) as f64;
        d /= 2.0 * (i32::MAX as u32) as f64 - 1.0;
        d
    }

    /// `Next()`
    pub fn next(&mut self) -> i32 {
        self.internal_sample()
    }

    /// `Next(int maxValue)`
    pub fn next_max(&mut self, max_value: i32) -> Result<i32, Exception> {
        if max_value < 0 {
            return Err(Exception::ArgumentOutOfRange(
                "'maxValue' must be greater than zero.\r\nParameter name: maxValue".into(),
            ));
        }
        Ok((self.sample() * max_value as f64) as i32)
    }

    /// `Next(int minValue, int maxValue)`
    pub fn next_range(&mut self, min_value: i32, max_value: i32) -> Result<i32, Exception> {
        if min_value > max_value {
            return Err(Exception::ArgumentOutOfRange(
                "'minValue' cannot be greater than maxValue.\r\nParameter name: minValue".into(),
            ));
        }
        let range = max_value as i64 - min_value as i64;
        if range <= i32::MAX as i64 {
            Ok(((self.sample() * range as f64) as i32).wrapping_add(min_value))
        } else {
            Ok(((self.get_sample_for_large_range() * range as f64) as i64 + min_value as i64) as i32)
        }
    }

    /// `NextDouble()`
    pub fn next_double(&mut self) -> f64 {
        self.sample()
    }

    /// `NextBytes(byte[] buffer)`
    pub fn next_bytes(&mut self, buffer: &mut [u8]) {
        for b in buffer.iter_mut() {
            *b = (self.internal_sample() % 256) as u8;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matches_net_framework_sequence() {
        // new Random(0).Next() x3 and new Random(42).NextDouble() on .NET Framework 4.8
        let mut r = Random::new(0);
        assert_eq!(r.next(), 1559595546);
        assert_eq!(r.next(), 1755192844);
        assert_eq!(r.next(), 1649316166);
        // a seed above MSEED makes the first entry negative; arithmetic wraps
        let _ = Random::new(i32::MAX).next();
        let _ = Random::new(i32::MIN).next_double();
        assert!(Random::new(1).next_max(-1).is_err());
        assert!(Random::new(1).next_range(2, 1).is_err());
    }
}

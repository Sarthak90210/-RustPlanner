//! `System.Random` as .NET Framework implements it (Knuth's subtractive generator), so a
//! seeded sequence matches the C# one value for value. `new Random()` seeds from
//! `Environment.TickCount`.

use super::Exception;

const MBIG: i32 = i32::MAX;
const MSEED: i32 = 161803398;

/// `System.Random`
#[derive(Debug, Clone)]
pub struct Random {
    inext: usize,
    inextp: usize,
    seed_array: [i32; 56],
}

/// `Environment.TickCount`: milliseconds since the process's clock origin, as an `int` that
/// wraps.
pub fn environment_tick_count() -> i32 {
    use std::sync::OnceLock;
    static START: OnceLock<std::time::Instant> = OnceLock::new();
    let start = START.get_or_init(std::time::Instant::now);
    start.elapsed().as_millis() as u64 as i32
}

impl Random {
    /// `new Random()`
    pub fn new() -> Self {
        Random::with_seed(environment_tick_count())
    }

    /// `new Random(int Seed)`
    pub fn with_seed(seed: i32) -> Self {
        let mut seed_array = [0i32; 56];
        let subtraction = if seed == i32::MIN { i32::MAX } else { seed.abs() };
        let mut mj = MSEED - subtraction;
        seed_array[55] = mj;
        let mut mk: i32 = 1;
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

    fn internal_sample(&mut self) -> i32 {
        let mut loc_inext = self.inext + 1;
        if loc_inext >= 56 {
            loc_inext = 1;
        }
        let mut loc_inextp = self.inextp + 1;
        if loc_inextp >= 56 {
            loc_inextp = 1;
        }
        let mut ret = self.seed_array[loc_inext].wrapping_sub(self.seed_array[loc_inextp]);
        if ret == MBIG {
            ret -= 1;
        }
        if ret < 0 {
            ret = ret.wrapping_add(MBIG);
        }
        self.seed_array[loc_inext] = ret;
        self.inext = loc_inext;
        self.inextp = loc_inextp;
        ret
    }

    fn sample(&mut self) -> f64 {
        self.internal_sample() as f64 * (1.0 / MBIG as f64)
    }

    /// `Next()`
    pub fn next_int(&mut self) -> i32 {
        self.internal_sample()
    }

    /// `Next(int maxValue)`
    pub fn next_max(&mut self, max_value: i32) -> Result<i32, Exception> {
        if max_value < 0 {
            return Err(Exception::ArgumentOutOfRange("'maxValue' must be greater than zero.\r\nParameter name: maxValue".into()));
        }
        Ok((self.sample() * max_value as f64) as i32)
    }

    /// `Next(int minValue, int maxValue)`
    pub fn next_range(&mut self, min_value: i32, max_value: i32) -> Result<i32, Exception> {
        if min_value > max_value {
            return Err(Exception::ArgumentOutOfRange("'minValue' cannot be greater than maxValue.\r\nParameter name: minValue".into()));
        }
        let range = max_value as i64 - min_value as i64;
        if range <= i32::MAX as i64 {
            return Ok(((self.sample() * range as f64) as i32).wrapping_add(min_value));
        }
        Ok((((self.get_sample_for_large_range() * range as f64) as i64) + min_value as i64) as i32)
    }

    fn get_sample_for_large_range(&mut self) -> f64 {
        let mut result = self.internal_sample();
        let negative = self.internal_sample() % 2 == 0;
        if negative {
            result = -result;
        }
        let mut d = result as f64;
        d += (i32::MAX - 1) as f64;
        d /= 2.0 * i32::MAX as f64 - 1.0;
        d
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

impl Default for Random {
    fn default() -> Self {
        Random::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn seeded_sequence_matches_net_framework() {
        // new Random(0).Next() x3 on .NET Framework
        let mut r = Random::with_seed(0);
        assert_eq!(r.next_int(), 1559595546);
        assert_eq!(r.next_int(), 1755192844);
        assert_eq!(r.next_int(), 1649316166);
        let mut r = Random::with_seed(42);
        assert_eq!(r.next_max(100).unwrap(), 66);
        assert!(r.next_max(-1).is_err());
        assert!(r.next_range(5, 1).is_err());
    }
}

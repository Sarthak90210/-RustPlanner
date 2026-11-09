//! Port of `ExtLibs/Core/Utils/MathUtils.cs`

use crate::dotnet::convert::to_int32_f64;
use crate::dotnet::random::Random;
use crate::dotnet::{cast_i32, math_round, DateTime, Exception};

/// `public static class MathUtils`
pub struct MathUtils;

impl MathUtils {
    /// `GetTrueRandom()`: a new `Random` seeded with `(int)DateTime.Now.Ticks` each call.
    pub fn get_true_random() -> f64 {
        let seed = DateTime::now().ticks as i32;
        Random::new(seed).next_double()
    }

    /// `GetTrueRandom(double low, double high)`
    pub fn get_true_random_range(low: f64, high: f64) -> f64 {
        low + (high - low) * Self::get_true_random()
    }

    /// `GetTrueRandom(int low, int high)`
    pub fn get_true_random_int(low: i32, high: i32) -> i32 {
        cast_i32(math_round(Self::get_true_random_range(low as f64, high as f64), 0))
    }

    /// `GetScale(double max)`: `int` arithmetic wraps as unchecked C# does; a zero step size
    /// makes `Convert.ToInt32` throw.
    pub fn get_scale(max: f64) -> Result<f64, Exception> {
        let mut i: i32 = 1;
        let state: i32;
        loop {
            i = i.wrapping_mul(10);
            if max < i as f64 {
                state = 1;
                break;
            }
            if max < i.wrapping_mul(2) as f64 {
                state = 2;
                break;
            }
            if max < i.wrapping_mul(5) as f64 {
                state = 5;
                break;
            }
        }
        let step_size = (i / 10).wrapping_mul(state);
        let num_steps = to_int32_f64((max / step_size as f64).ceil())?.wrapping_add(1);
        Ok(num_steps as f64 * step_size as f64)
    }

    /// `Pow(double x, int n)`: `Math.Abs(int.MinValue)` throws.
    pub fn pow(x: f64, n: i32) -> Result<f64, Exception> {
        if n == 0 {
            return Ok(1.0);
        }
        let mut ans = 1.0;
        let nn = n.checked_abs().ok_or_else(|| Exception::Overflow("Negating the minimum value of a twos complement number is invalid.".into()))?;
        for _ in 0..nn {
            ans *= x;
        }
        Ok(if n > 0 { ans } else { 1.0 / ans })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scale_and_pow() {
        assert_eq!(MathUtils::get_scale(7.0).unwrap(), 8.0);
        assert_eq!(MathUtils::get_scale(15.0).unwrap(), 18.0);
        assert_eq!(MathUtils::get_scale(42.0).unwrap(), 50.0);
        assert_eq!(MathUtils::get_scale(130.0).unwrap(), 160.0);
        assert_eq!(MathUtils::pow(2.0, 3).unwrap(), 8.0);
        assert_eq!(MathUtils::pow(2.0, -2).unwrap(), 0.25);
        assert!(MathUtils::pow(2.0, i32::MIN).is_err());
        let r = MathUtils::get_true_random_range(1.0, 2.0);
        assert!((1.0..2.0).contains(&r));
    }
}

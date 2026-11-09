//! Port of `ExtLibs/Core/Utils/ColorUtils.cs`

use std::collections::HashMap;
use std::sync::{Mutex, OnceLock};

use crate::dotnet::convert::{to_int32_f64, to_int32_from_base};
use crate::dotnet::drawing::Color;
use crate::dotnet::{math_min, Exception};

/// `public static class ColorUtils`
pub struct ColorUtils;

/// `private static Dictionary<Color, CIELAB> cielabLookup`
fn cielab_lookup() -> &'static Mutex<HashMap<Color, CIELAB>> {
    static L: OnceLock<Mutex<HashMap<Color, CIELAB>>> = OnceLock::new();
    L.get_or_init(|| Mutex::new(HashMap::new()))
}

/// `public struct CIELAB`
#[allow(non_snake_case)]
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct CIELAB {
    pub L: f64,
    pub a: f64,
    pub b: f64,
}

impl CIELAB {
    /// `DistanceTo(CIELAB c)`
    pub fn distance_to(&self, c: &CIELAB) -> f64 {
        let d_l = self.L - c.L;
        let da = self.a - c.a;
        let db = self.b - c.b;
        (0.5 * d_l * d_l + da * da + db * db).sqrt()
    }

    /// `DistanceTo(Color clr)`
    pub fn distance_to_color(&self, clr: &Color) -> f64 {
        self.distance_to(&ColorUtils::convert_to_cielab(clr))
    }
}

/// `string.Equals(a, b, StringComparison.OrdinalIgnoreCase)`: each character's invariant
/// simple upper case compared (so U+0131 matches `i`).
fn equals_ordinal_ignore_case(a: &str, b: &str) -> bool {
    let up = |c: char| {
        let mut u = c.to_uppercase();
        match (u.next(), u.next()) {
            (Some(x), None) => x,
            _ => c,
        }
    };
    a.encode_utf16().count() == b.encode_utf16().count() && a.chars().map(up).eq(b.chars().map(up))
}

/// `public class ColorTranslator` (nested in `ColorUtils`): only `#RRGGBB`, `#RGB` and
/// "LightGrey" are understood; anything else is `Color.Empty`.
pub struct ColorTranslator;

impl ColorTranslator {
    /// `FromHtml(string htmlColor)`: bad hex digits throw from `Convert.ToInt32(s, 16)`.
    pub fn from_html(html_color: Option<&str>) -> Result<Color, Exception> {
        let mut result = Color::EMPTY;
        let Some(html) = html_color else { return Ok(result) };
        let u: Vec<u16> = html.encode_utf16().collect();
        if u.is_empty() {
            return Ok(result);
        }
        if u[0] == b'#' as u16 && (u.len() == 7 || u.len() == 4) {
            let hex = |a: &[u16]| to_int32_from_base(Some(&String::from_utf16_lossy(a)), 16);
            if u.len() == 7 {
                result = Color::from_rgb(hex(&u[1..3])?, hex(&u[3..5])?, hex(&u[5..7])?)?;
            } else {
                let t1 = [u[1], u[1]];
                let t2 = [u[2], u[2]];
                let t3 = [u[3], u[3]];
                result = Color::from_rgb(hex(&t1)?, hex(&t2)?, hex(&t3)?)?;
            }
        }
        if result.is_empty() && equals_ordinal_ignore_case(html, "LightGrey") {
            result = Color::LIGHT_GRAY;
        }
        Ok(result)
    }

    /// `ToHtml(Color c)`: `#RRGGBB` in upper-case hex, alpha dropped.
    pub fn to_html(c: &Color) -> String {
        format!("#{:02X}{:02X}{:02X}", c.r(), c.g(), c.b())
    }
}

impl ColorUtils {
    /// `GetInbetweenColor(Color a, Color b, double aToB)`
    pub fn get_inbetween_color(a: &Color, b: &Color, a_to_b: f64) -> Result<Color, Exception> {
        let a_to_b = math_min(1.0, a_to_b);
        let red_d = b.r() as i32 - a.r() as i32;
        let green_d = b.g() as i32 - a.g() as i32;
        let blue_d = b.b() as i32 - a.b() as i32;
        let red = to_int32_f64(a.r() as f64 + red_d as f64 * a_to_b)?;
        let green = to_int32_f64(a.g() as f64 + green_d as f64 * a_to_b)?;
        let blue = to_int32_f64(a.b() as f64 + blue_d as f64 * a_to_b)?;
        Color::from_rgb(red, green, blue)
    }

    /// `ConvertToCIELAB(Color c)`: memoised per `Color` (value equality as `Color.Equals`).
    /// `16 / 116` is integer division in the C#, so the linear branch adds 0.
    pub fn convert_to_cielab(c: &Color) -> CIELAB {
        if let Some(v) = cielab_lookup().lock().unwrap_or_else(|e| e.into_inner()).get(c) {
            return *v;
        }
        let rf = Self::convert_channel(c.r()) * 100.0;
        let gf = Self::convert_channel(c.g()) * 100.0;
        let bf = Self::convert_channel(c.b()) * 100.0;

        let mut var_x = rf * 0.4124 + gf * 0.3576 + bf * 0.1805;
        let mut var_y = rf * 0.2126 + gf * 0.7152 + bf * 0.0722;
        let mut var_z = rf * 0.0193 + gf * 0.1192 + bf * 0.9505;

        var_x /= 95.047;
        var_y /= 100.000;
        var_z /= 108.883;

        let to_pow = 1.0 / 3.0;
        let int_div = (16 / 116) as f64;
        var_x = if var_x > 0.008856 { var_x.powf(to_pow) } else { (7.787 * var_x) + int_div };
        var_y = if var_y > 0.008856 { var_y.powf(to_pow) } else { (7.787 * var_y) + int_div };
        var_z = if var_z > 0.008856 { var_z.powf(to_pow) } else { (7.787 * var_z) + int_div };

        let ans = CIELAB { L: (116.0 * var_y) - 16.0, a: 500.0 * (var_x - var_y), b: 200.0 * (var_y - var_z) };
        cielab_lookup().lock().unwrap_or_else(|e| e.into_inner()).insert(c.clone(), ans);
        ans
    }

    fn convert_channel(channel: u8) -> f64 {
        let f = channel as f64 / 255.0;
        if f > 0.04045 {
            return ((f + 0.055) / 1.055).powf(2.4);
        }
        f / 12.92
    }

    /// `Invert(Color aColor)`: black for light colours, white for dark ones.
    pub fn invert(a_color: &Color) -> Color {
        let r = a_color.r() as i32;
        let g = a_color.g() as i32;
        let b = a_color.b() as i32;
        if (r + g + b) > 255 {
            Color::BLACK
        } else {
            Color::WHITE
        }
    }

    /// `GetDistinctColors()`
    pub fn get_distinct_colors() -> Vec<Color> {
        const HTML: &[&str] = &[
            "#BEE117", "#FFFF00", "#D8011D", "#0066FF", "#F16D9F", "#AC4CDC", "#FF9400", "#52A400", //
            "#FEBABA", "#80D50B", "#FFCE00", "#CBFF84", "#00A6FF", "#AD5701", "#990000", "#0000CC", //
            "#CC99CC", "#0D9D93", "#66CCFF", "#CC9966", "#CC6699", "#FDE6C1", "#FFE300", "#009933", "#C4E1FF",
            "#9999FF", "#E0FF00", //
            "#B3EA5F", "#DCDC78", "#FF6600", "#CCCCFF", "#843C3C", "#416529", "#00CC99", "#71437C", //
            "#66FF00", "#DBEF9A", "#429392", "#654229", "#FF6060", "#91FDA3", "#ED8787", "#8DC0AF", "#DDF4CA",
            "#FFD9FF", "#598A5F", "#FFFF99", "#CCFFFF", "#BE00CC", "#1A3D51", "#993366", "#996633", "#FF0000",
            "#33FFFF", "#006699", "#66FF99", "#006633", "#E639FF",
        ];
        let mut colors: Vec<Color> =
            HTML.iter().map(|h| ColorTranslator::from_html(Some(h)).expect("valid literal colour")).collect();
        colors.push(Color::BLACK);
        colors.push(Color::SILVER);
        colors.push(Color::GAINSBORO);
        colors.push(Color::WHITE);
        colors
    }

    /// `ColorsMatch(Color a, Color b, double distance)`
    pub fn colors_match_distance(a: &Color, b: &Color, distance: f64) -> bool {
        let a_lab = Self::convert_to_cielab(a);
        a_lab.distance_to_color(b) < distance
    }

    /// `ColorsMatch(Color a, Color b, int tol)`
    pub fn colors_match_tol(a: &Color, b: &Color, tol: i32) -> bool {
        if (a.r() as i32 - b.r() as i32).abs() > tol {
            return false;
        }
        if (a.g() as i32 - b.g() as i32).abs() > tol {
            return false;
        }
        if (a.b() as i32 - b.b() as i32).abs() > tol {
            return false;
        }
        true
    }

    /// `AlphaTest(Color col, float threshold)`: compared in single precision.
    pub fn alpha_test(col: &Color, threshold: f32) -> bool {
        col.a() as f32 > threshold * 255.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn html_round_trip() {
        let c = ColorTranslator::from_html(Some("#BEE117")).unwrap();
        assert_eq!((c.r(), c.g(), c.b()), (0xBE, 0xE1, 0x17));
        assert_eq!(ColorTranslator::to_html(&c), "#BEE117");
        let s = ColorTranslator::from_html(Some("#f0a")).unwrap();
        assert_eq!((s.r(), s.g(), s.b()), (0xFF, 0x00, 0xAA));
        assert!(ColorTranslator::from_html(Some("red")).unwrap().is_empty());
        assert!(ColorTranslator::from_html(None).unwrap().is_empty());
        assert_eq!(ColorTranslator::from_html(Some("LIGHTGREY")).unwrap(), Color::LIGHT_GRAY);
        assert!(ColorTranslator::from_html(Some("#GG0000")).is_err());
        assert_eq!(ColorUtils::get_distinct_colors().len(), 62);
    }

    #[test]
    fn color_math() {
        let blk = Color::from_rgb(0, 0, 0).unwrap();
        let wht = Color::from_rgb(255, 255, 255).unwrap();
        let mid = ColorUtils::get_inbetween_color(&blk, &wht, 0.5).unwrap();
        // 127.5 rounds to even
        assert_eq!(mid.r(), 128);
        assert!(ColorUtils::get_inbetween_color(&blk, &wht, f64::NAN).is_err());
        assert!(ColorUtils::get_inbetween_color(&blk, &wht, -1.0).is_err());
        assert_eq!(ColorUtils::invert(&wht), Color::BLACK);
        assert_eq!(ColorUtils::invert(&blk), Color::WHITE);
        let lab = ColorUtils::convert_to_cielab(&wht);
        assert!((lab.L - 100.0).abs() < 0.01);
        // dark channel takes the linear branch with the integer 16/116 == 0
        let lab = ColorUtils::convert_to_cielab(&blk);
        assert_eq!(lab.L, -16.0);
        assert!(ColorUtils::colors_match_tol(&blk, &Color::from_rgb(3, 0, 0).unwrap(), 3));
        assert!(!ColorUtils::colors_match_tol(&blk, &Color::from_rgb(4, 0, 0).unwrap(), 3));
        assert!(ColorUtils::colors_match_distance(&blk, &blk, 0.1));
        assert!(ColorUtils::alpha_test(&wht, 0.5));
        assert!(!ColorUtils::alpha_test(&wht, 1.0));
    }
}

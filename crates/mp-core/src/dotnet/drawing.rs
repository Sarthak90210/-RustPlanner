//! `System.Drawing` value types the ported code stores and compares (.NET Framework 4.7.2).
//!
//! Only the data side is here: no GDI+. `Color` keeps the four fields .NET keeps (value,
//! state, knownColor, name), so `==` behaves as it does in C#: `Color.White` is not equal to
//! `Color.FromArgb(255, 255, 255, 255)`, although both have the same ARGB value.

const STATE_KNOWN_COLOR_VALID: i16 = 0x0001;
const STATE_ARGB_VALUE_VALID: i16 = 0x0002;
const STATE_NAME_VALID: i16 = 0x0008;

/// `System.Drawing.KnownColor` members and their ARGB values (`KnownColorTable`). The system
/// colors (`ActiveBorder` .. `WindowText`, 1..=26 and 168..=174) depend on the OS theme and are
/// not listed; members are added when a ported file names them.
static KNOWN_COLORS: &[(i16, &str, u32)] = &[
    (27, "Transparent", 0x00FF_FFFF),
    (35, "Black", 0xFF00_0000),
    (37, "Blue", 0xFF00_00FF),
    (78, "Gray", 0xFF80_8080),
    (79, "Green", 0xFF00_8000),
    (91, "LightBlue", 0xFFAD_D8E6),
    (127, "Orange", 0xFFFF_A500),
    (137, "Pink", 0xFFFF_C0CB),
    (141, "Red", 0xFFFF_0000),
    (164, "White", 0xFFFF_FFFF),
    (166, "Yellow", 0xFFFF_FF00),
];

/// `public struct Color`
#[derive(Debug, Clone, PartialEq, Eq, Hash, Default)]
pub struct Color {
    value: i64,
    state: i16,
    known_color: i16,
    name: Option<String>,
}

impl Color {
    /// `Color.Empty`
    pub const EMPTY: Color = Color { value: 0, state: 0, known_color: 0, name: None };

    const fn known(k: i16) -> Color {
        Color { value: 0, state: STATE_KNOWN_COLOR_VALID, known_color: k, name: None }
    }

    /// `Color.Transparent`
    pub const TRANSPARENT: Color = Color::known(27);
    /// `Color.Black`
    pub const BLACK: Color = Color::known(35);
    /// `Color.Blue`
    pub const BLUE: Color = Color::known(37);
    /// `Color.Gray`
    pub const GRAY: Color = Color::known(78);
    /// `Color.Green`
    pub const GREEN: Color = Color::known(79);
    /// `Color.LightBlue`
    pub const LIGHT_BLUE: Color = Color::known(91);
    /// `Color.Orange`
    pub const ORANGE: Color = Color::known(127);
    /// `Color.Pink` (`Pens.Pink`)
    pub const PINK: Color = Color::known(137);
    /// `Color.Red`
    pub const RED: Color = Color::known(141);
    /// `Color.White`
    pub const WHITE: Color = Color::known(164);
    /// `Color.Yellow`
    pub const YELLOW: Color = Color::known(166);

    /// `Color.FromArgb(int argb)`
    pub fn from_argb(argb: i32) -> Color {
        Color { value: (argb as i64) & 0xffff_ffff, state: STATE_ARGB_VALUE_VALID, known_color: 0, name: None }
    }

    /// `Color.FromArgb(int alpha, int red, int green, int blue)`: each component must be 0..=255,
    /// otherwise .NET throws `ArgumentException`.
    pub fn from_argb4(alpha: i32, red: i32, green: i32, blue: i32) -> Result<Color, super::Exception> {
        for (v, n) in [(alpha, "alpha"), (red, "red"), (green, "green"), (blue, "blue")] {
            if !(0..=255).contains(&v) {
                return Err(super::Exception::Argument(format!(
                    "Value of '{v}' is not valid for '{n}'. '{n}' should be greater than or equal to 0 and less than or equal to 255."
                )));
            }
        }
        let argb = ((alpha as u32) << 24) | ((red as u32) << 16) | ((green as u32) << 8) | blue as u32;
        Ok(Color::from_argb(argb as i32))
    }

    /// `Color.FromArgb(int red, int green, int blue)`: alpha 255.
    pub fn from_rgb(red: i32, green: i32, blue: i32) -> Result<Color, super::Exception> {
        Color::from_argb4(255, red, green, blue)
    }

    fn argb_value(&self) -> i64 {
        if self.state & STATE_ARGB_VALUE_VALID != 0 {
            return self.value;
        }
        if self.state & STATE_KNOWN_COLOR_VALID != 0 {
            if let Some((_, _, v)) = KNOWN_COLORS.iter().find(|(k, _, _)| *k == self.known_color) {
                return *v as i64;
            }
        }
        0
    }

    /// `ToArgb()`
    pub fn to_argb(&self) -> i32 {
        self.argb_value() as u32 as i32
    }

    /// `A`
    pub fn a(&self) -> u8 {
        (self.argb_value() >> 24) as u8
    }

    /// `R`
    pub fn r(&self) -> u8 {
        (self.argb_value() >> 16) as u8
    }

    /// `G`
    pub fn g(&self) -> u8 {
        (self.argb_value() >> 8) as u8
    }

    /// `B`
    pub fn b(&self) -> u8 {
        self.argb_value() as u8
    }

    /// `IsEmpty`
    pub fn is_empty(&self) -> bool {
        self.state == 0
    }

    /// `IsKnownColor`
    pub fn is_known_color(&self) -> bool {
        self.state & STATE_KNOWN_COLOR_VALID != 0
    }

    /// `Name`: the given name, the known color's name, or the ARGB value in lower-case hex.
    pub fn name(&self) -> String {
        if self.state & STATE_NAME_VALID != 0 {
            return self.name.clone().unwrap_or_default();
        }
        if self.is_known_color() {
            if let Some((_, n, _)) = KNOWN_COLORS.iter().find(|(k, _, _)| *k == self.known_color) {
                return (*n).to_string();
            }
            return self.known_color.to_string();
        }
        format!("{:x}", self.value)
    }
}

impl std::fmt::Display for Color {
    /// `Color.ToString()`: `Color [Name]` for named colors, `Color [A=.., R=.., G=.., B=..]`
    /// otherwise, `Color [Empty]` for the empty color.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if self.state & STATE_NAME_VALID != 0 || self.is_known_color() {
            write!(f, "Color [{}]", self.name())
        } else if self.state & STATE_ARGB_VALUE_VALID != 0 {
            write!(f, "Color [A={}, R={}, G={}, B={}]", self.a(), self.r(), self.g(), self.b())
        } else {
            f.write_str("Color [Empty]")
        }
    }
}

/// `System.Drawing.PointF`: a single-precision point.
#[allow(non_snake_case)]
#[derive(Debug, Clone, Copy, Default)]
pub struct PointF {
    pub X: f32,
    pub Y: f32,
}

impl PointF {
    /// `new PointF(x, y)`
    pub fn new(x: f32, y: f32) -> Self {
        PointF { X: x, Y: y }
    }

    /// `IsEmpty`: both coordinates zero.
    pub fn is_empty(&self) -> bool {
        self.X == 0.0 && self.Y == 0.0
    }
}

impl PartialEq for PointF {
    /// `Equals` / `operator ==`: X and Y compared with `float ==` (NaN never equal).
    fn eq(&self, o: &PointF) -> bool {
        self.X == o.X && self.Y == o.Y
    }
}

impl std::fmt::Display for PointF {
    /// `ToString()`: `{X=1.5, Y=2}` in the current culture.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{{X={}, Y={}}}", super::float_to_string(self.X), super::float_to_string(self.Y))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn known_color_differs_from_same_argb() {
        let w = Color::from_argb4(255, 255, 255, 255).unwrap();
        assert_eq!(w.to_argb(), Color::WHITE.to_argb());
        assert_ne!(w, Color::WHITE);
        assert_eq!(Color::WHITE.to_string(), "Color [White]");
        assert_eq!(w.to_string(), "Color [A=255, R=255, G=255, B=255]");
        assert!(Color::from_argb4(256, 0, 0, 0).is_err());
    }
}

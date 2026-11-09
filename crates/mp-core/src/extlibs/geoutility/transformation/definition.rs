//! Port of `ExtLibs/GeoUtility/Transformation/Definition.cs`

use super::Transformation;

#[allow(dead_code)]
impl Transformation {
    /// `internal Transformation()`
    pub const fn new() -> Self {
        Transformation
    }

    // ===================== UTM =====================

    /// Scale factor
    pub(crate) const UTM_FAKTOR: f64 = 0.9996;
    pub(crate) const UTM_FALSE_EASTING: f64 = 500000.0;

    /// Semi-major axis
    pub(crate) const WGS84_HALBACHSE: f64 = 6378137.000;

    /// Flattening WGS84 = 298,257223563 (1/x)
    pub(crate) const WGS84_ABPLATTUNG: f64 = 3.35281066474748E-03;

    /// UTM bands C-X (the second X keeps the index in range at 84 degrees)
    pub(crate) const UTM_BAND: &'static str = "CDEFGHJKLMNPQRSTUVWXX";

    /// Polar radius of curvature
    pub(crate) const WGS84_POL: f64 = Self::WGS84_HALBACHSE / (1.0 - Self::WGS84_ABPLATTUNG);

    /// Eccentricities
    pub(crate) const WGS84_EXZENT: f64 =
        (2.0 * Self::WGS84_ABPLATTUNG) - (Self::WGS84_ABPLATTUNG * Self::WGS84_ABPLATTUNG);
    pub(crate) const WGS84_EXZENT2: f64 = ((2.0 * Self::WGS84_ABPLATTUNG) - (Self::WGS84_ABPLATTUNG * Self::WGS84_ABPLATTUNG))
        / ((1.0 - Self::WGS84_ABPLATTUNG) * (1.0 - Self::WGS84_ABPLATTUNG));
    pub(crate) const WGS84_EXZENT4: f64 = Self::WGS84_EXZENT2 * Self::WGS84_EXZENT2;
    pub(crate) const WGS84_EXZENT6: f64 = Self::WGS84_EXZENT4 * Self::WGS84_EXZENT2;
    pub(crate) const WGS84_EXZENT8: f64 = Self::WGS84_EXZENT4 * Self::WGS84_EXZENT4;

    /// Geographic limits of the UTM system in degrees
    pub(crate) const MIN_LAENGE: f64 = -180.0;
    pub(crate) const MAX_LAENGE: f64 = 180.0;
    pub(crate) const MIN_BREITE: f64 = -80.0;
    pub(crate) const MAX_BREITE: f64 = 84.0;

    // ===================== MGRS =====================

    pub(crate) const MGRS_EAST0: &'static str = "STUVWXYZ";
    pub(crate) const MGRS_EAST1: &'static str = "ABCDEFGH";
    pub(crate) const MGRS_EAST2: &'static str = "JKLMNPQR";
    /// `MGRS_EAST1 + MGRS_EAST2 + MGRS_EAST0`
    pub(crate) const MGRS_EAST: &'static str = "ABCDEFGHJKLMNPQRSTUVWXYZ";

    pub(crate) const MGRS_NORTH0: &'static str = "FGHJKLMNPQRSTUVABCDE";
    pub(crate) const MGRS_NORTH1: &'static str = "ABCDEFGHJKLMNPQRSTUV";

    pub(crate) const MIN_ZONE: i32 = 1;
    pub(crate) const MAX_ZONE: i32 = 60;
    pub(crate) const MIN_BAND: char = 'C';
    pub(crate) const MAX_BAND: char = 'X';

    // ===================== Gauss-Krueger =====================

    /// Semi-major axis and flattening, Bessel
    pub(crate) const HALBACHSE: f64 = 6377397.155;
    pub(crate) const ABPLATTUNG: f64 = 3.342773182E-03;

    /// Polar radius of curvature
    pub(crate) const POL: f64 = Self::HALBACHSE / (1.0 - Self::ABPLATTUNG);

    /// Eccentricities
    pub(crate) const EXZENT: f64 = (2.0 * Self::ABPLATTUNG) - (Self::ABPLATTUNG * Self::ABPLATTUNG);
    pub(crate) const EXZENT2: f64 =
        ((2.0 * Self::ABPLATTUNG) - (Self::ABPLATTUNG * Self::ABPLATTUNG)) / ((1.0 - Self::ABPLATTUNG) * (1.0 - Self::ABPLATTUNG));
    pub(crate) const EXZENT4: f64 = Self::EXZENT2 * Self::EXZENT2;
    pub(crate) const EXZENT6: f64 = Self::EXZENT4 * Self::EXZENT2;
    pub(crate) const EXZENT8: f64 = Self::EXZENT4 * Self::EXZENT4;

    /// Geographic limits of the Gauss-Krueger system in degrees
    pub(crate) const MIN_OST: f64 = 5.0;
    pub(crate) const MAX_OST: f64 = 16.0;
    pub(crate) const MIN_NORD: f64 = 46.0;
    pub(crate) const MAX_NORD: f64 = 56.0;

    /// Datum shift Potsdam -> WGS84 (the reverse shift uses the negated values)
    pub(crate) const POTSDAM_DATUM_SHIFT_X: f64 = 587.0;
    pub(crate) const POTSDAM_DATUM_SHIFT_Y: f64 = 16.0;
    pub(crate) const POTSDAM_DATUM_SHIFT_Z: f64 = 393.0;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mgrs_east_is_the_concatenation() {
        assert_eq!(
            Transformation::MGRS_EAST,
            format!("{}{}{}", Transformation::MGRS_EAST1, Transformation::MGRS_EAST2, Transformation::MGRS_EAST0)
        );
    }
}

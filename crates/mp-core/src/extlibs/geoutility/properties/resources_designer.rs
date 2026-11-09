//! Port of `ExtLibs/GeoUtility/Properties/Resources.Designer.cs`
//!
//! The generated class reads `GeoUtility.Properties.Resources.resources`, compiled from
//! `Properties/Resources.resx`. The table below holds that file's entries verbatim (the XML
//! reader turns the CR LF inside multi-line values into LF).

/// `Resources.ResourceManager.GetString(name)`: case-sensitive lookup, `null` when absent.
pub fn get_string(name: &str) -> Option<&'static str> {
    RESOURCES.iter().find(|(k, _)| *k == name).map(|(_, v)| *v)
}

static RESOURCES: &[(&str, &str)] = &[
    ("DE_ERROR_CONVERTFROM", "Eine Typ-Konvertierung zwischen {0} und {1} ist nicht erlaubt."),
    ("DE_ERROR_GAUSS_EAST", "Rechtswert außerhalb des für Deutschland definierten Gauss-Krüger-Systems: \nGeogr. Länge: 5 bis 16 E."),
    ("DE_ERROR_GAUSS_NORTH", "Hochwert außerhalb des für Deutschland definierten Gauss-Krüger-Systems: \nGeogr. Breite: 46 bis 55 N."),
    ("DE_ERROR_GEO2UTM", "Werte außerhalb des definierten Bereichs im WGS84 Datum: \nLänge: -180 bis +180 E; Breite: -80 bis +84 N."),
    ("DE_ERROR_GEO_LATITUDE", "Breitengrad ungültig. Definierter Bereich: -80 (südl.) bis +84 (nördl.)"),
    ("DE_ERROR_GEO_LONGITUDE", "Längengrad ungültig. Definierter Bereich: -180 (westl.) bis +180 (östl.)"),
    ("DE_ERROR_GK_OUT_OF_RANGE", "Werte außerhalb des für Deutschland definierten Gauss-Krüger-Systems: \nLänge: 5 bis 16 E; Breite: 46 bis 55 N."),
    ("DE_ERROR_MAPDIMENSION", "Ungültiger interner Schlüssel für das Satellitenbild."),
    ("DE_ERROR_MGRS", "Format des MGRS-Koordinatenstrings konnte nicht erkannt werden oder außerhalb des gültigen Bereichs."),
    ("DE_ERROR_MGRS_EAST", "East-Wert ungültig. "),
    ("DE_ERROR_MGRS_GRID", "Grid-Wert ungültig. Er besteht aus 2 Buchstaben."),
    ("DE_ERROR_MGRS_NORTH", "North-Wert ungültig."),
    ("DE_ERROR_TILE", "Ungültige Werte in dem von MapServiceTileBase abgeleiteten Parameter."),
    ("DE_ERROR_UTM_BAND", "Band ungültig. Definierter Bereich: C bis X (ohne I und O)."),
    ("DE_ERROR_UTM_EAST", "East-Wert ungültig. Definierter Bereich: ca. 200000 bis 800000"),
    ("DE_ERROR_UTM_NORTH", "Nord-Wert ungültig. Def.-Bereich: ca. 0 bis 9300000 (Nord), ca. 1100000 bis 10000000 (Süd)"),
    ("DE_ERROR_UTM_STRING", "Der UTM-String konnte nicht erkannt werden (Format unbekannt)."),
    ("DE_ERROR_UTM_ZONE", "{0}{1} ist eine ungültige UTM-Zone."),
    ("DE_ERROR_UTM_ZONE2", "Zone ungültig. Definierter Bereich: 1 bis 60."),
    ("DE_GAUSS_LONG_STRING", "Rechts: {0}; Hoch: {1}"),
    ("DE_GAUSS_SHORT_STRING", "R: {0}   H: {1}"),
    ("DE_GEOGRAPHIC_STRING", "Länge: {0}; Breite: {1}"),
    ("DE_MS_NO_IMAGE", "Satellitenbild in dieser Zoomstufe nicht verfügar."),
    ("DE_NO_ERROR_MESSAGE", "Nähere Angaben zum Fehler sind nicht verfügbar."),
    ("ERROR_CONVERTFROM", "Type conversion between {0} and {1} is not allowed."),
    ("ERROR_GAUSS_EAST", "East value out of range for the german Gauss-Krueger-System: \nLongitude: 5 bis 16 E."),
    ("ERROR_GAUSS_NORTH", "North value out of range for the german Gauss-Krueger-System: \nLatitude: 46 to 55 N."),
    ("ERROR_GEO2UTM", "Values out of range for UTM: \nLongitude: -180 to +180 E; Latitude: -80 to +84 N."),
    ("ERROR_GEO_LATITUDE", "Latitude invalid. Range: -80 (South) to +84 (North)"),
    ("ERROR_GEO_LONGITUDE", "Longitude invalid. Range: -180 (West) to +180 (East)"),
    ("ERROR_GK_OUT_OF_RANGE", "Values out of range for the german Gauss-Krueger-System: \nLongitude: 5 bis 16 E; Latitude: 46 to 55 N."),
    ("ERROR_MAPDIMENSION", "Invalid internal key for the satellite picture."),
    ("ERROR_MGRS", "Format of the MGRS string could not be parsed or out of range."),
    ("ERROR_MGRS_EAST", "East value invalid."),
    ("ERROR_MGRS_GRID", "Grid value invalid. It needs 2 letters."),
    ("ERROR_MGRS_NORTH", "North value invalid."),
    ("ERROR_TILE", "Your MapServiceTileBase derived object contains invalid values."),
    ("ERROR_UTM_BAND", "Band invalid. Range: C to X (without I and O)."),
    ("ERROR_UTM_EAST", "East value invalid. Range: approx. 200000 to 800000"),
    ("ERROR_UTM_NORTH", "North value invalid. Range: approx. 0 to 9300000 (North), approx. 1100000 to 10000000 (South)"),
    ("ERROR_UTM_STRING", "UTM string could not be parsed (format unknown)."),
    ("ERROR_UTM_ZONE", "{0}{1} is an invalid utm zone."),
    ("ERROR_UTM_ZONE2", "Zone invalid. Range: 1 to 60."),
    ("GAUSS_SHORT_STRING", "E {0}   N {1}"),
    ("GAUSS_LONG_STRING", "East: {0}; North: {1}"),
    ("GEOGRAPHIC_STRING", "Longitude: {0}; Latitude: {1}"),
    ("MS_NO_IMAGE", "Satellite picture with this zoom level not available ."),
    ("NO_ERROR_MESSAGE", "There are no error details available."),
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lookup_is_case_sensitive() {
        assert_eq!(get_string("ERROR_UTM_BAND"), Some("Band invalid. Range: C to X (without I and O)."));
        // UTM.TryParse and MGRS.TryParse ask for "ERROR_UTM_Band", which does not exist.
        assert_eq!(get_string("ERROR_UTM_Band"), None);
    }
}

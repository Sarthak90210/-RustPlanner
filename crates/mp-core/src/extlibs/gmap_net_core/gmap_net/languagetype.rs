//! Port of `ExtLibs/GMap.NET.Core/GMap.NET/LanguageType.cs`
//!
//! The `[Description]` attribute of each member is `description()`.

crate::cs_enum! {
    /// `public enum LanguageType`: all possible languages for the Google maps.
    pub LanguageType: i32 {
        Arabic = 0,
        Bulgarian = 1,
        Bengali = 2,
        Catalan = 3,
        Czech = 4,
        Danish = 5,
        German = 6,
        Greek = 7,
        English = 8,
        EnglishAustralian = 9,
        EnglishGreatBritain = 10,
        Spanish = 11,
        Basque = 12,
        FARSI = 13,
        Finnish = 14,
        Filipino = 15,
        French = 16,
        Galician = 17,
        Gujarati = 18,
        Hindi = 19,
        Croatian = 20,
        Hungarian = 21,
        Indonesian = 22,
        Italian = 23,
        Hebrew = 24,
        Japanese = 25,
        Kannada = 26,
        Korean = 27,
        Lithuanian = 28,
        Latvian = 29,
        Malayalam = 30,
        Marathi = 31,
        Dutch = 32,
        NorwegianNynorsk = 33,
        Norwegian = 34,
        Oriya = 35,
        Polish = 36,
        Portuguese = 37,
        PortugueseBrazil = 38,
        PortuguesePortugal = 39,
        Romansch = 40,
        Romanian = 41,
        Russian = 42,
        Slovak = 43,
        Slovenian = 44,
        Serbian = 45,
        Swedish = 46,
        TAGALOG = 47,
        Tamil = 48,
        Telugu = 49,
        Thai = 50,
        Turkish = 51,
        Ukrainian = 52,
        Vietnamese = 53,
        ChineseSimplified = 54,
        ChineseTraditional = 55,
    }
}

impl LanguageType {
    /// The `[Description("..")]` of the member, `None` for a value that is not a member.
    pub fn description(self) -> Option<&'static str> {
        match self.0 {
            0 => Some("ar"),
            1 => Some("bg"),
            2 => Some("bn"),
            3 => Some("ca"),
            4 => Some("cs"),
            5 => Some("da"),
            6 => Some("de"),
            7 => Some("el"),
            8 => Some("en"),
            9 => Some("en-AU"),
            10 => Some("en-GB"),
            11 => Some("es"),
            12 => Some("eu"),
            13 => Some("fa"),
            14 => Some("fi"),
            15 => Some("fil"),
            16 => Some("fr"),
            17 => Some("gl"),
            18 => Some("gu"),
            19 => Some("hi"),
            20 => Some("hr"),
            21 => Some("hu"),
            22 => Some("id"),
            23 => Some("it"),
            24 => Some("iw"),
            25 => Some("ja"),
            26 => Some("kn"),
            27 => Some("ko"),
            28 => Some("lt"),
            29 => Some("lv"),
            30 => Some("ml"),
            31 => Some("mr"),
            32 => Some("nl"),
            33 => Some("nn"),
            34 => Some("no"),
            35 => Some("or"),
            36 => Some("pl"),
            37 => Some("pt"),
            38 => Some("pt-BR"),
            39 => Some("pt-PT"),
            40 => Some("rm"),
            41 => Some("ro"),
            42 => Some("ru"),
            43 => Some("sk"),
            44 => Some("sl"),
            45 => Some("sr"),
            46 => Some("sv"),
            47 => Some("tl"),
            48 => Some("ta"),
            49 => Some("te"),
            50 => Some("th"),
            51 => Some("tr"),
            52 => Some("uk"),
            53 => Some("vi"),
            54 => Some("zh-CN"),
            55 => Some("zh-TW"),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn descriptions() {
        assert_eq!(LanguageType::get_values().len(), 56);
        assert_eq!(LanguageType::English.description(), Some("en"));
        assert_eq!(LanguageType::ChineseTraditional.description(), Some("zh-TW"));
        assert_eq!(LanguageType::Hebrew.description(), Some("iw"));
        assert_eq!(LanguageType(999).description(), None);
        assert_eq!(LanguageType::English.to_string(), "English");
    }
}

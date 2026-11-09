//! Port of `ExtLibs/GMap.NET.Core/GMap.NET/MapType.cs`
//!
//! The `[Obsolete]` attributes on the Google members have no runtime effect.

crate::cs_enum! {
    /// `public enum MapType`: types of great maps, legacy, not used anymore, left for old ids
    pub MapType: i32 {
        None = 0,
        GoogleMap = 1,
        GoogleSatellite = 4,
        GoogleLabels = 8,
        GoogleTerrain = 16,
        GoogleHybrid = 20,
        GoogleMapChina = 22,
        GoogleSatelliteChina = 24,
        GoogleLabelsChina = 26,
        GoogleTerrainChina = 28,
        GoogleHybridChina = 29,
        OpenStreetMap = 32,
        OpenStreetOsm = 33,
        OpenStreetMapSurfer = 34,
        OpenStreetMapSurferTerrain = 35,
        OpenSeaMapLabels = 36,
        OpenSeaMapHybrid = 37,
        OpenCycleMap = 38,
        YahooMap = 64,
        YahooSatellite = 128,
        YahooLabels = 256,
        YahooHybrid = 333,
        BingMap = 444,
        BingMap_New = 455,
        BingSatellite = 555,
        BingHybrid = 666,
        ArcGIS_StreetMap_World_2D = 777,
        ArcGIS_Imagery_World_2D = 788,
        ArcGIS_ShadedRelief_World_2D = 799,
        ArcGIS_Topo_US_2D = 811,
        ArcGIS_World_Physical_Map = 822,
        ArcGIS_World_Shaded_Relief = 833,
        ArcGIS_World_Street_Map = 844,
        ArcGIS_World_Terrain_Base = 855,
        ArcGIS_World_Topo_Map = 866,
        MapsLT_Map = 1000,
        MapsLT_OrtoFoto = 1001,
        MapsLT_Map_Labels = 1002,
        MapsLT_Map_Hybrid = 1003,
        MapsLT_Map_2_5D = 1004,
        MapsLT_OrtoFoto_2010 = 1101,
        MapsLT_Map_Hybrid_2010 = 1103,
        KarteLV_Map = 1500,
        PergoTurkeyMap = 2001,
        SigPacSpainMap = 3001,
        GoogleMapKorea = 4001,
        GoogleSatelliteKorea = 4002,
        GoogleLabelsKorea = 4003,
        GoogleHybridKorea = 4005,
        YandexMapRu = 5000,
        YandexMapRuSatellite = 5001,
        YandexMapRuLabels = 5002,
        YandexMapRuHybrid = 5003,
        MapBenderWMS = 6000,
        MapyCZ_Map = 7000,
        MapyCZ_MapTurist = 7001,
        MapyCZ_Satellite = 7002,
        MapyCZ_Labels = 7003,
        MapyCZ_Hybrid = 7004,
        MapyCZ_History = 7005,
        MapyCZ_HistoryHybrid = 7006,
        NearMap = 8000,
        NearMapSatellite = 8001,
        NearMapLabels = 8002,
        NearMapHybrid = 8003,
        OviMap = 9000,
        OviMapSatellite = 9001,
        OviMapHybrid = 9002,
        OviMapTerrain = 9003,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn values() {
        assert_eq!(MapType::get_values().len(), 69);
        assert_eq!(MapType::None.0, 0);
        assert_eq!(MapType::OviMapTerrain.0, 9003);
        assert_eq!(MapType(1500).to_string(), "KarteLV_Map");
    }
}

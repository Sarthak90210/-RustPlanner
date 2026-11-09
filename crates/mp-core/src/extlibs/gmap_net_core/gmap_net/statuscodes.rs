//! Port of `ExtLibs/GMap.NET.Core/GMap.NET/StatusCodes.cs`

crate::cs_enum! {
    /// `public enum GeoCoderStatusCode : int`
    pub GeoCoderStatusCode: i32 {
        /// unknow response
        Unknow = -1,
        /// No errors occurred; the address was successfully parsed and its geocode has been returned.
        G_GEO_SUCCESS = 200,
        /// A directions request could not be successfully parsed.
        G_GEO_BAD_REQUEST = 400,
        /// A geocoding or directions request could not be successfully processed, yet the exact reason for the failure is not known.
        G_GEO_SERVER_ERROR = 500,
        /// The HTTP q parameter was either missing or had no value.
        G_GEO_MISSING_QUERY = 601,
        /// Synonym for G_GEO_MISSING_QUERY.
        G_GEO_MISSING_ADDRESS = 601,
        /// No corresponding geographic location could be found for the specified address.
        G_GEO_UNKNOWN_ADDRESS = 602,
        /// The geocode for the given address or the route for the given directions query cannot be returned due to legal or contractual reasons.
        G_GEO_UNAVAILABLE_ADDRESS = 603,
        /// The GDirections object could not compute directions between the points mentioned in the query.
        G_GEO_UNKNOWN_DIRECTIONS = 604,
        /// The given key is either invalid or does not match the domain for which it was given.
        G_GEO_BAD_KEY = 610,
        /// The given key has gone over the requests limit in the 24 hour period or has submitted too many requests in too short a period of time.
        G_GEO_TOO_MANY_QUERIES = 620,
        /// indicates that exception occured during execution
        ExceptionInCode = 621,
    }
}

crate::cs_enum! {
    /// `public enum DirectionsStatusCode : int`
    pub DirectionsStatusCode: i32 {
        /// indicates the response contains a valid result.
        OK = 0,
        /// indicates at least one of the locations specified in the requests's origin, destination, or waypoints could not be geocoded.
        NOT_FOUND = 1,
        /// indicates no route could be found between the origin and destination.
        ZERO_RESULTS = 2,
        /// indicates that too many waypointss were provided in the request
        MAX_WAYPOINTS_EXCEEDED = 3,
        /// indicates that the provided request was invalid.
        INVALID_REQUEST = 4,
        /// indicates the service has received too many requests from your application within the allowed time period.
        OVER_QUERY_LIMIT = 5,
        /// indicates that the service denied use of the directions service by your application.
        REQUEST_DENIED = 6,
        /// indicates a directions request could not be processed due to a server error.
        UNKNOWN_ERROR = 7,
        /// indicates that exception occured during execution
        ExceptionInCode = 8,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn implicit_values_and_shared_value_names() {
        assert_eq!(GeoCoderStatusCode::ExceptionInCode.0, 621);
        assert_eq!(GeoCoderStatusCode::G_GEO_MISSING_ADDRESS, GeoCoderStatusCode::G_GEO_MISSING_QUERY);
        // Enum.GetName's binary search over two members valued 601
        assert_eq!(GeoCoderStatusCode(601).to_string(), "G_GEO_MISSING_QUERY");
        assert_eq!(GeoCoderStatusCode::Unknow.to_string(), "Unknow");
        assert_eq!(DirectionsStatusCode::ExceptionInCode.0, 8);
        assert_eq!(DirectionsStatusCode::ZERO_RESULTS.to_string(), "ZERO_RESULTS");
    }
}

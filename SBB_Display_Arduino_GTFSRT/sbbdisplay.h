/**
 * @copyright Hervé Cousin
 * @date      2024-07-12 / updated 2025
 * Identical to original - String members preserved as in working code.
 */

/**
 * @brief Current GPS-derived position, used for the top display row.
 */
struct GpsData {
  String x_coord;  ///< Reverse-geocoded street address (see fetchGPSAddress()),
                    ///< or "lat / lng" as a fallback if geocoding fails.
  String y_coord;   ///< Currently unused; reserved for a second address field.
  String pos_acc;   ///< Currently unused; reserved for GPS fix accuracy.
};

/**
 * @brief One candidate stop returned by the OJP LocationInformationRequest.
 *
 * Populated in fetchStationDataFromGPS(); up to `maxStations` entries
 * are stored in the global stationDataArray[].
 */
struct StationData {
  String gps_address;   ///< Name of the nearest stop overall (index 0's near_station),
                         ///< duplicated into every entry for the top display row.
  String near_station;  ///< Name of this specific stop (OJP StopPlaceName).
  int    distance;       ///< Haversine distance from the current position, in meters.
  String station_id;    ///< OJP StopPlaceRef, used as the StopPointRef in
                         ///< subsequent StopEventRequest calls.
};

/**
 * @brief One departure row shown on the e-paper board.
 *
 * Populated in ojpPostStream(); up to `numEntries` entries are stored
 * in the global stationBoardData[].
 */
struct StationBoardData {
  String line_operator;   ///< Currently unused; reserved for the operator name.
  String type;             ///< Currently unused; reserved for the vehicle/service type.
  String line;             ///< Published line name (e.g. "IC 5", "S3"), or "-" if unknown.
  String destination;      ///< Destination text (OJP DestinationText).
  String departure_time;   ///< Local departure time as "HH:MM" (see isoToHHMM()),
                            ///< or "--:--" if not yet available.
  int    delay;             ///< Delay in minutes vs. the timetabled time, clamped
                            ///< to [0, 120] by calcDelay(). 0 if on time or unknown.
};

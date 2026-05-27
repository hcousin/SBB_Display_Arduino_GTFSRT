/**
 * sbbdisplay.h
 *
 * @copyright Hervé Cousin (original), updated for GTFS-RT migration
 * @date      2024-07-12 / updated 2025
 *
 * Data structures used by the display sketch.
 *
 * GpsData has been removed: GPS-to-stop lookup is no longer used because
 * the new opentransportdata.swiss GTFS-RT API does not provide a nearby-
 * stop geocoding endpoint.  Stops are now configured statically in
 * credentials.h (STOP_IDS / STOP_NAMES).
 */

#pragma once

// ---------------------------------------------------------------------------
// Station Board Data
// One entry per departure row on the e-paper display.
// ---------------------------------------------------------------------------
struct StationBoardData {
  String line_operator;   // Transport operator (not available in GTFS-RT TU)
  String type;            // Vehicle type (not available in GTFS-RT TU)
  String line;            // Route ID from GTFS-RT (e.g. "IC 5", "S3")
  String destination;     // Headsign – not carried in GTFS-RT Trip Updates;
                          // pair with GTFS Static trips.txt for full info
  String departure_time;  // Scheduled departure + realtime offset, HH:MM
  int    delay;           // Realtime delay in minutes (negative = early)
};

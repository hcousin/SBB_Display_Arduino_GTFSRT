/**
 * @copyright Hervé Cousin
 * @date      2024-07-12 / updated 2025
 *
 * Structs use fixed char arrays instead of String objects to avoid
 * heap corruption during global/static initialisation on the ESP32-S3.
 */

#pragma once

#define MAX_STR 64   // max length for most string fields
#define MAX_ID  16   // max length for stop IDs

struct GpsData {
  char x_coord[MAX_STR];
  char y_coord[MAX_STR];
  char pos_acc[MAX_STR];
};

struct StationData {
  char gps_address  [MAX_STR];  // address at GPS fix  (shown after "GPS:")
  char near_station [MAX_STR];  // stop name           (shown after "Haltestelle:")
  int  distance;                // distance in metres
  char station_id   [MAX_ID];   // OJP / GTFS stop_id  (e.g. "8503000")
};

struct StationBoardData {
  char line_operator [MAX_STR];
  char type          [MAX_STR];
  char line          [MAX_STR];
  char destination   [MAX_STR];
  char departure_time[8];       // "HH:MM\0"
  int  delay;                   // minutes
};

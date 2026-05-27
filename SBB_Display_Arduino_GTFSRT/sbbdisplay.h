/**
 * @copyright Hervé Cousin
 * @date      2024-07-12 / updated 2025
 * Identical to original - String members preserved as in working code.
 */

struct GpsData {
  String x_coord;
  String y_coord;
  String pos_acc;
};

struct StationData {
  String gps_address;
  String near_station;
  int    distance;
  String station_id;
};

struct StationBoardData {
  String line_operator;
  String type;
  String line;
  String destination;
  String departure_time;
  int    delay;
};

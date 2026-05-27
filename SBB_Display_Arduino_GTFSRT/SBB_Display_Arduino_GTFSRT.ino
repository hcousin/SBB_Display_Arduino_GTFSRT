/**
 * @copyright Hervé Cousin (original), updated for GTFS-RT API
 * @date      2024-07-12 / updated 2025
 *
 * @note Migrated from transport.opendata.ch/v1 (discontinued) to
 *       opentransportdata.swiss GTFS-RT API.
 *
 *       API reference:
 *         https://opentransportdata.swiss/de/cookbook/realtime-prediction-cookbook/gtfs-rt/
 *
 *       Key changes vs. original:
 *         - fetchStationDataFromGPS() removed (no GPS-to-stop lookup in GTFS-RT).
 *           Nearby stops are now defined in a static list in credentials.h.
 *         - fetchStationBoardData() rewrites: calls the GTFS-RT JSON endpoint,
 *           filters trip_update entities by stop_id and collects the next N departures.
 *         - All HTTP calls upgraded to HTTPS with Bearer token in Authorization header.
 *         - User-Agent header added (required by the API).
 *         - Follow-redirect enabled (the API uses cache redirects).
 *         - Rate limit: max 2 requests/minute (30 s cache on server side) – sleep
 *           interval increased to 35 s to stay comfortably within the limit.
 *
 *       How to obtain an API key:
 *         1. Register at https://api-manager.opentransportdata.swiss/
 *         2. Create an Application and subscribe to the "GTFS-RT" API (Default Key).
 *         3. Copy the Bearer token into GTFS_RT_API_KEY in credentials.h.
 *
 *       How to find your GTFS stop IDs:
 *         Download stops.txt from https://data.opentransportdata.swiss/dataset/timetable-2026-gtfs2020
 *         and search for your station name.  IDs look like "8503000" (Zürich HB).
 *
 * @note Arduino IDE Settings
 *       Tools ->
 *             Board:"ESP32S3 Dev Module"
 *             USB CDC On Boot:"Enable"
 *             USB DFU On Boot:"Disable"
 *             Flash Size : "16MB(128Mb)"
 *             Flash Mode"QIO 80MHz
 *             Partition Scheme:"16M Flash(3M APP/9.9MB FATFS)"
 *             PSRAM:"OPI PSRAM"
 *             Upload Mode:"UART0/Hardware CDC"
 *             USB Mode:"Hardware CDC and JTAG"
 */

#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <Arduino.h>
#include "epd_driver.h"
#include "firasans.h"
#include "sbbdisplay.h"
#include "esp_adc_cal.h"
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <esp_sntp.h>
#include <SensorPCF8563.hpp>
#include <TinyGPSPlus.h>        // GPS parsing (TinyGPSPlus library)
#include <HardwareSerial.h>     // UART for GPS module
// WiFi credentials, API key and stop list
#include "credentials.h"

// Older versions of SensorLib did not export this constant publicly.
// Define it here as a fallback so the sketch compiles with any library version.
#ifndef PCF8563_SLAVE_ADDRESS
#define PCF8563_SLAVE_ADDRESS 0x51
#endif

// ---------------------------------------------------------------------------
// Pin definitions
// ---------------------------------------------------------------------------
#define BUTTON_1 (21)
#define BATT_PIN (14)

#define SD_MISO (16)
#define SD_MOSI (15)
#define SD_SCLK (11)
#define SD_CS   (42)

#define BOARD_SCL (17)
#define BOARD_SDA (18)

#define GPIO_MISO (45)
#define GPIO_MOSI (10)
#define GPIO_SCLK (48)
#define GPIO_CS   (39)

// ---------------------------------------------------------------------------
// GTFS-RT endpoint (JSON variant, for testing / embedded use)
// The server caches responses for 30 s, so 2 requests/minute is the effective
// maximum.  Production apps should use the binary protobuf endpoint (no
// ?format=JSON) for better performance.
// ---------------------------------------------------------------------------
#define GTFS_RT_URL "https://api.opentransportdata.swiss/la/gtfs-rt?format=JSON"

// ---------------------------------------------------------------------------
// OJP LocationInformationRequest endpoint (nearest stop lookup by GPS)
// Same Bearer token as GTFS-RT.
// ---------------------------------------------------------------------------
#define OJP_URL "https://api.opentransportdata.swiss/ojp2020"

// ---------------------------------------------------------------------------
// GPS module UART pins (adjust to your wiring)
// ---------------------------------------------------------------------------
#define GPS_RX_PIN  (44)   // ESP32-S3 pin connected to GPS TX
#define GPS_TX_PIN  (43)   // ESP32-S3 pin connected to GPS RX
#define GPS_BAUD    (9600)
#define GPS_SERIAL  Serial1

// Radius (metres) passed to OJP GeoRestriction Circle
#define OJP_RADIUS_M 500

// How long to wait for a GPS fix before giving up (ms)
#define GPS_FIX_TIMEOUT_MS 30000

// ---------------------------------------------------------------------------
// Display
// ---------------------------------------------------------------------------
uint8_t *framebuffer;
int vref = 1100;

// ---------------------------------------------------------------------------
// Station / board state
// ---------------------------------------------------------------------------
int  stationIndex = 0;                     // index into STOP_IDS[] / STOP_NAMES[]
const int numEntries = 4;                  // rows shown on the departure board
StationBoardData stationBoardData[numEntries];

// ---------------------------------------------------------------------------
// GPS / nearest-stop state
// Dynamic stop (overrides the static list when a GPS fix is obtained)
// ---------------------------------------------------------------------------
TinyGPSPlus gps;
bool  gpsStopActive  = false;   // true when nearestStopId was set by GPS lookup
String nearestStopId   = "";
String nearestStopName = "";

// ---------------------------------------------------------------------------
// Button / timing
// ---------------------------------------------------------------------------
volatile bool buttonPressed = false;
// 35 s > 30 s server-side cache window; keeps us well within 2 req/min limit
const unsigned long sleepInterval = 35000;

// ---------------------------------------------------------------------------
// RTC / NTP
// ---------------------------------------------------------------------------
SensorPCF8563 rtc;
char buf[128];
const char *ntpServer1       = "pool.ntp.org";
const char *ntpServer2       = "time.nist.gov";
const long  gmtOffset_sec    = 3600;
const int   daylightOffset_sec = 3600;
const char *time_zone        = "CET-1CEST,M3.5.0/2,M10.5.0/3";

// ---------------------------------------------------------------------------
// ISR
// ---------------------------------------------------------------------------
void IRAM_ATTR selectStationID() {
  buttonPressed = true;
}

// ---------------------------------------------------------------------------
// NTP callback
// ---------------------------------------------------------------------------
void timeavailable(struct timeval *t) {
  Serial.println("[WiFi]: Got time adjustment from NTP!");
  rtc.hwClockWrite();
}

// ===========================================================================
// setup()
// ===========================================================================
void setup() {
  Serial.begin(115200);

  connectWifi();

  pinMode(BUTTON_1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_1), selectStationID, RISING);

  sntp_set_time_sync_notification_cb(timeavailable);
  configTzTime(time_zone, ntpServer1, ntpServer2);

  Wire.begin(BOARD_SDA, BOARD_SCL);
  Wire.beginTransmission(PCF8563_SLAVE_ADDRESS);
  if (Wire.endTransmission() == 0) {
    rtc.begin(Wire, PCF8563_SLAVE_ADDRESS, BOARD_SDA, BOARD_SCL);
    Serial.println("RTC initialization failed!");
  } else {
    Serial.println("RTC is online");
  }

  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(
      ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
    Serial.printf("eFuse Vref:%u mV", adc_chars.vref);
    vref = adc_chars.vref;
  }

  epd_init();
  framebuffer = (uint8_t *)ps_calloc(sizeof(uint8_t), EPD_WIDTH * EPD_HEIGHT / 2);
  if (!framebuffer) {
    Serial.println("alloc memory failed !!!");
    while (1);
  }
  memset(framebuffer, 0xFF, EPD_WIDTH * EPD_HEIGHT / 2);

  epd_poweron();
  epd_clear();
  epd_poweroff();

  Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());

  // Try GPS-based nearest stop first; fall back to static list if no fix
  GPS_SERIAL.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  fetchNearestStop();

  title();
  displayStationHeader();
  fetchStationBoardData();
  displayStationBoardData();
}

// ===========================================================================
// loop()
// ===========================================================================
void loop() {
  struct tm timeinfo;
  rtc.getDateTime(&timeinfo);
  strftime(buf, 64, "➸ %b %d %Y %H:%M:%S", &timeinfo);
  Serial.print("RTC: ");
  Serial.println(buf);

  readBatVoltage();
  displayTime();

  if (buttonPressed) {
    buttonPressed = false;
    if (gpsStopActive) {
      // Second press: leave GPS mode, cycle to first static stop
      gpsStopActive = false;
      stationIndex  = 0;
    } else {
      // Cycle static list; when it wraps, try GPS again
      stationIndex = (stationIndex + 1) % NUM_STOPS;
      if (stationIndex == 0) {
        fetchNearestStop();   // attempt fresh GPS fix at wrap-around
      }
    }
    displayStationHeader();
    fetchStationBoardData();
    displayStationBoardData();
  } else {
    fetchStationBoardData();
    displayStationBoardData();
  }

  unsigned long startTime = millis();
  while (!buttonPressed && (millis() - startTime < sleepInterval)) {
    delay(10);
  }
}

// ===========================================================================
// fetchStationBoardData()
//
// Calls the GTFS-RT JSON endpoint and extracts the next N departures from the
// currently selected stop (STOP_IDS[stationIndex]).
//
// GTFS-RT JSON structure (abbreviated):
// {
//   "Header": { ... },
//   "Entities": [
//     {
//       "Id": "...",
//       "TripUpdate": {
//         "Trip": { "TripId": "...", "RouteId": "...", ... },
//         "StopTimeUpdate": [
//           {
//             "StopId": "8503000",
//             "Departure": { "Delay": 120, "Time": 1727430000 }
//           }, ...
//         ]
//       }
//     }, ...
//   ]
// }
//
// We iterate over all entities, look for a StopTimeUpdate whose StopId
// matches our target stop, and collect the soonest departures.
// ===========================================================================
void fetchStationBoardData() {
  const String targetStopId = gpsStopActive
                                ? nearestStopId
                                : String(STOP_IDS[stationIndex]);
  Serial.println("Fetching GTFS-RT for stop: " + targetStopId);

  WiFiClientSecure client;
  client.setInsecure();  // Skip TLS certificate validation on embedded device.
                         // For production, load the root CA into client.setCACert().

  HTTPClient http;
  http.begin(client, GTFS_RT_URL);
  http.addHeader("Authorization", "Bearer " + String(GTFS_RT_API_KEY));
  http.addHeader("User-Agent", "SBB-EPaper-Display/2.0 ESP32");
  http.addHeader("Accept-Encoding", "gzip, deflate");
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);  // required by API

  int httpCode = http.GET();
  if (httpCode != HTTP_CODE_OK) {
    Serial.println("GTFS-RT HTTP error: " + String(httpCode));
    if (httpCode > 0) {
      // Print body so we can see rate-limit messages, etc.
      Serial.println(http.getString().substring(0, 200));
    }
    http.end();
    return;
  }

  // The JSON response can be up to ~11 MB; we stream-parse it with a filter
  // document so that ArduinoJson only keeps what we need in RAM.
  //
  // Filter: keep only the fields used below.
  StaticJsonDocument<256> filter;
  filter["Entities"][0]["TripUpdate"]["Trip"]["RouteId"]             = true;
  filter["Entities"][0]["TripUpdate"]["StopTimeUpdate"][0]["StopId"] = true;
  filter["Entities"][0]["TripUpdate"]["StopTimeUpdate"][0]["Departure"]["Time"]  = true;
  filter["Entities"][0]["TripUpdate"]["StopTimeUpdate"][0]["Departure"]["Delay"] = true;

  // Allocate a large dynamic document.  Adjust if you see allocation errors.
  DynamicJsonDocument doc(131072);  // 128 KB – enough for the filtered payload

  DeserializationError error = deserializeJson(doc, http.getStream(),
                                               DeserializationOption::Filter(filter));
  http.end();

  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  // Collect departures from the target stop
  int found = 0;
  time_t now;
  time(&now);  // current Unix timestamp

  JsonArray entities = doc["Entities"].as<JsonArray>();
  for (JsonObject entity : entities) {
    if (found >= numEntries) break;

    JsonObject tripUpdate = entity["TripUpdate"];
    if (tripUpdate.isNull()) continue;

    String routeId = tripUpdate["Trip"]["RouteId"].as<String>();

    JsonArray stopTimeUpdates = tripUpdate["StopTimeUpdate"].as<JsonArray>();
    for (JsonObject stu : stopTimeUpdates) {
      String stopId = stu["StopId"].as<String>();
      if (stopId != targetStopId) continue;

      JsonObject dep = stu["Departure"];
      if (dep.isNull()) continue;

      long  depTime  = dep["Time"].as<long>();
      int   delay    = dep["Delay"].as<int>() / 60;  // seconds → minutes

      // Skip departures that are already in the past (>30 s ago)
      if (depTime < (now - 30)) continue;

      // Format departure time as HH:MM from Unix timestamp
      time_t t = (time_t)depTime;
      struct tm *tm_info = localtime(&t);
      char timeBuf[6];
      strftime(timeBuf, sizeof(timeBuf), "%H:%M", tm_info);

      stationBoardData[found].line          = routeId;
      stationBoardData[found].destination   = "";  // not in GTFS-RT Trip Updates
      stationBoardData[found].departure_time = String(timeBuf);
      stationBoardData[found].delay         = delay;
      stationBoardData[found].type          = "";
      stationBoardData[found].line_operator = "";
      found++;
      break;  // only one departure per trip at this stop
    }
  }

  // Clear any unused rows
  for (int i = found; i < numEntries; i++) {
    stationBoardData[i].line          = "-";
    stationBoardData[i].destination   = "";
    stationBoardData[i].departure_time = "--:--";
    stationBoardData[i].delay         = 0;
  }

  Serial.printf("Found %d departures for stop %s\n", found, targetStopId.c_str());
}

// ===========================================================================
// Display helpers
// ===========================================================================

void readBatVoltage() {
  delay(10);
  uint16_t v = analogRead(BATT_PIN);
  float battery_voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
  if (battery_voltage >= 4.2) battery_voltage = 4.2;
  Serial.println("➸ Voltage: " + String(battery_voltage) + "V");
}

void title() {
  int32_t cursor_x, cursor_y;

  cursor_x = 30;  cursor_y = 50;
  writeln((GFXfont *)&FiraSans, (char *)"Abfahrt:", &cursor_x, &cursor_y, NULL);
}

// Show the current stop name on the display
void displayStationHeader() {
  epd_clear_area({ 250, 10, 700, 50 });

  int32_t cursor_x = 250, cursor_y = 50;
  String label = gpsStopActive ? nearestStopName : String(STOP_NAMES[stationIndex]);
  char buf_local[label.length() + 1];
  label.toCharArray(buf_local, label.length() + 1);
  writeln((GFXfont *)&FiraSans, buf_local, &cursor_x, &cursor_y, NULL);
}

void displayStationBoardData() {
  int32_t cursor_x, cursor_y;

  int base_y     = 300;
  int row_height = 55;
  int col1_x     = 30;   // line/route
  int col2_x     = 100;  // destination (empty in GTFS-RT TU, reserved)
  int col3_x     = 765;  // departure time
  int col4_x     = 855;  // delay

  for (int i = 0; i < numEntries; i++) {
    int current_y = base_y + (i * row_height);
    epd_clear_area({ col1_x - 2, current_y - 45, 910, row_height + 5 });

    // Line / Route ID
    cursor_x = col1_x;  cursor_y = current_y;
    char line[stationBoardData[i].line.length() + 1];
    stationBoardData[i].line.toCharArray(line, stationBoardData[i].line.length() + 1);
    writeln((GFXfont *)&FiraSans, line, &cursor_x, &cursor_y, NULL);

    // Destination (GTFS-RT Trip Updates don't carry headsign; left blank)
    cursor_x = col2_x;
    char dest[stationBoardData[i].destination.length() + 2];
    stationBoardData[i].destination.toCharArray(dest, stationBoardData[i].destination.length() + 2);
    writeln((GFXfont *)&FiraSans, dest, &cursor_x, &cursor_y, NULL);

    // Departure time
    cursor_x = col3_x;
    char depTime[stationBoardData[i].departure_time.length() + 1];
    stationBoardData[i].departure_time.toCharArray(depTime, stationBoardData[i].departure_time.length() + 1);
    writeln((GFXfont *)&FiraSans, depTime, &cursor_x, &cursor_y, NULL);

    // Delay (only display if non-zero)
    cursor_x = col4_x;
    String delayStr = (stationBoardData[i].delay > 0)
                        ? "+" + String(stationBoardData[i].delay)
                        : "  ";
    char delay[delayStr.length() + 1];
    delayStr.toCharArray(delay, delayStr.length() + 1);
    writeln((GFXfont *)&FiraSans, delay, &cursor_x, &cursor_y, NULL);
  }
}

void displayTime() {
  int32_t cursor_x = 500, cursor_y = 100;
  int col_x = (int)cursor_x, col_y = (int)cursor_y;
  int row_height = 50;

  struct tm timeInfo;
  rtc.getDateTime(&timeInfo);

  Rect_t clearRect = { col_x - 2, col_y - 45, 125, row_height + 5 };
  epd_clear_area(clearRect);

  char timeBuffer[6];
  strftime(timeBuffer, sizeof(timeBuffer), "%H:%M", &timeInfo);
  writeln((GFXfont *)&FiraSans, timeBuffer, &cursor_x, &cursor_y, NULL);
}

// ===========================================================================
// fetchNearestStop()
//
// 1. Reads NMEA sentences from the GPS module until a valid fix is obtained
//    (or GPS_FIX_TIMEOUT_MS elapses).
// 2. Sends an OJP LocationInformationRequest (XML/POST) to the OJP API with
//    the GPS coordinates and a GeoRestriction Circle of OJP_RADIUS_M metres.
// 3. Parses the first StopPointRef and its LocationName from the XML response
//    using simple string search – no full XML library needed on the ESP32.
// 4. On success, sets gpsStopActive=true and stores nearestStopId / Name.
//    On failure (no fix, HTTP error, parse error) leaves the static stop list
//    in effect.
//
// OJP API reference:
//   https://opentransportdata.swiss/cookbook/ojplocationinformationrequest/
// ===========================================================================
void fetchNearestStop() {
  Serial.println("[GPS] Waiting for fix...");

  double lat = 0.0, lng = 0.0;
  bool   fixOk = false;
  unsigned long startMs = millis();

  while (millis() - startMs < GPS_FIX_TIMEOUT_MS) {
    while (GPS_SERIAL.available()) {
      gps.encode(GPS_SERIAL.read());
    }
    if (gps.location.isValid() && gps.location.age() < 2000) {
      lat   = gps.location.lat();
      lng   = gps.location.lng();
      fixOk = true;
      break;
    }
    delay(100);
  }

  if (!fixOk) {
    Serial.println("[GPS] No fix – using static stop list.");
    gpsStopActive = false;
    return;
  }

  Serial.printf("[GPS] Fix: lat=%.6f lng=%.6f\n", lat, lng);

  // -------------------------------------------------------------------------
  // Build OJP LocationInformationRequest XML
  // Uses GeoRestriction/Circle so the server returns the nearest stops.
  // RequestorRef must end in _prod / _int / _test per API convention.
  // -------------------------------------------------------------------------
  String reqBody =
    "<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
    "<OJP xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\""
    " xmlns:xsd=\"http://www.w3.org/2001/XMLSchema\""
    " xmlns=\"http://www.siri.org.uk/siri\" version=\"1.0\""
    " xmlns:ojp=\"http://www.vdv.de/ojp\""
    " xsi:schemaLocation=\"http://www.siri.org.uk/siri ../ojp-xsd-v1.0/OJP.xsd\">"
    "<OJPRequest><ServiceRequest>"
    "<RequestTimestamp>2020-01-01T00:00:00Z</RequestTimestamp>"
    "<RequestorRef>SBB-EPaper-Display_prod</RequestorRef>"
    "<ojp:OJPLocationInformationRequest>"
    "<RequestTimestamp>2020-01-01T00:00:00Z</RequestTimestamp>"
    "<MessageIdentifier>1</MessageIdentifier>"
    "<ojp:InitialInput>"
    "<ojp:GeoRestriction>"
    "<ojp:Circle>"
    "<ojp:Center>"
    "<Longitude>" + String(lng, 6) + "</Longitude>"
    "<Latitude>"  + String(lat, 6) + "</Latitude>"
    "</ojp:Center>"
    "<ojp:Radius>" + String(OJP_RADIUS_M) + "</ojp:Radius>"
    "</ojp:Circle>"
    "</ojp:GeoRestriction>"
    "</ojp:InitialInput>"
    "<ojp:Restrictions>"
    "<ojp:Type>stop</ojp:Type>"
    "<ojp:NumberOfResults>1</ojp:NumberOfResults>"
    "<ojp:IncludePtModes>false</ojp:IncludePtModes>"
    "</ojp:Restrictions>"
    "</ojp:OJPLocationInformationRequest>"
    "</ServiceRequest></OJPRequest></OJP>";

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  http.begin(client, OJP_URL);
  http.addHeader("Content-Type",  "application/xml");
  http.addHeader("Authorization", "Bearer " + String(GTFS_RT_API_KEY));
  http.addHeader("User-Agent",    "SBB-EPaper-Display/2.0 ESP32");
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  int httpCode = http.POST(reqBody);
  if (httpCode != HTTP_CODE_OK) {
    Serial.println("[OJP] HTTP error: " + String(httpCode));
    http.end();
    gpsStopActive = false;
    return;
  }

  String response = http.getString();
  http.end();

  // -------------------------------------------------------------------------
  // Lightweight XML parse: extract first StopPointRef and LocationName text.
  //
  // Response snippet we're looking for:
  //   <siri:StopPointRef>8503000</siri:StopPointRef>
  //   ...
  //   <ojp:LocationName><ojp:Text>Zürich HB</ojp:Text></ojp:LocationName>
  // -------------------------------------------------------------------------
  auto extractTag = [](const String &xml, const String &openTag, const String &closeTag) -> String {
    int start = xml.indexOf(openTag);
    if (start < 0) return "";
    start += openTag.length();
    int end = xml.indexOf(closeTag, start);
    if (end < 0) return "";
    return xml.substring(start, end);
  };

  String stopRef  = extractTag(response, "<siri:StopPointRef>", "</siri:StopPointRef>");
  String stopName = extractTag(response, "<ojp:Text>",          "</ojp:Text>");

  if (stopRef.length() == 0) {
    Serial.println("[OJP] Could not parse StopPointRef – keeping static list.");
    gpsStopActive = false;
    return;
  }

  nearestStopId   = stopRef;
  nearestStopName = (stopName.length() > 0) ? stopName : stopRef;
  gpsStopActive   = true;

  Serial.println("[OJP] Nearest stop: " + nearestStopName + " (" + nearestStopId + ")");
}

// ===========================================================================
void connectWifi() {
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected – IP: " + WiFi.localIP().toString());
}

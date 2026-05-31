/**
 * @copyright Hervé Cousin (original), updated 2025 for OJP API
 * @date      2024-07-12 / updated 2025
 *
 * Migrated from transport.opendata.ch/v1 (discontinued) to
 * opentransportdata.swiss OJP API.
 *
 * This file is kept as close as possible to the original to avoid
 * introducing new crash sources. Only the API call functions and
 * credentials differ from the original.
 *
 * Arduino IDE Settings:
 *   Board:            ESP32S3 Dev Module
 *   USB CDC On Boot:  Enable
 *   Flash Size:       16MB(128Mb)
 *   Partition Scheme: 16M Flash(3M APP/9.9MB FATFS)
 *   PSRAM:            OPI PSRAM
 *   Upload Mode:      UART0/Hardware CDC
 *   USB Mode:         Hardware CDC and JTAG
 */

// ---------------------------------------------------------------------------
// Includes – identical to original except:
//   ArduinoJson    removed  (not needed for OJP XML parsing)
//   WiFiClientSecure added  (HTTPS for OJP API) – used only inside functions
//   TinyGPSPlus    added    (GPS NMEA parsing)  – used only inside fetchGPSFix()
// ---------------------------------------------------------------------------
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
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
#include <TinyGPSPlus.h>
#include "credentials.h"

#ifndef PCF8563_SLAVE_ADDRESS
#define PCF8563_SLAVE_ADDRESS 0x51
#endif

// ---------------------------------------------------------------------------
// Pin definitions – identical to original
// ---------------------------------------------------------------------------
#define BUTTON_1  (21)
#define BATT_PIN  (14)
#define SD_MISO   (16)
#define SD_MOSI   (15)
#define SD_SCLK   (11)
#define SD_CS     (42)
#define BOARD_SCL (17)
#define BOARD_SDA (18)
#define GPIO_MISO (45)
#define GPIO_MOSI (10)
#define GPIO_SCLK (48)
#define GPIO_CS   (39)

// GPS UART pins
#define GPS_RX_PIN         (44)
#define GPS_TX_PIN         (43)
#define GPS_BAUD           (9600)
#define GPS_FIX_TIMEOUT_MS (30000)

// OJP API - note: OJP 2.0 endpoint is /ojp20 (not /ojp2020)
#define OJP_URL      "https://api.opentransportdata.swiss/ojp20"
#define OJP_RADIUS_M 500

// Set to false if no GPS module is connected – skips the 30s wait
// and uses DEFAULT_LAT / DEFAULT_LNG from credentials.h immediately
#define GPS_ENABLED  false

// ---------------------------------------------------------------------------
// Globals – identical to original
// ---------------------------------------------------------------------------
uint8_t *framebuffer;
int vref = 1100;

GpsData gpsData;
String xCoord = DEFAULT_LAT;
String yCoord = DEFAULT_LNG;

int stationIndex = 0;
const int maxStations = 10;
StationData stationDataArray[maxStations];
int stationsFound = 0;   // actual number of stops returned by OJP
String stationID = "";
const int numEntries = 4;
StationBoardData stationBoardData[numEntries];

volatile int buttonPressed = false;
const unsigned long sleepInterval = 45000;

SensorPCF8563 rtc;
char buf[128];
const char *ntpServer1        = "pool.ntp.org";
const char *ntpServer2        = "time.nist.gov";
const long  gmtOffset_sec     = 3600;
const int   daylightOffset_sec = 3600;
const char *time_zone = "CET-1CEST,M3.5.0/2,M10.5.0/3";

// ---------------------------------------------------------------------------
// ISR – identical to original
// ---------------------------------------------------------------------------
void IRAM_ATTR selectStationID() {
  buttonPressed = true;
}

void timeavailable(struct timeval *t) {
  Serial.println("[WiFi]: Got time adjustment from NTP!");
  rtc.hwClockWrite();
}

// ===========================================================================
// setup() – identical to original
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

  fetchGPSFix();
  fetchGPSAddress();
  fetchStationDataFromGPS();
  epd_poweron();
  title();
  displayStationData();
  fetchStationBoardData();
  displayStationBoardData();
  epd_poweroff();
}

// ===========================================================================
// loop() – identical to original
// ===========================================================================
void loop() {
  struct tm timeinfo;
  rtc.getDateTime(&timeinfo);
  strftime(buf, 64, "➸ %b %d %Y %H:%M:%S", &timeinfo);
  Serial.print("RTC: ");
  Serial.println(buf);

  readBatVoltage();
  epd_poweron();
  displayTime();
  epd_poweroff();

  Serial.println(buttonPressed);
  Serial.print("Station Index before if = ");
  Serial.println(stationIndex);

  if (buttonPressed) {
    if (stationIndex >= stationsFound - 1) {
      stationIndex = 0;
    } else {
      stationIndex++;
    }
    buttonPressed = false;
    epd_poweron();
    displayStationData();
    fetchStationBoardData();
    displayStationBoardData();
    epd_poweroff();
  } else {
    fetchStationBoardData();
    epd_poweron();
    displayStationBoardData();
    epd_poweroff();
  }

  unsigned long startTime = millis();
  while (!buttonPressed && (millis() - startTime < sleepInterval)) {
    delay(10);
  }
  Serial.println("Nach 45s");
}

// ===========================================================================
// fetchGPSAddress()
// Reverse-geocodes xCoord/yCoord to a street address using Nominatim
// (OpenStreetMap). No API key required.
// Fills gpsData.x_coord with a formatted address string, e.g.:
//   "Dufaux-Str. 65, Glattpark (Opfikon)"
// Falls back to showing coordinates if the request fails.
// ===========================================================================
void fetchGPSAddress() {
  String url = "https://nominatim.openstreetmap.org/reverse?format=json&lat=" +
               xCoord + "&lon=" + yCoord + "&zoom=18&addressdetails=1";

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.begin(client, url);
  http.addHeader("User-Agent", "SBB-EPaper-Display/2.0 ESP32");
  http.setTimeout(10000);

  int code = http.GET();
  if (code != HTTP_CODE_OK) {
    Serial.println("[Nominatim] HTTP error: " + String(code));
    gpsData.x_coord = xCoord + " / " + yCoord;
    http.end();
    return;
  }

  String response = http.getString();
  http.end();

  // Parse the display_name field from JSON – simple string search
  // "display_name":"Dufaux-Strasse, 65, Glattpark, Opfikon, ..."
  auto extractJson = [](const String &json, const String &key) -> String {
    String search = "\"" + key + "\":\"";
    int s = json.indexOf(search);
    if (s < 0) return "";
    s += search.length();
    int e = json.indexOf("\"", s);
    if (e < 0) return "";
    return json.substring(s, e);
  };

  // Build a short address from individual fields rather than the full
  // display_name (which includes country, canton etc.)
  String road    = extractJson(response, "road");
  String houseNo = extractJson(response, "house_number");
  String suburb  = extractJson(response, "suburb");
  String city    = extractJson(response, "city");
  if (city.length() == 0) city = extractJson(response, "town");
  if (city.length() == 0) city = extractJson(response, "village");

  String address = "";
  if (road.length() > 0) {
    address = road;
    if (houseNo.length() > 0) address += " " + houseNo;
  }
  if (suburb.length() > 0 && suburb != city) {
    address += (address.length() > 0 ? ", " : "") + suburb;
  }
  if (city.length() > 0 && city != suburb) {
    address += (address.length() > 0 ? " (" : "") + city;
    if (suburb.length() > 0 && suburb != city) address += ")";
  }

  if (address.length() == 0) address = xCoord + " / " + yCoord;

  Serial.println("[Nominatim] Address: " + address);
  gpsData.x_coord = address;
}

// ===========================================================================
// NEW: reads NMEA from GPS module, updates xCoord/yCoord if fix obtained.
// TinyGPSPlus instantiated locally to avoid global constructor issues.
// ===========================================================================
void fetchGPSFix() {
  if (!GPS_ENABLED) {
    Serial.println("[GPS] GPS_ENABLED=false – using fallback coordinates.");
    Serial.println("[GPS] lat=" + xCoord + " lng=" + yCoord);
    return;
  }
  Serial.println("[GPS] Waiting for fix...");
  Serial1.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  TinyGPSPlus gpsLocal;
  unsigned long startMs = millis();
  while (millis() - startMs < GPS_FIX_TIMEOUT_MS) {
    while (Serial1.available()) gpsLocal.encode(Serial1.read());
    if (gpsLocal.location.isValid() && gpsLocal.location.age() < 2000) {
      xCoord = String(gpsLocal.location.lat(), 9);
      yCoord = String(gpsLocal.location.lng(), 9);
      Serial.println("[GPS] Fix: lat=" + xCoord + " lng=" + yCoord);
      return;
    }
    delay(100);
  }
  Serial.println("[GPS] No fix – using fallback coordinates.");
}

// ===========================================================================
// ojpPost() – shared HTTPS POST helper for OJP API calls
// WiFiClientSecure instantiated locally to avoid global constructor issues.
// ===========================================================================
String ojpPost(const String &body) {
  WiFiClientSecure client;
  client.setInsecure();
  client.setTimeout(10000);  // 10s TCP timeout
  HTTPClient http;
  http.begin(client, OJP_URL);
  http.setTimeout(10000);    // 10s HTTP timeout
  http.addHeader("Content-Type",  "application/xml");
  http.addHeader("Authorization", String("Bearer ") + OJP_API_KEY);
  http.addHeader("User-Agent",    "SBB-EPaper-Display/2.0 ESP32");
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  Serial.println("[OJP] POST to " + String(OJP_URL));
  int code = http.POST(body);
  Serial.println("[OJP] Response code: " + String(code));
  if (code != HTTP_CODE_OK) {
    Serial.println("[OJP] Error body: " + http.getString().substring(0, 300));
    http.end();
    return "";
  }
  String response = http.getString();
  Serial.println("[OJP] Response length: " + String(response.length()));
  Serial.println("[OJP] First 200 chars: " + response.substring(0, 200));
  http.end();
  return response;
}

// ===========================================================================
// extractTag() – lightweight XML field extractor
// ===========================================================================
String extractTag(const String &xml, const String &open,
                  const String &close, int from = 0) {
  int s = xml.indexOf(open, from);
  if (s < 0) return "";
  s += open.length();
  int e = xml.indexOf(close, s);
  if (e < 0) return "";
  return xml.substring(s, e);
}

// ===========================================================================
// fetchStationDataFromGPS()
// CHANGED: uses OJP LocationInformationRequest instead of
//          transport.opendata.ch/v1/locations
// Fills stationDataArray[] with identical fields to the original.
// ===========================================================================
void fetchStationDataFromGPS() {
  Serial.println("[OJP] LocationInfo lat=" + xCoord + " lng=" + yCoord);

  time_t now; time(&now);
  struct tm *utc = gmtime(&now);
  char tsNow[25];
  strftime(tsNow, sizeof(tsNow), "%Y-%m-%dT%H:%M:%SZ", utc);

  String body =
    "<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
    "<OJP xmlns=\"http://www.vdv.de/ojp\""
    " xmlns:siri=\"http://www.siri.org.uk/siri\""
    " version=\"2.0\""
    " xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\">"
    "<OJPRequest>"
    "<siri:ServiceRequest>"
    "<siri:RequestTimestamp>" + String(tsNow) + "</siri:RequestTimestamp>"
    "<siri:RequestorRef>SBB-EPaper-Display_prod</siri:RequestorRef>"
    "<OJPLocationInformationRequest>"
    "<siri:RequestTimestamp>" + String(tsNow) + "</siri:RequestTimestamp>"
    "<siri:MessageIdentifier>LIR-1</siri:MessageIdentifier>"
    "<InitialInput>"
    "<GeoRestriction>"
    "<Circle>"
    "<Center>"
    "<siri:Longitude>" + yCoord + "</siri:Longitude>"
    "<siri:Latitude>"  + xCoord + "</siri:Latitude>"
    "</Center>"
    "<Radius>" + String(OJP_RADIUS_M) + "</Radius>"
    "</Circle>"
    "</GeoRestriction>"
    "</InitialInput>"
    "<Restrictions>"
    "<Type>stop</Type>"
    "<NumberOfResults>" + String(maxStations) + "</NumberOfResults>"
    "</Restrictions>"
    "</OJPLocationInformationRequest>"
    "</siri:ServiceRequest>"
    "</OJPRequest>"
    "</OJP>";

  String response = ojpPost(body);
  if (response.length() == 0) return;
  Serial.println("[OJP] LocationInfo full response: " + response);

  const String LOC_OPEN  = "<PlaceResult>";
  const String LOC_CLOSE = "</PlaceResult>";
  int found = 0, pos = 0;
  String firstStopName = "";

  while (found < maxStations) {
    int bs = response.indexOf(LOC_OPEN, pos);
    if (bs < 0) break;
    int be = response.indexOf(LOC_CLOSE, bs);
    if (be < 0) break;
    String block = response.substring(bs, be + LOC_CLOSE.length());
    pos = be + LOC_CLOSE.length();

    // OJP 2.0 response structure:
    // <StopPlaceRef>8590633</StopPlaceRef>
    // <StopPlaceName><Text xml:lang="de">Glattpark, Chavez-Allee</Text></StopPlaceName>
    // <GeoPosition><siri:Longitude>8.56</siri:Longitude><siri:Latitude>47.42</siri:Latitude></GeoPosition>
    String stopRef  = extractTag(block, "<StopPlaceRef>",  "</StopPlaceRef>");
    String stopName = extractTag(block, "<StopPlaceName><Text xml:lang=\"de\">", "</Text>");
    if (stopName.length() == 0)
      stopName = extractTag(block, "<StopPlaceName><Text>", "</Text>");
    String latStr   = extractTag(block, "<siri:Latitude>",  "</siri:Latitude>");
    String lngStr   = extractTag(block, "<siri:Longitude>", "</siri:Longitude>");

    if (stopRef.length() == 0 || stopName.length() == 0) continue;
    if (found == 0) firstStopName = stopName;

    // Haversine distance
    double myLat = xCoord.toDouble(), myLng = yCoord.toDouble();
    double stopLat = latStr.toDouble(), stopLng = lngStr.toDouble();
    int dist = 0;
    if (stopLat != 0.0 && stopLng != 0.0) {
      double dLat = (stopLat - myLat) * PI / 180.0;
      double dLng = (stopLng - myLng) * PI / 180.0;
      double a = sin(dLat/2)*sin(dLat/2) +
                 cos(myLat*PI/180.0)*cos(stopLat*PI/180.0)*sin(dLng/2)*sin(dLng/2);
      dist = (int)(6371000.0 * 2.0 * atan2(sqrt(a), sqrt(1.0 - a)));
    }

    stationDataArray[found].gps_address  = firstStopName;
    stationDataArray[found].near_station = stopName;
    stationDataArray[found].distance     = dist;
    stationDataArray[found].station_id   = stopRef;

    Serial.printf("[OJP] Stop %d: %s  id=%s  dist=%dm\n",
                  found, stopName.c_str(), stopRef.c_str(), dist);
    found++;
  }

  if (found > 0) {
    stationsFound = found;
    stationID = stationDataArray[stationIndex].station_id;
    Serial.println("[OJP] Active: " + stationDataArray[stationIndex].near_station +
                   " (" + String(stationDataArray[stationIndex].distance) + "m)");
  }
}

// ===========================================================================
// fetchStationBoardData()
// CHANGED: uses OJP StopEventRequest instead of
//          transport.opendata.ch/v1/stationboard
// Fills stationBoardData[] with identical fields to the original.
// ===========================================================================
void fetchStationBoardData() {
  if (stationDataArray[stationIndex].station_id.length() == 0) return;

  stationID = stationDataArray[stationIndex].station_id;
  Serial.println("[OJP] StopEvent for: " + stationID);

  time_t now; time(&now);
  struct tm *utc = gmtime(&now);
  char tsNow[25];
  strftime(tsNow, sizeof(tsNow), "%Y-%m-%dT%H:%M:%SZ", utc);

  String body =
    "<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
    "<OJP xmlns=\"http://www.vdv.de/ojp\""
    " xmlns:siri=\"http://www.siri.org.uk/siri\""
    " xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\""
    " version=\"2.0\">"
    "<OJPRequest>"
    "<siri:ServiceRequest>"
    "<siri:RequestTimestamp>" + String(tsNow) + "</siri:RequestTimestamp>"
    "<siri:RequestorRef>SBB-EPaper-Display_prod</siri:RequestorRef>"
    "<OJPStopEventRequest>"
    "<siri:RequestTimestamp>" + String(tsNow) + "</siri:RequestTimestamp>"
    "<siri:MessageIdentifier>SER-1</siri:MessageIdentifier>"
    "<Location>"
    "<PlaceRef>"
    "<siri:StopPointRef>" + stationID + "</siri:StopPointRef>"
    "<Name><Text>stop</Text></Name>"
    "</PlaceRef>"
    "<DepArrTime>" + String(tsNow) + "</DepArrTime>"
    "</Location>"
    "<Params>"
    "<NumberOfResults>" + String(numEntries) + "</NumberOfResults>"
    "<StopEventType>departure</StopEventType>"
    "<UseRealtimeData>full</UseRealtimeData>"
    "</Params>"
    "</OJPStopEventRequest>"
    "</siri:ServiceRequest>"
    "</OJPRequest>"
    "</OJP>";

  String response = ojpPost(body);
  if (response.length() == 0) return;
  Serial.println("[OJP] StopEvent length=" + String(response.length()));
  Serial.println("[OJP] StopEvent chars 600-1200: " + response.substring(600, 1200));
  auto isoToHHMM = [](const String &iso) -> String {
    if (iso.length() < 16) return "--:--";
    struct tm t = {};
    t.tm_year  = iso.substring(0,4).toInt() - 1900;
    t.tm_mon   = iso.substring(5,7).toInt() - 1;
    t.tm_mday  = iso.substring(8,10).toInt();
    t.tm_hour  = iso.substring(11,13).toInt();
    t.tm_min   = iso.substring(14,16).toInt();
    t.tm_sec   = 0;
    t.tm_isdst = -1;
    time_t utcT = mktime(&t);
    struct tm *local = localtime(&utcT);
    char out[6];
    strftime(out, sizeof(out), "%H:%M", local);
    return String(out);
  };

  const String SE_OPEN  = "<StopEvent>";
  const String SE_CLOSE = "</StopEvent>";
  int found = 0, pos = 0;

  // Debug: print first StopEvent block
  int dbgStart = response.indexOf(SE_OPEN);
  int dbgEnd   = response.indexOf(SE_CLOSE, dbgStart);
  if (dbgStart >= 0 && dbgEnd >= 0) {
    Serial.println("[OJP] First StopEvent block:");
    Serial.println(response.substring(dbgStart, dbgEnd + SE_CLOSE.length()));
  }

  while (found < numEntries) {
    int bs = response.indexOf(SE_OPEN, pos);
    if (bs < 0) break;
    int be = response.indexOf(SE_CLOSE, bs);
    if (be < 0) break;
    String block = response.substring(bs, be + SE_CLOSE.length());
    pos = be + SE_CLOSE.length();

    String timetabled = extractTag(block, "<TimetabledTime>", "</TimetabledTime>");
    String estimated  = extractTag(block, "<EstimatedTime>",  "</EstimatedTime>");

    String depTime = isoToHHMM(estimated.length() > 0 ? estimated : timetabled);

    int delayMin = 0;
    if (estimated.length() > 0 && timetabled.length() > 0) {
      int tH = timetabled.substring(11,13).toInt();
      int tM = timetabled.substring(14,16).toInt();
      int eH = estimated.substring(11,13).toInt();
      int eM = estimated.substring(14,16).toInt();
      delayMin = (eH*60 + eM) - (tH*60 + tM);
      if (delayMin < 0) delayMin += 1440;
    }

    String lineName = extractTag(block, "<PublishedLineName><Text>", "</Text>");
    String dest     = extractTag(block, "<DestinationText><Text>",   "</Text>");

    stationBoardData[found].line_operator = "";
    stationBoardData[found].type          = "";
    stationBoardData[found].line          = lineName.length() > 0 ? lineName : "-";
    stationBoardData[found].destination   = dest;
    stationBoardData[found].departure_time = depTime;
    stationBoardData[found].delay         = delayMin;

    Serial.printf("[OJP] %d: %s → %s  %s  +%d\n",
                  found, stationBoardData[found].line.c_str(),
                  stationBoardData[found].destination.c_str(),
                  stationBoardData[found].departure_time.c_str(), delayMin);
    found++;
  }

  for (int i = found; i < numEntries; i++) {
    stationBoardData[i].line           = "-";
    stationBoardData[i].destination    = "";
    stationBoardData[i].departure_time = "--:--";
    stationBoardData[i].delay          = 0;
  }
}

// ===========================================================================
// Display functions – IDENTICAL to original
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
  cursor_x = 30; cursor_y = 50;
  writeln((GFXfont *)&FiraSans, (char *)"GPS: ", &cursor_x, &cursor_y, NULL);
  cursor_x = 30; cursor_y = 200;
  writeln((GFXfont *)&FiraSans, (char *)"Haltestelle: ", &cursor_x, &cursor_y, NULL);
}

void displayStationData() {
  int32_t cursor_x, cursor_y;
  epd_clear_area({ 250, 10,  700, 50 });
  epd_clear_area({ 250, 160, 700, 50 });

  // Row 1: reverse-geocoded street address
  cursor_x = 250; cursor_y = 50;
  char pos[gpsData.x_coord.length() + 1];
  gpsData.x_coord.toCharArray(pos, sizeof(pos));
  writeln((GFXfont *)&FiraSans, pos, &cursor_x, &cursor_y, NULL);

  // Row 2: nearest stop name + distance
  cursor_x = 250; cursor_y = 200;
  String nStation = stationDataArray[stationIndex].near_station + " " +
                    String(stationDataArray[stationIndex].distance) + " m";
  char nst[nStation.length() + 1];
  nStation.toCharArray(nst, sizeof(nst));
  writeln((GFXfont *)&FiraSans, nst, &cursor_x, &cursor_y, NULL);
}

void displayStationBoardData() {
  int32_t cursor_x, cursor_y;
  const int base_y     = 300;
  const int row_height = 55;
  const int col1_x     = 30;
  const int col2_x     = 100;
  const int col3_x     = 765;
  const int col4_x     = 855;

  for (int i = 0; i < numEntries; i++) {
    int current_y = base_y + (i * row_height);
    epd_clear_area({ col1_x - 2, current_y - 45, 910, row_height + 5 });

    cursor_x = col1_x; cursor_y = current_y;
    char line[stationBoardData[i].line.length() + 1];
    stationBoardData[i].line.toCharArray(line, sizeof(line));
    writeln((GFXfont *)&FiraSans, line, &cursor_x, &cursor_y, NULL);

    cursor_x = col2_x;
    char dest[stationBoardData[i].destination.length() + 1];
    stationBoardData[i].destination.toCharArray(dest, sizeof(dest));
    writeln((GFXfont *)&FiraSans, dest, &cursor_x, &cursor_y, NULL);

    cursor_x = col3_x;
    char depTime[stationBoardData[i].departure_time.length() + 1];
    stationBoardData[i].departure_time.toCharArray(depTime, sizeof(depTime));
    writeln((GFXfont *)&FiraSans, depTime, &cursor_x, &cursor_y, NULL);

    cursor_x = col4_x;
    String delayStr = " + " + String(stationBoardData[i].delay);
    char delay[delayStr.length() + 1];
    delayStr.toCharArray(delay, sizeof(delay));
    writeln((GFXfont *)&FiraSans, delay, &cursor_x, &cursor_y, NULL);
  }
}

void displayTime() {
  int32_t cursor_x = 500, cursor_y = 100;
  Rect_t clearRect = { 498, 55, 125, 55 };
  epd_clear_area(clearRect);
  struct tm timeInfo;
  rtc.getDateTime(&timeInfo);
  char timeBuffer[6];
  strftime(timeBuffer, sizeof(timeBuffer), "%H:%M", &timeInfo);
  writeln((GFXfont *)&FiraSans, timeBuffer, &cursor_x, &cursor_y, NULL);
}

void connectWifi() {
  Serial.println("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println(WiFi.localIP());
}

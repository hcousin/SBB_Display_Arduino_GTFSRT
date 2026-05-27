/**
 * @copyright Hervé Cousin (original), updated 2025 for OJP API
 * @date      2024-07-12 / updated 2025
 *
 * Migrated from transport.opendata.ch/v1 (discontinued) to
 * opentransportdata.swiss OJP API.
 *
 * Display layout is IDENTICAL to the original:
 *   Row 1:  "GPS:"         <gps_address>
 *   Row 2:  "Haltestelle:" <near_station> <distance> m
 *   Row 3:  <time HH:MM>
 *   Rows 4-7: <line>  <destination>  <HH:MM>  + <delay>
 *
 * Key fix vs previous version:
 *   Struct members changed from String to char[] to prevent heap
 *   corruption during global/static initialisation (boot crash).
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
#include <HardwareSerial.h>
#include "credentials.h"

#ifndef PCF8563_SLAVE_ADDRESS
#define PCF8563_SLAVE_ADDRESS 0x51
#endif

// ---------------------------------------------------------------------------
// Pin definitions
// ---------------------------------------------------------------------------
#define BUTTON_1  (21)
#define BATT_PIN  (14)
#define BOARD_SCL (17)
#define BOARD_SDA (18)

// GPS UART
#define GPS_RX_PIN         (44)
#define GPS_TX_PIN         (43)
#define GPS_BAUD           (9600)
#define GPS_SERIAL         Serial1
#define GPS_FIX_TIMEOUT_MS (30000)

// OJP
#define OJP_URL      "https://api.opentransportdata.swiss/ojp2020"
#define OJP_RADIUS_M 500

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
uint8_t *framebuffer;
int vref = 1100;

// GPS
TinyGPSPlus gps;
GpsData gpsData;
// Default coordinates (Glattpark, Opfikon) used when no GPS fix obtained
char xCoord[32] = "47.421250255895124";
char yCoord[32] = "8.562216925810933";

// Station arrays
int stationIndex = 1;
const int maxStations = 10;
StationData stationDataArray[maxStations];   // zero-initialised (global)
const int numEntries = 4;
StationBoardData stationBoardData[numEntries]; // zero-initialised (global)

// Button
volatile bool buttonPressed = false;
const unsigned long sleepInterval = 45000;

// RTC / NTP
SensorPCF8563 rtc;
char buf[128];
const char *ntpServer1     = "pool.ntp.org";
const char *ntpServer2     = "time.nist.gov";
const char *time_zone      = "CET-1CEST,M3.5.0/2,M10.5.0/3";

// ---------------------------------------------------------------------------
// ISR
// ---------------------------------------------------------------------------
void IRAM_ATTR selectStationID() {
  buttonPressed = true;
}

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

  // Try live GPS fix; updates xCoord/yCoord on success
  GPS_SERIAL.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  fetchGPSFix();

  strncpy(gpsData.x_coord, xCoord, sizeof(gpsData.x_coord) - 1);
  strncpy(gpsData.y_coord, yCoord, sizeof(gpsData.y_coord) - 1);
  strncpy(gpsData.pos_acc, "5m",   sizeof(gpsData.pos_acc)  - 1);

  // ADC calibration
  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(
      ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
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

  fetchStationDataFromGPS();
  title();
  displayStationData();
  fetchStationBoardData();
  displayStationBoardData();
}

// ===========================================================================
// loop()
// ===========================================================================
void loop() {
  struct tm timeinfo;
  rtc.getDateTime(&timeinfo);
  strftime(buf, sizeof(buf), "RTC: %b %d %Y %H:%M:%S", &timeinfo);
  Serial.println(buf);

  readBatVoltage();
  displayTime();

  if (buttonPressed) {
    stationIndex = (stationIndex >= maxStations - 1) ? 1 : stationIndex + 1;
    buttonPressed = false;
    displayStationData();
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
// fetchGPSFix()
// ===========================================================================
void fetchGPSFix() {
  Serial.println("[GPS] Waiting for fix...");
  unsigned long startMs = millis();
  while (millis() - startMs < GPS_FIX_TIMEOUT_MS) {
    while (GPS_SERIAL.available()) gps.encode(GPS_SERIAL.read());
    if (gps.location.isValid() && gps.location.age() < 2000) {
      snprintf(xCoord, sizeof(xCoord), "%.9f", gps.location.lat());
      snprintf(yCoord, sizeof(yCoord), "%.9f", gps.location.lng());
      Serial.printf("[GPS] Fix: lat=%s lng=%s\n", xCoord, yCoord);
      return;
    }
    delay(100);
  }
  Serial.println("[GPS] No fix – using default coordinates.");
}

// ===========================================================================
// fetchStationDataFromGPS()
// OJP LocationInformationRequest – fills stationDataArray[]
// ===========================================================================
void fetchStationDataFromGPS() {
  Serial.printf("[OJP] LocationInfo lat=%s lng=%s\n", xCoord, yCoord);

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
    "<ojp:GeoRestriction><ojp:Circle>"
    "<ojp:Center>"
    "<Longitude>" + String(yCoord) + "</Longitude>"
    "<Latitude>"  + String(xCoord) + "</Latitude>"
    "</ojp:Center>"
    "<ojp:Radius>" + String(OJP_RADIUS_M) + "</ojp:Radius>"
    "</ojp:Circle></ojp:GeoRestriction>"
    "</ojp:InitialInput>"
    "<ojp:Restrictions>"
    "<ojp:Type>stop</ojp:Type>"
    "<ojp:NumberOfResults>" + String(maxStations) + "</ojp:NumberOfResults>"
    "<ojp:IncludePtModes>false</ojp:IncludePtModes>"
    "</ojp:Restrictions>"
    "</ojp:OJPLocationInformationRequest>"
    "</ServiceRequest></OJPRequest></OJP>";

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.begin(client, OJP_URL);
  http.addHeader("Content-Type",  "application/xml");
  http.addHeader("Authorization", String("Bearer ") + OJP_API_KEY);
  http.addHeader("User-Agent",    "SBB-EPaper-Display/2.0 ESP32");
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  int httpCode = http.POST(reqBody);
  if (httpCode != HTTP_CODE_OK) {
    Serial.printf("[OJP] LocationInfo error %d\n", httpCode);
    http.end();
    return;
  }

  String response = http.getString();
  http.end();

  // Helper: extract text between tags
  auto extractTag = [](const String &xml, const String &open,
                       const String &close, int from = 0) -> String {
    int s = xml.indexOf(open, from);
    if (s < 0) return "";
    s += open.length();
    int e = xml.indexOf(close, s);
    if (e < 0) return "";
    return xml.substring(s, e);
  };

  // Haversine distance in metres
  auto haversineM = [](double lat1, double lng1, double lat2, double lng2) -> int {
    const double R = 6371000.0;
    double dLat = (lat2 - lat1) * PI / 180.0;
    double dLng = (lng2 - lng1) * PI / 180.0;
    double a = sin(dLat/2)*sin(dLat/2) +
               cos(lat1*PI/180.0)*cos(lat2*PI/180.0)*sin(dLng/2)*sin(dLng/2);
    return (int)(R * 2.0 * atan2(sqrt(a), sqrt(1.0 - a)));
  };

  double myLat = atof(xCoord);
  double myLng = atof(yCoord);

  const String LOC_OPEN  = "<ojp:Location>";
  const String LOC_CLOSE = "</ojp:Location>";
  int found = 0, searchPos = 0;
  char addressStation[MAX_STR] = "";

  while (found < maxStations) {
    int bs = response.indexOf(LOC_OPEN, searchPos);
    if (bs < 0) break;
    int be = response.indexOf(LOC_CLOSE, bs);
    if (be < 0) break;
    String block = response.substring(bs, be + LOC_CLOSE.length());
    searchPos = be + LOC_CLOSE.length();

    String stopRef  = extractTag(block, "<siri:StopPointRef>", "</siri:StopPointRef>");
    String stopName = extractTag(block, "<ojp:LocationName><ojp:Text>", "</ojp:Text>");
    if (stopName.length() == 0)
      stopName = extractTag(block, "<ojp:Text>", "</ojp:Text>");
    String latStr = extractTag(block, "<siri:Latitude>",  "</siri:Latitude>");
    String lngStr = extractTag(block, "<siri:Longitude>", "</siri:Longitude>");

    if (stopRef.length() == 0 || stopName.length() == 0) continue;

    if (found == 0) strncpy(addressStation, stopName.c_str(), MAX_STR - 1);

    double stopLat = latStr.toDouble();
    double stopLng = lngStr.toDouble();
    int dist = (stopLat != 0.0 && stopLng != 0.0)
                 ? haversineM(myLat, myLng, stopLat, stopLng) : 0;

    strncpy(stationDataArray[found].gps_address,  addressStation,      MAX_STR - 1);
    strncpy(stationDataArray[found].near_station, stopName.c_str(),    MAX_STR - 1);
    strncpy(stationDataArray[found].station_id,   stopRef.c_str(),     MAX_ID  - 1);
    stationDataArray[found].distance = dist;

    Serial.printf("[OJP] Stop %d: %s  id=%s  dist=%dm\n",
                  found, stopName.c_str(), stopRef.c_str(), dist);
    found++;
  }

  if (found == 0) {
    Serial.println("[OJP] No stops found.");
    return;
  }
  Serial.printf("[OJP] Active stop: %s  id=%s\n",
                stationDataArray[stationIndex].near_station,
                stationDataArray[stationIndex].station_id);
}

// ===========================================================================
// fetchStationBoardData()
// OJP StopEventRequest – fills stationBoardData[]
// ===========================================================================
void fetchStationBoardData() {
  if (strlen(stationDataArray[stationIndex].station_id) == 0) {
    Serial.println("[OJP] No station ID – skipping");
    return;
  }

  const char *stopId = stationDataArray[stationIndex].station_id;
  Serial.printf("[OJP] StopEvent for stop: %s\n", stopId);

  time_t now; time(&now);
  struct tm *utc = gmtime(&now);
  char tsNow[25];
  strftime(tsNow, sizeof(tsNow), "%Y-%m-%dT%H:%M:%SZ", utc);

  String reqBody =
    "<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
    "<OJP xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\""
    " xmlns:xsd=\"http://www.w3.org/2001/XMLSchema\""
    " xmlns=\"http://www.siri.org.uk/siri\" version=\"1.0\""
    " xmlns:ojp=\"http://www.vdv.de/ojp\""
    " xsi:schemaLocation=\"http://www.siri.org.uk/siri ../ojp-xsd-v1.0/OJP.xsd\">"
    "<OJPRequest><ServiceRequest>"
    "<RequestTimestamp>" + String(tsNow) + "</RequestTimestamp>"
    "<RequestorRef>SBB-EPaper-Display_prod</RequestorRef>"
    "<ojp:OJPStopEventRequest>"
    "<RequestTimestamp>" + String(tsNow) + "</RequestTimestamp>"
    "<MessageIdentifier>2</MessageIdentifier>"
    "<ojp:Location>"
    "<ojp:PlaceRef>"
    "<ojp:StopPlaceRef>" + String(stopId) + "</ojp:StopPlaceRef>"
    "<ojp:LocationName><ojp:Text>stop</ojp:Text></ojp:LocationName>"
    "</ojp:PlaceRef>"
    "<ojp:DepArrTime>" + String(tsNow) + "</ojp:DepArrTime>"
    "</ojp:Location>"
    "<ojp:Params>"
    "<ojp:NumberOfResults>" + String(numEntries) + "</ojp:NumberOfResults>"
    "<ojp:StopEventType>departure</ojp:StopEventType>"
    "<ojp:IncludeRealtimeData>true</ojp:IncludeRealtimeData>"
    "</ojp:Params>"
    "</ojp:OJPStopEventRequest>"
    "</ServiceRequest></OJPRequest></OJP>";

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.begin(client, OJP_URL);
  http.addHeader("Content-Type",  "application/xml");
  http.addHeader("Authorization", String("Bearer ") + OJP_API_KEY);
  http.addHeader("User-Agent",    "SBB-EPaper-Display/2.0 ESP32");
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  int httpCode = http.POST(reqBody);
  if (httpCode != HTTP_CODE_OK) {
    Serial.printf("[OJP] StopEvent error %d\n", httpCode);
    if (httpCode > 0) Serial.println(http.getString().substring(0, 300));
    http.end();
    return;
  }

  String response = http.getString();
  http.end();

  auto extractTag = [](const String &xml, const String &open,
                       const String &close, int from = 0) -> String {
    int s = xml.indexOf(open, from);
    if (s < 0) return "";
    s += open.length();
    int e = xml.indexOf(close, s);
    if (e < 0) return "";
    return xml.substring(s, e);
  };

  // ISO8601 UTC → local HH:MM
  auto isoToHHMM = [](const String &iso, char *out, size_t outLen) {
    if (iso.length() < 16) { strncpy(out, "--:--", outLen); return; }
    struct tm t = {};
    t.tm_year = iso.substring(0,4).toInt() - 1900;
    t.tm_mon  = iso.substring(5,7).toInt() - 1;
    t.tm_mday = iso.substring(8,10).toInt();
    t.tm_hour = iso.substring(11,13).toInt();
    t.tm_min  = iso.substring(14,16).toInt();
    t.tm_sec  = 0;
    t.tm_isdst = -1;
    time_t utcT = mktime(&t);
    struct tm *local = localtime(&utcT);
    strftime(out, outLen, "%H:%M", local);
  };

  const String SE_OPEN  = "<ojp:StopEvent>";
  const String SE_CLOSE = "</ojp:StopEvent>";
  int found = 0, searchPos = 0;

  while (found < numEntries) {
    int bs = response.indexOf(SE_OPEN, searchPos);
    if (bs < 0) break;
    int be = response.indexOf(SE_CLOSE, bs);
    if (be < 0) break;
    String block = response.substring(bs, be + SE_CLOSE.length());
    searchPos = be + SE_CLOSE.length();

    String timetabled = extractTag(block, "<ojp:TimetabledTime>", "</ojp:TimetabledTime>");
    String estimated  = extractTag(block, "<ojp:EstimatedTime>",  "</ojp:EstimatedTime>");

    char depTimeBuf[8];
    isoToHHMM((estimated.length() > 0) ? estimated : timetabled,
              depTimeBuf, sizeof(depTimeBuf));

    int delayMin = 0;
    if (estimated.length() > 0 && timetabled.length() > 0) {
      int tH = timetabled.substring(11,13).toInt();
      int tM = timetabled.substring(14,16).toInt();
      int eH = estimated.substring(11,13).toInt();
      int eM = estimated.substring(14,16).toInt();
      delayMin = (eH * 60 + eM) - (tH * 60 + tM);
      if (delayMin < 0) delayMin += 1440;
    }

    // Line name – try both compact and whitespace variants
    String lineName = extractTag(block, "<ojp:PublishedLineName><ojp:Text>", "</ojp:Text>");
    if (lineName.length() == 0)
      lineName = extractTag(block, "<ojp:PublishedLineName>\n      <ojp:Text>", "</ojp:Text>");

    // Destination
    String dest = extractTag(block, "<ojp:DestinationText><ojp:Text>", "</ojp:Text>");
    if (dest.length() == 0)
      dest = extractTag(block, "<ojp:DestinationText>\n      <ojp:Text>", "</ojp:Text>");

    String mode = extractTag(block, "<ojp:ShortName><ojp:Text>", "</ojp:Text>");

    strncpy(stationBoardData[found].line,          lineName.c_str(), MAX_STR - 1);
    strncpy(stationBoardData[found].destination,   dest.c_str(),     MAX_STR - 1);
    strncpy(stationBoardData[found].departure_time, depTimeBuf,      7);
    strncpy(stationBoardData[found].type,          mode.c_str(),     MAX_STR - 1);
    stationBoardData[found].delay         = delayMin;
    stationBoardData[found].line_operator[0] = '\0';

    Serial.printf("[OJP] %d: %s → %s  %s  +%d\n",
                  found,
                  stationBoardData[found].line,
                  stationBoardData[found].destination,
                  stationBoardData[found].departure_time,
                  delayMin);
    found++;
  }

  // Clear unused rows
  for (int i = found; i < numEntries; i++) {
    strncpy(stationBoardData[i].line,           "-",     MAX_STR - 1);
    strncpy(stationBoardData[i].destination,    "",      MAX_STR - 1);
    strncpy(stationBoardData[i].departure_time, "--:--", 7);
    stationBoardData[i].delay = 0;
  }
}

// ===========================================================================
// readBatVoltage() – unchanged from original
// ===========================================================================
void readBatVoltage() {
  delay(10);
  uint16_t v = analogRead(BATT_PIN);
  float battery_voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
  if (battery_voltage >= 4.2) battery_voltage = 4.2;
  Serial.println("Voltage: " + String(battery_voltage) + "V");
}

// ===========================================================================
// title() – UNCHANGED from original
// ===========================================================================
void title() {
  int32_t cursor_x, cursor_y;
  cursor_x = 30; cursor_y = 50;
  writeln((GFXfont *)&FiraSans, (char *)"GPS: ", &cursor_x, &cursor_y, NULL);
  cursor_x = 30; cursor_y = 200;
  writeln((GFXfont *)&FiraSans, (char *)"Haltestelle: ", &cursor_x, &cursor_y, NULL);
}

// ===========================================================================
// displayStationData() – UNCHANGED from original
// ===========================================================================
void displayStationData() {
  int32_t cursor_x, cursor_y;

  epd_clear_area({ 250, 10,  700, 50 });
  epd_clear_area({ 250, 160, 700, 50 });

  cursor_x = 250; cursor_y = 50;
  writeln((GFXfont *)&FiraSans,
          stationDataArray[stationIndex].gps_address,
          &cursor_x, &cursor_y, NULL);

  cursor_x = 250; cursor_y = 200;
  char nst[MAX_STR + 16];
  snprintf(nst, sizeof(nst), "%s %d m",
           stationDataArray[stationIndex].near_station,
           stationDataArray[stationIndex].distance);
  writeln((GFXfont *)&FiraSans, nst, &cursor_x, &cursor_y, NULL);
}

// ===========================================================================
// displayStationBoardData() – UNCHANGED from original
// ===========================================================================
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
    writeln((GFXfont *)&FiraSans, stationBoardData[i].line,
            &cursor_x, &cursor_y, NULL);

    cursor_x = col2_x;
    writeln((GFXfont *)&FiraSans, stationBoardData[i].destination,
            &cursor_x, &cursor_y, NULL);

    cursor_x = col3_x;
    writeln((GFXfont *)&FiraSans, stationBoardData[i].departure_time,
            &cursor_x, &cursor_y, NULL);

    cursor_x = col4_x;
    char delayStr[12];
    snprintf(delayStr, sizeof(delayStr), " + %d", stationBoardData[i].delay);
    writeln((GFXfont *)&FiraSans, delayStr, &cursor_x, &cursor_y, NULL);
  }
}

// ===========================================================================
// displayTime() – UNCHANGED from original
// ===========================================================================
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

// ===========================================================================
// connectWifi() – UNCHANGED from original
// ===========================================================================
void connectWifi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nWiFi connected – IP: " + WiFi.localIP().toString());
}

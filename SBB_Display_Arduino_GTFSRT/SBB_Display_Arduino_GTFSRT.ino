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

// OJP API
#define OJP_URL      "https://api.opentransportdata.swiss/ojp2020"
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

int stationIndex = 1;
const int maxStations = 10;
StationData stationDataArray[maxStations];
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
  fetchStationDataFromGPS();
  title();
  displayStationData();
  fetchStationBoardData();
  displayStationBoardData();
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
  displayTime();

  Serial.println(buttonPressed);
  Serial.print("Station Index before if = ");
  Serial.println(stationIndex);

  if (buttonPressed) {
    if (stationIndex >= maxStations) {
      stationIndex = 1;
    } else {
      stationIndex++;
    }
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
  Serial.println("Nach 45s");
}

// ===========================================================================
// fetchGPSFix()
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
  HTTPClient http;
  http.begin(client, OJP_URL);
  http.addHeader("Content-Type",  "application/xml");
  http.addHeader("Authorization", String("Bearer ") + OJP_API_KEY);
  http.addHeader("User-Agent",    "SBB-EPaper-Display/2.0 ESP32");
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  int code = http.POST(body);
  if (code != HTTP_CODE_OK) {
    Serial.printf("[OJP] HTTP error %d\n", code);
    if (code > 0) Serial.println(http.getString().substring(0, 200));
    http.end();
    return "";
  }
  String response = http.getString();
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

  String body =
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
    "<ojp:InitialInput><ojp:GeoRestriction><ojp:Circle>"
    "<ojp:Center>"
    "<Longitude>" + yCoord + "</Longitude>"
    "<Latitude>"  + xCoord + "</Latitude>"
    "</ojp:Center>"
    "<ojp:Radius>" + String(OJP_RADIUS_M) + "</ojp:Radius>"
    "</ojp:Circle></ojp:GeoRestriction></ojp:InitialInput>"
    "<ojp:Restrictions>"
    "<ojp:Type>stop</ojp:Type>"
    "<ojp:NumberOfResults>" + String(maxStations) + "</ojp:NumberOfResults>"
    "<ojp:IncludePtModes>false</ojp:IncludePtModes>"
    "</ojp:Restrictions>"
    "</ojp:OJPLocationInformationRequest>"
    "</ServiceRequest></OJPRequest></OJP>";

  String response = ojpPost(body);
  if (response.length() == 0) return;

  const String LOC_OPEN  = "<ojp:Location>";
  const String LOC_CLOSE = "</ojp:Location>";
  int found = 0, pos = 0;
  String firstStopName = "";

  while (found < maxStations) {
    int bs = response.indexOf(LOC_OPEN, pos);
    if (bs < 0) break;
    int be = response.indexOf(LOC_CLOSE, bs);
    if (be < 0) break;
    String block = response.substring(bs, be + LOC_CLOSE.length());
    pos = be + LOC_CLOSE.length();

    String stopRef  = extractTag(block, "<siri:StopPointRef>", "</siri:StopPointRef>");
    String stopName = extractTag(block, "<ojp:LocationName><ojp:Text>", "</ojp:Text>");
    if (stopName.length() == 0)
      stopName = extractTag(block, "<ojp:Text>", "</ojp:Text>");
    String latStr = extractTag(block, "<siri:Latitude>",  "</siri:Latitude>");
    String lngStr = extractTag(block, "<siri:Longitude>", "</siri:Longitude>");

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
    stationID = stationDataArray[stationIndex].station_id;
    Serial.println("[OJP] Active: " + stationDataArray[stationIndex].near_station);
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
    "<ojp:StopPlaceRef>" + stationID + "</ojp:StopPlaceRef>"
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

  String response = ojpPost(body);
  if (response.length() == 0) return;

  // ISO8601 UTC → local HH:MM
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

  const String SE_OPEN  = "<ojp:StopEvent>";
  const String SE_CLOSE = "</ojp:StopEvent>";
  int found = 0, pos = 0;

  while (found < numEntries) {
    int bs = response.indexOf(SE_OPEN, pos);
    if (bs < 0) break;
    int be = response.indexOf(SE_CLOSE, bs);
    if (be < 0) break;
    String block = response.substring(bs, be + SE_CLOSE.length());
    pos = be + SE_CLOSE.length();

    String timetabled = extractTag(block, "<ojp:TimetabledTime>", "</ojp:TimetabledTime>");
    String estimated  = extractTag(block, "<ojp:EstimatedTime>",  "</ojp:EstimatedTime>");

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

    String lineName = extractTag(block, "<ojp:PublishedLineName><ojp:Text>", "</ojp:Text>");
    if (lineName.length() == 0)
      lineName = extractTag(block, "<ojp:PublishedLineName>\n      <ojp:Text>", "</ojp:Text>");

    String dest = extractTag(block, "<ojp:DestinationText><ojp:Text>", "</ojp:Text>");
    if (dest.length() == 0)
      dest = extractTag(block, "<ojp:DestinationText>\n      <ojp:Text>", "</ojp:Text>");

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

  cursor_x = 250; cursor_y = 50;
  char pos[stationDataArray[stationIndex].gps_address.length() + 1];
  stationDataArray[stationIndex].gps_address.toCharArray(pos, sizeof(pos));
  writeln((GFXfont *)&FiraSans, pos, &cursor_x, &cursor_y, NULL);

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

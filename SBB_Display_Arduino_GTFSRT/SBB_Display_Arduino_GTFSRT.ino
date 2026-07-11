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
#define BUTTON_1 (21)
#define BATT_PIN (14)
#define SD_MISO (16)
#define SD_MOSI (15)
#define SD_SCLK (11)
#define SD_CS (42)
#define BOARD_SCL (17)
#define BOARD_SDA (18)
#define GPIO_MISO (45)
#define GPIO_MOSI (10)
#define GPIO_SCLK (48)
#define GPIO_CS (39)

// GPS UART pins
#define GPS_RX_PIN (44)
#define GPS_TX_PIN (43)
#define GPS_BAUD (9600)
#define GPS_FIX_TIMEOUT_MS (30000)

// OJP API - note: OJP 2.0 endpoint is /ojp20 (not /ojp2020)
#define OJP_URL "https://api.opentransportdata.swiss/ojp20"
#define OJP_RADIUS_M 500

// Set to false if no GPS module is connected – skips the 30s wait
// and uses DEFAULT_LAT / DEFAULT_LNG from credentials.h immediately
#define GPS_ENABLED false

// ---------------------------------------------------------------------------
// Debug logging
// ---------------------------------------------------------------------------
// Set to 0 to compile all diagnostic Serial output out of the binary
// entirely (the DBG_* calls below expand to nothing, they are not
// just silenced at runtime). Useful once the device is set up and
// running unattended, to save the per-call formatting/UART time in
// loop() and drop the log string literals from flash.
#define DEBUG_LOG 1

#if DEBUG_LOG
#define DBG_PRINT(...) Serial.print(__VA_ARGS__)
#define DBG_PRINTLN(...) Serial.println(__VA_ARGS__)
#define DBG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define DBG_PRINT(...)
#define DBG_PRINTLN(...)
#define DBG_PRINTF(...)
#endif

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
int stationsFound = 0;  // actual number of stops returned by OJP
String stationID = "";
const int numEntries = 4;
StationBoardData stationBoardData[numEntries];

volatile int buttonPressed = false;
const unsigned long sleepInterval = 45000;

SensorPCF8563 rtc;
char buf[128];
const char *ntpServer1 = "pool.ntp.org";
const char *ntpServer2 = "time.nist.gov";
const long gmtOffset_sec = 3600;
const int daylightOffset_sec = 3600;
const char *time_zone = "CET-1CEST,M3.5.0/2,M10.5.0/3";

// ---------------------------------------------------------------------------
// ISR – identical to original
// ---------------------------------------------------------------------------

/**
 * @brief Button interrupt handler (rising edge on BUTTON_1).
 *
 * Only sets a flag; the actual station-cycling logic runs in loop()
 * to keep the ISR itself minimal, as required on the ESP32.
 */
void IRAM_ATTR selectStationID() {
  buttonPressed = true;
}

/**
 * @brief SNTP callback, invoked once the system time has been
 *        synchronized from the configured NTP servers.
 * @param t Pointer to the new time value (unused, informational only).
 */
void timeavailable(struct timeval *t) {
  DBG_PRINTLN("[WiFi]: Got time adjustment from NTP!");
}

// ===========================================================================
// setup() – identical to original
// ===========================================================================
/**
 * @brief Arduino setup routine, run once at boot.
 *
 * Performs, in order:
 *  1. WiFi connection
 *  2. RTC (PCF8563) detection and init over I2C
 *  3. Button interrupt attachment
 *  4. NTP time sync (blocks until a valid local time is available)
 *  5. ADC calibration for battery voltage readings
 *  6. E-paper display init and framebuffer allocation (PSRAM)
 *  7. Initial GPS fix + reverse geocoding + nearest-stop lookup
 *  8. First render of station name and departure board
 */
void setup() {
  Serial.begin(115200);
  connectWifi();

  Wire.begin(BOARD_SDA, BOARD_SCL);
  delay(100);

  Wire.beginTransmission(PCF8563_SLAVE_ADDRESS);

  if (Wire.endTransmission() == 0) {
    rtc.begin(Wire, PCF8563_SLAVE_ADDRESS, BOARD_SDA, BOARD_SCL);
    DBG_PRINTLN("RTC is online");
  } else {
    DBG_PRINTLN("RTC initialization failed!");
  }

  pinMode(BUTTON_1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_1), selectStationID, RISING);

  sntp_set_time_sync_notification_cb(timeavailable);
  configTzTime(time_zone, ntpServer1, ntpServer2);

  struct tm t;
  while (!getLocalTime(&t)) {
    delay(100);
  }

  while (sntp_get_sync_status() != SNTP_SYNC_STATUS_COMPLETED) {
    delay(100);
  }

  DBG_PRINTF("Setup local: %02d:%02d:%02d\n",
                t.tm_hour,
                t.tm_min,
                t.tm_sec);

  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(
    ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
    DBG_PRINTF("eFuse Vref:%u mV", adc_chars.vref);
    vref = adc_chars.vref;
  }

  epd_init();
  framebuffer = (uint8_t *)ps_calloc(sizeof(uint8_t), EPD_WIDTH * EPD_HEIGHT / 2);
  if (!framebuffer) {
    DBG_PRINTLN("alloc memory failed !!!");
    while (1)
      ;
  }
  memset(framebuffer, 0xFF, EPD_WIDTH * EPD_HEIGHT / 2);

  epd_poweron();
  epd_clear();
  epd_poweroff();

  DBG_PRINTF("Free heap: %d bytes\n", ESP.getFreeHeap());

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
/**
 * @brief Main loop, runs continuously after setup().
 *
 * On each cycle:
 *  - Logs NTP/RTC time to Serial and refreshes the time display.
 *  - If the button was pressed since the last cycle, advances to the
 *    next known station (wrapping back to index 0, which retries GPS
 *    on the following cycle) and re-fetches its departure board.
 *  - Otherwise simply refreshes the departure board for the current
 *    station.
 *  - Sleeps for `sleepInterval` ms (or until the button is pressed
 *    again) before repeating.
 */
void loop() {
  struct tm timeInfo;

  if (getLocalTime(&timeInfo)) {
    strftime(buf, sizeof(buf), "➸ %b %d %Y %H:%M:%S", &timeInfo);
    DBG_PRINT("NTP: ");
    DBG_PRINTLN(buf);
  }
  strftime(buf, 64, "➸ %b %d %Y %H:%M:%S", &timeInfo);
  DBG_PRINT("RTC: ");
  DBG_PRINTLN(buf);

  readBatVoltage();
  epd_poweron();
  displayTime();
  epd_poweroff();

  DBG_PRINTLN(buttonPressed);
  DBG_PRINT("Station Index before if = ");
  DBG_PRINTLN(stationIndex);

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
  DBG_PRINTLN("Nach 45s");
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
  String url = "https://nominatim.openstreetmap.org/reverse?format=json&lat=" + xCoord + "&lon=" + yCoord + "&zoom=18&addressdetails=1";

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.begin(client, url);
  http.addHeader("User-Agent", "SBB-EPaper-Display/2.0 ESP32");
  http.setTimeout(10000);

  int code = http.GET();
  if (code != HTTP_CODE_OK) {
    DBG_PRINTLN("[Nominatim] HTTP error: " + String(code));
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
  String road = extractJson(response, "road");
  String houseNo = extractJson(response, "house_number");
  String suburb = extractJson(response, "suburb");
  String city = extractJson(response, "city");
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

  DBG_PRINTLN("[Nominatim] Address: " + address);
  gpsData.x_coord = address;
}

// ===========================================================================
// NEW: reads NMEA from GPS module, updates xCoord/yCoord if fix obtained.
// TinyGPSPlus instantiated locally to avoid global constructor issues.
// ===========================================================================
void fetchGPSFix() {
  if (!GPS_ENABLED) {
    DBG_PRINTLN("[GPS] GPS_ENABLED=false – using fallback coordinates.");
    DBG_PRINTLN("[GPS] lat=" + xCoord + " lng=" + yCoord);
    return;
  }
  DBG_PRINTLN("[GPS] Waiting for fix...");
  Serial1.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  TinyGPSPlus gpsLocal;
  unsigned long startMs = millis();
  while (millis() - startMs < GPS_FIX_TIMEOUT_MS) {
    while (Serial1.available()) gpsLocal.encode(Serial1.read());
    if (gpsLocal.location.isValid() && gpsLocal.location.age() < 2000) {
      xCoord = String(gpsLocal.location.lat(), 9);
      yCoord = String(gpsLocal.location.lng(), 9);
      DBG_PRINTLN("[GPS] Fix: lat=" + xCoord + " lng=" + yCoord);
      return;
    }
    delay(100);
  }
  DBG_PRINTLN("[GPS] No fix – using fallback coordinates.");
}

// ===========================================================================
// setupOjpClient()
// Shared setup for every OJP HTTPS POST request (used by ojpPost() and
// ojpPostStream()), to avoid duplicating the TLS/HTTP header boilerplate.
// ===========================================================================
/**
 * @brief Configures a WiFiClientSecure + HTTPClient pair for an OJP
 *        API request and starts a POST call.
 *
 * Sets up TLS (certificate check disabled – OJP is accessed only via
 * its known HTTPS endpoint, no cert bundle is pinned), the shared
 * timeout, standard headers (Content-Type, Bearer auth, User-Agent),
 * strict redirect handling, and logs the request. `client` and
 * `http` must be declared by the caller and stay in scope for as
 * long as `http` is used, since HTTPClient keeps a reference to
 * `client` internally.
 *
 * @param client    Uninitialized WiFiClientSecure, owned by the caller.
 * @param http      Uninitialized HTTPClient, owned by the caller.
 * @param body      Raw OJP XML request body to POST.
 * @param timeoutMs Timeout in milliseconds, applied to both the TLS
 *                  client and the HTTP client.
 * @param logTag    Short tag appended to the "[OJP] POST" log line,
 *                  e.g. "" or " (stream)", to tell call sites apart
 *                  in the Serial log.
 * @return The HTTP status code returned by http.POST(), or a
 *         negative HTTPClient error code on failure.
 */
int startOjpPost(WiFiClientSecure &client, HTTPClient &http,
                  const String &body, unsigned long timeoutMs,
                  const char *logTag) {
  client.setInsecure();
  client.setTimeout(timeoutMs);
  http.begin(client, OJP_URL);
  http.setTimeout(timeoutMs);
  http.addHeader("Content-Type", "application/xml");
  http.addHeader("Authorization", String("Bearer ") + OJP_API_KEY);
  http.addHeader("User-Agent", "SBB-EPaper-Display/2.0 ESP32");
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  DBG_PRINTLN("[OJP] POST" + String(logTag) + " to " + String(OJP_URL));

  int code = http.POST(body);
  DBG_PRINTLN("[OJP] Response code: " + String(code));
  return code;
}

// ===========================================================================
// ojpPost() – used only for small responses (LocationInfo ~2KB)
// ===========================================================================
/**
 * @brief Sends an OJP XML request via HTTPS POST and returns the full
 *        response body in memory.
 *
 * Intended for small responses only (e.g. LocationInformationRequest,
 * ~2 KB). For the larger StopEventRequest response, use
 * ojpPostStream() instead to avoid buffering the whole reply.
 *
 * @param body Raw OJP XML request body.
 * @return Response body as a String, or "" on any HTTP/network error.
 */
String ojpPost(const String &body) {
  WiFiClientSecure client;
  HTTPClient http;
  int code = startOjpPost(client, http, body, 10000, "");
  if (code != HTTP_CODE_OK) {
    DBG_PRINTLN("[OJP] Error: " + http.getString().substring(0, 200));
    http.end();
    return "";
  }
  String response = http.getString();
  DBG_PRINTLN("[OJP] Response length: " + String(response.length()));
  http.end();
  return response;
}

// ===========================================================================
// ISO8601 UTC -> lokale HH:MM
// ===========================================================================
/**
 * @brief Converts an ISO 8601 UTC timestamp ("YYYY-MM-DDTHH:MM:SSZ",
 *        as returned by OJP) into a local "HH:MM" string.
 *
 * The local offset is derived at call time from the difference
 * between localtime_r() and gmtime_r() on the current system clock
 * (already NTP/timezone-adjusted via configTzTime() in setup()), so
 * this correctly handles both CET and CEST without hardcoding an
 * offset.
 *
 * @param iso UTC timestamp string, e.g. "2026-07-09T12:31:00Z".
 * @return Local time formatted as "HH:MM".
 */
String isoToHHMM(const String &iso) {
  // Erwartet: 2026-07-09T12:31:00Z

  int hour = iso.substring(11, 13).toInt();
  int minute = iso.substring(14, 16).toInt();

  // OJP liefert UTC.
  // Schweiz im Sommer = UTC+2
  // Schweiz im Winter = UTC+1

  time_t now;
  time(&now);

  struct tm localNow;
  localtime_r(&now, &localNow);

  // Offset zwischen UTC und Lokalzeit bestimmen
  struct tm utcNow;
  gmtime_r(&now, &utcNow);

  int offset =
    (localNow.tm_hour * 60 + localNow.tm_min) - (utcNow.tm_hour * 60 + utcNow.tm_min);

  // Mitternacht berücksichtigen
  if (offset < -720) offset += 1440;
  if (offset > 720) offset -= 1440;

  int total = hour * 60 + minute + offset;

  while (total < 0) total += 1440;
  while (total >= 1440) total -= 1440;

  char buf[6];
  sprintf(buf, "%02d:%02d", total / 60, total % 60);

  return String(buf);
}

// ===========================================================================
// Berechnet die Verspätung in Minuten
// ===========================================================================
/**
 * @brief Computes the delay in whole minutes between a planned and an
 *        estimated departure timestamp.
 *
 * Both timestamps must be in "YYYY-MM-DDTHH:MM:SSZ" format. The
 * result is clamped to the range [0, 120] minutes: early departures
 * (negative delay) are reported as 0, and unrealistic values (e.g.
 * from a parsing glitch) are capped at 120 rather than displayed
 * as-is.
 *
 * @param planned   Timetabled departure time (OJP `TimetabledTime`).
 * @param estimated Real-time estimated departure time (OJP `EstimatedTime`).
 * @return Delay in minutes, clamped to [0, 120]; 0 if either
 *         timestamp fails to parse.
 */
int calcDelay(const String &planned,
              const String &estimated) {
  struct tm tmPlanned = {};
  struct tm tmEstimated = {};

  if (!strptime(planned.c_str(),
                "%Y-%m-%dT%H:%M:%SZ",
                &tmPlanned))
    return 0;

  if (!strptime(estimated.c_str(),
                "%Y-%m-%dT%H:%M:%SZ",
                &tmEstimated))
    return 0;

  time_t t1 = mktime(&tmPlanned);
  time_t t2 = mktime(&tmEstimated);

  int delay = (t2 - t1) / 60;

  if (delay < 0)
    delay = 0;

  if (delay > 120)
    delay = 120;

  return delay;
}

// ===========================================================================
// ojpPostStream()
// Streams the StopEvent response and extracts departure fields on-the-fly
// without loading the full ~10KB response into RAM.
// Reads in 1KB chunks with 512-byte overlap to catch tags across boundaries.
// Fills stationBoardData[] directly.
// ===========================================================================
bool ojpPostStream(const String &body) {
  WiFiClientSecure client;
  HTTPClient http;
  int code = startOjpPost(client, http, body, 15000, " (stream)");
  if (code != HTTP_CODE_OK) {
    DBG_PRINTLN("[OJP] Error: " + http.getString().substring(0, 200));
    http.end();
    return false;
  }

  WiFiClient *stream = http.getStreamPtr();
  const int CHUNK = 1024;
  // Pre-reserve for the steady-state size (one chunk plus the
  // trailing overlap kept after each trim below), so the buffer
  // settles into that allocation instead of growing/reallocating on
  // every one of the first few chunks.
  String buf = "";
  buf.reserve(CHUNK + 2048 + 64);
  int found = 0;

  // Clear stationBoardData
  for (int i = 0; i < numEntries; i++) {
    stationBoardData[i].line = "-";
    stationBoardData[i].destination = "";
    stationBoardData[i].departure_time = "--:--";
    stationBoardData[i].delay = 0;
    stationBoardData[i].type = "";
    stationBoardData[i].line_operator = "";
  }

  unsigned long t0 = millis();
  while ((stream->connected() || stream->available()) && found < numEntries) {
    if (millis() - t0 > 14000) {
      DBG_PRINTLN("[OJP] Stream timeout");
      break;
    }

    if (stream->available()) {
      uint8_t tmp[CHUNK];
      int n = stream->readBytes(tmp, CHUNK);
      if (n > 0) {
        // concat(ptr, length) appends the chunk directly into the
        // already-reserved buffer, instead of constructing a
        // temporary String((char*)tmp) just to append it.
        buf.concat((const char *)tmp, n);
      }
    } else {
      delay(1);
      continue;
    }

    // Extract all complete <StopEventResult>…</StopEventResult> blocks
    // StopEventResult contains both <StopEvent> (times) and <Service>
    // (line name, destination) as siblings - must use the outer wrapper
    while (found < numEntries) {

      int start = buf.indexOf("<StopEventResult>");
      if (start < 0)
        break;

      int end = buf.indexOf("</StopEventResult>", start);
      if (end < 0)
        break;

      // ---------- Timetabled ----------
      String timetabled = "";
      int p = buf.indexOf("<TimetabledTime>", start);
      if (p >= 0 && p < end) {
        p += strlen("<TimetabledTime>");
        int q = buf.indexOf("</TimetabledTime>", p);
        if (q > p && q < end)
          timetabled = buf.substring(p, q);
      }

      // ---------- Estimated ----------
      String estimated = "";
      p = buf.indexOf("<EstimatedTime>", start);
      if (p >= 0 && p < end) {
        p += strlen("<EstimatedTime>");
        int q = buf.indexOf("</EstimatedTime>", p);
        if (q > p && q < end)
          estimated = buf.substring(p, q);
      }

      // ---------- Line ----------
      String lineName = "";

      // 1. PublicCode (preferred)
      p = buf.indexOf("<PublicCode>", start);
      if (p >= 0 && p < end) {

        p += strlen("<PublicCode>");

        int q = buf.indexOf("</PublicCode>", p);

        if (q > p && q < end)
          lineName = buf.substring(p, q);
      }

      // 2. Fallback: PublishedServiceName
      if (lineName.length() == 0) {

        p = buf.indexOf("<PublishedServiceName>", start);

        if (p >= 0 && p < end) {

          p = buf.indexOf("<Text", p);

          if (p >= 0 && p < end) {

            p = buf.indexOf(">", p);

            if (p >= 0 && p < end) {

              p++;

              int q = buf.indexOf("</Text>", p);

              if (q > p && q < end)
                lineName = buf.substring(p, q);
            }
          }
        }
      }

      // ---------- Destination ----------
      String dest = "";

      p = buf.indexOf("<DestinationText>", start);
      if (p >= 0 && p < end) {

        p = buf.indexOf("<Text", p);

        if (p >= 0 && p < end) {

          p = buf.indexOf(">", p);

          if (p >= 0 && p < end) {

            p++;

            int q = buf.indexOf("</Text>", p);

            if (q > p && q < end)
              dest = buf.substring(p, q);
          }
        }
      }

      DBG_PRINTLN("----------------");
      DBG_PRINTLN("Timetabled = " + timetabled);
      DBG_PRINTLN("Estimated  = " + estimated);

      // Show the planned (timetabled) time, with any delay shown
      // separately via stationBoardData[].delay (rendered as a
      // "+N" badge in displayStationBoardData()). Falls back to the
      // estimated time only if no timetabled time was provided by
      // OJP, so a time is still shown in that edge case.
      String depTime = isoToHHMM(
        timetabled.length() ? timetabled : estimated);

      int delayMin = 0;

      if (!timetabled.isEmpty() && !estimated.isEmpty()) {
        delayMin = calcDelay(timetabled, estimated);
      }

      stationBoardData[found].line =
        lineName.length() ? lineName : "-";

      stationBoardData[found].destination = dest;
      stationBoardData[found].departure_time = depTime;
      stationBoardData[found].delay = delayMin;

      DBG_PRINTF(
        "[OJP] %d: '%s' -> '%s' %s +%d\n",
        found,
        lineName.c_str(),
        dest.c_str(),
        depTime.c_str(),
        delayMin);

      found++;

      // Remove processed StopEventResult
      buf.remove(0, end + strlen("</StopEventResult>"));
    }

    // Trim buffer but keep overlap for cross-chunk tags
    // StopEventResult blocks can be ~2KB so keep 2KB overlap.
    // remove() shifts the kept tail left in place; unlike
    // buf = buf.substring(...), it doesn't allocate a new buffer and
    // copy into it.
    if (buf.length() > (unsigned)(CHUNK + 2048))
      buf.remove(0, buf.length() - 2048);
  }

  http.end();
  DBG_PRINTF("[OJP] Stream done, %d departures, heap=%d\n", found, ESP.getFreeHeap());
  return found > 0;
}

// ===========================================================================
// extractTag() – lightweight XML field extractor
// ===========================================================================
/**
 * @brief Extracts the text content between the first matching
 *        `open`/`close` tag pair found in `xml`, starting at `from`.
 *
 * This is a minimal string-search based XML reader used instead of a
 * full XML parser, to keep memory usage low on the ESP32. It does
 * not handle nested tags of the same name.
 *
 * @param xml   Source XML/text to search in.
 * @param open  Opening tag or marker to search for, e.g. "<StopPlaceRef>".
 * @param close Closing tag or marker, e.g. "</StopPlaceRef>".
 * @param from  Index to start searching from (default 0).
 * @return Text between `open` and `close`, or "" if either is not found.
 */
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
/**
 * @brief Finds the nearest stops to the current xCoord/yCoord via
 *        OJP's LocationInformationRequest and fills stationDataArray[].
 *
 * Requests up to `maxStations` stops within OJP_RADIUS_M meters,
 * computes each stop's great-circle (Haversine) distance from the
 * current position, and stores id/name/distance in
 * stationDataArray[]. On success, updates `stationsFound` and sets
 * `stationID` to the currently selected station (stationIndex).
 * Does nothing (leaves prior data untouched) if the request fails or
 * returns no usable results.
 */
void fetchStationDataFromGPS() {
  DBG_PRINTLN("[OJP] LocationInfo lat=" + xCoord + " lng=" + yCoord);

  time_t now;
  time(&now);
  struct tm *utc = gmtime(&now);
  char tsNow[25];
  strftime(tsNow, sizeof(tsNow), "%Y-%m-%dT%H:%M:%SZ", utc);

  // Pre-reserve the buffer once, sized generously for the final XML
  // (~650 chars), so the incremental += calls below fill in place
  // instead of triggering repeated heap reallocations/copies as the
  // String grows chunk by chunk.
  String body;
  body.reserve(700);
  body =
    "<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
    "<OJP xmlns=\"http://www.vdv.de/ojp\""
    " xmlns:siri=\"http://www.siri.org.uk/siri\""
    " version=\"2.0\""
    " xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\">"
    "<OJPRequest>"
    "<siri:ServiceRequest>"
    "<siri:RequestTimestamp>";
  body += tsNow;
  body += "</siri:RequestTimestamp>"
          "<siri:RequestorRef>SBB-EPaper-Display_prod</siri:RequestorRef>"
          "<OJPLocationInformationRequest>"
          "<siri:RequestTimestamp>";
  body += tsNow;
  body += "</siri:RequestTimestamp>"
          "<siri:MessageIdentifier>LIR-1</siri:MessageIdentifier>"
          "<InitialInput>"
          "<GeoRestriction>"
          "<Circle>"
          "<Center>"
          "<siri:Longitude>";
  body += yCoord;
  body += "</siri:Longitude>"
          "<siri:Latitude>";
  body += xCoord;
  body += "</siri:Latitude>"
          "</Center>"
          "<Radius>";
  body += OJP_RADIUS_M;
  body += "</Radius>"
          "</Circle>"
          "</GeoRestriction>"
          "</InitialInput>"
          "<Restrictions>"
          "<Type>stop</Type>"
          "<NumberOfResults>";
  body += maxStations;
  body += "</NumberOfResults>"
          "</Restrictions>"
          "</OJPLocationInformationRequest>"
          "</siri:ServiceRequest>"
          "</OJPRequest>"
          "</OJP>";

  String response = ojpPost(body);
  if (response.length() == 0) return;
  DBG_PRINTLN("[OJP] LocationInfo full response: " + response);

  const String LOC_OPEN = "<PlaceResult>";
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
    String stopRef = extractTag(block, "<StopPlaceRef>", "</StopPlaceRef>");
    String stopName = extractTag(block, "<StopPlaceName><Text xml:lang=\"de\">", "</Text>");
    if (stopName.length() == 0)
      stopName = extractTag(block, "<StopPlaceName><Text>", "</Text>");
    String latStr = extractTag(block, "<siri:Latitude>", "</siri:Latitude>");
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
      double a = sin(dLat / 2) * sin(dLat / 2) + cos(myLat * PI / 180.0) * cos(stopLat * PI / 180.0) * sin(dLng / 2) * sin(dLng / 2);
      dist = (int)(6371000.0 * 2.0 * atan2(sqrt(a), sqrt(1.0 - a)));
    }

    stationDataArray[found].gps_address = firstStopName;
    stationDataArray[found].near_station = stopName;
    stationDataArray[found].distance = dist;
    stationDataArray[found].station_id = stopRef;

    DBG_PRINTF("[OJP] Stop %d: %s  id=%s  dist=%dm\n",
                  found, stopName.c_str(), stopRef.c_str(), dist);
    found++;
  }

  if (found > 0) {
    stationsFound = found;
    stationID = stationDataArray[stationIndex].station_id;
    DBG_PRINTLN("[OJP] Active: " + stationDataArray[stationIndex].near_station + " (" + String(stationDataArray[stationIndex].distance) + "m)");
  }
}

// ===========================================================================
// fetchStationBoardData()
// CHANGED: uses OJP StopEventRequest instead of
//          transport.opendata.ch/v1/stationboard
// Fills stationBoardData[] with identical fields to the original.
// ===========================================================================
/**
 * @brief Fetches the next departures for the currently selected
 *        station and fills stationBoardData[] via ojpPostStream().
 *
 * Builds an OJP StopEventRequest for `stationDataArray[stationIndex]`
 * requesting `numEntries` real-time departures, and streams the
 * response. Returns immediately without making a request if no
 * station is currently selected (empty station_id).
 */
void fetchStationBoardData() {
  if (stationDataArray[stationIndex].station_id.length() == 0) return;

  stationID = stationDataArray[stationIndex].station_id;
  DBG_PRINTLN("[OJP] StopEvent for: " + stationID);

  time_t now;
  time(&now);
  struct tm *utc = gmtime(&now);
  char tsNow[25];
  strftime(tsNow, sizeof(tsNow), "%Y-%m-%dT%H:%M:%SZ", utc);

  // Pre-reserve once; this body is rebuilt on every loop() cycle, so
  // avoiding repeated reallocation here matters more than for the
  // LocationInformationRequest above.
  String body;
  body.reserve(750);
  body =
    "<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
    "<OJP xmlns=\"http://www.vdv.de/ojp\""
    " xmlns:siri=\"http://www.siri.org.uk/siri\""
    " xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\""
    " version=\"2.0\">"
    "<OJPRequest>"
    "<siri:ServiceRequest>"
    "<siri:RequestTimestamp>";
  body += tsNow;
  body += "</siri:RequestTimestamp>"
          "<siri:RequestorRef>SBB-EPaper-Display_prod</siri:RequestorRef>"
          "<OJPStopEventRequest>"
          "<siri:RequestTimestamp>";
  body += tsNow;
  body += "</siri:RequestTimestamp>"
          "<siri:MessageIdentifier>SER-1</siri:MessageIdentifier>"
          "<Location>"
          "<PlaceRef>"
          "<siri:StopPointRef>";
  body += stationID;
  body += "</siri:StopPointRef>"
          "<Name><Text>stop</Text></Name>"
          "</PlaceRef>"
          "<DepArrTime>";
  body += tsNow;
  body += "</DepArrTime>"
          "</Location>"
          "<Params>"
          "<NumberOfResults>";
  body += numEntries;
  body += "</NumberOfResults>"
          "<StopEventType>departure</StopEventType>"
          "<UseRealtimeData>full</UseRealtimeData>"
          "</Params>"
          "</OJPStopEventRequest>"
          "</siri:ServiceRequest>"
          "</OJPRequest>"
          "</OJP>";

  ojpPostStream(body);
}


// ===========================================================================
// Display functions – IDENTICAL to original
// ===========================================================================
/**
 * @brief Reads the battery voltage via the ADC and logs it to Serial.
 *
 * Converts the raw ADC reading using the eFuse-calibrated `vref`
 * (see setup()), the on-board 1:2 voltage divider, and the ADC
 * reference voltage. Values are clamped at 4.2 V (typical Li-Ion
 * full-charge voltage). The result is currently logged only; it is
 * not yet rendered on the display.
 */
void readBatVoltage() {
  delay(10);
  uint16_t v = analogRead(BATT_PIN);
  float battery_voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
  if (battery_voltage >= 4.2) battery_voltage = 4.2;
  DBG_PRINTLN("➸ Voltage: " + String(battery_voltage) + "V");
}

/**
 * @brief Draws the static "GPS:" / "Haltestelle:" (Stop:) labels.
 *
 * Called once from setup() before the first render; the labels are
 * outside the areas cleared/redrawn by displayStationData() and
 * displayStationBoardData(), so they persist across refreshes.
 */
void title() {
  int32_t cursor_x, cursor_y;
  cursor_x = 30;
  cursor_y = 50;
  writeln((GFXfont *)&FiraSans, (char *)"GPS: ", &cursor_x, &cursor_y, NULL);
  cursor_x = 30;
  cursor_y = 200;
  writeln((GFXfont *)&FiraSans, (char *)"Haltestelle: ", &cursor_x, &cursor_y, NULL);
}

/**
 * @brief Renders the current GPS address and nearest-stop info.
 *
 * Clears and redraws two rows:
 *  - Row 1: the reverse-geocoded street address (gpsData.x_coord).
 *  - Row 2: the name and distance (in meters) of the currently
 *    selected stop, from stationDataArray[stationIndex].
 *
 * Must be called between epd_poweron()/epd_poweroff(); relies on
 * fetchGPSAddress() and fetchStationDataFromGPS() having already
 * populated their respective data.
 */
void displayStationData() {
  int32_t cursor_x, cursor_y;
  epd_clear_area({ 250, 10, 700, 50 });
  epd_clear_area({ 250, 160, 700, 50 });

  // Row 1: reverse-geocoded street address
  cursor_x = 250;
  cursor_y = 50;
  // Fixed-size buffer instead of a VLA sized from gpsData.x_coord:
  // the address comes from an external Nominatim response, so an
  // unexpectedly long value must not size a stack allocation.
  // toCharArray() truncates safely to fit and null-terminates.
  char pos[128];
  gpsData.x_coord.toCharArray(pos, sizeof(pos));
  writeln((GFXfont *)&FiraSans, pos, &cursor_x, &cursor_y, NULL);

  // Row 2: nearest stop name + distance
  cursor_x = 250;
  cursor_y = 200;
  String nStation = stationDataArray[stationIndex].near_station + " " + String(stationDataArray[stationIndex].distance) + " m";
  // Same reasoning: fixed size instead of a VLA sized from the
  // (externally-sourced) stop name.
  char nst[128];
  nStation.toCharArray(nst, sizeof(nst));
  writeln((GFXfont *)&FiraSans, nst, &cursor_x, &cursor_y, NULL);
}

/**
 * @brief Renders the departure board (up to `numEntries` rows).
 *
 * For each entry in stationBoardData[], clears its row area and
 * draws four columns: line name, destination, departure time, and
 * (if delayed) the delay in minutes prefixed with "+". Rows with no
 * data show the placeholder values set in ojpPostStream()
 * ("-", "", "--:--", 0).
 *
 * Must be called between epd_poweron()/epd_poweroff(); relies on
 * fetchStationBoardData() having already populated stationBoardData[].
 */
void displayStationBoardData() {
  int32_t cursor_x, cursor_y;
  const int base_y = 300;
  const int row_height = 55;
  const int col1_x = 30;
  const int col2_x = 100;
  const int col3_x = 765;
  const int col4_x = 855;

  for (int i = 0; i < numEntries; i++) {
    int current_y = base_y + (i * row_height);
    epd_clear_area({ col1_x - 2, current_y - 45, 910, row_height + 5 });

    cursor_x = col1_x;
    cursor_y = current_y;
    // Fixed-size buffers instead of VLAs sized from OJP response
    // data: line names, times and delay text are all bounded in
    // practice, but sizing a stack array directly from
    // externally-sourced String::length() is a stack-overflow risk
    // if the API ever returns something unexpectedly large.
    // toCharArray() truncates safely to fit and null-terminates.
    char line[24];
    stationBoardData[i].line.toCharArray(line, sizeof(line));
    writeln((GFXfont *)&FiraSans, line, &cursor_x, &cursor_y, NULL);

    cursor_x = col2_x;
    char dest[128];
    stationBoardData[i].destination.toCharArray(dest, sizeof(dest));
    writeln((GFXfont *)&FiraSans, dest, &cursor_x, &cursor_y, NULL);

    cursor_x = col3_x;
    char depTime[8];  // "HH:MM" + margin
    stationBoardData[i].departure_time.toCharArray(depTime, sizeof(depTime));
    writeln((GFXfont *)&FiraSans, depTime, &cursor_x, &cursor_y, NULL);

    cursor_x = col4_x;

    if (stationBoardData[i].delay > 0) {
      String delayStr = "+" + String(stationBoardData[i].delay);
      char delay[8];  // "+120" + margin, delay is clamped to [0,120] by calcDelay()
      delayStr.toCharArray(delay, sizeof(delay));
      writeln((GFXfont *)&FiraSans, delay, &cursor_x, &cursor_y, NULL);
    }
  }
}

/**
 * @brief Renders the current local time as "HH:MM" in the top area.
 *
 * Clears its fixed-size area first, then draws the time obtained via
 * getLocalTime(). Silently does nothing if the local time is not
 * (yet) available.
 *
 * Must be called between epd_poweron()/epd_poweroff().
 */
void displayTime() {
  int32_t cursor_x = 500, cursor_y = 100;
  Rect_t clearRect = { 498, 55, 125, 55 };
  epd_clear_area(clearRect);

  struct tm timeInfo;

  if (getLocalTime(&timeInfo)) {

    char timeBuffer[6];
    strftime(timeBuffer, sizeof(timeBuffer), "%H:%M", &timeInfo);

    writeln((GFXfont *)&FiraSans, timeBuffer, &cursor_x, &cursor_y, NULL);
  }
}

/**
 * @brief Connects to WiFi using credentials from credentials.h.
 *
 * Blocks (polling every 500 ms) until WiFi.status() reports
 * WL_CONNECTED. There is currently no timeout or retry limit, so a
 * wrong SSID/password will hang here indefinitely.
 */
void connectWifi() {
  DBG_PRINTLN("Connecting to ");
  DBG_PRINTLN(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    DBG_PRINT(".");
  }
  DBG_PRINTLN("");
  DBG_PRINTLN("WiFi connected");
  DBG_PRINTLN(WiFi.localIP());
}

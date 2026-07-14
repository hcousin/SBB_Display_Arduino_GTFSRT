# SBB Display – Arduino / ESP32S3 (OJP edition)

E-paper departure board for Swiss public transport, running on an ESP32-S3.

## What changed from the original

The original sketch used the `transport.opendata.ch/v1` API (now discontinued).
This version uses the official **OJP (Open Journey Planner)** API from
[opentransportdata.swiss](https://opentransportdata.swiss/en/cookbook/open-journey-planner-landing-page/).

| | Old | New |
|---|---|---|
| API base URL | `http://transport.opendata.ch/v1/` | `https://api.opentransportdata.swiss/ojp2020` |
| Auth | none | Bearer token in `Authorization` header |
| Protocol | HTTP | **HTTPS** (WiFiClientSecure) |
| Departures | `/v1/stationboard` (JSON) | OJP **StopEventRequest** (XML) |
| Station lookup | GPS → `/v1/locations` | OJP **LocationInformationRequest** (GPS + radius) |
| Destination shown | ✅ | ✅ (from `ojp:DestinationText`) |
| Line name | route ID only | Full published name (e.g. "IC 5", "S3") |
| Real-time delay | basic | Computed from `TimetabledTime` vs `EstimatedTime` |
| JSON dependency | ArduinoJson | **removed** – OJP responses parsed with string search |

## What is displayed per departure row

| Column | Source |
|---|---|
| Line | `ojp:PublishedLineName` |
| Destination | `ojp:DestinationText` |
| Time | `ojp:EstimatedTime` (if delayed) or `ojp:TimetabledTime` |
| Delay | Difference between estimated and timetabled times (minutes) |

## Hardware

**Board:** LilyGO T5-4.7-S3 ("Screen-4.7-S3"), silkscreened revision
**V2.3, 2021-6-10**. ESP32-S3-WROOM-1 MCU, 4.7" e-paper panel, onboard
PCF8563 RTC, microSD slot, LiPo charging circuit, STEMMA QT/Qwiic
JST-SH 4-pin connector. Board-level pin assignments below are specific
to this revision; other LilyGO e-paper boards may differ.

### Pin reference

| Define | GPIO | Purpose | Status |
|---|---|---|---|
| `BUTTON_1` | 21 | Station-cycling push button (ISR, rising edge) | Active |
| `BATT_PIN` | 14 | Battery voltage ADC input | Active (read in `readBatVoltage()`, logged only - not yet shown on the display, see [issue #8](https://github.com/hcousin/SBB_Display_Arduino_GTFSRT/issues/8)) |
| `BOARD_SDA` / `BOARD_SCL` | 18 / 17 | I2C bus to the onboard PCF8563 RTC | Active, but currently not responding - see "Known hardware issues" below |
| `GPS_RX_PIN` / `GPS_TX_PIN` | 39 / 45 | UART to external GPS module (TX/RX) | Active - see "GPS module wiring" below for full wiring incl. power |
| `GPIO_MOSI` / `GPIO_MISO` | 10 / 48 | Free/unconnected GPIOs (LilyGO-documented) | Reserved, unused |
| `SD_MISO` / `SD_MOSI` / `SD_SCLK` / `SD_CS` | 16 / 15 / 11 / 42 | microSD card SPI | **Defined but dead code** - `SD.begin()` is never called, no SD card functionality is implemented despite `#include <SD.h>` |

The e-paper panel itself is driven by the `epd47`/`LilyGo-EPD47`
library internally and doesn't need separate pin defines in this
sketch.

### Known hardware issues

- **RTC (PCF8563) not detected** - open, unresolved. An I2C scan
  (both SDA/SCL orientations) found no device at all on the bus,
  which only has the RTC on it on this board revision. Likely a
  soldering/manufacturing defect on this specific unit; next steps
  would be a visual inspection of the RTC chip's solder joints and/or
  a multimeter check of its VDD pin. Not currently blocking - the
  sketch gets its time from NTP at boot regardless (see `setup()`),
  the RTC would only help retain time across reboots without WiFi.
- **Antenna connector proximity to the backup battery holder** -
  resolved, but a recurring risk on reassembly. See "Power and
  antenna placement" under "GPS module wiring" below.

## Setup

### 1 – Get an API key

1. Register at <https://api-manager.opentransportdata.swiss/>
2. Create an Application and subscribe to the **OJP** API (Default Key plan).
3. Copy the Bearer token into `GTFS_RT_API_KEY` in `credentials.h`.

### 2 – Configure credentials.h

```cpp
const char *ssid            = "YOUR_WIFI_SSID";
const char *password        = "YOUR_WIFI_PASSWORD";
const char *GTFS_RT_API_KEY = "eyJ...your token...";

// Fallback stops when GPS is unavailable
const char *STOP_IDS[]   = { "8503000", "8507000" };
const char *STOP_NAMES[] = { "Zürich HB", "Bern" };
const int   NUM_STOPS    = 2;
```

### 3 – GPS module wiring (optional but recommended)

The device attempts a GPS fix on startup and uses the OJP
`LocationInformationRequest` to find the nearest stop automatically.
If no fix is obtained within 30 s it falls back to the static stop list.

Default UART pins (adjustable at the top of the `.ino`):

| ESP32-S3 pin | GPS module | Notes |
|---|---|---|
| GPIO 39 | TX (output from GPS) | Silkscreen label "CS" on the P8 header (left edge of the board, 3rd row from the top of the labeled section) |
| GPIO 45 | RX (input to GPS, optional) | Silkscreen label "SCL" on the P8 header, right next to GPIO 39 |
| VBUS | VCC | Topmost row of the same P8 header, left column. **5 V, only powered while USB-C is connected** - see "Power and antenna placement" below |
| GND | GND | Any GND pin on the P8 header |

GPIO 43/44 (the ESP32-S3's default UART0 TX/RX, also used by the
onboard USB-serial console) are intentionally **not** used for the
GPS, to avoid conflicting with Serial output and firmware uploads.
(GPIO 10/48 remain free for other peripherals - see the Hardware
pin reference above.)

**Power and antenna placement:** VCC is wired to VBUS (topmost row
of the P8 header), since this board has no dedicated 3.3 V pin -
only VBUS (raw USB-C 5 V, present only while USB-C is connected).
The tested GPS module (ATGM336H-5N-31 / GOOUUU-GPS-BD) is
seller-specified for 5 V, and its AT6558 chip needs only 2.7–3.6 V,
so VBUS fits. GPS therefore only has power while USB-C is connected;
on battery-only operation it falls back to the static stop list, as
if no GPS were wired. A true 3.3 V rail would be needed for
battery-powered GPS.

The module once ran hot; the cause was a short between the backup
battery's (+) terminal and the antenna's GND/shield, not VBUS. Fixed,
but the connector sits close enough to the battery holder that a
slight twist can reintroduce the short. Insulate the holder's
contacts with Kapton/electrical tape and secure the antenna cable
(hot glue or a cable tie) before closing the enclosure.

### 4 – Required Arduino libraries

Install via Library Manager (or "Add .ZIP Library" if not indexed):

- **LilyGo-EPD47** by Xinyuan-LilyGO — e-paper panel driver
  (`epd_driver.h`). Use the **`esp32s3` branch** specifically
  (`git clone -b esp32s3 https://github.com/Xinyuan-LilyGO/LilyGo-EPD47`),
  the default branch targets older, non-S3 LilyGo e-paper boards.
- **SensorLib** by lewisxhe — RTC driver (`SensorPCF8563.hpp`),
  used directly by this sketch for the onboard PCF8563. Also a
  declared dependency of LilyGo-EPD47 itself.
- **Button2** by lennarthennigs — not used directly by this sketch
  (button handling here is a plain `attachInterrupt`), but it's a
  declared dependency of LilyGo-EPD47's `library.json`. Arduino
  Library Manager installs it automatically alongside LilyGo-EPD47;
  if you instead install LilyGo-EPD47 via "Add .ZIP Library" or
  manual git clone, install this one too to avoid a missing-header
  build error.
- **TinyGPSPlus** by Mikal Hart — GPS NMEA parsing.

Everything else the sketch includes (`WiFi.h`, `HTTPClient.h`,
`WiFiClientSecure.h`, `SPI.h`, `SD.h`, `Wire.h`, `esp_adc_cal.h`,
`esp_sntp.h`) ships with the ESP32 Arduino core - no separate
install needed. `firasans.h` and `sbbdisplay.h` are project files
already included in this repo's sketch folder, not libraries.

ArduinoJson is **no longer required**.

### 5 – Arduino IDE settings

- Board: **ESP32S3 Dev Module**
- USB CDC On Boot: Enable
- Flash Size: 16 MB (128 Mb)
- Partition Scheme: 16M Flash (3M APP / 9.9 MB FATFS)
- PSRAM: OPI PSRAM

## Button behaviour

| State | Button press |
|---|---|
| GPS stop active | Leave GPS mode → first static stop |
| Static list | Cycle to next stop; wrap-around retries GPS |

## API reference

- OJP StopEventService: <https://opentransportdata.swiss/en/cookbook/ojp-stopeventservice/>
- OJP LocationInformationRequest: <https://opentransportdata.swiss/cookbook/ojplocationinformationrequest/>
- API key howto: <https://opentransportdata.swiss/howto-access-apis>

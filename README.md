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
| VBUS | VCC | Topmost row of the same P8 header, left column. **5 V, only powered while USB-C is connected** (see note below) |
| GND | GND | Any GND pin on the P8 header |

These are 2 of the 4 GPIOs LilyGO documents as free/unconnected on the
T5-4.7-S3 (the other two, GPIO 10 and GPIO 48, are unused and available
for other peripherals). GPIO 43/44 (the ESP32-S3's default UART0
TX/RX, also used by the onboard USB-serial console) are intentionally
**not** used for the GPS, to avoid conflicting with Serial output and
firmware uploads.

**Power note:** this board does not break out a dedicated 3.3 V pin
on the P8 header, only VBUS (raw USB-C 5 V, present only when USB is
plugged in) and various signal/GND pins. The tested GPS module here
is an ATGM336H-5N-31 (GOOUUU-GPS-BD breakout), whose seller-specified
VCC range is 5 V DC (the underlying AT6558 chip itself only needs
2.7–3.6 V, so VBUS is safely within range) — VBUS is used for VCC.
**Consequence: GPS is only powered while the device is connected via
USB-C. On battery-only operation (BAT+/BAT-), the GPS module loses
power and no fix will be obtained** — the device falls back to the
static stop list in that case, same as if no GPS were wired at all.
If your project needs GPS on battery power, you'll need to source a
true 3.3 V rail (e.g. probe the board's voltage regulator output
directly with a multimeter) rather than VBUS.

### 4 – Required Arduino libraries

Install via Library Manager:
- **TinyGPSPlus** by Mikal Hart (GPS NMEA parsing)

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

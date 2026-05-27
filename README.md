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

| ESP32-S3 pin | GPS module |
|---|---|
| GPIO 44 | TX (output from GPS) |
| GPIO 43 | RX (input to GPS, optional) |

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

# SBB Display – Arduino / ESP32S3 (GTFS-RT edition)

E-paper departure board for Swiss public transport, running on an ESP32-S3.

## What changed from the original

The original sketch used the `transport.opendata.ch/v1` API (now discontinued).
This version uses the official **GTFS Realtime** feed from
[opentransportdata.swiss](https://opentransportdata.swiss/de/cookbook/realtime-prediction-cookbook/gtfs-rt/).

| | Old | New |
|---|---|---|
| API base URL | `http://transport.opendata.ch/v1/` | `https://api.opentransportdata.swiss/la/gtfs-rt` |
| Auth | none | Bearer token in `Authorization` header |
| Protocol | HTTP | **HTTPS** (WiFiClientSecure) |
| Format | JSON (custom) | GTFS-RT (binary protobuf, or `?format=JSON` for embedded use) |
| Station lookup | GPS → `/v1/locations` | Static list in `credentials.h` |
| Rate limit | none published | **2 requests/minute** (30 s server cache) |
| Redirects | not needed | **required** (`setFollowRedirects`) |

## Setup

### 1 – Get an API key

1. Register at <https://api-manager.opentransportdata.swiss/>
2. Create an Application and subscribe to the **GTFS-RT** API (Default Key plan).
3. Copy the Bearer token into `GTFS_RT_API_KEY` in `credentials.h`.

### 2 – Find your GTFS stop IDs

Download `stops.txt` from the current GTFS Static package:  
<https://data.opentransportdata.swiss/dataset/timetable-2026-gtfs2020>

Search for your station name; copy the `stop_id` value (e.g. `8503000` for
Zürich HB) into `STOP_IDS[]` in `credentials.h`.  Add a matching display name
to `STOP_NAMES[]`.

### 3 – Configure credentials.h

```cpp
const char *ssid            = "YOUR_WIFI_SSID";
const char *password        = "YOUR_WIFI_PASSWORD";
const char *GTFS_RT_API_KEY = "eyJ...your token...";

const char *STOP_IDS[]   = { "8503000", "8507000" };
const char *STOP_NAMES[] = { "Zürich HB", "Bern" };
const int   NUM_STOPS    = 2;
```

### 4 – Arduino IDE settings

- Board: **ESP32S3 Dev Module**
- USB CDC On Boot: Enable
- Flash Size: 16 MB (128 Mb)
- Partition Scheme: 16M Flash (3M APP / 9.9 MB FATFS)
- PSRAM: OPI PSRAM

## Known limitations

- **Destination (headsign)** is not part of a GTFS-RT Trip Update message.  
  To show it, you would need to download GTFS Static `trips.txt` and join
  on `trip_id` – not practical on the ESP32.  The destination column is left
  blank for now.
- **GPS-based stop discovery** has been removed.  Use the static stop list.
- The JSON endpoint (`?format=JSON`) is described by opentransportdata.swiss
  as *"for testing only"* and can be up to ~11 MB.  The sketch uses
  ArduinoJson's **filter** feature to parse only the relevant fields and keep
  RAM usage manageable.  For a production deployment, switching to the binary
  protobuf endpoint and a protobuf library (e.g. `nanopb`) is recommended.
- TLS certificate validation is disabled (`client.setInsecure()`).  For a
  secure deployment, load the DigiCert root CA with `client.setCACert()`.

## API reference

- GTFS-RT cookbook: <https://opentransportdata.swiss/de/cookbook/realtime-prediction-cookbook/gtfs-rt/>
- API key howto: <https://opentransportdata.swiss/howto-access-apis>
- GTFS-RT spec: <https://developers.google.com/transit/gtfs-realtime/>

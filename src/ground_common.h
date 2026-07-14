#pragma once
// Shared ground-station logic — used by BOTH ground_main.cpp (Heltec ESP32 V3)
// and ground_wio_main.cpp (Wio Tracker L1 nRF52840). The two firmwares must
// stay functionally in sync: platform-independent behavior belongs here, not
// forked into the mains. Header-only because each env's build_src_filter
// compiles only its own main .cpp.
//
// Portability: nRF52 newlib-nano strips %f from sscanf/printf, so parsing
// uses strtof and formatting uses String(x, digits).

#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <Adafruit_GFX.h>
#include "gps_data.h"
#include "lora_config.h"

// Ages beyond this are highlighted as stale (bold on displays, yellow in the
// mobile app — keep in sync with STALE_LOC_SEC in mobile/app/index.tsx)
static const uint32_t STALE_AGE_SEC = 5;

// Max distinct trackers a ground station follows at once; oldest is evicted
#define MAX_TRACKERS 4

inline const char* gpsNextField(const char* p) {
    const char* comma = strchr(p, ',');
    return comma ? comma + 1 : nullptr;
}

// Pointer to the idx'th comma-separated field (0-based), or nullptr
inline const char* gpsFieldAt(const char* p, int idx) {
    while (idx-- > 0 && p) p = gpsNextField(p);
    return p;
}

// Copy one comma-terminated field, trimming trailing whitespace
inline void gpsCopyField(char* dst, size_t dstSize, const char* src) {
    size_t i = 0;
    while (src && src[i] && src[i] != ',' && i < dstSize - 1) {
        dst[i] = src[i];
        i++;
    }
    dst[i] = '\0';
    while (i > 0 && (dst[i - 1] == '\r' || dst[i - 1] == '\n' || dst[i - 1] == ' ')) {
        dst[--i] = '\0';
    }
}

// Record for this tracker — matched by hardware ID when present (stable even
// across renames), by name otherwise; else a free slot; else the stalest slot
inline GPSData* findTrackerSlot(GPSData* trackers, const char* id, const char* name) {
    for (int i = 0; i < MAX_TRACKERS; i++) {
        if (!trackers[i].dataValid) continue;
        if (id[0] ? (strcmp(trackers[i].trackerId, id) == 0)
                  : (strcmp(trackers[i].trackerName, name) == 0)) {
            return &trackers[i];
        }
    }
    for (int i = 0; i < MAX_TRACKERS; i++) {
        if (!trackers[i].dataValid) return &trackers[i];
    }
    GPSData* oldest = &trackers[0];
    for (int i = 1; i < MAX_TRACKERS; i++) {
        if (trackers[i].lastUpdate < oldest->lastUpdate) oldest = &trackers[i];
    }
    return oldest;
}

// Hardware IDs are exactly 6 hex digits
inline bool isTrackerId(const char* s) {
    int n = 0;
    for (; s[n]; n++) {
        if (!isxdigit((unsigned char)s[n])) return false;
    }
    return n == 6;
}

// Display label: manually assigned name wins, then hardware ID
inline const char* trackerLabel(const GPSData& t) {
    if (t.trackerName[0]) return t.trackerName;
    if (t.trackerId[0]) return t.trackerId;
    return "Tracker";
}

// Most recently heard tracker, or nullptr if none yet — used by the displays
inline GPSData* mostRecentTracker(GPSData* trackers) {
    GPSData* best = nullptr;
    for (int i = 0; i < MAX_TRACKERS; i++) {
        if (!trackers[i].dataValid) continue;
        if (!best || trackers[i].lastUpdate > best->lastUpdate) best = &trackers[i];
    }
    return best;
}

// Parses "GPS:state,lat,lon,alt,speed,sats,hdop[,id[,name[,battV]]]" or
// "GPS:state,NoFix,sats,hdop[,id[,name[,battV]]]" into the per-tracker record for the
// trailing hardware ID (name is display-only and empty unless manually
// assigned; both are absent from older vehicles). On NoFix the record's
// previous lat/lon/lastGPSFixTime survive so the station can show a last-known
// position with its age. Returns the updated record, or nullptr if the packet
// wasn't a GPS packet.
inline GPSData* parseGPSPacket(const char* data, GPSData* trackers) {
    if (strncmp(data, "GPS:", 4) != 0) return nullptr;
    const char* p = data + 4;

    // ID and name are the trailing fields; extract first to pick the record
    const char* f1 = gpsFieldAt(p, 1);
    bool noFix = f1 && strncmp(f1, "NoFix", 5) == 0;
    char id[sizeof(((GPSData*)0)->trackerId)] = "";
    char name[sizeof(((GPSData*)0)->trackerName)] = "";
    const char* idField = gpsFieldAt(p, noFix ? 4 : 7);
    const char* nameField = gpsFieldAt(p, noFix ? 5 : 8);
    gpsCopyField(id, sizeof(id), idField);
    gpsCopyField(name, sizeof(name), nameField);
    if (idField && !nameField && !isTrackerId(id)) {
        // Intermediate vehicle firmware sent only a name where the ID now
        // lives — treat it as the name
        strncpy(name, id, sizeof(name) - 1);
        name[sizeof(name) - 1] = '\0';
        id[0] = '\0';
    }

    GPSData& gpsData = *findTrackerSlot(trackers, id, name);
    bool sameTracker = gpsData.dataValid &&
        (id[0] ? strcmp(gpsData.trackerId, id) == 0
               : strcmp(gpsData.trackerName, name) == 0);
    if (!sameTracker) {
        // Slot is new or evicted from another tracker — clear stale state
        memset(&gpsData, 0, sizeof(gpsData));
    }
    // Name may change on a live record (rename via BLE); ID never does
    strncpy(gpsData.trackerId, id, sizeof(gpsData.trackerId) - 1);
    strncpy(gpsData.trackerName, name, sizeof(gpsData.trackerName) - 1);

    // Battery volts trails the name (absent from older vehicles — keep last value)
    const char* battField = gpsFieldAt(p, noFix ? 6 : 9);
    if (battField) gpsData.batteryVoltage = strtof(battField, nullptr);

    gpsData.lastUpdate = millis();
    gpsData.dataValid = true;

    gpsData.vehicleState = (FlightState)strtol(p, nullptr, 10);
    p = gpsNextField(p);
    if (!p) return &gpsData;

    if (noFix) {
        gpsData.hasGPSFix = false;
        p = gpsNextField(p);
        if (p) {
            gpsData.satellites = strtol(p, nullptr, 10);
            p = gpsNextField(p);
            if (p) gpsData.hdop = strtof(p, nullptr);
        }
        Serial.print("Received [");
        Serial.print(trackerLabel(gpsData));
        Serial.print(" ");
        Serial.print(flightStateName(gpsData.vehicleState));
        Serial.println("]: No GPS Fix");
    } else {
        gpsData.hasGPSFix = true;
        gpsData.lastGPSFixTime = millis();
        gpsData.latitude = strtof(p, nullptr);
        p = gpsNextField(p);
        if (!p) return &gpsData;
        gpsData.longitude = strtof(p, nullptr);
        p = gpsNextField(p);
        if (!p) return &gpsData;
        gpsData.altitude = strtof(p, nullptr);
        p = gpsNextField(p);
        if (!p) return &gpsData;
        gpsData.speed = strtof(p, nullptr);
        p = gpsNextField(p);
        if (!p) return &gpsData;
        gpsData.satellites = strtol(p, nullptr, 10);
        p = gpsNextField(p);
        if (p) gpsData.hdop = strtof(p, nullptr);

        Serial.print("Received [");
        Serial.print(trackerLabel(gpsData));
        Serial.print(" ");
        Serial.print(flightStateName(gpsData.vehicleState));
        Serial.print("]: ");
        Serial.print(gpsData.latitude, 6);
        Serial.print(",");
        Serial.print(gpsData.longitude, 6);
        Serial.print(" Alt:");
        Serial.print(gpsData.altitude, 0);
        Serial.println("m");
    }

    return &gpsData;
}

// Telemetry JSON sent over BLE — ONE tracker per notification so every
// payload fits in a single MTU (no long reads, which tear when the value is
// rewritten mid-read). The app merges per-tracker payloads by "id"/"trk".
// Identical payload from both stations so the mobile app works against either.
inline String buildTrackerJSON(const GPSData& t, unsigned long currentTime, float loraFrequency) {
    String json = "{\"freq\":" + String(loraFrequency, 1);
    json += ",\"trk\":\"" + String(t.trackerName) + "\"";
    json += ",\"id\":\"" + String(t.trackerId) + "\"";
    json += ",\"state\":" + String((int)t.vehicleState);
    json += ",\"stateName\":\"" + String(flightStateName(t.vehicleState)) + "\"";
    json += ",\"fix\":" + String(t.hasGPSFix ? "true" : "false");
    json += ",\"locAge\":" + String(t.lastGPSFixTime > 0 ? (int32_t)((currentTime - t.lastGPSFixTime) / 1000) : -1);
    json += ",\"lat\":" + String(t.latitude, 6);
    json += ",\"lon\":" + String(t.longitude, 6);
    json += ",\"alt\":" + String(t.altitude, 1);
    json += ",\"spd\":" + String(t.speed, 1);
    json += ",\"sat\":" + String(t.satellites);
    json += ",\"hdop\":" + String(t.hdop, 2);
    json += ",\"rssi\":" + String(t.rssi, 0);
    json += ",\"snr\":" + String(t.snr, 1);
    json += ",\"batt\":" + String(t.batteryVoltage, 2);
    json += ",\"age\":" + String((currentTime - t.lastUpdate) / 1000);
    json += "}";
    return json;
}

// Fake-bold for the default GFX 5x7 font: reprint 1px right of the original.
// Leaves the cursor where a normal print would, so surrounding text is unaffected.
inline void printBoldIf(Adafruit_GFX& gfx, const char* text, bool bold) {
    int16_t x = gfx.getCursorX();
    int16_t y = gfx.getCursorY();
    gfx.print(text);
    if (bold) {
        int16_t xEnd = gfx.getCursorX();
        gfx.setCursor(x + 1, y);
        gfx.print(text);
        gfx.setCursor(xEnd, y);
    }
}

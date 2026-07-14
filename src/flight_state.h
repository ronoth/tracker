#pragma once

#include <Arduino.h>
#include <math.h>

// States from RocketState enum in sensor_data.proto
// Linear forward-only progression
enum FlightState : uint8_t {
    STATE_ERROR           = 0,
    STATE_INITIALIZING    = 1,
    STATE_WAITING_ON_GPS  = 2,
    STATE_ARMED           = 10,
    STATE_PAD_IDLE        = 11,
    STATE_ASCENT          = 22,
    STATE_APOGEE          = 25,
    STATE_DESCENT         = 41,
    STATE_RECOVERY        = 62
};

// Events: detected and logged, but don't drive state transitions
enum FlightEventType : uint8_t {
    EVENT_NONE             = 0,
    EVENT_IGNITION         = 1,
    EVENT_BURNOUT          = 2,
    EVENT_DROGUE_DEPLOY    = 3,
    EVENT_MAIN_DEPLOY      = 4,
    EVENT_TOUCHDOWN        = 5,
    EVENT_MACH_TRANSITION  = 6,
    EVENT_SUPERSONIC       = 7
};

struct FlightEvent {
    uint32_t timestamp;
    FlightEventType type;
    float value;  // relevant sensor value at detection time
};

// Callback type for event notifications
typedef void (*FlightEventCallback)(const FlightEvent& event);

// ----- Configurable thresholds -----
constexpr float FSM_LAUNCH_ACCEL_THRESHOLD  = 5.0f;    // m/s² above baseline to detect launch
constexpr uint8_t FSM_LAUNCH_SAMPLE_COUNT   = 3;       // consecutive samples above threshold
constexpr float FSM_APOGEE_PRESSURE_MARGIN  = 2.0f;    // hPa above min pressure to confirm apogee
constexpr uint8_t FSM_APOGEE_SAMPLE_COUNT   = 8;       // consecutive samples of rising pressure
constexpr float FSM_LANDING_HEIGHT_TOLERANCE = 4.0f;   // meters — max height variation to detect landing
constexpr uint32_t FSM_LANDING_STABLE_MS    = 5000;    // ms of stable altitude to confirm landing
constexpr uint32_t FSM_PAD_BASELINE_MS      = 10000;   // ms of stable readings to auto-arm
constexpr float FSM_PAD_ACCEL_TOLERANCE     = 0.5f;    // m/s² jitter allowed during baseline
constexpr float FSM_PAD_PRESSURE_TOLERANCE  = 1.0f;    // hPa jitter allowed during baseline
constexpr float FSM_BURNOUT_ACCEL_THRESHOLD = 2.0f;    // m/s² total — near freefall
constexpr uint8_t FSM_BURNOUT_SAMPLE_COUNT  = 3;       // consecutive samples below threshold
constexpr float FSM_MAIN_DEPLOY_AGL         = 300.0f;  // meters above ground level
constexpr float FSM_SUPERSONIC_SPEED        = 248.0f;  // m/s — baro unreliable above this (matches AltOS)
constexpr float FSM_SUBSONIC_SPEED          = 328.0f;  // m/s — baro fully distrusted above this

// Kalman filter tuning — pre-computed gains for 26Hz (~0.0385s step)
// These are starting values; tune against flight data
constexpr float KF_DT                       = 0.0385f; // 1/26 seconds
constexpr float KF_DT2_OVER_2               = 0.000741f; // dt²/2

// Kalman gains for baro+accel fused correction (ascent)
constexpr float KF_BOTH_K00 = 0.15f;   // height  <- baro error
constexpr float KF_BOTH_K01 = 0.005f;  // height  <- accel error
constexpr float KF_BOTH_K10 = 0.60f;   // speed   <- baro error
constexpr float KF_BOTH_K11 = 0.02f;   // speed   <- accel error
constexpr float KF_BOTH_K20 = 1.20f;   // accel   <- baro error
constexpr float KF_BOTH_K21 = 0.10f;   // accel   <- accel error

// Kalman gains for baro-only correction (descent)
constexpr float KF_BARO_K0 = 0.20f;    // height  <- baro error
constexpr float KF_BARO_K1 = 0.80f;    // speed   <- baro error
constexpr float KF_BARO_K2 = 1.50f;    // accel   <- baro error

// ----- State name helper -----
inline const char* flightStateName(FlightState s) {
    switch (s) {
        case STATE_ERROR:           return "ERROR";
        case STATE_INITIALIZING:    return "INIT";
        case STATE_WAITING_ON_GPS:  return "WAIT_GPS";
        case STATE_PAD_IDLE:        return "PAD_IDLE";
        case STATE_ARMED:           return "ARMED";
        case STATE_ASCENT:          return "ASCENT";
        case STATE_APOGEE:          return "APOGEE";
        case STATE_DESCENT:         return "DESCENT";
        case STATE_RECOVERY:        return "RECOVERY";
        default:                    return "UNKNOWN";
    }
}

inline const char* flightEventName(FlightEventType e) {
    switch (e) {
        case EVENT_NONE:            return "";
        case EVENT_IGNITION:        return "IGNITION";
        case EVENT_BURNOUT:         return "BURNOUT";
        case EVENT_DROGUE_DEPLOY:   return "DROGUE";
        case EVENT_MAIN_DEPLOY:     return "MAIN_DEPLOY";
        case EVENT_TOUCHDOWN:       return "TOUCHDOWN";
        case EVENT_MACH_TRANSITION: return "MACH";
        case EVENT_SUPERSONIC:      return "SUPERSONIC";
        default:                    return "";
    }
}

// ----- 3-State Kalman Filter -----
// Fuses barometer (height) and accelerometer during ascent.
// Baro-only during descent (gyro/accel orientation unknown under chute).
// Transonic distrust: scales down baro correction when speed > 248 m/s.
struct KalmanState {
    float height;   // meters AGL
    float speed;    // m/s (positive = up)
    float accel;    // m/s²

    void reset() {
        height = 0;
        speed = 0;
        accel = 0;
    }

    // Predict step: kinematic model
    void predict() {
        height += speed * KF_DT + accel * KF_DT2_OVER_2;
        speed  += accel * KF_DT;
    }

    // Correct step: fused baro + accel (used during ascent)
    // baroHeight: measured height AGL from barometer
    // measuredAccel: measured acceleration along vertical axis (m/s²)
    // distrust: 0.0 = full baro trust, 1.0 = fully distrust baro
    void correctBoth(float baroHeight, float measuredAccel, float distrust) {
        float errH = (baroHeight - height) * (1.0f - distrust);
        float errA = measuredAccel - accel;

        height += KF_BOTH_K00 * errH + KF_BOTH_K01 * errA;
        speed  += KF_BOTH_K10 * errH + KF_BOTH_K11 * errA;
        accel  += KF_BOTH_K20 * errH + KF_BOTH_K21 * errA;
    }

    // Correct step: baro-only (used during descent)
    void correctBaro(float baroHeight) {
        float errH = baroHeight - height;

        height += KF_BARO_K0 * errH;
        speed  += KF_BARO_K1 * errH;
        accel  += KF_BARO_K2 * errH;
    }
};

// ----- Flight State Machine -----
class FlightStateMachine {
public:
    void begin(bool autoArm = true, int armPin = -1) {
        _state = STATE_INITIALIZING;
        _autoArmEnabled = autoArm;
        _armPin = armPin;
        _eventCallback = nullptr;
        _lastEvent = EVENT_NONE;

        _baselinePressure = 0;
        _baselineAccelMag = 0;
        _stateEntryTime = millis();

        _launchConsecutive = 0;
        _apogeeConsecutive = 0;
        _burnoutConsecutive = 0;
        _burnoutDetected = false;
        _supersonicDetected = false;
        _mainDeployDetected = false;

        _baselineSampleCount = 0;
        _baselinePressureSum = 0;
        _baselineAccelSum = 0;
        _padStableStart = 0;

        _kf.reset();
        _kfInitialized = false;

        _landingIntervalEnd = 0;
        _landingMinHeight = 0;
        _landingMaxHeight = 0;

        if (_armPin >= 0) {
            pinMode(_armPin, INPUT_PULLDOWN);
        }
    }

    void onEvent(FlightEventCallback cb) {
        _eventCallback = cb;
    }

    // Call once per sensor sample (~26Hz)
    // accelMag: magnitude of acceleration vector (m/s²)
    // pressure: barometric pressure (hPa)
    // gpsAlt: GPS altitude (meters)
    // gpsSpeed: GPS speed (m/s)
    // now: millis()
    FlightState update(float accelMag, float pressure, float gpsAlt, float gpsSpeed, uint32_t now) {
        _lastEvent = EVENT_NONE;

        // Convert pressure to height AGL (rough: 1 hPa ≈ 8.3m near sea level)
        float baroHeight = (_baselinePressure > 0)
            ? (_baselinePressure - pressure) * 8.3f
            : 0.0f;

        // Run Kalman filter when we have a baseline
        if (_kfInitialized) {
            _kf.predict();

            if (_state >= STATE_DESCENT) {
                // Baro-only during descent — gyro/accel orientation unknown under chute
                _kf.correctBaro(baroHeight);
            } else {
                // Fused baro+accel during ascent with transonic distrust
                float distrust = computeBaroDistrust();
                _kf.correctBoth(baroHeight, accelMag - 9.81f, distrust);
            }
        }

        switch (_state) {
            case STATE_INITIALIZING:
                // Transitions handled externally by vehicle_main
                break;

            case STATE_WAITING_ON_GPS:
                // Transitions handled externally by vehicle_main
                break;

            case STATE_PAD_IDLE:
                updatePadIdle(accelMag, pressure, now);
                break;

            case STATE_ARMED:
                updateArmed(accelMag, now);
                break;

            case STATE_ASCENT:
                updateAscent(accelMag, gpsSpeed, now);
                break;

            case STATE_APOGEE:
                // Immediate transition to DESCENT (APOGEE is a momentary state)
                fireEvent(EVENT_DROGUE_DEPLOY, _kf.height, now);
                initLandingDetection(now);
                transitionTo(STATE_DESCENT, now);
                break;

            case STATE_DESCENT:
                updateDescent(now);
                break;

            case STATE_RECOVERY:
                // Terminal state — stay here
                break;

            case STATE_ERROR:
            default:
                break;
        }

        return _state;
    }

    FlightState getState() const { return _state; }
    FlightEventType getLastEvent() const { return _lastEvent; }
    float getBaselinePressure() const { return _baselinePressure; }
    float getBaselineAccelMag() const { return _baselineAccelMag; }

    // Kalman filter outputs — use these for telemetry/logging
    float getKFHeight() const { return _kf.height; }
    float getKFSpeed() const { return _kf.speed; }
    float getKFAccel() const { return _kf.accel; }

    // External state transitions (called by vehicle_main during init)
    void setInitialized() {
        if (_state == STATE_INITIALIZING) {
            transitionTo(STATE_WAITING_ON_GPS, millis());
        }
    }

    void setGPSReady() {
        if (_state == STATE_WAITING_ON_GPS) {
            transitionTo(STATE_PAD_IDLE, millis());
        }
    }

    void setError() {
        transitionTo(STATE_ERROR, millis());
    }

private:
    FlightState _state;
    FlightEventCallback _eventCallback;
    FlightEventType _lastEvent;

    // Arming
    bool _autoArmEnabled;
    int _armPin;

    // Baseline
    float _baselinePressure;
    float _baselineAccelMag;
    uint32_t _baselineSampleCount;
    float _baselinePressureSum;
    float _baselineAccelSum;
    uint32_t _padStableStart;

    // Tracking
    uint32_t _stateEntryTime;

    // Kalman filter
    KalmanState _kf;
    bool _kfInitialized;

    // Consecutive-sample counters for noise rejection
    uint8_t _launchConsecutive;
    uint8_t _apogeeConsecutive;
    uint8_t _burnoutConsecutive;
    bool _burnoutDetected;
    bool _supersonicDetected;
    bool _mainDeployDetected;

    // Landing detection: altitude stability over interval
    uint32_t _landingIntervalEnd;
    float _landingMinHeight;
    float _landingMaxHeight;

    void transitionTo(FlightState newState, uint32_t now) {
        Serial.print("[FSM] ");
        Serial.print(flightStateName(_state));
        Serial.print(" -> ");
        Serial.println(flightStateName(newState));
        _state = newState;
        _stateEntryTime = now;
    }

    void fireEvent(FlightEventType type, float value, uint32_t now) {
        _lastEvent = type;
        Serial.print("[FSM EVENT] ");
        Serial.print(flightEventName(type));
        Serial.print(" value=");
        Serial.println(value, 2);

        if (_eventCallback) {
            FlightEvent evt;
            evt.timestamp = now;
            evt.type = type;
            evt.value = value;
            _eventCallback(evt);
        }
    }

    // Compute barometer distrust factor based on Kalman speed.
    // 0.0 at FSM_SUPERSONIC_SPEED (248 m/s), 1.0 at FSM_SUBSONIC_SPEED (328 m/s).
    float computeBaroDistrust() const {
        float spd = fabsf(_kf.speed);
        if (spd <= FSM_SUPERSONIC_SPEED) return 0.0f;
        if (spd >= FSM_SUBSONIC_SPEED) return 1.0f;
        return (spd - FSM_SUPERSONIC_SPEED) / (FSM_SUBSONIC_SPEED - FSM_SUPERSONIC_SPEED);
    }

    // Initialize landing detection interval tracking
    void initLandingDetection(uint32_t now) {
        _landingIntervalEnd = now + FSM_LANDING_STABLE_MS;
        _landingMinHeight = _kf.height;
        _landingMaxHeight = _kf.height;
    }

    // ----- State update handlers -----

    void updatePadIdle(float accelMag, float pressure, uint32_t now) {
        // Accumulate baseline
        _baselineSampleCount++;
        _baselinePressureSum += pressure;
        _baselineAccelSum += accelMag;

        float runningAvgPressure = _baselinePressureSum / _baselineSampleCount;
        float runningAvgAccel = _baselineAccelSum / _baselineSampleCount;

        // Check stability
        bool accelStable = fabsf(accelMag - runningAvgAccel) < FSM_PAD_ACCEL_TOLERANCE;
        bool pressureStable = fabsf(pressure - runningAvgPressure) < FSM_PAD_PRESSURE_TOLERANCE;

        if (accelStable && pressureStable) {
            if (_padStableStart == 0) _padStableStart = now;
        } else {
            _padStableStart = 0;
        }

        // Check hardware arm pin
        if (_armPin >= 0 && digitalRead(_armPin) == HIGH) {
            _baselinePressure = runningAvgPressure;
            _baselineAccelMag = runningAvgAccel;
            _kf.reset();
            _kfInitialized = true;
            transitionTo(STATE_ARMED, now);
            return;
        }

        // Auto-arm after stable baseline period
        if (_autoArmEnabled && _padStableStart > 0 &&
            (now - _padStableStart) >= FSM_PAD_BASELINE_MS) {
            _baselinePressure = runningAvgPressure;
            _baselineAccelMag = runningAvgAccel;
            _kf.reset();
            _kfInitialized = true;
            transitionTo(STATE_ARMED, now);
        }
    }

    void updateArmed(float accelMag, uint32_t now) {
        float threshold = _baselineAccelMag + FSM_LAUNCH_ACCEL_THRESHOLD;

        if (accelMag > threshold) {
            _launchConsecutive++;
            if (_launchConsecutive == 1) {
                fireEvent(EVENT_IGNITION, accelMag, now);
            }
            if (_launchConsecutive >= FSM_LAUNCH_SAMPLE_COUNT) {
                transitionTo(STATE_ASCENT, now);
            }
        } else {
            _launchConsecutive = 0;
        }
    }

    void updateAscent(float accelMag, float gpsSpeed, uint32_t now) {
        // Detect supersonic: GPS speed exceeds baro reliability threshold
        if (!_supersonicDetected && gpsSpeed >= FSM_SUPERSONIC_SPEED) {
            _supersonicDetected = true;
            fireEvent(EVENT_SUPERSONIC, gpsSpeed, now);
        }

        // Detect burnout: acceleration drops to near freefall
        if (!_burnoutDetected) {
            if (accelMag < FSM_BURNOUT_ACCEL_THRESHOLD) {
                _burnoutConsecutive++;
                if (_burnoutConsecutive >= FSM_BURNOUT_SAMPLE_COUNT) {
                    _burnoutDetected = true;
                    fireEvent(EVENT_BURNOUT, accelMag, now);
                }
            } else {
                _burnoutConsecutive = 0;
            }
        }

        // Detect apogee: Kalman speed goes negative (descending)
        if (_kf.speed < 0) {
            _apogeeConsecutive++;
            if (_apogeeConsecutive >= FSM_APOGEE_SAMPLE_COUNT) {
                transitionTo(STATE_APOGEE, now);
            }
        } else {
            _apogeeConsecutive = 0;
        }
    }

    void updateDescent(uint32_t now) {
        // Detect main deployment altitude (baro-only)
        if (!_mainDeployDetected && _kf.height <= FSM_MAIN_DEPLOY_AGL) {
            _mainDeployDetected = true;
            fireEvent(EVENT_MAIN_DEPLOY, _kf.height, now);
        }

        // Track min/max height over interval
        if (_kf.height < _landingMinHeight) _landingMinHeight = _kf.height;
        if (_kf.height > _landingMaxHeight) _landingMaxHeight = _kf.height;

        // Check landing: altitude stable over interval
        if (now >= _landingIntervalEnd) {
            float variation = _landingMaxHeight - _landingMinHeight;

            if (variation <= FSM_LANDING_HEIGHT_TOLERANCE) {
                fireEvent(EVENT_TOUCHDOWN, _kf.height, now);
                transitionTo(STATE_RECOVERY, now);
                return;
            }

            // Reset interval
            _landingIntervalEnd = now + FSM_LANDING_STABLE_MS;
            _landingMinHeight = _kf.height;
            _landingMaxHeight = _kf.height;
        }
    }
};

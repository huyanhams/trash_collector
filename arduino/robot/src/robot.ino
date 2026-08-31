#include <Arduino.h>
#include "Config.h"
#include "SerialProtocol.h"
#include "MotorControl.h"
#include "ServoControl.h"
#include "DistanceSensors.h"
#include "SafetyLogic.h"

static SerialProtocol  g_serial;
static MotorControl    g_motors;
static ServoControl    g_rudders;
static DistanceSensors g_sensors;
static SafetyLogic     g_safety;
static SerialCommand   g_lastCmd = {
};

static const unsigned long STATUS_EMIT_INTERVAL_MS = 500UL;
static unsigned long       g_lastStatusEmitMs      = 0;

static const uint16_t LED_BLINK_SLOW_ON_MS   = 250;
static const uint16_t LED_BLINK_SLOW_OFF_MS  = 750;
static const uint16_t LED_BLINK_DOUBLE_ON_MS = 100;
static const uint16_t LED_BLINK_DOUBLE_OFF_MS = 100;
static const uint16_t LED_BLINK_DOUBLE_GAP_MS = 800;
static const uint16_t LED_BLINK_FAST_ON_MS   = 100;
static const uint16_t LED_BLINK_FAST_OFF_MS  = 100;

static void setStatusLed(bool on) {
    digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
}

static void updateStatusLed(OverrideState state) {
    const unsigned long now = millis();
    static unsigned long s_phaseStartMs = 0;
    static uint8_t       s_doubleBlinkStep = 0; 

    auto restart = [&](uint16_t durationMs) {
        s_phaseStartMs = now;
    };
    auto elapsed = [&]() -> bool {
        return (now - s_phaseStartMs) >= phaseDuration(state, s_doubleBlinkStep);
    };

    switch (state) {
        case OverrideState::None:
            setStatusLed(false);
            s_doubleBlinkStep = 0;
            return;

        case OverrideState::Avoid:
            if (digitalRead(PIN_STATUS_LED) == HIGH) {
                if ((now - s_phaseStartMs) >= LED_BLINK_SLOW_ON_MS) {
                    setStatusLed(false);
                    s_phaseStartMs = now;
                }
            } else {
                if ((now - s_phaseStartMs) >= LED_BLINK_SLOW_OFF_MS) {
                    setStatusLed(true);
                    s_phaseStartMs = now;
                }
            }
            s_doubleBlinkStep = 0;
            return;

        case OverrideState::AvoidAmbiguousStop:
            switch (s_doubleBlinkStep) {
                case 0:  // first on
                    setStatusLed(true);
                    s_phaseStartMs = now;
                    s_doubleBlinkStep = 1;
                    return;
                case 1:  // first off
                    if ((now - s_phaseStartMs) >= LED_BLINK_DOUBLE_ON_MS) {
                        setStatusLed(false);
                        s_phaseStartMs = now;
                        s_doubleBlinkStep = 2;
                    }
                    return;
                case 2:  // second on
                    if ((now - s_phaseStartMs) >= LED_BLINK_DOUBLE_OFF_MS) {
                        setStatusLed(true);
                        s_phaseStartMs = now;
                        s_doubleBlinkStep = 3;
                    }
                    return;
                case 3:  // gap (off)
                default:
                    if ((now - s_phaseStartMs) >= LED_BLINK_DOUBLE_ON_MS) {
                        setStatusLed(false);
                        s_phaseStartMs = now;
                        s_doubleBlinkStep = 4;
                    } else {
                        return;
                    }
                    // fall through to step 4 below
                    [[fallthrough]];
                case 4:  // long off gap
                    if ((now - s_phaseStartMs) >= LED_BLINK_DOUBLE_GAP_MS) {
                        s_doubleBlinkStep = 0;
                    }
                    return;
            }

        case OverrideState::Failsafe:
            // Fast blink: 100 ms on, 100 ms off, repeat.
            if (digitalRead(PIN_STATUS_LED) == HIGH) {
                if ((now - s_phaseStartMs) >= LED_BLINK_FAST_ON_MS) {
                    setStatusLed(false);
                    s_phaseStartMs = now;
                }
            } else {
                if ((now - s_phaseStartMs) >= LED_BLINK_FAST_OFF_MS) {
                    setStatusLed(true);
                    s_phaseStartMs = now;
                }
            }
            s_doubleBlinkStep = 0;
            return;
    }
}

static uint16_t phaseDuration(OverrideState /*state*/, uint8_t step) {
    switch (step) {
        case 0:  return LED_BLINK_DOUBLE_ON_MS;   // first on
        case 1:  return LED_BLINK_DOUBLE_OFF_MS;  // first off
        case 2:  return LED_BLINK_DOUBLE_ON_MS;   // second on
        case 3:  return LED_BLINK_DOUBLE_OFF_MS;  // (unused but safe)
        case 4:  return LED_BLINK_DOUBLE_GAP_MS;  // long gap
        default: return LED_BLINK_DOUBLE_GAP_MS;
    }
}
void setup() {
    g_serial.begin();

    pinMode(PIN_STATUS_LED, OUTPUT);
    setStatusLed(true);

    g_rudders.begin();

    g_sensors.begin();

    g_motors.begin();

    delay(ESC_ARM_DELAY_MS);

    g_safety.attach(g_motors, g_rudders, g_sensors, g_serial);

    setStatusLed(false);
    g_lastStatusEmitMs = millis();
}


void loop() {
    g_serial.service(&g_lastCmd);

    g_sensors.service();

    g_safety.update(g_lastCmd);

    const unsigned long now = millis();
    if ((now - g_lastStatusEmitMs) >= STATUS_EMIT_INTERVAL_MS) {
        char statusBuf[32];
        const bool overrideActive =
            (g_safety.lastOverride() != OverrideState::None);
        const size_t n = SerialProtocol::formatStatus(
            statusBuf, sizeof(statusBuf),
            g_sensors.latestCm(SENSOR_INDEX_FRONT),
            g_sensors.latestCm(SENSOR_INDEX_LEFT),
            g_sensors.latestCm(SENSOR_INDEX_RIGHT),
            overrideActive);
        if (n > 0) {
            Serial.write(statusBuf, n);
            Serial.write(PROTOCOL_TERMINATOR);
        }
        g_lastStatusEmitMs = now;
    }

    updateStatusLed(g_safety.lastOverride());
}

// robot.ino
// ============================================================================
// Trash-collecting catamaran — Arduino UNO orchestration entry point.
//
// THIS FILE IS ORCHESTRATION ONLY. The actual behavior lives in the
// paired modules:
//   SerialProtocol  — Pi <-> Arduino command/status framing
//   MotorControl    — ESC pulse generation + arming
//   ServoControl    — Rudder servo pulse generation + clamping
//   DistanceSensors — Non-blocking round-robin ultrasonic reads
//   SafetyLogic     — FAILSAFE > AVOID > Pi-command priority chain
//
// setup() wires the modules together and runs the ESC arming sequence.
// loop() drives the per-iteration pipeline:
//
//   1. Drain any incoming CMD from the Pi (SerialProtocol::service).
//   2. Drive the ultrasonic round-robin (DistanceSensors::service).
//   3. Evaluate the priority chain and dispatch to actuators
//      (SafetyLogic::update).
//   4. Optionally emit a STATUS line at ~1-2 Hz for Pi-side logging.
//   5. Blink the status LED to reflect the current override state
//      (purely local observability; not on the wire).
//
// All constants come from Config.h. agent_rules.md §5 honored: no
// String, no malloc/new, no delay() in loop().
// ============================================================================

#include <Arduino.h>
#include "Config.h"
#include "SerialProtocol.h"
#include "MotorControl.h"
#include "ServoControl.h"
#include "DistanceSensors.h"
#include "SafetyLogic.h"

// ----------------------------------------------------------------------------
// Module instances (one of each, lifetime = program lifetime).
// ----------------------------------------------------------------------------

static SerialProtocol  g_serial;
static MotorControl    g_motors;
static ServoControl    g_rudders;
static DistanceSensors g_sensors;
static SafetyLogic     g_safety;

// Most-recent successfully-parsed CMD from the Pi. Reused across loops
// without modification; SerialProtocol updates it in place when a new
// valid line is accepted. Initialized to a safe "stop" state so the
// first few loops (before any CMD has arrived) are at least not
// dangerous.
static SerialCommand   g_lastCmd = {
    /* leftMotorPct   = */ 0,
    /* rightMotorPct  = */ 0,
    /* rudderAngleDeg = */ RUDDER_CENTER_DEG,
    /* hasValid       = */ false,
};

// ----------------------------------------------------------------------------
// Telemetry (STATUS) emit cadence
// ----------------------------------------------------------------------------

// We emit one STATUS line at this interval so the Pi can log/debug
// without flooding the serial link with the Pi's own CMD traffic. 500 ms
// = 2 Hz, well under the Pi's 10 Hz CMD rate and well within the
// 115200 baud link's capacity. Adjust here (or move to Config.h as a
// named constant) if you want different cadence.
static const unsigned long STATUS_EMIT_INTERVAL_MS = 500UL;
static unsigned long       g_lastStatusEmitMs      = 0;

// ----------------------------------------------------------------------------
// Status-LED blink patterns
// ----------------------------------------------------------------------------
//
// Purely local observability — these blink patterns are NOT transmitted
// to the Pi; STATUS only carries a binary overrideActive flag (see
// SerialProtocol::formatStatus and design_spec.md §4). The LED is
// useful for bench debugging when no serial monitor is attached.
//
//   OverrideState::None               -> LED off (all quiet)
//   OverrideState::Avoid              -> slow blink (250 ms on, 750 ms off)
//   OverrideState::AvoidAmbiguousStop -> double-blink (two 100 ms pulses,
//                                          800 ms gap, repeat)
//   OverrideState::Failsafe           -> fast blink (100 ms on, 100 ms off)
// ----------------------------------------------------------------------------

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

// Drives the LED based on the current override state. Must be called
// every loop (cheap — just a digitalWrite and a few comparisons).
// Tracks its own phase per state to avoid blocking delays.
static void updateStatusLed(OverrideState state) {
    const unsigned long now = millis();
    static unsigned long s_phaseStartMs = 0;
    static uint8_t       s_doubleBlinkStep = 0;  // 0 = first on, 1 = first off,
                                                 // 2 = second on, 3 = gap

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
            // Slow blink: 250 ms on, 750 ms off, repeat. We toggle the
            // LED on every phase transition; phaseDuration is decided
            // by the current LED state, so we just track that.
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
            // Double-blink: on 100, off 100, on 100, gap 800, repeat.
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

// Helper for the double-blink state machine: returns the duration (ms)
// of the current phase so the state machine above can decide when to
// advance. Pulled out as a separate function so the switch above stays
// readable.
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

// ----------------------------------------------------------------------------
// setup() — runs once at power-on / reset.
// ----------------------------------------------------------------------------

void setup() {
    // 1. Serial link.
    g_serial.begin();

    // 2. Status LED pin configured as an output BEFORE we try to
    //    light it during the ESC arming hold (step 5). This is the
    //    visual confirmation that arming is in progress.
    pinMode(PIN_STATUS_LED, OUTPUT);
    setStatusLed(true);

    // 3. Rudder servos next, so the rudders are at a known safe
    //    center pose during the entire ESC arming sequence.
    g_rudders.begin();

    // 4. Ultrasonic sensors (constructs the three NewPing objects on
    //    the heap; this is the only place dynamic allocation is
    //    allowed, per agent_rules.md §5).
    g_sensors.begin();

    // 5. Motor ESCs: attach servos and write the arming pulse to both.
    g_motors.begin();

    // 6. Hold minimum throttle for ESC_ARM_DELAY_MS so the ESCs arm.
    //    This is a setup-only delay; loop() is fully non-blocking.
    //    LED stays on throughout so a human can see we're arming.
    delay(ESC_ARM_DELAY_MS);

    // 7. Wire up SafetyLogic. From here on, the priority chain owns
    //    the actuators.
    g_safety.attach(g_motors, g_rudders, g_sensors, g_serial);

    // 8. Leave LED in the "not yet running" state — loop() will pick
    //    the appropriate pattern on the first iteration.
    setStatusLed(false);
    g_lastStatusEmitMs = millis();
}

// ----------------------------------------------------------------------------
// loop() — runs continuously after setup().
// ----------------------------------------------------------------------------

void loop() {
    // 1. Drain any incoming CMD from the Pi. Updates g_lastCmd in
    //    place; updates SerialProtocol's internal lastValidCmdMs
    //    timestamp on a successful parse.
    g_serial.service(&g_lastCmd);

    // 2. Drive the ultrasonic round-robin.
    g_sensors.service();

    // 3. Evaluate FAILSAFE > AVOID > Pi-command and dispatch to
    //    actuators. The first few loops, before any CMD has arrived,
    //    g_lastCmd.hasValid is false but SafetyLogic's FAILSAFE branch
    //    will catch that case via the millis-since-lastCmd check
    //    (lastValidCmdMs() == 0 is treated as "no command ever").
    g_safety.update(g_lastCmd);

    // 4. Emit STATUS at ~2 Hz. Clamps distances on the way out via
    //    SerialProtocol::formatStatus (so a "no echo" 0 from
    //    DistanceSensors becomes SENSOR_MAX_DISTANCE_CM on the wire).
    const unsigned long now = millis();
    if ((now - g_lastStatusEmitMs) >= STATUS_EMIT_INTERVAL_MS) {
        char statusBuf[32];  // generous: "STATUS,200,200,200,1\n" = 21 chars
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

    // 5. Status LED reflects the current override state for bench
    //    debugging. Purely local — not on the wire.
    updateStatusLed(g_safety.lastOverride());
}

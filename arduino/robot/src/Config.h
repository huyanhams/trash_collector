// Config.h
// ============================================================================
// Pin assignments and all Arduino-side constants for the trash-collecting
// catamaran robot.
//
// EVERY value in this file is either:
//   (a) FIXED — a hardware or protocol fact from docs/technical_spec.md.
//       Do not change without updating the spec AND the physical wiring.
//   (b) TUNABLE DEFAULT — a starting point expected to be adjusted after
//       real hardware testing. Implemented as a named constant here so the
//       logic code can reference it without hardcoding literals.
//
// Source of truth ordering (per README.md): if anything here conflicts with
// another document, docs/technical_spec.md wins.
// ============================================================================

#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <Arduino.h>

// ----------------------------------------------------------------------------
// 1. Pin assignments (FIXED — docs/technical_spec.md §1)
//
// D0 and D1 are reserved for the USB-to-serial link to the Pi and MUST NOT
// be used for any other purpose. D13 and A0–A5 are reserved for future
// expansion.
// ----------------------------------------------------------------------------

// --- Reserved (do not assign to anything) ---
static const uint8_t PIN_RESERVED_RX        = 0;   // Reserved — USB serial RX
static const uint8_t PIN_RESERVED_TX        = 1;   // Reserved — USB serial TX

// --- Ultrasonic sensors (JSN-SR04T, 5-pin mode: separate TRIG and ECHO) ---
static const uint8_t PIN_ULTRA_FRONT_TRIG   = 2;
static const uint8_t PIN_ULTRA_FRONT_ECHO   = 3;
static const uint8_t PIN_ULTRA_LEFT_TRIG    = 4;
static const uint8_t PIN_ULTRA_LEFT_ECHO    = 5;
static const uint8_t PIN_ULTRA_RIGHT_TRIG   = 6;
static const uint8_t PIN_ULTRA_RIGHT_ECHO   = 7;

// --- Actuators ---
static const uint8_t PIN_ESC_LEFT           = 8;   // A2212 BLDC ESC signal (via Servo lib)
static const uint8_t PIN_ESC_RIGHT          = 9;   // A2212 BLDC ESC signal (via Servo lib)
static const uint8_t PIN_SERVO_RUDDER_LEFT  = 10;  // MG996R rudder servo (left hull)
static const uint8_t PIN_SERVO_RUDDER_RIGHT = 11;  // MG996R rudder servo (right hull)

// --- Status / spare ---
static const uint8_t PIN_STATUS_LED         = 12;  // Optional: blink to indicate FAILSAFE/AVOID
static const uint8_t PIN_RESERVED_LED_13    = 13;  // Onboard LED — reserved, unused by default

// ----------------------------------------------------------------------------
// 2. Communication protocol constants (FIXED — docs/technical_spec.md §2)
// ----------------------------------------------------------------------------

static const uint32_t SERIAL_BAUD_RATE       = 115200;  // Must match Pi side
static const uint16_t FAILSAFE_TIMEOUT_MS   = 1000;    // No valid CMD within this window → fail-safe

// Rudder angle clamps (degrees). Both rudders move together; clamp guards
// against bad serial input over-driving the servos.
static const uint8_t  RUDDER_MIN_DEG         = 60;
static const uint8_t  RUDDER_MAX_DEG         = 120;
static const uint8_t  RUDDER_CENTER_DEG      = 90;     // Mechanically zeroed so 90° = straight

// ============================================================================
// HARDWARE BENCH-TEST ITEM (do not silently flip — verify, then flip):
//
//   ASSUMPTION: RUDDER_MIN_DEG turns the robot's nose LEFT, and
//               RUDDER_MAX_DEG turns it RIGHT.
//
//   NOT YET VERIFIED ON HARDWARE.
//
//   Once the Arduino is wired to real servos, this must be physically
//   confirmed (and the values swapped, or the linkage mechanically
//   reversed, if wrong). The chosen convention also has to be
//   consistent with the Pi-side APPROACH steering sign convention
//   (positive bbox offset from frame center = positive rudder
//   deflection, per the open question recorded in the project's
//   2026-07-10 design-review notes).
//
//   Both decisions are intentionally deferred until hardware is on
//   the bench. Do not pick a side in code without updating the
//   mechanical zero point first.
// ============================================================================

// Motor throttle range sent over serial (percent). 0 = stopped, 100 = full.
static const uint8_t  MOTOR_MIN_PCT         = 0;
static const uint8_t  MOTOR_MAX_PCT         = 100;

// Serial protocol framing
static const char     CMD_PREFIX            = 'C';    // "CMD,..."  (Pi -> Arduino)
static const char     STATUS_PREFIX         = 'S';    // "STATUS,..." (Arduino -> Pi, optional telemetry)
static const char     PROTOCOL_DELIM        = ',';
static const char     PROTOCOL_TERMINATOR   = '\n';

// CMD line buffer size: "CMD," + 3 fields (max "100") + commas + '\n'
// "CMD,100,100,120\n" = 16 chars. Use 32 for headroom.
static const uint8_t  CMD_LINE_MAX_LEN      = 32;

// ----------------------------------------------------------------------------
// 3. ESC behavior (FIXED — docs/technical_spec.md §3)
//
// Standard hobby ESCs require a minimum-throttle hold at startup to arm
// before they will respond to higher throttle commands. robot.ino's setup()
// MUST execute this arming sequence before entering loop().
// ----------------------------------------------------------------------------

static const uint16_t ESC_ARM_PULSE_US      = 1000;   // 0% throttle pulse width
static const uint16_t ESC_MAX_PULSE_US      = 2000;   // 100% throttle pulse width
static const uint16_t ESC_ARM_DELAY_MS      = 2000;   // Hold min throttle this long to arm

// Servo pulse endpoints. Many servos (including MG996R) accept 1000–2000 µs;
// we use the same endpoints as the ESCs for simplicity and consistency.
static const uint16_t SERVO_MIN_PULSE_US    = 1000;
static const uint16_t SERVO_MAX_PULSE_US    = 2000;

// ----------------------------------------------------------------------------
// 4. Distance sensor timing (TUNABLE DEFAULT — docs/technical_spec.md §4)
// ----------------------------------------------------------------------------

// Minimum gap between pings on the SAME sensor, to avoid echo interference.
static const uint16_t SENSOR_READ_INTERVAL_MS = 60;

// Approximate time to read all 3 sensors once when pinging sequentially.
// Not used as a hard deadline — just for cycle accounting.
static const uint16_t SENSOR_CYCLE_MS         = 180;

// NewPing library max distance (cm). Used to bound pulseIn and keep loop()
// non-blocking. 200 cm is well past the AVOID threshold and within sensor spec.
static const uint16_t SENSOR_MAX_DISTANCE_CM  = 200;

// Below this, AVOID triggers on that sensor (FIXED threshold value, see §5).
static const uint8_t  AVOID_DISTANCE_THRESHOLD_CM = 40;

// ----------------------------------------------------------------------------
// 5. Arduino AVOID behavior (TUNABLE DEFAULT — docs/technical_spec.md §5)
// ----------------------------------------------------------------------------

// Speed of the "active" motor during an avoidance pivot. The other motor is 0.
static const uint8_t  AVOID_PIVOT_SPEED_PCT      = 30;

// During AVOID, the rudder is driven to its full clamp value toward the
// clearer side. RUDDER_MIN_DEG and RUDDER_MAX_DEG are used directly for that
// (left clear → RUDDER_MIN_DEG, right clear → RUDDER_MAX_DEG).

#endif  // ROBOT_CONFIG_H

// MotorControl.h
//
// Drives the two brushless ESCs (one per hull). Translates a 0..100
// throttle percentage into the correct ESC pulse width and writes it
// via the Servo library (ESCs accept the same PWM protocol as servos).
//
// One-direction only — see docs/design_spec.md §2 and §7. The motors on
// this robot physically cannot run in reverse, so this module never
// accepts a negative or "reverse" command. Negative inputs are clamped
// to MOTOR_MIN_PCT; values > MOTOR_MAX_PCT are clamped to MOTOR_MAX_PCT.
//
// All numeric constants come from Config.h. Do not hardcode here.

#ifndef ROBOT_MOTOR_CONTROL_H
#define ROBOT_MOTOR_CONTROL_H

#include <Arduino.h>
#include <Servo.h>
#include "Config.h"

class MotorControl {
public:
    MotorControl();

    // Attach the two ESC signal pins and write the arming pulse to both.
    // MUST be called from setup() and the caller MUST then wait
    // ESC_ARM_DELAY_MS before issuing any non-zero throttle — that wait
    // is implemented in robot.ino (where it can also keep the status LED
    // blinking for visual feedback) so this method stays synchronous and
    // testable.
    void begin();

    // Set both motor speeds as a percentage of full throttle.
    //   leftPct, rightPct: 0..100 (will be clamped to that range)
    // A value of 0 means "minimum throttle" / "stopped" for the ESC.
    void setBoth(uint8_t leftPct, uint8_t rightPct);

    // Set the two motors independently. Same clamping rules as setBoth.
    void setLeft(uint8_t pct);
    void setRight(uint8_t pct);

    // Force both motors to the minimum pulse (effectively 0% throttle).
    // Called by SafetyLogic on FAILSAFE / AVOID-stop.
    void stopBoth();

    // Returns the most recently commanded throttle percentage for each
    // motor (after clamping). Useful for STATUS reporting or logging.
    uint8_t lastLeftPct()  const { return _lastLeftPct;  }
    uint8_t lastRightPct() const { return _lastRightPct; }

private:
    Servo _leftEsc;
    Servo _rightEsc;
    bool  _attached;

    // Last commanded percent, post-clamp, for each motor.
    uint8_t _lastLeftPct;
    uint8_t _lastRightPct;

    // Internal helper: clamp a percent to [MOTOR_MIN_PCT, MOTOR_MAX_PCT]
    // and convert it to the corresponding ESC pulse width in microseconds.
    static uint16_t _pctToPulseUs(uint8_t pct);
};

#endif // ROBOT_MOTOR_CONTROL_H

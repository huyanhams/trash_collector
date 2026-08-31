// ServoControl.h
//
// Drives the two rudder servos (one per hull). Both servos move together —
// the design calls for a single rudder signal controlling both hulls, like
// the front wheels of a car. See docs/design_spec.md §2 and §4.
//
// Translates a requested rudder angle in degrees into the correct pulse
// width, clamps to [RUDDER_MIN_DEG, RUDDER_MAX_DEG], and writes both
// servos via the Servo library (using writeMicroseconds so the pulse
// endpoints are exactly SERVO_MIN_PULSE_US / SERVO_MAX_PULSE_US).
//
// All numeric constants come from Config.h. Do not hardcode here.

#ifndef ROBOT_SERVO_CONTROL_H
#define ROBOT_SERVO_CONTROL_H

#include <Arduino.h>
#include <Servo.h>
#include "Config.h"

class ServoControl {
public:
    ServoControl();

    // Attach both rudder servos to their pins and center them
    // (RUDDER_CENTER_DEG). Call once from setup(), BEFORE the arming
    // sequence in MotorControl, so the rudders are at a known safe
    // angle during ESC arming.
    void begin();

    // Set both rudder servos to the same angle. `angleDeg` is clamped
    // to [RUDDER_MIN_DEG, RUDDER_MAX_DEG] so a bad serial value can
    // never over-deflect the rudders.
    void setBoth(uint8_t angleDeg);

    // Convenience: re-center the rudders (RUDDER_CENTER_DEG).
    // Used by SafetyLogic on FAILSAFE and by the COLLECT / "going
    // straight" paths.
    void center();

    // Returns the most recently commanded angle, post-clamp. Useful for
    // STATUS reporting.
    uint8_t lastAngleDeg() const { return _lastAngleDeg; }

private:
    Servo    _leftRudder;
    Servo    _rightRudder;
    bool     _attached;
    uint8_t  _lastAngleDeg;

    // Clamp the requested angle to the safe range, then convert to
    // microseconds for the Servo library. Returns the pulse width.
    static uint16_t _angleToPulseUs(uint8_t angleDeg);
};

#endif // ROBOT_SERVO_CONTROL_H

// ServoControl.cpp
//
// Drives the two MG996R rudder servos. See ServoControl.h for the contract.
//
// Key design choices (per docs/design_spec.md §2, §4, §7 and agent_rules.md §5):
// - Both rudders move together. There is no API to set them independently
//   — the design treats this as a single-rudder boat, not vectored thrust.
// - We use writeMicroseconds() (not Servo.write(angle)) so the pulse
//   endpoints are exactly SERVO_MIN_PULSE_US / SERVO_MAX_PULSE_US. The
//   angle-based write() silently remaps to the Servo library's default
//   pulse range, which we don't want.
// - begin() centers the rudders (RUDDER_CENTER_DEG) immediately. This
//   gives the robot a known safe pose during the ESC arming sequence
//   that MotorControl::begin() runs in setup().
// - All inputs are clamped to [RUDDER_MIN_DEG, RUDDER_MAX_DEG] so a bad
//   serial value can never over-deflect the servos.
// - No String class, no malloc, no new. State is plain old data.

#include "ServoControl.h"

ServoControl::ServoControl()
    : _attached(false),
      _lastAngleDeg(RUDDER_CENTER_DEG) {
}

void ServoControl::begin() {
    _leftRudder.attach(PIN_SERVO_RUDDER_LEFT);
    _rightRudder.attach(PIN_SERVO_RUDDER_RIGHT);

    // Center both rudders immediately. The Servo library's
    // writeMicroseconds call is non-blocking and cheap, so this can run
    // before MotorControl::begin() and the ESC arming delay.
    const uint16_t centerPulse = _angleToPulseUs(RUDDER_CENTER_DEG);
    _leftRudder.writeMicroseconds(centerPulse);
    _rightRudder.writeMicroseconds(centerPulse);

    _lastAngleDeg = RUDDER_CENTER_DEG;
    _attached = true;
}

void ServoControl::setBoth(uint8_t angleDeg) {
    // Clamp the requested angle to the safe range. RUDDER_MIN_DEG and
    // RUDDER_MAX_DEG are both uint8_t, so we can do the comparison
    // directly without sign-extension surprises.
    if (angleDeg < RUDDER_MIN_DEG) angleDeg = RUDDER_MIN_DEG;
    if (angleDeg > RUDDER_MAX_DEG) angleDeg = RUDDER_MAX_DEG;

    if (_attached) {
        const uint16_t pulse = _angleToPulseUs(angleDeg);
        _leftRudder.writeMicroseconds(pulse);
        _rightRudder.writeMicroseconds(pulse);
    }
    _lastAngleDeg = angleDeg;
}

void ServoControl::center() {
    setBoth(RUDDER_CENTER_DEG);
}

uint16_t ServoControl::_angleToPulseUs(uint8_t angleDeg) {
    // Linear map: RUDDER_MIN_DEG -> SERVO_MIN_PULSE_US
    //            RUDDER_CENTER_DEG -> midpoint pulse
    //            RUDDER_MAX_DEG -> SERVO_MAX_PULSE_US
    //
    // For the design's symmetric clamp (60..120) around a center of 90,
    // the midpoint pulse is exactly (SERVO_MIN_PULSE_US +
    // SERVO_MAX_PULSE_US) / 2. We don't special-case the center, though
    // — the linear map handles it correctly and the formula stays
    // robust if the clamp is ever changed to be asymmetric.
    //
    // Compute in uint32_t to avoid 8-bit truncation artifacts at the
    // endpoints, same defensive style as MotorControl's mapping.

    const uint32_t minAngle = RUDDER_MIN_DEG;
    const uint32_t maxAngle = RUDDER_MAX_DEG;
    const uint32_t rangeDeg = maxAngle - minAngle;  // degrees over which the pulse ramps
    const uint32_t minPulse = SERVO_MIN_PULSE_US;
    const uint32_t maxPulse = SERVO_MAX_PULSE_US;
    const uint32_t rangeUs  = maxPulse - minPulse;  // µs over rangeDeg

    // (angleDeg - minAngle) * rangeUs * 10 / (rangeDeg * 10)
    //   == (angleDeg - minAngle) * rangeUs / rangeDeg
    // Multiplying by 10 first then dividing by 10*rangeDeg keeps
    // precision across the integer divide. We use a fixed scale of
    // 1000 instead of 10 because the angle range (60) and pulse range
    // (1000) don't share a clean factor of 10, and 1000 gives sub-µs
    // precision across the whole map.
    const uint32_t scaled  = static_cast<uint32_t>(angleDeg - minAngle) * rangeUs * 1000UL;
    const uint32_t offset  = scaled / (rangeDeg * 1000UL);
    return static_cast<uint16_t>(minPulse + offset);
}

#include "ServoControl.h"

ServoControl::ServoControl()
    : _attached(false),
      _lastAngleDeg(RUDDER_CENTER_DEG) {
}

void ServoControl::begin() {
    _leftRudder.attach(PIN_SERVO_RUDDER_LEFT);
    _rightRudder.attach(PIN_SERVO_RUDDER_RIGHT);
    const uint16_t centerPulse = _angleToPulseUs(RUDDER_CENTER_DEG);
    _leftRudder.writeMicroseconds(centerPulse);
    _rightRudder.writeMicroseconds(centerPulse);

    _lastAngleDeg = RUDDER_CENTER_DEG;
    _attached = true;
}

void ServoControl::setBoth(uint8_t angleDeg) {
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
    const uint32_t minAngle = RUDDER_MIN_DEG;
    const uint32_t maxAngle = RUDDER_MAX_DEG;
    const uint32_t rangeDeg = maxAngle - minAngle;
    const uint32_t minPulse = SERVO_MIN_PULSE_US;
    const uint32_t maxPulse = SERVO_MAX_PULSE_US;
    const uint32_t rangeUs  = maxPulse - minPulse;
    
    const uint32_t scaled  = static_cast<uint32_t>(angleDeg - minAngle) * rangeUs * 1000UL;
    const uint32_t offset  = scaled / (rangeDeg * 1000UL);
    return static_cast<uint16_t>(minPulse + offset);
}

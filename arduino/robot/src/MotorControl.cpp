#include "MotorControl.h"

MotorControl::MotorControl()
    : _attached(false),
      _lastLeftPct(MOTOR_MIN_PCT),
      _lastRightPct(MOTOR_MIN_PCT) {
}

void MotorControl::begin() {
    _leftEsc.attach(PIN_ESC_LEFT);
    _rightEsc.attach(PIN_ESC_RIGHT);

    const uint16_t armPulse = ESC_ARM_PULSE_US;
    _leftEsc.writeMicroseconds(armPulse);
    _rightEsc.writeMicroseconds(armPulse);

    _lastLeftPct  = MOTOR_MIN_PCT;
    _lastRightPct = MOTOR_MIN_PCT;
    _attached = true;
}

void MotorControl::setBoth(uint8_t leftPct, uint8_t rightPct) {
    setLeft(leftPct);
    setRight(rightPct);
}

void MotorControl::setLeft(uint8_t pct) {
    if (pct < MOTOR_MIN_PCT) pct = MOTOR_MIN_PCT;
    if (pct > MOTOR_MAX_PCT) pct = MOTOR_MAX_PCT;

    if (_attached) {
        _leftEsc.writeMicroseconds(_pctToPulseUs(pct));
    }
    _lastLeftPct = pct;
}

void MotorControl::setRight(uint8_t pct) {
    if (pct < MOTOR_MIN_PCT) pct = MOTOR_MIN_PCT;
    if (pct > MOTOR_MAX_PCT) pct = MOTOR_MAX_PCT;

    if (_attached) {
        _rightEsc.writeMicroseconds(_pctToPulseUs(pct));
    }
    _lastRightPct = pct;
}

void MotorControl::stopBoth() {
    if (_attached) {
        _leftEsc.writeMicroseconds(ESC_ARM_PULSE_US);
        _rightEsc.writeMicroseconds(ESC_ARM_PULSE_US);
    }
    _lastLeftPct  = MOTOR_MIN_PCT;
    _lastRightPct = MOTOR_MIN_PCT;
}

uint16_t MotorControl::_pctToPulseUs(uint8_t pct) {
    const uint32_t armPulse = ESC_ARM_PULSE_US;
    const uint32_t maxPulse = ESC_MAX_PULSE_US;
    const uint32_t range    = maxPulse - armPulse;  // µs over the 0..100 range

    uint32_t offset = (static_cast<uint32_t>(pct) * range * 10UL) / 1000UL;
    return static_cast<uint16_t>(armPulse + offset);
}

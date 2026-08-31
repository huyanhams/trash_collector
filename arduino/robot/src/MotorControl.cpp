// MotorControl.cpp
//
// Drives the two brushless ESCs (one per hull) via the Servo library.
// See MotorControl.h for the contract.
//
// Key design choices (per docs/design_spec.md §7 and agent_rules.md §5):
// - Motors are ONE-DIRECTION ONLY. This module never produces a pulse
//   below ESC_ARM_PULSE_US (1000 µs). A "0% throttle" command is the
//   arming minimum, not "reverse." There is intentionally no API to
//   command reverse — out of scope per agent_rules.md.
// - begin() performs the arming sequence: attaches the Servo objects
//   and writes the arming pulse to both ESCs. The caller is responsible
//   for the 2-second hold at minimum throttle (ESC_ARM_DELAY_MS), since
//   a blocking delay() in a library is a bad idea on a 2KB-SRAM chip
//   and robot.ino is the right place to coordinate setup timing.
// - All inputs are clamped; we never write garbage to the ESC. Clamping
//   uses the FIXED MOTOR_MIN_PCT / MOTOR_MAX_PCT and ESC_ARM_PULSE_US /
//   ESC_MAX_PULSE_US from Config.h — no magic numbers in this file.
// - No String class, no malloc, no new. State is plain old data.

#include "MotorControl.h"

MotorControl::MotorControl()
    : _attached(false),
      _lastLeftPct(MOTOR_MIN_PCT),
      _lastRightPct(MOTOR_MIN_PCT) {
}

void MotorControl::begin() {
    _leftEsc.attach(PIN_ESC_LEFT);
    _rightEsc.attach(PIN_ESC_RIGHT);

    // Arming sequence: drive both ESCs to the minimum pulse. The
    // caller (robot.ino) must hold this state for ESC_ARM_DELAY_MS
    // before allowing any non-zero throttle to be commanded. We do NOT
    // delay() here — setup() in robot.ino handles the wait.
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
    // Stop = minimum pulse (the ESC arming minimum). This is the same
    // pulse as "0% throttle" and is what the FAILSAFE / AVOID-stop
    // path uses. Crucially, we do NOT attempt to command reverse —
    // the motors physically cannot do it.
    if (_attached) {
        _leftEsc.writeMicroseconds(ESC_ARM_PULSE_US);
        _rightEsc.writeMicroseconds(ESC_ARM_PULSE_US);
    }
    _lastLeftPct  = MOTOR_MIN_PCT;
    _lastRightPct = MOTOR_MIN_PCT;
}

uint16_t MotorControl::_pctToPulseUs(uint8_t pct) {
    // Linear map: 0%  -> ESC_ARM_PULSE_US (1000 µs)
    //             100% -> ESC_MAX_PULSE_US   (2000 µs)
    //
    // To preserve integer precision across the full 0..100 range, we
    // compute the range in tenths of a percent and divide at the end.
    // This avoids the classic 8-bit truncation bug where (pct * range)
    // overflows for any non-trivial range. (For our actual 1000-µs
    // range it doesn't strictly overflow in uint16_t, but doing it
    // correctly is cheap and keeps us safe if the constants change.)
    const uint32_t armPulse = ESC_ARM_PULSE_US;
    const uint32_t maxPulse = ESC_MAX_PULSE_US;
    const uint32_t range    = maxPulse - armPulse;  // µs over the 0..100 range

    // (pct * range * 10) / 1000  ==  pct * range / 100, computed in a
    // larger type to keep precision.
    uint32_t offset = (static_cast<uint32_t>(pct) * range * 10UL) / 1000UL;
    return static_cast<uint16_t>(armPulse + offset);
}

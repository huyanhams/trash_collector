#include "SafetyLogic.h"
#include "MotorControl.h"
#include "ServoControl.h"

SafetyLogic::SafetyLogic()
    : _motors(nullptr),
      _rudders(nullptr),
      _sensors(nullptr),
      _serial(nullptr),
      _lastOverride(OverrideState::Failsafe) {
}

void SafetyLogic::attach(MotorControl& motors,
                         ServoControl& rudders,
                         DistanceSensors& sensors,
                         SerialProtocol& serial) {
    _motors  = &motors;
    _rudders = &rudders;
    _sensors = &sensors;
    _serial  = &serial;
    _lastOverride = OverrideState::Failsafe;
}

OverrideState SafetyLogic::update(const SerialCommand& lastCmd) {
    if (_motors == nullptr || _rudders == nullptr ||
        _sensors == nullptr || _serial == nullptr) {
        _lastOverride = OverrideState::Failsafe;
        return _lastOverride;
    }

    const unsigned long lastCmdMs = _serial->lastValidCmdMs();
    const unsigned long now       = millis();
    const bool failsafeActive = (lastCmdMs == 0) ||
                                ((now - lastCmdMs) >=
                                 static_cast<unsigned long>(FAILSAFE_TIMEOUT_MS));
    if (failsafeActive) {
        _applyFailsafe();
        _lastOverride = OverrideState::Failsafe;
        return _lastOverride;
    }
    const bool frontBlocked = _sensors->isObstacleWithin(SENSOR_INDEX_FRONT);
    const bool leftBlocked  = _sensors->isObstacleWithin(SENSOR_INDEX_LEFT);
    const bool rightBlocked = _sensors->isObstacleWithin(SENSOR_INDEX_RIGHT);

    if (frontBlocked || leftBlocked || rightBlocked) {
        _applyAvoid(frontBlocked, leftBlocked, rightBlocked);
        if (_lastOverride != OverrideState::AvoidAmbiguousStop) {
            _lastOverride = OverrideState::Avoid;
        }
        return _lastOverride;
    }

    _applyPassThrough(lastCmd);
    _lastOverride = OverrideState::None;
    return _lastOverride;
}

void SafetyLogic::_applyFailsafe() {
    if (_motors  != nullptr) _motors->stopBoth();
    if (_rudders != nullptr) _rudders->center();
}

void SafetyLogic::_applyAvoid(bool frontBlocked, bool leftBlocked, bool rightBlocked) {
    if (leftBlocked && rightBlocked) {
        _applyFailsafe();
        _lastOverride = OverrideState::AvoidAmbiguousStop;
        return;
    }
    if (frontBlocked && (leftBlocked || rightBlocked)) {
        _applyFailsafe();
        _lastOverride = OverrideState::AvoidAmbiguousStop;
        return;
    }
    if (frontBlocked) {
        _applyFailsafe();
        _lastOverride = OverrideState::AvoidAmbiguousStop;
        return;
    }

    if (leftBlocked) {
        _motors->setLeft(MOTOR_MIN_PCT);
        _motors->setRight(AVOID_PIVOT_SPEED_PCT);
        _rudders->setBoth(RUDDER_MAX_DEG);
        return;
    }
    _motors->setLeft(AVOID_PIVOT_SPEED_PCT);
    _motors->setRight(MOTOR_MIN_PCT);
    _rudders->setBoth(RUDDER_MIN_DEG);
}

void SafetyLogic::_applyPassThrough(const SerialCommand& cmd) {
    _motors->setBoth(cmd.leftMotorPct, cmd.rightMotorPct);
    _rudders->setBoth(cmd.rudderAngleDeg);
}

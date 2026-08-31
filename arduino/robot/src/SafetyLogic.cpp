// SafetyLogic.cpp
//
// Implements the FAILSAFE > AVOID > PASS-THROUGH priority chain for
// the Arduino side. See SafetyLogic.h for the contract and the spec
// references.
//
// Carry-forward rule (per user instruction): we do NOT special-case 0
// readings here. The "no echo == clear" semantics live in
// DistanceSensors (isObstacleWithin / indexOfClosestObstacle), and we
// trust those helpers. Calling isObstacleWithin() on a sensor that
// returned 0 returns false, exactly like a sensor that simply hasn't
// pinged yet — both cases mean "no obstacle known," which is the
// correct conservative default (don't AVOID on a phantom obstacle).
//
// agent_rules.md §5 honored: no String, no malloc, no new, no delay().
// update() is non-blocking and safe to call on every loop().

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
    // Defensive: if attach() wasn't called, fail safe (motors already
    // un-driven, but the orchestrator would have to handle that
    // situation anyway; we just don't crash).
    if (_motors == nullptr || _rudders == nullptr ||
        _sensors == nullptr || _serial == nullptr) {
        _lastOverride = OverrideState::Failsafe;
        return _lastOverride;
    }

    // -------------------------------------------------------------------
    // Priority 1: FAILSAFE
    //
    // If no valid CMD has arrived within FAILSAFE_TIMEOUT_MS, the Pi
    // is considered lost. Stop motors, center rudder, and hold until
    // valid commands resume. Per the design, this is independent of
    // sensor state — even if an obstacle is detected, we don't try to
    // AVOID during failsafe, because the Pi is presumed unhealthy and
    // any complex behavior is more dangerous than a clean stop.
    // -------------------------------------------------------------------
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

    // -------------------------------------------------------------------
    // Priority 2: AVOID
    //
    // Sample the three sensors via the isObstacleWithin() helper, which
    // already handles the "0 == clear" carry-forward rule. We do NOT
    // special-case 0 here — we trust DistanceSensors.
    // -------------------------------------------------------------------
    const bool frontBlocked = _sensors->isObstacleWithin(SENSOR_INDEX_FRONT);
    const bool leftBlocked  = _sensors->isObstacleWithin(SENSOR_INDEX_LEFT);
    const bool rightBlocked = _sensors->isObstacleWithin(SENSOR_INDEX_RIGHT);

    if (frontBlocked || leftBlocked || rightBlocked) {
        _applyAvoid(frontBlocked, leftBlocked, rightBlocked);
        // _applyAvoid may have set the actuators identically to a
        // failsafe stop (ambiguous case); report that distinction
        // locally via _lastOverride so the status LED can reflect it,
        // but the actuation state is the same.
        if (_lastOverride != OverrideState::AvoidAmbiguousStop) {
            _lastOverride = OverrideState::Avoid;
        }
        return _lastOverride;
    }

    // -------------------------------------------------------------------
    // Priority 3: PASS-THROUGH
    //
    // No failsafe, no obstacle. Apply the Pi's last good command. The
    // command's own field values are already clamped by SerialProtocol
    // (motor percent to [0,100], rudder to [60..120]) and by
    // MotorControl / ServoControl on the way out, so we don't need to
    // re-clamp here.
    // -------------------------------------------------------------------
    _applyPassThrough(lastCmd);
    _lastOverride = OverrideState::None;
    return _lastOverride;
}

void SafetyLogic::_applyFailsafe() {
    if (_motors  != nullptr) _motors->stopBoth();
    if (_rudders != nullptr) _rudders->center();
}

void SafetyLogic::_applyAvoid(bool frontBlocked, bool leftBlocked, bool rightBlocked) {
    // We have three sensors and three possible pivot directions. With
    // the design's no-reverse constraint, "stop both and wait" is the
    // only safe response when we don't have a clearly-clear side to
    // pivot toward. The four cases:
    //
    //   front only            : stop both, center rudder. (We can't
    //                            safely turn without moving forward;
    //                            and forward is the blocked direction.)
    //   left only             : pivot RIGHT (right motor = pivot
    //                            speed, left motor = 0, rudder to
    //                            RUDDER_MAX_DEG).
    //   right only            : pivot LEFT (mirror of above).
    //   front+left, front+right, all three, etc. — ambiguous: stop
    //                            both and center.
    //
    // We don't try to use the magnitude of the readings to bias the
    // decision; if any side is "blocked," we treat that side as
    // unusable. That's intentionally coarse — the spec is explicit
    // that obstacle avoidance is a coarse safety net, not a path
    // planner.

    if (leftBlocked && rightBlocked) {
        // Both sides blocked → no safe pivot direction. Stop and hold.
        // (Front may or may not also be blocked, doesn't matter.)
        _applyFailsafe();
        _lastOverride = OverrideState::AvoidAmbiguousStop;
        return;
    }
    if (frontBlocked && (leftBlocked || rightBlocked)) {
        // Front blocked AND at least one side blocked → ambiguous
        // (might be a corner; can't safely pivot without knowing which
        // side is "more" open). Stop and hold.
        _applyFailsafe();
        _lastOverride = OverrideState::AvoidAmbiguousStop;
        return;
    }
    if (frontBlocked) {
        // Only front blocked → stop both, center. The spec calls this
        // out implicitly: with one-direction-only motors and a blocked
        // front, the only safe action is to stop until the obstacle
        // moves.
        _applyFailsafe();
        _lastOverride = OverrideState::AvoidAmbiguousStop;
        return;
    }

    // At this point, front is clear and exactly one of {left, right}
    // is blocked. Pivot away from the blocked side.
    if (leftBlocked) {
        // Pivot to the right: right motor drives at pivot speed, left
        // motor at 0, rudder deflected to its maximum right value
        // (RUDDER_MAX_DEG) to assist the pivot.
        _motors->setLeft(MOTOR_MIN_PCT);
        _motors->setRight(AVOID_PIVOT_SPEED_PCT);
        _rudders->setBoth(RUDDER_MAX_DEG);
        return;
    }
    // rightBlocked (front is clear, left is clear).
    _motors->setLeft(AVOID_PIVOT_SPEED_PCT);
    _motors->setRight(MOTOR_MIN_PCT);
    _rudders->setBoth(RUDDER_MIN_DEG);
}

void SafetyLogic::_applyPassThrough(const SerialCommand& cmd) {
    // SerialProtocol already guarantees the command is fully validated
    // (all three fields in range). We apply it directly.
    _motors->setBoth(cmd.leftMotorPct, cmd.rightMotorPct);
    _rudders->setBoth(cmd.rudderAngleDeg);
}

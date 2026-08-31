// SafetyLogic.h
//
// The single owner of the Arduino-side control priority chain:
//
//   1. FAILSAFE   — no valid CMD from the Pi within FAILSAFE_TIMEOUT_MS
//   2. AVOID      — any ultrasonic reads below AVOID_DISTANCE_THRESHOLD_CM
//   3. PASS-THROUGH — apply the last good CMD from the Pi as-is
//
// See docs/design_spec.md §5 for the priority semantics and §6 for the
// state descriptions.
//
// Update() is the only entry point. It evaluates the chain in strict
// order, decides what the motors and rudder should do this loop, and
// dispatches the result to MotorControl and ServoControl. It returns
// an OverrideState enum that the orchestrator can use for STATUS
// reporting.
//
// Carry-forward rule (per user instruction): a raw 0 reading from any
// ultrasonic sensor is treated as "no obstacle in range" (clear), not
// as "0 cm to obstacle." This is enforced inside DistanceSensors
// (isObstacleWithin / indexOfClosestObstacle) — SafetyLogic simply
// calls those helpers and never has to special-case 0 itself.
//
// All numeric constants come from Config.h. Do not hardcode here.

#ifndef ROBOT_SAFETY_LOGIC_H
#define ROBOT_SAFETY_LOGIC_H

#include <Arduino.h>
#include "Config.h"
#include "SerialProtocol.h"   // for SerialCommand
#include "DistanceSensors.h"  // for the sensor abstraction

class MotorControl;   // forward decl — defined in MotorControl.h
class ServoControl;  // forward decl — defined in ServoControl.h

// Which branch of the priority chain won this loop. The orchestrator
// (robot.ino) passes this to SerialProtocol::formatStatus() so the Pi
// can log/debug which override (if any) is active. See design_spec.md §4
// for the STATUS message format.
//
// `AvoidAmbiguousStop` is a finer-grained subdivision of "AVOID" used
// purely for local observability (status-LED blink pattern in
// robot.ino). On the wire, STATUS still serializes a single boolean
// `overrideActive` (1 for any of {Avoid, AvoidAmbiguousStop, Failsafe},
// 0 for None), per the spec — see SerialProtocol::formatStatus.
enum class OverrideState : uint8_t {
    None               = 0,  // Pi's last command is being passed through
    Avoid              = 1,  // Ultrasonic-triggered pivot away (clearer side known)
    AvoidAmbiguousStop = 2,  // Ultrasonic-triggered stop (no safe pivot direction; same actuation as failsafe)
    Failsafe           = 3,  // No recent valid CMD from the Pi
};

class SafetyLogic {
public:
    SafetyLogic();

    // Wire up references to the other modules. Call once from setup()
    // before the first update(). No allocation, no copies.
    void attach(MotorControl& motors,
                ServoControl& rudders,
                DistanceSensors& sensors,
                SerialProtocol& serial);

    // Evaluate the priority chain and dispatch the result.
    //
    //   1. If no valid CMD has been received within
    //      FAILSAFE_TIMEOUT_MS (per SerialProtocol::lastValidCmdMs()),
    //      FAILSAFE: stop both motors and center the rudder.
    //
    //   2. Otherwise, ask DistanceSensors::isObstacleWithin() on each
    //      sensor. If any sensor reports an obstacle, AVOID: stop the
    //      motor on the obstructed side, drive the other motor at
    //      AVOID_PIVOT_SPEED_PCT, and deflect the rudder to the full
    //      clamp value toward the clearer side. The "clearer side" is
    //      whichever side does NOT have an obstacle (front-with-clear
    //      sides → just stop; we don't know a safe direction). If both
    //      sides AND front have obstacles, we hold the safest known
    //      command: stop both motors and center the rudder.
    //
    //   3. Otherwise, PASS-THROUGH: apply SerialCommand::leftMotorPct,
    //      rightMotorPct, rudderAngleDeg to the actuators.
    //
    // Returns the override state so the caller can format STATUS.
    OverrideState update(const SerialCommand& lastCmd);

    // For STATUS / debug: the override state from the most recent
    // update() call. Defaults to Failsafe until the first update().
    OverrideState lastOverride() const { return _lastOverride; }

private:
    MotorControl*    _motors;
    ServoControl*    _rudders;
    DistanceSensors* _sensors;
    SerialProtocol*  _serial;

    OverrideState _lastOverride;

    // -- Override handlers -------------------------------------------------

    // Stop both motors and center the rudder. Used by FAILSAFE and by
    // the "all sides obstructed" sub-case of AVOID.
    void _applyFailsafe();

    // AVOID response: pivot away from the obstacle using differential
    // thrust + full rudder deflection toward the clearer side.
    //
    // `frontBlocked` / `leftBlocked` / `rightBlocked` are the results
    // of DistanceSensors::isObstacleWithin(...) for the three sensors
    // in this loop. The pivot direction is chosen by picking the
    // non-blocked side; if no side is clear, _applyFailsafe() is
    // called instead.
    void _applyAvoid(bool frontBlocked, bool leftBlocked, bool rightBlocked);

    // PASS-THROUGH: apply the Pi's last good command.
    void _applyPassThrough(const SerialCommand& cmd);
};

#endif // ROBOT_SAFETY_LOGIC_H

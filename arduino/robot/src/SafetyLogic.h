#ifndef ROBOT_SAFETY_LOGIC_H
#define ROBOT_SAFETY_LOGIC_H

#include <Arduino.h>
#include "Config.h"
#include "SerialProtocol.h"  
#include "DistanceSensors.h"  

class MotorControl;  
class ServoControl; 

enum class OverrideState : uint8_t {
    None               = 0,
    Avoid              = 1,
    AvoidAmbiguousStop = 2,
    Failsafe           = 3,
};

class SafetyLogic {
public:
    SafetyLogic();

    void attach(MotorControl& motors,
                ServoControl& rudders,
                DistanceSensors& sensors,
                SerialProtocol& serial);

    OverrideState update(const SerialCommand& lastCmd);

    OverrideState lastOverride() const { return _lastOverride; }

private:
    MotorControl*    _motors;
    ServoControl*    _rudders;
    DistanceSensors* _sensors;
    SerialProtocol*  _serial;

    OverrideState _lastOverride;

    void _applyFailsafe();

    void _applyAvoid(bool frontBlocked, bool leftBlocked, bool rightBlocked);

    void _applyPassThrough(const SerialCommand& cmd);
};

#endif
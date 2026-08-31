#ifndef ROBOT_SERVO_CONTROL_H
#define ROBOT_SERVO_CONTROL_H

#include <Arduino.h>
#include <Servo.h>
#include "Config.h"

class ServoControl {
public:
    ServoControl();

    void begin();

    void setBoth(uint8_t angleDeg);

    void center();

    uint8_t lastAngleDeg() const { return _lastAngleDeg; }

private:
    Servo    _leftRudder;
    Servo    _rightRudder;
    bool     _attached;
    uint8_t  _lastAngleDeg;

    static uint16_t _angleToPulseUs(uint8_t angleDeg);
};

#endif
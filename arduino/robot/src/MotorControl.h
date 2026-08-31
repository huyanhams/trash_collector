#ifndef ROBOT_MOTOR_CONTROL_H
#define ROBOT_MOTOR_CONTROL_H

#include <Arduino.h>
#include <Servo.h>
#include "Config.h"

class MotorControl {
public:
    MotorControl();

    void begin();

    void setBoth(uint8_t leftPct, uint8_t rightPct);

    void setLeft(uint8_t pct);
    void setRight(uint8_t pct);

    void stopBoth();

    uint8_t lastLeftPct()  const { return _lastLeftPct;  }
    uint8_t lastRightPct() const { return _lastRightPct; }

private:
    Servo _leftEsc;
    Servo _rightEsc;
    bool  _attached;

    uint8_t _lastLeftPct;
    uint8_t _lastRightPct;

    static uint16_t _pctToPulseUs(uint8_t pct);
};

#endif 
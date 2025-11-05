#pragma once

extern Servo rudderLeft, rudderRight;

// Rudder pins
extern const int RUDDER_LEFT_PIN;
extern const int RUDDER_RIGHT_PIN;

// Servo neutral and extreme throttles
extern const int RUDDER_LEFT_NEUTRAL_THROTTLE;
extern const int RUDDER_LEFT_LEFT_THROTTLE;
extern const int RUDDER_LEFT_RIGHT_THROTTLE;
extern const int RUDDER_RIGHT_NEUTRAL_THROTTLE;
extern const int RUDDER_RIGHT_LEFT_THROTTLE;
extern const int RUDDER_RIGHT_RIGHT_THROTTLE;

// Turning rudders left, middle and right
void test_rudders_direction();
void set_rudders(int leftPosition, int rightPosition);
void turn_rudders(int angle);
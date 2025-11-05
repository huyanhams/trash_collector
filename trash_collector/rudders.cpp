#include <Arduino.h>
#include <Servo.h>
#include "rudders.hpp"

// Rudder pins
Servo rudderLeft, rudderRight;
const int RUDDER_LEFT_PIN = 5;
const int RUDDER_RIGHT_PIN = 6;

// Rudder positions
// int rudderLeftPosition;
// int rudderRightPosition;

// Servo neutral and extreme throttles
const int RUDDER_LEFT_NEUTRAL_THROTTLE = 85;
const int RUDDER_LEFT_LEFT_THROTTLE = 10;
const int RUDDER_LEFT_RIGHT_THROTTLE = 180;
const int RUDDER_RIGHT_NEUTRAL_THROTTLE = 105;
const int RUDDER_RIGHT_LEFT_THROTTLE = 0;
const int RUDDER_RIGHT_RIGHT_THROTTLE = 180;

// Turning rudders left, middle and right
void test_rudders_direction()
{
  Serial.println("Turning rudders left");
  rudderLeft.write(0);
  rudderRight.write(0);
  delay(2000);

  Serial.println("Turning rudders middle");
  rudderLeft.write(90);
  rudderRight.write(90);
  delay(2000);

  Serial.println("Turning rudders right");
  rudderLeft.write(180);
  rudderRight.write(180);
  delay(2000);
}

void set_rudders(int leftPosition, int rightPosition)
{
  rudderLeft.write(leftPosition);
  rudderRight.write(rightPosition);
  // rudderLeftPosition = LeftPosition;
  // rudderRightPosition = RightPosition;
}

// Turn rudders to the right a same angle
void turn_rudders(int angle)
{
  rudderLeft.write(RUDDER_LEFT_NEUTRAL_THROTTLE + angle);
  rudderRight.write(RUDDER_RIGHT_NEUTRAL_THROTTLE + angle);
  // rudderLeftPosition += angle;
  // rudderRightPosition += angle;
}
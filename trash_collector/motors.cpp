// 1000: stop
// > 1000, motors turn counter-clockwise (looking from the back / the face of the props) and move forward
// both props are left handed

// right servo is corrent
// left servo is a little to the right (the right is good, center is a bit to the right and left is to the right)
// right: write(0 -> 90 -> 180)
// left: write (10 -> 95 -> 180)

#include <Arduino.h>
#include <Servo.h>
#include "motors.hpp"

Servo escLeft, escRight;

// Motor pins
const int ESC_LEFT_PIN = 9;
const int ESC_RIGHT_PIN = 10;

// Motor and rudder speeds
int escLeftSpeed;
int escRightSpeed;

// Calibrate by setting the ESCs to max throttle
void calibrate_esc()
{
  Serial.println("Start calibration");

  Serial.println("Set ESCs to max throttle");
  escLeft.writeMicroseconds(2000); // max throttle
  escRight.writeMicroseconds(2000);
  delay(5000);
  
  Serial.println("Done calibration");
  delay(1000);
}

// Arming the ESCs for 3 seconds at min throttle
void arm_esc()
{
  Serial.println("Setting the ESCs to min throttle");
  escLeft.writeMicroseconds(1000);
  escRight.writeMicroseconds(1000);
  delay(5000);
}

// Turning 2 motors at one direction, middle, then the other direction
void test_motors_direction()
{
  Serial.println("Turning motors backward");
  escLeft.writeMicroseconds(1000);
  escRight.writeMicroseconds(1000);
  delay(1000);

  Serial.println("Stopping motors");
  escLeft.writeMicroseconds(1500);
  escRight.writeMicroseconds(1500);
  delay(1000);

  Serial.println("Turning motors forward");
  escLeft.writeMicroseconds(2000);
  escRight.writeMicroseconds(2000);
  delay(1000);
}

void set_motors(int leftSpeed, int rightSpeed)
{
  escLeft.writeMicroseconds(leftSpeed);
  escRight.writeMicroseconds(rightSpeed);
}
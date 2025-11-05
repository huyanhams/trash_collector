#pragma once

extern Servo escLeft, escRight;

// Motor pins
extern const int ESC_LEFT_PIN;
extern const int ESC_RIGHT_PIN;

// Motor speeds
extern int escLeftSpeed;
extern int escRightSpeed;

// Calibrate by setting the ESCs to max throttle
void calibrate_esc();

// Arming the ESCs for 3 seconds at min throttle
void arm_esc();

// Turning 2 motors at one direction, middle, then the other direction
void test_motors_direction();

void set_motors(int leftSpeed, int rightSpeed);
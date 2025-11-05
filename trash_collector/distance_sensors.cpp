#include <Arduino.h>
#include "distance_sensors.hpp"

// Distance sensor pins
const int SENSOR_FRONT_TRIG = 7;
const int SENSOR_FRONT_ECHO = 8;
const int SENSOR_LEFT_TRIG = 12;
const int SENSOR_LEFT_ECHO = 13;
const int SENSOR_RIGHT_TRIG = 2;
const int SENSOR_RIGHT_ECHO = 4;

// Measure distances
const unsigned long DISTANCE_SENSOR_TIMEOUT = 20000UL; // 20ms -> 3m distance
unsigned long frontSensorDuration;
unsigned long rightSensorDuration;
float frontDistance;
float rightDistance;

// Sensor thresholds (m)
const float OBSTACLE_DISTANCE = 1.0;
const float ALIGN_DISTANCE = 2.0;

float get_front_distance()
{
  delay(60);
  digitalWrite(SENSOR_FRONT_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(SENSOR_FRONT_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(SENSOR_FRONT_TRIG, LOW);

  frontSensorDuration = pulseIn(SENSOR_FRONT_ECHO, HIGH, DISTANCE_SENSOR_TIMEOUT);
  if (frontSensorDuration == 0) return -1;
  return (float)((frontSensorDuration*.0343)/2.0);
}

float get_right_distance()
{
  delay(60);
  digitalWrite(SENSOR_RIGHT_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(SENSOR_RIGHT_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(SENSOR_RIGHT_TRIG, LOW);

  rightSensorDuration = pulseIn(SENSOR_RIGHT_ECHO, HIGH, DISTANCE_SENSOR_TIMEOUT);
  if (rightSensorDuration == 0) return -1;
  return (float)((rightSensorDuration*.0343)/2.0);
}
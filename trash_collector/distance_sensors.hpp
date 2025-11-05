#pragma once

extern const int SENSOR_FRONT_TRIG;
extern const int SENSOR_FRONT_ECHO;
extern const int SENSOR_LEFT_TRIG;
extern const int SENSOR_LEFT_ECHO;
extern const int SENSOR_RIGHT_TRIG;
extern const int SENSOR_RIGHT_ECHO;

// Measure distances
extern const unsigned long DISTANCE_SENSOR_TIMEOUT;
extern unsigned long frontSensorDuration;
extern unsigned long rightSensorDuration;
extern float frontDistance;
extern float rightDistance;

// Sensor thresholds (m)
extern const float OBSTACLE_DISTANCE;
extern const float ALIGN_DISTANCE;

float get_front_distance();
float get_right_distance();
// float[] get_distances();
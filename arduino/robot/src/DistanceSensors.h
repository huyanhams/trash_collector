#ifndef ROBOT_DISTANCE_SENSORS_H
#define ROBOT_DISTANCE_SENSORS_H

#include <Arduino.h>
#include <NewPing.h>
#include "Config.h"

static const uint8_t SENSOR_INDEX_FRONT = 0;
static const uint8_t SENSOR_INDEX_LEFT  = 1;
static const uint8_t SENSOR_INDEX_RIGHT = 2;
static const uint8_t SENSOR_COUNT       = 3;

class DistanceSensors {
public:
    DistanceSensors();

    void begin();

    void service();

    uint16_t latestCm(uint8_t index) const;

    bool isFresh(uint8_t index) const;

    bool isObstacleWithin(uint8_t index) const;

    uint8_t indexOfClosestObstacle() const;

    unsigned long lastReadingMs(uint8_t index) const;

private:
    NewPing* _sensors[SENSOR_COUNT];

    uint16_t       _latestCm[SENSOR_COUNT];
    unsigned long  _lastReadingMs[SENSOR_COUNT];
    bool           _fresh[SENSOR_COUNT];

    uint8_t        _nextToStart;     // SENSOR_INDEX_*; which sensor gets serviced next
    unsigned long  _lastPingStartMs; // millis() of the most recent ping start (any sensor)
    bool           _pingInFlight;    // true between ping start and read completion
    uint8_t        _pingSensorIndex; // which sensor the in-flight ping belongs to

    void _maybeStartNextPing();

    void _pollInFlightPing();
};

#endif
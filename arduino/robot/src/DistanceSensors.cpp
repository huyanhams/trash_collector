#include "DistanceSensors.h"

DistanceSensors::DistanceSensors()
    : _nextToStart(SENSOR_INDEX_FRONT),
      _lastPingStartMs(0),
      _pingInFlight(false),
      _pingSensorIndex(SENSOR_INDEX_FRONT) {
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        _sensors[i]       = nullptr;
        _latestCm[i]      = 0;
        _lastReadingMs[i] = 0;
        _fresh[i]         = false;
    }
}

void DistanceSensors::begin() {
    if (_sensors[SENSOR_INDEX_FRONT] == nullptr) {
        _sensors[SENSOR_INDEX_FRONT] = new NewPing(PIN_ULTRA_FRONT_TRIG,
                                                   PIN_ULTRA_FRONT_ECHO,
                                                   SENSOR_MAX_DISTANCE_CM);
    }
    if (_sensors[SENSOR_INDEX_LEFT] == nullptr) {
        _sensors[SENSOR_INDEX_LEFT] = new NewPing(PIN_ULTRA_LEFT_TRIG,
                                                  PIN_ULTRA_LEFT_ECHO,
                                                  SENSOR_MAX_DISTANCE_CM);
    }
    if (_sensors[SENSOR_INDEX_RIGHT] == nullptr) {
        _sensors[SENSOR_INDEX_RIGHT] = new NewPing(PIN_ULTRA_RIGHT_TRIG,
                                                   PIN_ULTRA_RIGHT_ECHO,
                                                   SENSOR_MAX_DISTANCE_CM);
    }
}

void DistanceSensors::service() {
    if (_pingInFlight) {
        _pollInFlightPing();
    }

    if (!_pingInFlight) {
        _maybeStartNextPing();
    }
}

void DistanceSensors::_maybeStartNextPing() {
    const unsigned long now = millis();
    if (_lastPingStartMs != 0 &&
        (now - _lastPingStartMs) < static_cast<unsigned long>(SENSOR_READ_INTERVAL_MS)) {
        return;
    }
    NewPing* s = _sensors[_nextToStart];
    if (s == nullptr) return;  

    s->ping_timer();
    _pingInFlight    = true;
    _pingSensorIndex = _nextToStart;
    _lastPingStartMs = now;
}

void DistanceSensors::_pollInFlightPing() {
    NewPing* s = _sensors[_pingSensorIndex];
    if (s == nullptr) {
        _pingInFlight = false;
        return;
    }

    if (!s->check_timer()) {
        return;  
    }

    const uint16_t rawCm = s->ping_result();
    _latestCm[_pingSensorIndex]      = rawCm;
    _lastReadingMs[_pingSensorIndex] = millis();
    _fresh[_pingSensorIndex]         = true;

    _pingInFlight = false;

    _nextToStart = static_cast<uint8_t>((_pingSensorIndex + 1U) % SENSOR_COUNT);
}

uint16_t DistanceSensors::latestCm(uint8_t index) const {
    if (index >= SENSOR_COUNT) return 0;
    return _latestCm[index];
}

bool DistanceSensors::isFresh(uint8_t index) const {
    if (index >= SENSOR_COUNT) return false;
    return _fresh[index];
}

bool DistanceSensors::isObstacleWithin(uint8_t index) const {
    if (index >= SENSOR_COUNT) return false;
    if (!_fresh[index])        return false;

    const uint16_t d = _latestCm[index];
    if (d == 0)                return false;  // no echo == clear
    if (d < static_cast<uint16_t>(AVOID_DISTANCE_THRESHOLD_CM)) return true;
    return false;
}

uint8_t DistanceSensors::indexOfClosestObstacle() const {
    uint8_t  bestIdx  = SENSOR_COUNT;
    uint16_t bestDist = 0xFFFF; 

    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        if (!_fresh[i])        continue;  
        const uint16_t d = _latestCm[i];
        if (d == 0)            continue;  
        if (d < bestDist) {
            bestDist = d;
            bestIdx  = i;
        }
    }
    return bestIdx;
}

unsigned long DistanceSensors::lastReadingMs(uint8_t index) const {
    if (index >= SENSOR_COUNT) return 0;
    return _lastReadingMs[index];
}

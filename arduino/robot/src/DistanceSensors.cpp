// DistanceSensors.cpp
//
// Round-robin, non-blocking driver for the three JSN-SR04T ultrasonic
// sensors. See DistanceSensors.h for the contract and the "no echo"
// semantics.
//
// Architecture:
//   We never have more than ONE ping in flight. service() either starts
//   a new ping (if the inter-ping gap SENSOR_READ_INTERVAL_MS has
//   elapsed and no ping is in flight) or checks whether the current
//   in-flight ping has completed (NewPing's check() method is
//   non-blocking; it just reads the hardware echo pin and computes
//   distance from a previously-laid timer). The maximum time a single
//   ping can stay in flight is bounded by SENSOR_MAX_DISTANCE_CM — at
//   200 cm and the speed of sound, that's ~12 ms round-trip, so
//   loop() is never delayed by more than a few hundred microseconds
//   per call (the time to read one digital pin and do an integer
//   compare).
//
// "No echo" / raw-zero handling (CARRY-FORWARD RULE):
//   NewPing returns 0 cm when no echo arrives within the max-distance
//   window. Per the user's explicit carry-forward instruction, this
//   value is treated as "no obstacle in range" (clear), NOT as "0 cm
//   to obstacle." The contract is enforced in three places:
//     1. _pollInFlightPing() stores 0 as-is, but also marks the
//        reading as "fresh" — so a 0 reading can never accidentally
//        become "stale" and produce an undefined result.
//     2. isObstacleWithin() returns false for any reading of 0 (and
//        also returns false for any reading >= AVOID_DISTANCE_THRESHOLD_CM).
//        This is the helper SafetyLogic will use, so it never has to
//        remember the special case itself.
//     3. indexOfClosestObstacle() treats 0 as "no obstacle known" and
//        ignores it when picking the "clearer side" for AVOID pivots.

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
    // The NewPing library takes (trigPin, echoPin, maxDistanceCm).
    // We construct them on the heap because the library is a C++ class
    // without a static-config mode, and we want each one to be a
    // distinct object so the round-robin scheduler can address them
    // independently. This allocation happens during setup() (one-time,
    // before the no-malloc rule kicks in per agent_rules.md §5).
    // Allocated objects are kept for the life of the program; no free.
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
    // Step 1: if a ping is in flight, check whether it has finished
    // (non-blocking). If so, store the result and advance the round
    // robin. Only one ping in flight at a time — the NewPing hardware
    // protocol requires it.
    if (_pingInFlight) {
        _pollInFlightPing();
    }

    // Step 2: if no ping is in flight, try to start the next one,
    // subject to the inter-ping gap (SENSOR_READ_INTERVAL_MS) on the
    // same sensor. The gap exists to avoid feeding the previous
    // sensor's echo into the next reading on the same sensor; it is
    // sensor-scoped, not global, so we can rotate between sensors
    // faster than the gap if we wanted to (we don't, but the option
    // is there).
    if (!_pingInFlight) {
        _maybeStartNextPing();
    }
}

void DistanceSensors::_maybeStartNextPing() {
    const unsigned long now = millis();
    if (_lastPingStartMs != 0 &&
        (now - _lastPingStartMs) < static_cast<unsigned long>(SENSOR_READ_INTERVAL_MS)) {
        // Not enough time has passed since the last ping on this
        // sensor. Skip; we'll try again next time service() is called.
        return;
    }
    NewPing* s = _sensors[_nextToStart];
    if (s == nullptr) return;  // begin() hasn't run; bail.

    // ping_timer() is the non-blocking ping starter: it triggers the
    // trig pulse and starts the echo timer in hardware. It returns
    // immediately.
    s->ping_timer();
    _pingInFlight    = true;
    _pingSensorIndex = _nextToStart;
    _lastPingStartMs = now;
}

void DistanceSensors::_pollInFlightPing() {
    NewPing* s = _sensors[_pingSensorIndex];
    if (s == nullptr) {
        // Defensive: if the sensor was never constructed, drop the
        // in-flight state and move on.
        _pingInFlight = false;
        return;
    }

    // check_timer() is the non-blocking completion check. It returns
    // true exactly once per ping, when the echo window has closed
    // (either because we got a real echo or because the max-distance
    // timeout elapsed). After that, ping_result() returns the
    // measured distance in cm, or 0 if no echo arrived.
    if (!s->check_timer()) {
        return;  // still waiting
    }

    const uint16_t rawCm = s->ping_result();
    _latestCm[_pingSensorIndex]      = rawCm;
    _lastReadingMs[_pingSensorIndex] = millis();
    _fresh[_pingSensorIndex]         = true;

    _pingInFlight = false;

    // Advance round-robin to the next sensor so the next call to
    // service() (after the inter-ping gap) pings a different one.
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
    // The single place where the AVOID threshold comparison happens.
    // Treats a missing reading (not fresh) and a "no echo" reading
    // (raw 0) the same way: as "no obstacle known," i.e. NOT within
    // threshold. This is the carry-forward rule.
    if (index >= SENSOR_COUNT) return false;
    if (!_fresh[index])        return false;

    const uint16_t d = _latestCm[index];
    if (d == 0)                return false;  // no echo == clear
    if (d < static_cast<uint16_t>(AVOID_DISTANCE_THRESHOLD_CM)) return true;
    return false;
}

uint8_t DistanceSensors::indexOfClosestObstacle() const {
    // Find the sensor with the smallest non-zero reading. Sensors
    // reporting 0 (no echo) are treated as "no obstacle known" and
    // skipped entirely — they can never "win" as the closest obstacle.
    // If no sensor has a non-zero reading, return SENSOR_COUNT so the
    // caller can decide its own safe behavior (e.g. hold course vs.
    // stop).
    uint8_t  bestIdx  = SENSOR_COUNT;
    uint16_t bestDist = 0xFFFF;  // sentinel larger than any real reading

    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        if (!_fresh[i])        continue;  // no data — skip
        const uint16_t d = _latestCm[i];
        if (d == 0)            continue;  // no echo — skip (carry-forward rule)
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

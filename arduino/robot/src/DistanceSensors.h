// DistanceSensors.h
//
// Reads the three JSN-SR04T waterproof ultrasonic sensors (front, left,
// right). All reads are NON-BLOCKING — service() returns immediately,
// and consumers (SafetyLogic) must call service() every loop and then
// ask "do I have a fresh reading on each sensor yet?" via isFresh() /
// latestCm().
//
// Why non-blocking: agent_rules.md §5 forbids long pulseIn() calls in
// the main loop. NewPing's ping_timer() with a max-distance bound keeps
// each call under a few ms even when no echo arrives.
//
// IMPORTANT — "no echo" semantics (carry-forward rule from the user):
//   NewPing returns 0 when no echo is received within the max-distance
//   window. Per user instruction, a raw 0 reading MUST be treated as
//   "no obstacle in range" (i.e. clear), NOT as "0 cm, something is
//   touching the sensor." The downstream SafetyLogic AVOID-threshold
//   comparison depends on this convention — see comments in .cpp and
//   the isObstacleWithin() helper for how 0 is normalized.
//
// All numeric constants come from Config.h. Do not hardcode here.

#ifndef ROBOT_DISTANCE_SENSORS_H
#define ROBOT_DISTANCE_SENSORS_H

#include <Arduino.h>
#include <NewPing.h>
#include "Config.h"

// Indices for the three sensors. Used to address arrays in this class
// and in SafetyLogic. Order matches the spec's "front, left, right"
// sensor layout. The right sensor is intentionally LAST so a future
// "no rear sensor" comment in the design spec stays visible in code.
static const uint8_t SENSOR_INDEX_FRONT = 0;
static const uint8_t SENSOR_INDEX_LEFT  = 1;
static const uint8_t SENSOR_INDEX_RIGHT = 2;
static const uint8_t SENSOR_COUNT       = 3;

class DistanceSensors {
public:
    DistanceSensors();

    // Construct the three NewPing sensor objects on their pins. Call
    // once from setup() BEFORE entering loop(). Does NOT start any
    // pings — those are kicked off by service().
    void begin();

    // Drive the round-robin sensor scheduler. To avoid cross-talk
    // between the three sensors, we ping at most one sensor per call,
    // and enforce a minimum gap (SENSOR_READ_INTERVAL_MS) between
    // pings on the same sensor. The "current" sensor rotates
    // front -> left -> right -> front -> ...
    //
    // This method never blocks. Each call either:
    //   (a) starts a new ping on the current sensor (if the inter-ping
    //       gap has elapsed), or
    //   (b) checks whether the previously-started ping has finished
    //       (non-blocking, bounded by SENSOR_MAX_DISTANCE_CM) and, if
    //       so, stores the result and advances to the next sensor.
    //
    // Must be called from loop() on every iteration. SafetyLogic is
    // expected to call it before reading the latest readings.
    void service();

    // Returns the most recent reading on the requested sensor, in
    // centimetres. The reading is normalized so that:
    //   - A value of 0 means "no echo / out of range" (per the
    //     carry-forward rule, this is "clear", NOT "0 cm to obstacle").
    //   - Any value > 0 and <= SENSOR_MAX_DISTANCE_CM is a real
    //     measurement in centimetres.
    //   - Values are not clamped at the upper end; the caller may want
    //     the raw value for telemetry, even if it's "max distance."
    //
    // `index` must be one of SENSOR_INDEX_FRONT / LEFT / RIGHT.
    uint16_t latestCm(uint8_t index) const;

    // True if the requested sensor has produced at least one reading
    // since begin(). False until the first complete round-robin cycle.
    // SafetyLogic should treat "not fresh yet" the same as "no echo"
    // (i.e. clear) to avoid a startup false-positive AVOID.
    bool isFresh(uint8_t index) const;

    // True if a fresh, valid reading exists and the measured distance
    // is strictly less than AVOID_DISTANCE_THRESHOLD_CM. A "no echo"
    // reading (0) returns FALSE here — that is the entire point of this
    // helper, and the reason it exists: so SafetyLogic can call
    // isObstacleWithin(SENSOR_INDEX_FRONT) and never have to remember
    // to special-case 0.
    //
    // If the sensor is not yet fresh, also returns false (we have no
    // evidence of an obstacle).
    bool isObstacleWithin(uint8_t index) const;

    // Returns the sensor index that has the smallest valid (non-zero)
    // distance reading, or SENSOR_COUNT if none of the three sensors
    // have a valid reading. Used by SafetyLogic to pick which side
    // is "clearer" during an AVOID pivot: a sensor returning 0 (no
    // echo) is treated as "no obstacle known" and is therefore a
    // candidate for the clearer side, but if every sensor reports 0
    // (all echoes lost), we return SENSOR_COUNT so the caller can
    // decide its own safe-stop behavior.
    uint8_t indexOfClosestObstacle() const;

    // Convenience: monotonic millis() timestamp of the most recent
    // completed reading on the requested sensor, or 0 if no reading
    // has completed yet.
    unsigned long lastReadingMs(uint8_t index) const;

private:
    NewPing* _sensors[SENSOR_COUNT];

    // Most recent reading in cm. 0 means "no echo / not yet read."
    uint16_t       _latestCm[SENSOR_COUNT];
    // millis() of the most recent completed reading, 0 if none yet.
    unsigned long  _lastReadingMs[SENSOR_COUNT];
    // True after the first complete reading has been stored.
    bool           _fresh[SENSOR_COUNT];

    // Round-robin state.
    uint8_t        _nextToStart;     // SENSOR_INDEX_*; which sensor gets serviced next
    unsigned long  _lastPingStartMs; // millis() of the most recent ping start (any sensor)
    bool           _pingInFlight;    // true between ping start and read completion
    uint8_t        _pingSensorIndex; // which sensor the in-flight ping belongs to

    // Kick off a new ping on `_nextToStart` if the inter-ping gap has
    // elapsed. No-op if a ping is already in flight or the gap hasn't
    // passed.
    void _maybeStartNextPing();

    // Check whether the in-flight ping has completed; if so, store the
    // result, clear the in-flight flag, and advance `_nextToStart`.
    void _pollInFlightPing();
};

#endif // ROBOT_DISTANCE_SENSORS_H

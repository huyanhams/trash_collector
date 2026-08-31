// SerialProtocol.h
//
// Parses incoming CMD lines from the Raspberry Pi and (optionally) emits
// STATUS telemetry back. Tracks when the last valid CMD was received so
// SafetyLogic can implement the FAILSAFE timeout.
//
// Protocol (see docs/design_spec.md §4 and docs/technical_spec.md §2):
//   Pi -> Arduino:  CMD,<leftMotor 0-100>,<rightMotor 0-100>,<rudderAngle 60-120>\n
//   Arduino -> Pi: STATUS,<distFront>,<distLeft>,<distRight>,<overrideActive>\n
//
// All parsing uses a fixed-size char buffer (no String class) and is
// non-blocking — call service() from loop() as often as practical.
//
// All numeric constants come from Config.h. Do not hardcode here.

#ifndef ROBOT_SERIAL_PROTOCOL_H
#define ROBOT_SERIAL_PROTOCOL_H

#include <Arduino.h>
#include "Config.h"

// Most-recent successfully parsed command from the Pi. Valid only when
// `hasValid` is true; cleared when the line is malformed or out-of-range
// (the timestamp is still updated only on accept, so SafetyLogic's
// failsafe check is meaningful).
struct SerialCommand {
    uint8_t leftMotorPct;   // 0..100
    uint8_t rightMotorPct;  // 0..100
    uint8_t rudderAngleDeg; // RUDDER_MIN_DEG..RUDDER_MAX_DEG (60..120)
    bool    hasValid;       // true if the most recent service() produced a valid line
};

class SerialProtocol {
public:
    SerialProtocol();

    // Begin serial at SERIAL_BAUD_RATE. Call once from setup().
    void begin();

    // Drain any available bytes from the serial port, accumulate into the
    // line buffer, and attempt to parse one full newline-terminated line.
    // If a line is complete and valid, the fields in `outCmd` are filled
    // and outCmd->hasValid is set to true. If a line is complete but
    // malformed, outCmd->hasValid is left at its prior value (the prior
    // command is NOT overwritten by a bad one).
    //
    // On a valid CMD, updates the internal "last-valid-CMD" timestamp
    // (millis()) that SafetyLogic reads for FAILSAFE.
    //
    // Must be called from loop() — never block on Serial here.
    void service(SerialCommand* outCmd);

    // Returns the millis() timestamp of the most recently accepted CMD
    // (i.e. the last time service() parsed a line that passed all
    // validation). Returns 0 if no command has ever been accepted.
    unsigned long lastValidCmdMs() const;

    // Build a STATUS,<front>,<left>,<right>,<overrideActive> line into the
    // provided fixed-size buffer. `overrideActive` is true when the
    // Arduino is currently overriding the Pi (AVOID or FAILSAFE), false
    // when the Pi's last command is being passed through.
    //
    // Distance values are clamped to [0, SENSOR_MAX_DISTANCE_CM]. A reading
    // of SENSOR_MAX_DISTANCE_CM (or 0) means "no echo / out of range" —
    // the receiver should treat either as "no obstacle known" rather
    // than as a real distance.
    //
    // Returns the number of characters written to `buf` (excluding the
    // terminating NUL), or 0 if the buffer is too small to hold the
    // longest possible line. The caller is responsible for writing those
    // bytes to Serial and appending '\n'.
    static size_t formatStatus(char* buf, size_t bufLen,
                               uint16_t distFrontCm,
                               uint16_t distLeftCm,
                               uint16_t distRightCm,
                               bool overrideActive);

private:
    // Buffer for one incoming line. CMD lines are short
    // ("CMD,100,100,090\n" = 14 chars; CMD_LINE_MAX_LEN in Config.h is
    // a safe upper bound). Using a fixed buffer keeps us off the heap
    // and off the String class, per agent_rules.md §5.
    char     _buf[CMD_LINE_MAX_LEN];
    uint8_t  _len;            // current fill count (0..CMD_LINE_MAX_LEN-1)
    bool     _overflowed;      // sticky: a byte arrived when the buffer was full; discard the rest of that line

    unsigned long _lastValidCmdMs;

    // Attempt to parse the line currently in _buf[0.._len) as a CMD.
    // On success, fills `out`, updates _lastValidCmdMs, and returns true.
    // On any validation failure, returns false and leaves *out untouched.
    bool _tryParseCmd(char* line, uint8_t len, SerialCommand* out);

    // Reset the line buffer (used after a successful parse or after a
    // discarded line).
    void _resetLine();
};

#endif // ROBOT_SERIAL_PROTOCOL_H

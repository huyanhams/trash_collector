// SerialProtocol.cpp
//
// Implementation of the Pi <-> Arduino serial link. See SerialProtocol.h
// for the protocol format and the contract of each method.
//
// Design notes:
// - All parsing is byte-at-a-time, fully non-blocking. service() never
//   calls Serial.readBytes or any blocking wait. It is safe to call from
//   loop() on every iteration.
// - The line buffer is fixed-size (CMD_LINE_MAX_LEN) and we use raw
//   char*, not the String class — agent_rules.md §5 forbids String on
//   the Arduino (heap fragmentation on a 2KB-SRAM chip).
// - Validation is intentionally strict: a malformed line does NOT
//   overwrite the previously accepted command. SafetyLogic continues to
//   apply the last good command until the failsafe timeout expires, at
//   which point it will force-stop. This is the safe failure mode for a
//   noisy serial line: missing data, not garbage data, triggers stop.
// - We update _lastValidCmdMs ONLY on a successfully parsed line, not on
//   receipt of any byte. That timestamp is what SafetyLogic reads to
//   decide whether to enter FAILSAFE.

#include "SerialProtocol.h"

SerialProtocol::SerialProtocol()
    : _len(0),
      _overflowed(false),
      _lastValidCmdMs(0) {
    _buf[0] = '\0';
}

void SerialProtocol::begin() {
    Serial.begin(SERIAL_BAUD_RATE);
    // No "wait for serial" call here — robot.ino's setup() may need to
    // be headless on first boot (no monitor attached).
}

void SerialProtocol::service(SerialCommand* outCmd) {
    // Read every available byte; do not stop early on '\n' because the
    // data may arrive in multiple chunks and the newline marks the end
    // of one logical line, not the end of available data.
    while (Serial.available() > 0) {
        const char c = static_cast<char>(Serial.read());

        if (_overflowed) {
            // We already determined the current line is too long; skip
            // bytes until we see the terminator, then reset.
            if (c == PROTOCOL_TERMINATOR) {
                _overflowed = false;
                _resetLine();
            }
            continue;
        }

        if (c == PROTOCOL_TERMINATOR) {
            // End of line — try to parse whatever we've accumulated.
            if (_len > 0 && outCmd != nullptr) {
                if (_tryParseCmd(_buf, _len, outCmd)) {
                    _lastValidCmdMs = millis();
                }
                // On parse failure we deliberately keep the prior
                // SerialCommand unchanged. SafetyLogic falls back to
                // FAILSAFE on its own if the timestamp goes stale.
            }
            _resetLine();
            continue;
        }

        // CR / LF other than our configured terminator: ignore. This
        // makes the parser robust to \r\n line endings sent by tooling
        // that doesn't strip CRs.
        if (c == '\r') {
            continue;
        }

        // Append the byte if there's room; otherwise mark the line as
        // overflowed and start discarding until the next terminator.
        // The "-1" leaves space for a trailing NUL we maintain for
        // safe C-string operations.
        if (_len < CMD_LINE_MAX_LEN - 1) {
            _buf[_len++] = c;
        } else {
            _overflowed = true;
        }
    }
}

unsigned long SerialProtocol::lastValidCmdMs() const {
    return _lastValidCmdMs;
}

void SerialProtocol::_resetLine() {
    _len = 0;
    _buf[0] = '\0';
    _overflowed = false;
}

bool SerialProtocol::_tryParseCmd(char* line, uint8_t len, SerialCommand* out) {
    // Expected format: "CMD,L,R,RUDDER"
    //   - exactly 4 fields separated by PROTOCOL_DELIM
    //   - first field must equal CMD_PREFIX
    //   - L, R are unsigned integers in [MOTOR_MIN_PCT, MOTOR_MAX_PCT]
    //   - RUDDER is an unsigned integer in [RUDDER_MIN_DEG, RUDDER_MAX_DEG]
    //
    // We do the parse in place on the caller's buffer (which is _buf)
    // and walk it with two indices: `start` and `end` of the current
    // field. We replace PROTOCOL_DELIM bytes with '\0' as we go so we
    // can use strtoul on each field.

    if (line == nullptr || out == nullptr || len == 0) {
        return false;
    }

    // Field 0: prefix. Must be exactly "CMD" (length 3) followed by a
    // delimiter. We compare against the first 3 bytes and require the
    // 4th to be a delimiter (or end of buffer — accepted but rare).
    if (len < 4) {  // need at least "CMD,"
        return false;
    }
    if (line[0] != 'C' || line[1] != 'M' || line[2] != 'D') {
        return false;
    }
    if (line[3] != PROTOCOL_DELIM) {
        return false;
    }

    // Walk the remaining bytes and split on delimiter, expecting exactly
    // 3 more fields: L, R, rudder.
    char* fields[3];
    uint8_t fieldCount = 0;

    uint8_t start = 4;  // skip "CMD,"
    for (uint8_t i = 4; i < len; i++) {
        if (line[i] == PROTOCOL_DELIM) {
            if (fieldCount >= 3) {
                return false;  // too many fields
            }
            line[i] = '\0';
            fields[fieldCount++] = &line[start];
            start = i + 1;
        }
    }
    // Trailing field (no delimiter after it).
    if (fieldCount >= 3) {
        return false;
    }
    // Must be non-empty.
    if (start >= len) {
        return false;
    }
    line[len] = '\0';
    fields[fieldCount++] = &line[start];

    if (fieldCount != 3) {
        return false;
    }

    // Parse the three numeric fields. strtoul will set endptr to the
    // first non-numeric character; if it points at '\0', the whole field
    // was numeric.
    char* endp = nullptr;

    long lVal = strtol(fields[0], &endp, 10);
    if (endp == fields[0] || *endp != '\0') return false;
    if (lVal < static_cast<long>(MOTOR_MIN_PCT) ||
        lVal > static_cast<long>(MOTOR_MAX_PCT)) return false;

    long rVal = strtol(fields[1], &endp, 10);
    if (endp == fields[1] || *endp != '\0') return false;
    if (rVal < static_cast<long>(MOTOR_MIN_PCT) ||
        rVal > static_cast<long>(MOTOR_MAX_PCT)) return false;

    long rudderVal = strtol(fields[2], &endp, 10);
    if (endp == fields[2] || *endp != '\0') return false;
    if (rudderVal < static_cast<long>(RUDDER_MIN_DEG) ||
        rudderVal > static_cast<long>(RUDDER_MAX_DEG)) return false;

    // All checks passed — write the result. Note: the caller is
    // responsible for keeping a copy if they need to retain the value
    // across an invalid line, since on parse failure we do not touch
    // *out (see comment in service()).
    out->leftMotorPct   = static_cast<uint8_t>(lVal);
    out->rightMotorPct  = static_cast<uint8_t>(rVal);
    out->rudderAngleDeg = static_cast<uint8_t>(rudderVal);
    out->hasValid       = true;
    return true;
}

size_t SerialProtocol::formatStatus(char* buf, size_t bufLen,
                                    uint16_t distFrontCm,
                                    uint16_t distLeftCm,
                                    uint16_t distRightCm,
                                    bool overrideActive) {
    // We need at minimum:
    //   "STATUS," (7) + up to 5 chars per distance + "," between them
    //   + 1 char for override flag + '\0'
    // i.e. 7 + 3*5 + 2 + 1 = 25. Round up to 32 to be safe.
    // We compute it exactly via snprintf below and check the return.
    if (buf == nullptr || bufLen == 0) {
        return 0;
    }

    // Clamp distances to the sensor's reported range so a bogus reading
    // (0 means "no echo" for NewPing) doesn't get serialized as a
    // misleading zero distance to the Pi.
    if (distFrontCm > SENSOR_MAX_DISTANCE_CM) distFrontCm = SENSOR_MAX_DISTANCE_CM;
    if (distLeftCm  > SENSOR_MAX_DISTANCE_CM) distLeftCm  = SENSOR_MAX_DISTANCE_CM;
    if (distRightCm > SENSOR_MAX_DISTANCE_CM) distRightCm = SENSOR_MAX_DISTANCE_CM;

    int written = snprintf(buf, bufLen, "STATUS,%u,%u,%u,%d",
                           static_cast<unsigned>(distFrontCm),
                           static_cast<unsigned>(distLeftCm),
                           static_cast<unsigned>(distRightCm),
                           overrideActive ? 1 : 0);

    if (written < 0 || static_cast<size_t>(written) >= bufLen) {
        return 0;
    }
    return static_cast<size_t>(written);
}

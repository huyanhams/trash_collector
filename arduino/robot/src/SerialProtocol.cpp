#include "SerialProtocol.h"

SerialProtocol::SerialProtocol()
    : _len(0),
      _overflowed(false),
      _lastValidCmdMs(0) {
    _buf[0] = '\0';
}

void SerialProtocol::begin() {
    Serial.begin(SERIAL_BAUD_RATE);
}

void SerialProtocol::service(SerialCommand* outCmd) {
    while (Serial.available() > 0) {
        const char c = static_cast<char>(Serial.read());

        if (_overflowed) {
            if (c == PROTOCOL_TERMINATOR) {
                _overflowed = false;
                _resetLine();
            }
            continue;
        }

        if (c == PROTOCOL_TERMINATOR) {
            if (_len > 0 && outCmd != nullptr) {
                if (_tryParseCmd(_buf, _len, outCmd)) {
                    _lastValidCmdMs = millis();
                }
            }
            _resetLine();
            continue;
        }

        if (c == '\r') {
            continue;
        }

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

    if (line == nullptr || out == nullptr || len == 0) {
        return false;
    }

    if (len < 4) {  // need at least "CMD,"
        return false;
    }
    if (line[0] != 'C' || line[1] != 'M' || line[2] != 'D') {
        return false;
    }
    if (line[3] != PROTOCOL_DELIM) {
        return false;
    }

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
    if (fieldCount >= 3) {
        return false;
    }
    if (start >= len) {
        return false;
    }
    line[len] = '\0';
    fields[fieldCount++] = &line[start];

    if (fieldCount != 3) {
        return false;
    }

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
    if (buf == nullptr || bufLen == 0) {
        return 0;
    }

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

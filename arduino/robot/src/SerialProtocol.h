#ifndef ROBOT_SERIAL_PROTOCOL_H
#define ROBOT_SERIAL_PROTOCOL_H

#include <Arduino.h>
#include "Config.h"

struct SerialCommand {
    uint8_t leftMotorPct;   
    uint8_t rightMotorPct;  
    uint8_t rudderAngleDeg; 
    bool    hasValid;       
};

class SerialProtocol {
public:
    SerialProtocol();

    void begin();
    void service(SerialCommand* outCmd);

    unsigned long lastValidCmdMs() const;

    static size_t formatStatus(char* buf, size_t bufLen,
                               uint16_t distFrontCm,
                               uint16_t distLeftCm,
                               uint16_t distRightCm,
                               bool overrideActive);

private:
    char     _buf[CMD_LINE_MAX_LEN];
    uint8_t  _len;          
    bool     _overflowed;   

    unsigned long _lastValidCmdMs;

    bool _tryParseCmd(char* line, uint8_t len, SerialCommand* out);

    void _resetLine();
};

#endif
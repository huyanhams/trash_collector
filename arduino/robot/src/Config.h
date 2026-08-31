#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <Arduino.h>

static const uint8_t PIN_RESERVED_RX        = 0;   // Reserved — USB serial RX
static const uint8_t PIN_RESERVED_TX        = 1;   // Reserved — USB serial TX

static const uint8_t PIN_ULTRA_FRONT_TRIG   = 2;
static const uint8_t PIN_ULTRA_FRONT_ECHO   = 3;
static const uint8_t PIN_ULTRA_LEFT_TRIG    = 4;
static const uint8_t PIN_ULTRA_LEFT_ECHO    = 5;
static const uint8_t PIN_ULTRA_RIGHT_TRIG   = 6;
static const uint8_t PIN_ULTRA_RIGHT_ECHO   = 7;

// --- Actuators ---
static const uint8_t PIN_ESC_LEFT           = 8;   // A2212 BLDC ESC signal (via Servo lib)
static const uint8_t PIN_ESC_RIGHT          = 9;   // A2212 BLDC ESC signal (via Servo lib)
static const uint8_t PIN_SERVO_RUDDER_LEFT  = 10;  // MG996R rudder servo (left hull)
static const uint8_t PIN_SERVO_RUDDER_RIGHT = 11;  // MG996R rudder servo (right hull)

// --- Status / spare ---
static const uint8_t PIN_STATUS_LED         = 12;  
static const uint8_t PIN_RESERVED_LED_13    = 13;  

static const uint32_t SERIAL_BAUD_RATE       = 115200;  
static const uint16_t FAILSAFE_TIMEOUT_MS   = 1000;    

static const uint8_t  RUDDER_MIN_DEG         = 60;
static const uint8_t  RUDDER_MAX_DEG         = 120;
static const uint8_t  RUDDER_CENTER_DEG      = 90;    
static const uint8_t  MOTOR_MIN_PCT         = 0;
static const uint8_t  MOTOR_MAX_PCT         = 100;

static const char     CMD_PREFIX            = 'C';   
static const char     STATUS_PREFIX         = 'S';   
static const char     PROTOCOL_DELIM        = ',';
static const char     PROTOCOL_TERMINATOR   = '\n';

static const uint8_t  CMD_LINE_MAX_LEN      = 32;

static const uint16_t ESC_ARM_PULSE_US      = 1000; 
static const uint16_t ESC_MAX_PULSE_US      = 2000; 
static const uint16_t ESC_ARM_DELAY_MS      = 2000; 

static const uint16_t SERVO_MIN_PULSE_US    = 1000;
static const uint16_t SERVO_MAX_PULSE_US    = 2000;
static const uint16_t SENSOR_READ_INTERVAL_MS = 60;

static const uint16_t SENSOR_CYCLE_MS         = 180;

static const uint16_t SENSOR_MAX_DISTANCE_CM  = 200;

static const uint8_t  AVOID_DISTANCE_THRESHOLD_CM = 40;

static const uint8_t  AVOID_PIVOT_SPEED_PCT      = 30;


#endif 
#include <Servo.h>
#include "motors.hpp"
#include "rudders.hpp"
#include "distance_sensors.hpp"

// Finite state machine
enum State {
  IDLE, // Waiting for startup or command from Pi
  CRUISE, // Normal movement, following the lake edge
  AVOID_OBSTACLE, // Reacting to distance sensors to avoid collision
  ALIGN_FOR_COLLECTION, // Adjusting heading and position to line up with trash (based on Pi YOLO detection)
  COLLECT_TRASH, // Slow approach to trash and collecting it
  SHARP_TURN, // Turning in place if stuck or to change direction quickly
  ERROR_STATE // Emergency stop or failure state
};
State currentState = IDLE;
unsigned long long stateStartTime = 0;
void changeState(State newState) {
  currentState = newState;
  stateStartTime = millis();
}


// Failsafe if not getting Pi's command for a long period
const unsigned long PI_FAILSAFE_TIMEOUT = 2000;
unsigned long piLastCommandTime = 0;


void setup() {
  Serial.begin(9600);

  // Configurating ESC and servo pins
  escLeft.attach(ESC_LEFT_PIN);
  escRight.attach(ESC_RIGHT_PIN);
  rudderLeft.attach(RUDDER_LEFT_PIN);
  rudderRight.attach(RUDDER_RIGHT_PIN);

  // Configurating distance sensor pins
  pinMode(SENSOR_FRONT_TRIG, OUTPUT);
  pinMode(SENSOR_FRONT_ECHO, INPUT);
  pinMode(SENSOR_LEFT_TRIG, OUTPUT);
  pinMode(SENSOR_LEFT_ECHO, INPUT);
  pinMode(SENSOR_RIGHT_TRIG, OUTPUT);
  pinMode(SENSOR_RIGHT_ECHO, INPUT);

  turn_rudders(0);
  // Arm ESCs
  arm_esc();
}


void loop() {
  // frontDistance = get_front_distance();
  // rightDistance = get_right_distance();

  // switch (currentState)
  // {
  //   case IDLE:
  //     set_motors(1000, 1000);
  //     turn_rudders(0);
  //     if (millis() - stateStartTime > 3000) changeState(CRUISE);
  //     break;
    
  //   case CRUISE:
  //     set_motors(1500, 1500);
  //     if (rightDistance < 0.5) turn_rudders(-20);      // steer away
  //     else if (rightDistance > 1.0) turn_rudders(20); // steer closer
  //     else turn_rudders(0);

  //     if (frontDistance < OBSTACLE_DISTANCE) changeState(AVOID_OBSTACLE);
  //     break;
    
  //   case AVOID_OBSTACLE:
  //     set_motors(1500, 1500);
  //     set_rudders(-40);
  //     if (millis() - stateStartTime > 1500) changeState(CRUISE);
  //     break;
    
  //   case ALIGN_FOR_COLLECTION:
  //     break;
    
  //   case COLLECT_TRASH:
  //     break;
    
  //   case SHARP_TURN:
  //     break;
    
  //   case ERROR_STATE:
  //     break;
  // }

  turn_rudders(0);
  set_motors(1500, 1500);

  // Serial.println(get_right_distance());
  // Serial.println(get_front_distance());
}

// Get 3 seconds to plug the lipo in after running the code
void plug_in_lipo()
{
  Serial.println("Serial starts");
  Serial.println("Plug in the LiPo in 3 seconds");
  delay(6000);
}

// Receiving the Raspberry Pi's command if trash is detected
void get_pi_command()
{
  if (Serial.available())
  {
    // Command: "EL1800 ER2000 RL1000 RR1200"
    // String command = Serial.readStringUntil('\n');
    // if (sscanf(command.c_str(), "EL%d ER%d RL%d RR%d", &escLeftSpeed, &escRightSpeed, &rudderLeftSpeed, &rudderRightSpeed) == 4)
    // {
    //   escLeft.writeMicroseconds(escLeftSpeed);
    //   escRight.writeMicroseconds(escRightSpeed);
    //   rudderLeft.writeMicroseconds(rudderLeftSpeed);
    //   rudderRight.writeMicroseconds(rudderRightSpeed);
    //   piLastCommandTime = millis();
    // }

    if (millis() - piLastCommandTime > PI_FAILSAFE_TIMEOUT)
    {
      escLeft.writeMicroseconds(1000);
      escRight.writeMicroseconds(1000);
      rudderLeft.writeMicroseconds(1500);
      rudderRight.writeMicroseconds(1500);
    }
  }
}


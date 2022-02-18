#ifndef RPR_MANIPULATOR_DRIVER_H_
#define RPR_MANIPULATOR_DRIVER_H_

// include librarys 
#include <ArduinoQueue.h>
//#include <Arduino.h>
#include <Servo.h>

#include "RPR_base_servo.h"
#include "RPR_linear_actuator.h"
#include "RPR_end_servo.h"

/* Things to make defining Arm_Move_Command structs easier */
// Adds in the appropriate boolean array to specify which joints to move
#define BASE_ONLY {1,0,0}
#define LA_ONLY {0,1,0}
#define END_ONLY {0,0,1}
#define ALL_JOINTS {1,1,1}
// Adds in an empty LA_Move_Command when the command will not involve the linear actuator moving
#define LA_NO_MOVE_CMD {0,DEFAULT_MODE,0}
// Command to use when homing the linear actuator
#define HOMING_CMD {LA_ONLY,0,{0,HOMING_MODE,0},0}
// Command to use when sending all joints to their zero position
#define GO_HOME_CMD {ALL_JOINTS,BASE_ZERO_POSITION_PULSE_WIDTH,{0,DEFAULT_MODE,LA_MAX_VEL_NO_ACCEL},END_ZERO_POSITION_PULSE_WIDTH}
// Template Arm_Move_Command that initializes everything to default values so that
  // only the members which are changed need to be specified
#define LA_MAVE_CMD_INIT(...) {.joints_to_move = {0,0,0}, \
                               .base_cmd = 0, \
                               .LA_cmd = LA_NO_MOVE_CMD, \
                               .end_cmd = 0, \
                               __VA_ARGS__}

// Struct for storing the information that defines a motion task
struct Arm_Move_Command {
  bool joints_to_move [3];
  int base_cmd;
  LA_Move_Command LA_cmd;
  int end_cmd;
};
// Queue for storing motion tasks
#define QUEUE_MAX_SIZE 10
ArduinoQueue<Arm_Move_Command> Arm_Move_Queue(QUEUE_MAX_SIZE);

/****** High-level arm motion functions ******/
void Prepare_Arm_Motion(Arm_Move_Command cmd);
void Queue_Return_To_Home();
void Go_To_Home(); // Being replaced
void Queue_Pick_Up_Block(float block_x, float block_z);
void Queue_Simple_Move_To_Position(float goal_x, float goal_z);
/****** Test functions ******/
void Test_Base();
void Test_LA();
void Test_End();

#endif 

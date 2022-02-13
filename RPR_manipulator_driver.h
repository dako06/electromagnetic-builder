#ifndef RPR_MANIPULATOR_DRIVER_H_
#define RPR_MANIPULATOR_DRIVER_H_

// include librarys 
#include <ArduinoQueue.h>
//#include <Arduino.h>
#include <Servo.h>

#include "RPR_base_servo.h"
#include "RPR_linear_actuator.h"
#include "RPR_end_servo.h"

#define QUEUE_MAX_SIZE 10

struct Arm_Move_Command {
  bool joints_to_move [3];
  int base_cmd;
  LA_Move_Command LA_cmd;
  int end_cmd;
};

ArduinoQueue<Arm_Move_Command> Arm_Move_Queue(QUEUE_MAX_SIZE);

/****** High-level arm motion functions ******/
void Prepare_Arm_Motion(Arm_Move_Command cmd);
void Go_To_Home(); // Being replaced
//void Start_Next_Motion(); // Being replaced
void Queue_Pick_Up_Block(float block_x, float block_z);
void Queue_Simple_Move_To_Position(float goal_x, float goal_z);
/****** Test functions ******/
void Test_Joints();

#endif 

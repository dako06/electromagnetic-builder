#ifndef RPR_DRIVER_H_
#define RPR_DRIVER_H_

// include librarys 
#include <ArduinoQueue.h>
#include <Servo.h>

#define _TASK_MICRO_RES
#include <TaskScheduler.h>
#include <TaskSchedulerDeclarations.h>
#include <TaskSchedulerSleepMethods.h>

//////////////////////////////////////////////////////////////////////////////////////////
/**** Stuff for base servo ****/
Servo Base_joint;
#define BASE_SIGNAL_PIN 3
#define BASE_FEEDBACK_PIN A0
#define BASE_MIN_PULSE_WIDTH 1200 // The mechanical limit is 900, but only part of the full range is used
#define BASE_ZERO_POSITION_PULSE_WIDTH 1500
#define BASE_MAX_PULSE_WIDTH 2100
// Each time the servo position is changed, the new position is written to this address in the EEPROM (after being divided by 10
  // to ensure that it fits in 8 bits) so that it can be read next time the board is powered on (Not currently used)
#define BASE_LAST_POSITION_ADDR 0x00
// Parameters for setting the velocity of the base servo
  // Current setting: 2.86 degrees per second
#define BASE_WIDTH_INCREMENT 5
#define BASE_DELAY_US 90000
//#define BASE_DEFAULT_VEL 0.218 // Converted to radians
#define BASE_DEFAULT_VEL 0.05 // Converted to radians
int base_curr_position;
int base_goal_position;
int base_move_dir;
float base_angle_tan; // Used in calculating the velocity for the linear actuator; updated
                        // each time the base angle changes
//////////////////////////////////////////////////////////////////////////////////////////
/**** Stuff for linear actuator ****/
// Stepper motor wires should be in this order (starting furthest from the capacitor): blue, red, black, green
#define STEP_PIN 5
#define DIR_PIN 4
#define STEP_SIZE_MM 0.01
#define FULL_STROKE_STEPS 10000
// Do not allow the linear actuator to get within this many steps from its limits
//#define SAFETY_BUFFER_STEPS 100
#define SAFETY_BUFFER_STEPS 0 // Disabled for testing
//#define LA_MIN_PULSE_WIDTH 350
#define LA_MIN_PULSE_WIDTH 200
#define LA_MAX_PULSE_WIDTH 1000
#define LA_ACCEL_START_PULSE_WIDTH 625 // LA_MAX_VEL_NO_ACCEL converted to pulse width
#define LA_MAX_VEL_NO_ACCEL 8 // The maximum velocity that the linear actuator can begin moving to from rest, in mm/s
#define L0 215
unsigned int LA_curr_position;
unsigned int LA_goal_position;
unsigned int LA_accel_rate;
unsigned int LA_curr_pulse_width;
unsigned int LA_goal_pulse_width;
int LA_move_dir; // 1 for forwards, -1 for reverse
bool do_accel;
bool block_place_mode;
byte step_pin_state;
//////////////////////////////////////////////////////////////////////////////////////////
/**** Linear actuator move commands ****/
struct LA_Move_Command {
  unsigned int goal_position;
  bool block_place_mode;
  float vel_in_mm_per_sec;
};
#define QUEUE_MAX_SIZE 10
//////////////////////////////////////////////////////////////////////////////////////////
/**** Stuff for end servo ****/
Servo End_joint;
#define END_SIGNAL_PIN 6
#define END_MIN_PULSE_WIDTH 800 
#define END_ZERO_POSITION_PULSE_WIDTH 1350
#define END_MAX_PULSE_WIDTH 2300
// Each time the servo position is changed, the new position is written to this address in the EEPROM (after being divided by 10
  // to ensure that it fits in 8 bits) so that it can be read next time the board is powered on
#define END_LAST_POSITION_ADDR 0x04
// Parameters for setting the velocity of the end servo
  // Current setting: 16 degrees per second (same as base servo)
#define END_WIDTH_INCREMENT 5
#define END_DELAY_US 90000
int end_curr_position;
int end_goal_position;
int end_move_dir;
//////////////////////////////////////////////////////////////////////////////////////////
/**** Stuff for coordinating the arm as a whole ****/
struct Arm_Move_Command {
  int base_cmd;
  LA_Move_Command LA_cmd;
  int end_cmd;
};
ArduinoQueue<Arm_Move_Command> Arm_Move_Queue(QUEUE_MAX_SIZE);
//////////////////////////////////////////////////////////////////////////////////////////
Scheduler Arm_Move_Scheduler;
/****** T_Base_Servo: Task which moves the base servo to its next position ******/
void Base_Servo_OnDisable();
void Base_Servo_Move_Callback();
void Prepare_Base_Servo_Move_Task(int goal_position);
float Base_PWM_to_Rad(int PWM_us);
float Base_Rad_to_PWM(float radians);
Task T_Base_Servo(0,TASK_FOREVER,&Base_Servo_Move_Callback,&Arm_Move_Scheduler,0,NULL,&Base_Servo_OnDisable);
bool base_servo_move_complete; // flag to tell if the task has completed
/****** T_Lin_Act: Task which moves the linear actuator to its next position ******/
void LA_Extend();
void LA_Retract();
void Stepper_Driver_Setup();
void Lin_Act_OnDisable();
void Lin_Act_Move_Callback();
void Prepare_LA_Move_Task(LA_Move_Command LA_cmd);
bool LA_Is_Valid_Position(unsigned int pos);
void LA_Update_Pulse_Width();
void LA_Toggle_Step_Pin();
void LA_Set_Dir(unsigned int goal_position);
void LA_Set_Accel_Parameters(float vel_in_mm_per_sec);
float LA_Pos_mm();
void LA_Reverse(); // Only a temporary measure for moving the linear actuator backwards when the program fails to do so
Task T_Lin_Act(0,TASK_FOREVER,&Lin_Act_Move_Callback,&Arm_Move_Scheduler,0,NULL,&Lin_Act_OnDisable);
bool LA_move_complete; // flag to tell if the task has completed
/****** T_End_Servo: Task which moves the base servo to its next position ******/
void End_Servo_OnDisable();
void End_Servo_Move_Callback();
void Prepare_End_Servo_Move_Task(int goal_position);
float End_PWM_to_Rad(int PWM_us);
float End_Rad_to_PWM(float radians);
Task T_End_Servo(0,TASK_FOREVER,&End_Servo_Move_Callback,&Arm_Move_Scheduler,0,NULL,&End_Servo_OnDisable);
bool end_servo_move_complete; // flag to tell if the task has completed
/****** High-level arm motion functions ******/
void Go_To_Home();
void Start_Next_Motion();

#endif 
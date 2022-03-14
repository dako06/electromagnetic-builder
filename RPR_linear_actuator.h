#ifndef RPR_LINEAR_ACTUATOR_H_
#define RPR_LINEAR_ACTUATOR_H_

// UART used to communicate with the Arduino Uno controlling the linear actuator
#define LA_CNTRL_SERIAL Serial4

/*** Incoming messages ***/
// When the Uno is sending the current position of the linear actuator to the OpenCR, it sends this byte first
#define CURR_POSITION_MSG 0x01
// In homing mode, when the new zero position for the linear actuator is established this message is sent automatically to the OpenCR
#define HOMING_COMPLETE_MSG 0x03

/*** Outgoing messages ***/
// The first byte in a message determines what information is being requested or will be sent. The upper nibble of this byte is all
  // ones. If the OpenCR is sending information, it is sent 7 bits at a time with the MSB being zero
// This message is sent in order to set the goal position for the linear actuator
#define SET_GOAL_POSITION_MSG 0xF1
// This message is sent in order to set the goal velocity for the linear actuator in block place mode (given in terms of the microseconds per pulse) 
#define SET_GOAL_VELOCITY_MSG 0xF2
// This message is sent in order to request the current position of the linear actuator in steps
#define GET_CURR_POSITION_MSG 0xF3
// These messages are sent to set the mode of the linear actuator
#define SET_MODE_DEFAULT_MSG 0xF4
#define SET_MODE_BLOCK_PLACE_MSG 0xF5
#define SET_MODE_HOMING_MSG 0xF6

#define STEP_SIZE_MM 0.01
#define FULL_STROKE_STEPS 10000
// Do not allow the linear actuator to get within this many steps from its limits
#define SAFETY_BUFFER_STEPS 100
//#define SAFETY_BUFFER_STEPS 0 // For testing
//#define LA_MIN_PULSE_WIDTH 350
#define LA_MIN_PULSE_WIDTH 200
#define LA_MAX_PULSE_WIDTH 1000
#define LA_HOMING_PULSE_WIDTH 625
#define LA_ACCEL_START_PULSE_WIDTH 625 // LA_MAX_VEL_NO_ACCEL converted to pulse width
#define LA_MAX_VEL_NO_ACCEL 8 // The maximum velocity that the linear actuator can begin moving to from rest, in mm/s
#define LA_DEFAULT_ACCEL_RATE 1
#define LA_DEFAULT_VEL 10
unsigned int LA_curr_position;
unsigned int LA_goal_position;
bool block_place_mode;
bool LA_is_moving{false}; // flag to tell if the task has completed
bool LA_is_homing;
// bool step_pin_state;
void Init_Linear_Actuator();
void LA_Update_Curr_Position();
void LA_Send_Goal_Position(unsigned int goal_position);
bool LA_Is_Valid_Position(unsigned int pos);
void LA_Set_Accel_Parameters(float vel_in_mm_per_sec);
float LA_Pos_mm();
unsigned int LA_Vel_To_Pulse_Width(float vel_in_mm_per_sec);
unsigned int LA_Get_Max_Achievable_Pulse_Width();
void LA_Send_Goal_Velocity(unsigned int pulse_width);

typedef enum {
  DEFAULT_MODE,
  BLOCK_PLACE_MODE,
  HOMING_MODE
} LA_Move_Mode;

struct LA_Move_Command {
  unsigned int goal_position;
  LA_Move_Mode move_mode;
  float vel_in_mm_per_sec;
};

void Lin_Act_Move_Callback();
void Prepare_LA_Move_Task(LA_Move_Command LA_cmd);
void LA_Update_Pulse_Width();
void Homing_Callback();
void Start_LA_Homing();

#endif

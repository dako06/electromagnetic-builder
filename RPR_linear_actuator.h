#ifndef RPR_LINEAR_ACTUATOR_H_
#define RPR_LINEAR_ACTUATOR_H_

// Stepper motor wires should be in this order (starting furthest from the capacitor): blue, red, black, green
#define STEP_PIN 5
#define DIR_PIN 4
#define HOMING_SWITCH_PIN 7
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
#define LA_DEFAULT_ACCEL_RATE 1
#define LA_DEFAULT_VEL 10
#define L0 215 // The distance in the x direction from the base joint to the end joint with
                // the linear actuator fully retracted
unsigned int LA_curr_position;
unsigned int LA_goal_position;
unsigned int LA_accel_rate;
unsigned int LA_curr_pulse_width;
unsigned int LA_goal_pulse_width;
int LA_move_dir; // 1 for forwards, -1 for reverse
bool LA_is_accelerating;
bool LA_is_decelerating;
bool block_place_mode;
bool step_pin_state;
void Init_Linear_Actuator();
void LA_Extend();
void LA_Retract();
bool LA_Is_Valid_Position(unsigned int pos);
void LA_Toggle_Step_Pin();
void LA_Set_Dir(unsigned int goal_position);
void LA_Set_Accel_Parameters(float vel_in_mm_per_sec);
float LA_Pos_mm();
unsigned int LA_Vel_To_Pulse_Width(float vel_in_mm_per_sec);

struct LA_Move_Command {
  unsigned int goal_position;
  bool block_place_mode;
  float vel_in_mm_per_sec;
};

void Lin_Act_Move_Callback();
void Prepare_LA_Move_Task(LA_Move_Command LA_cmd);
void LA_Update_Pulse_Width();
bool LA_is_moving; // flag to tell if the task has completed
bool LA_is_homing;
void Homing_Callback();
void Start_LA_Homing();

#endif
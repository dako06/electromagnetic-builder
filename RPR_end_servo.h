#ifndef RPR_END_SERVO_H_
#define RPR_END_SERVO_H_

Servo End_joint;
#define END_SIGNAL_PIN 6
#define END_MIN_PULSE_WIDTH 800 
#define END_ZERO_POSITION_PULSE_WIDTH 1400
#define END_MAX_PULSE_WIDTH 2300
// Parameters for setting the velocity of the end servo
  // Current setting: 16 degrees per second (same as base servo)
#define END_WIDTH_INCREMENT 5
#define END_DELAY_US 90000
int end_curr_position;
int end_goal_position;
int end_move_dir;
void Init_End_Servo();
float End_PWM_to_Rad(int PWM_us);
float End_Rad_to_PWM(float radians);
bool End_Is_Valid_Position(int position); // Need to add to .cpp

void End_Servo_Move_Callback();
void Prepare_End_Servo_Move_Task(int goal_position);
bool end_is_moving; // flag to tell if the task has completed

#endif
#ifndef RPR_BASE_SERVO_H_
#define RPR_BASE_SERVO_H_

Servo Base_joint;
#define BASE_SIGNAL_PIN 3
#define BASE_FEEDBACK_PIN A0
#define BASE_MIN_PULSE_WIDTH 1200 // The mechanical limit is 900, but only part of the full range is used
#define BASE_ZERO_POSITION_PULSE_WIDTH 1500
#define BASE_MAX_PULSE_WIDTH 2100

// Parameters for setting the velocity of the base servo
  // Current setting: 2.86 degrees per second
#define BASE_WIDTH_INCREMENT 5
#define BASE_DELAY_US 90000 // Delay between setting new goal positions; this is already implemented by the main loop
//#define BASE_DEFAULT_VEL 0.218 // Converted to radians
#define BASE_DEFAULT_VEL 0.05 // Converted to radians
int base_curr_position;
int base_goal_position;
int base_move_dir; // TODO: remove this global variable and determine direction from within the callback function
bool base_is_moving;
float base_angle_tan; // Used in calculating the velocity for the linear actuator; updated
                        // each time the base angle changes
void Init_Base_Servo();
float Base_PWM_to_Rad(int PWM_us);
float Base_Rad_to_PWM(float radians);
bool Base_Is_Valid_Position(int position);
void Update_Base_Angle_Tan(int new_position);
void Base_Servo_Move_Callback();
void Prepare_Base_Servo_Move_Task(int goal_position);

#endif

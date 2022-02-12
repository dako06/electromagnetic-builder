#include "RPR_base_servo.h"
#include "RPR_linear_actuator.h"
#include "RPR_end_servo.h"

void Start_Next_Motion() {
  //while(!Serial.available());
   // Serial.read();
  if (!Arm_Move_Queue.isEmpty()) {
    Serial.println("Executing move");
    Arm_Move_Command next = Arm_Move_Queue.dequeue();
    Prepare_Base_Servo_Move_Task(next.base_cmd);
    Prepare_LA_Move_Task(next.LA_cmd);
    Prepare_End_Servo_Move_Task(next.end_cmd);
    T_Base_Servo.restart();
    T_Lin_Act.restart();
    T_End_Servo.restart();
  }
  else {
    Go_To_Home();  // For testing purposes, return to home position after the test finishes
  }
}

// All parameters should be in mm
void Queue_Pick_Up_Block(float block_x, float block_z) {
  // First move: move to a point 2 cm above the top of the block (given by block_z)
  // Use inverse kinematics to determine the required joint positions
  float start_x = block_x;
  float start_z = block_z + 20;
  float theta1 = atan2(start_z,start_x);
  float theta2 = start_x/cos(theta1) - L0;
  float theta3 = -1*theta1;
  int base_start_pos = Base_Rad_to_PWM(theta1);
  unsigned int LA_start_pos = theta2 * 100;
  int end_start_pos = End_Rad_to_PWM(theta3);
  Arm_Move_Command Move_1 = {base_start_pos, {LA_start_pos, false, 5}, end_start_pos};
  Arm_Move_Queue.enqueue(Move_1);
  // Second move: perform downward vertical motion to make contact with the block
  theta1 = atan2(block_z,block_x);
  theta2 = block_x/cos(theta1) - L0;
  theta3 = -1*theta1;
  float base_grab_pos = Base_Rad_to_PWM(theta1);
  float LA_grab_pos = theta2 * 100;
  float end_grab_pos = End_Rad_to_PWM(theta3);
  Arm_Move_Command Move_2 = {base_grab_pos, {LA_grab_pos, true, 0}, end_grab_pos};
  Arm_Move_Queue.enqueue(Move_2);
  // Third move: perform upward vertical motion to return to the start position
  Arm_Move_Command Move_3 = {base_start_pos, {LA_start_pos, true, 0}, end_start_pos};  
  Arm_Move_Queue.enqueue(Move_3);
}

void Queue_Simple_Move_To_Position(float goal_x, float goal_z) {
  float theta1 = atan2(goal_z,goal_x);
  float theta2 = goal_x/cos(theta1) - L0;
  float theta3 = -1*theta1;
  int base_goal_pos = Base_Rad_to_PWM(theta1);
  unsigned int LA_goal_pos = theta2 * 100;
  int end_goal_pos = End_Rad_to_PWM(theta3);
  Arm_Move_Command Move_1 = {base_goal_pos, {LA_goal_pos, false, LA_DEFAULT_VEL}, end_goal_pos};
  Arm_Move_Queue.enqueue(Move_1);
}

// Note: All functionality has been moved to the tb3_builder_core setup function,
/* void setup() {
  Serial.begin(115200);
  ////////////////////////// Base servo setup
  Init_Base_Servo();
  ////////////////////////// Linear actuator setup
  Init_Linear_Actuator();
  ////////////////////////// End servo setup
  Init_End_Servo();
  /////////////////////////////////////////////////
  while(!Serial.available());
  Serial.read();
  Serial.println("Begin");

  //Queue_Simple_Move_To_Position(240, 40);
  //Queue_Pick_Up_Block(250, 10);
  //Queue_Simple_Move_To_Position(220, 40);

  Arm_Move_Scheduler.addTask(T_Base_Servo);
  Arm_Move_Scheduler.addTask(T_Lin_Act);
  Arm_Move_Scheduler.addTask(T_End_Servo);
  Arm_Move_Scheduler.addTask(T_Homing);
  //Start_Next_Motion();
  Start_LA_Homing();
} */

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*********************************** Base servo functions ****************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Init_Base_Servo() {
  analogReference(INTERNAL);
  pinMode(BASE_SIGNAL_PIN, OUTPUT);
  pinMode(BASE_FEEDBACK_PIN, INPUT);
  Base_joint.attach(BASE_SIGNAL_PIN);
  base_curr_position = BASE_ZERO_POSITION_PULSE_WIDTH; // Arm is assumed to start in its home configuration
  base_move_dir = 1;
  base_is_moving = false;
}
void Base_Servo_Move_Callback() {
  // The base servo position needs to be updated as the base servo arrives at the position that was calculated 
    // in the previous iteration. When this function is first called, the base servo will be arriving at the position which
    // was assigned to new_position at the end of the previous iteration, and the current position is updated to reflect this
  static int new_position = 0;
  if (new_position != 0) {
    base_curr_position = new_position;
  }
  int new_position = base_curr_position + (base_move_dir * BASE_WIDTH_INCREMENT);
  if (Base_Is_Valid_Position(new_position)) {
    // Do not start a for loop if we are already within 1 iteration of the target position
    if (abs(new_position - base_goal_position) <= BASE_WIDTH_INCREMENT) {
      Base_joint.write(new_position);
      // Goal position reached
      base_is_moving = false;
    }
    else {
      Base_joint.write(new_position);
    }
    Update_Base_Angle_Tan(new_position);
  }
  else {
    Serial.print("new_position was invalid in Base_Servo_Move_Callback: ");
    Serial.println(new_position);
    base_is_moving = false;
  }
}
void Prepare_Base_Servo_Move_Task(int goal_position) {
  if (goal_position > BASE_MAX_PULSE_WIDTH || goal_position < BASE_MIN_PULSE_WIDTH) {
    Serial.println("goal_position was invalid in call to Prepare_Base_Servo_Move_Task");
  }
  T_Base_Servo.setInterval(BASE_DELAY_US);
  base_goal_position = goal_position;
  Serial.print("Base goal position: "); Serial.println(base_goal_position);
  base_move_dir = (base_curr_position > base_goal_position) ? -1 : 1; // -1 for CW, 1 for CCW
  base_servo_move_complete = 0;
  base_angle_tan = tan(Base_PWM_to_Rad(base_curr_position)); // starting value
}
float Base_PWM_to_Rad(int PWM_us) {
  return (((float)PWM_us - BASE_ZERO_POSITION_PULSE_WIDTH)/10)*DEG_TO_RAD;
}
float Base_Rad_to_PWM(float radians) {
  return (int)(radians*RAD_TO_DEG)*10 + BASE_ZERO_POSITION_PULSE_WIDTH;
}
bool Base_Is_Valid_Position(int position) {
  return position >= BASE_MIN_PULSE_WIDTH && new_position <= BASE_MAX_PULSE_WIDTH;
}
void Update_Base_Angle_Tan(int new_position) {
  // The tangent calculation is based on the average of the starting angle and the final angle for this
      // step in the base servo's movement
    int pwm_avg = (new_position + base_curr_position)/2;
    base_angle_tan = fabs(tan(Base_PWM_to_Rad(pwm_avg)));
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*********************************** Linear Actuator functions ***********************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Init_Linear_Actuator() {
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  // Mode initialization (Full step)
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(STEP_PIN, LOW);
  pinMode(HOMING_SWITCH_PIN, INPUT);
  LA_Extend();
  LA_curr_position = 0;
  LA_curr_pulse_width = LA_ACCEL_START_PULSE_WIDTH;
  LA_move_dir = 1;
  step_pin_state = LOW;
  LA_is_moving = false; // flag to tell if the task has completed
  LA_is_homing = false;
  LA_is_accelerating = false;
  LA_is_decelerating = false;
  block_place_mode = false;
}

void Homing_Callback() {
  // Keep moving the linear actuator backwards until it trips the homing switch
  // We do not track steps here because the final position that the linear actuator
    // ends up in will be considered the new zero position
  // When the homing switch is not pressed, the pin connected to the OpenCR is 
    // connected to ground. When the switch is pressed, it is pulled up to 5V
  if (digitalRead(HOMING_SWITCH_PIN)) {
    LA_curr_position = 0; 
    LA_is_homing = false;
    LA_is_moving = false;
    return;
  }
  else {
    LA_Toggle_Step_Pin();
  }
}

// Bring the linear actuator back until it trips the homing limit switch
/* void Start_LA_Homing() {
  if (T_Base_Servo.isEnabled() || T_Lin_Act.isEnabled() || T_End_Servo.isEnabled()) {
    Serial.println("Could not execute homing, another motion task is currently enabled");
  }
  else {
    Serial.println("Begin homing");
  }
  LA_Retract();
  LA_move_complete = 0;
  T_Homing.restart();
} */

void Lin_Act_Move_Callback() {
  // Check for out-of-bounds motion
  if (!LA_Is_Valid_Position(LA_curr_position)) {
    Serial.println("Linear actuator limit reached");
    LA_is_moving = false;
  }
  // Check if goal has been reached
  if (LA_curr_position == LA_goal_position) {
    digitalWrite(STEP_PIN, LOW); // Since a pulse is in progress, set the pin low before stopping
    LA_is_moving = false;
    return;
  }
  LA_Update_Pulse_Width();
  LA_Toggle_Step_Pin();
  if (step_pin_state == HIGH) {
    LA_curr_position += LA_move_dir;
  }
}
void Prepare_LA_Move_Task(LA_Move_Command LA_cmd) {
  //Serial.println("Entered Prepare_LA_Move_Task");
  unsigned int goal_position = LA_cmd.goal_position;
  if (!LA_Is_Valid_Position(goal_position)) {
    Serial.println("goal_position was invalid in call to Prepare_LA_Move_Task");
    return;
  }
  LA_goal_position = goal_position;
  Serial.print("Linear Actuator goal position: "); Serial.println(LA_goal_position);
  // Need to add velocity bounds check
  LA_Set_Dir(goal_position);
  float start_vel;
  if (LA_cmd.block_place_mode) {
    start_vel = BASE_DEFAULT_VEL * (L0 + LA_Pos_mm()) * base_angle_tan;
    block_place_mode = true;
  }
  else {
    start_vel = LA_cmd.vel_in_mm_per_sec;
    block_place_mode = false;
  }
  LA_Set_Accel_Parameters(start_vel);
  T_Lin_Act.setInterval(LA_curr_pulse_width);
  LA_move_complete = 0;
  //Serial.println("Finished Prepare_LA_Move_Task");
}

void LA_Set_Accel_Parameters(float vel_in_mm_per_sec) {
  LA_is_accelerating = false; // Will be set true if the LA needs to accelerate (unless in block place mode)
  // If the final velocity will be large, add additional acceleration at the start
  /* if (block_place_mode && (abs(base_goal_position - 1500) > abs(base_curr_position - 1500))) {
    float final_vel = BASE_DEFAULT_VEL * (L0 + LA_goal_position/100) * fabs(tan(Base_PWM_to_Rad(base_goal_position)));
    LA_accel_rate = 1;
    LA_goal_pulse_width = (unsigned int)((1.0/final_vel)*5000 + 0.5);
    LA_curr_pulse_width = LA_ACCEL_START_PULSE_WIDTH;
  } */
  if (block_place_mode) {
    // Goal pulse width is not used since it updates over time
    float vel_to_match = BASE_DEFAULT_VEL * (L0 + LA_Pos_mm()) * base_angle_tan;
    LA_curr_pulse_width = LA_Vel_To_Pulse_Width(vel_to_match);
  }
  else if (vel_in_mm_per_sec > LA_MAX_VEL_NO_ACCEL) {
    LA_is_accelerating = true;
    LA_goal_pulse_width = (unsigned int)((1.0/vel_in_mm_per_sec)*5000 + 0.5);
    LA_curr_pulse_width = LA_ACCEL_START_PULSE_WIDTH;
  }
  else {
    LA_curr_pulse_width = (unsigned int)((1.0/vel_in_mm_per_sec)*5000 + 0.5);
  }
}
void LA_Update_Pulse_Width() {
  // In order to cut down on unnecessary calculations, the velocity to match is only calculated on every 100 executions
    // of this function; the velocity is still updated on every execution however
  static unsigned long vel_update_counter = 0;
  static unsigned int pulse_width_to_match = LA_MAX_PULSE_WIDTH;
  vel_update_counter = LA_is_moving ? (vel_update_counter + 1) : 0;
  if (block_place_mode) {
    if (vel_update_counter % 100 == 0) {
      float vel_to_match = BASE_DEFAULT_VEL * (L0 + LA_Pos_mm()) * base_angle_tan;
      pulse_width_to_match = LA_Vel_To_Pulse_Width(vel_to_match);
    }
    if (abs(LA_curr_pulse_width - pulse_width_to_match) <= LA_accel_rate) {
      LA_curr_pulse_width = pulse_width_to_match;
    }
    else {
      if (LA_curr_pulse_width > pulse_width_to_match) {
        LA_curr_pulse_width -= LA_accel_rate;
      }
      else if (LA_curr_pulse_width < pulse_width_to_match) {
        LA_curr_pulse_width += LA_accel_rate;
      }
    }
  }
  else {
    if (LA_is_accelerating) {
      LA_curr_pulse_width -= min(LA_curr_pulse_width - LA_goal_pulse_width, LA_accel_rate);
      if (LA_curr_pulse_width <= LA_goal_pulse_width) {
        LA_is_accelerating = 0;
      }
    }
    else if (LA_is_decelerating) {
      LA_curr_pulse_width += min(LA_goal_pulse_width - LA_curr_pulse_width, LA_accel_rate);
      if (LA_curr_pulse_width >= LA_goal_pulse_width) {
        LA_is_decelerating = 0;
      }
    }
  }
 // Just to be safe
  if (LA_curr_pulse_width < LA_MIN_PULSE_WIDTH) {
    LA_curr_pulse_width = LA_MIN_PULSE_WIDTH;
  }
  if (LA_curr_pulse_width > LA_MAX_PULSE_WIDTH) {
    LA_curr_pulse_width = LA_MAX_PULSE_WIDTH;
  }
}
float LA_Pos_mm() {
  return (float)LA_curr_position * 0.01;
}
void LA_Set_Dir(unsigned int goal_position) {
  if (goal_position >= LA_curr_position) {
    LA_Extend();
  }
  else {
    LA_Retract();
  }
}
void LA_Toggle_Step_Pin() {
  step_pin_state = step_pin_state ? LOW : HIGH;
  digitalWrite(STEP_PIN, step_pin_state);
}
bool LA_Is_Valid_Position(unsigned int pos) {
  return (pos < (FULL_STROKE_STEPS - SAFETY_BUFFER_STEPS)) && (pos >= SAFETY_BUFFER_STEPS);
}
void LA_Extend() {
  digitalWrite(DIR_PIN, HIGH);
  LA_move_dir = 1;
}
void LA_Retract() {
  digitalWrite(DIR_PIN, LOW);
  LA_move_dir = -1;
}
unsigned int LA_Vel_To_Pulse_Width(float vel_in_mm_per_sec) {
  return (1.0/vel_in_mm_per_sec)*5000 + 0.5;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*********************************** End servo functions ****************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Init_End_Servo() {
  pinMode(END_SIGNAL_PIN, OUTPUT);
  End_joint.attach(END_SIGNAL_PIN);
  End_joint.write(END_ZERO_POSITION_PULSE_WIDTH);
  end_curr_position = END_ZERO_POSITION_PULSE_WIDTH;
  end_move_dir = 1;
  end_is_moving = false;
}
void End_Servo_Move_Callback() {
  // The end servo position needs to be updated as the end servo arrives at the position that was calculated 
    // in the previous iteration. When this function is first called, the end servo will be arriving at the position which
    // was assigned to new_position at the end of the previous iteration, and the current position is updated to reflect this
  static int new_position = 0;
  if (new_position != 0) {
    end_curr_position = new_position;
  }
  int new_position = end_curr_position + (end_move_dir * END_WIDTH_INCREMENT);
  if (new_position >= END_MIN_PULSE_WIDTH && new_position <= END_MAX_PULSE_WIDTH) {
    // Do not start a for loop if we are already within 1 iteration of the target position
    if (abs(new_position - end_goal_position) <= END_WIDTH_INCREMENT) {
      End_joint.write(new_position);
      // Goal position reached
      end_is_moving = false;
    }
    else {
      End_joint.write(new_position);
    }
  }
  else {
    Serial.print("new_position was invalid in End_Servo_Move_Callback: ");
    Serial.println(new_position);
    end_is_moving = false;
  }
}
void Prepare_End_Servo_Move_Task(int goal_position) {
  if (goal_position > END_MAX_PULSE_WIDTH || goal_position < END_MIN_PULSE_WIDTH) {
    Serial.println("goal_position was invalid in call to Prepare_End_Servo_Move_Task");
  }
  T_End_Servo.setInterval(END_DELAY_US);
  end_goal_position = goal_position;
  Serial.print("End goal position: "); Serial.println(end_goal_position);
  end_move_dir = (end_curr_position > end_goal_position) ? -1 : 1; // -1 for CW, 1 for CCW
  end_servo_move_complete = 0;
}
float End_PWM_to_Rad(int PWM_us) {
  return (((float)PWM_us - END_ZERO_POSITION_PULSE_WIDTH)/10)*DEG_TO_RAD;
}
float End_Rad_to_PWM(float radians) {
  return (int)(radians*RAD_TO_DEG)*10 + END_ZERO_POSITION_PULSE_WIDTH;
}
bool End_Is_Valid_Position(int position) {
  return position >= END_MIN_PULSE_WIDTH && new_position <= END_MAX_PULSE_WIDTH;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*********************************** High-level arm motion functions *****************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Go_To_Home() {
  Serial.println("Waiting for signal to return to home");
  while(!Serial.available());
  Serial.read();
  bool base_done = false;
  bool LA_done = false;
  bool end_done = false;
  if (base_curr_position != BASE_ZERO_POSITION_PULSE_WIDTH) {
    Prepare_Base_Servo_Move_Task(BASE_ZERO_POSITION_PULSE_WIDTH);
  }
  else {
    base_done = true;
  }
  if (LA_curr_position != SAFETY_BUFFER_STEPS) {
    Prepare_LA_Move_Task({SAFETY_BUFFER_STEPS, false, 5});
  }
  else {
    LA_done = true;
  }
  if (end_curr_position != END_ZERO_POSITION_PULSE_WIDTH) {
    Prepare_End_Servo_Move_Task(END_ZERO_POSITION_PULSE_WIDTH);
  }
  else {
    end_done = true;
  }
  if (base_done && LA_done && end_done) {
    Serial.println("All done");
    while(1);
  }
  T_Base_Servo.restart();
  T_Lin_Act.restart();
  T_End_Servo.restart();
}
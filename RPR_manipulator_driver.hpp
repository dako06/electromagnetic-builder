#include "RPR_manipulator_driver.h"

// All parameters should be in mm
//void Queue_Pick_Up_Block(float block_x, float block_z) {
//  // First move: move to a point 2 cm above the top of the block (given by block_z)
//  // Use inverse kinematics to determine the required joint positions
//  float start_x = block_x;
//  float start_z = block_z + 20;
//  float theta1 = atan2(start_z,start_x);
//  float theta2 = start_x/cos(theta1) - L0;
//  float theta3 = -1*theta1;
//  int base_start_pos = Base_Rad_to_PWM(theta1);
//  unsigned int LA_start_pos = theta2 * 100;
//  int end_start_pos = End_Rad_to_PWM(theta3);
//  Arm_Move_Command Move_1 = {base_start_pos, {LA_start_pos, false, 5}, end_start_pos};
////  Arm_Move_Queue.enqueue(Move_1);
//  // Second move: perform downward vertical motion to make contact with the block
//  theta1 = atan2(block_z,block_x);
//  theta2 = block_x/cos(theta1) - L0;
//  theta3 = -1*theta1;
//  float base_grab_pos = Base_Rad_to_PWM(theta1);
//  float LA_grab_pos = theta2 * 100;
//  float end_grab_pos = End_Rad_to_PWM(theta3);
//  Arm_Move_Command Move_2 = {base_grab_pos, {LA_grab_pos, true, 0}, end_grab_pos};
//  Arm_Move_Queue.enqueue(Move_2);
//  // Third move: perform upward vertical motion to return to the start position
//  Arm_Move_Command Move_3 = {base_start_pos, {LA_start_pos, true, 0}, end_start_pos};  
//  Arm_Move_Queue.enqueue(Move_3);
//}

//void Queue_Simple_Move_To_Position(float goal_x, float goal_z) {
//  float theta1 = atan2(goal_z,goal_x);
//  float theta2 = goal_x/cos(theta1) - L0;
//  float theta3 = -1*theta1;
//  int base_goal_pos = Base_Rad_to_PWM(theta1);
//  unsigned int LA_goal_pos = theta2 * 100;
//  int end_goal_pos = End_Rad_to_PWM(theta3);
//  Arm_Move_Command Move_1 = {base_goal_pos, {LA_goal_pos, false, LA_DEFAULT_VEL}, end_goal_pos};
//  Arm_Move_Queue.enqueue(Move_1);
//}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*********************************** Base servo functions ****************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Init_Base_Servo() {
//  analogReference(DEFAULT);     // TODO resolve scope declaration
  pinMode(BASE_SIGNAL_PIN, OUTPUT);
  pinMode(BASE_FEEDBACK_PIN, INPUT);
  Base_joint.attach(BASE_SIGNAL_PIN);
  base_curr_position = BASE_ZERO_POSITION_PULSE_WIDTH; // Arm is assumed to start in its home configuration
  base_goal_position = base_curr_position;
  base_move_dir = 1;
  base_is_moving = false;
}

void Base_Servo_Move_Callback() {
  // The base servo position needs to be updated as the base servo arrives at the position that was calculated 
    // in the previous iteration. When this function is first called, the base servo will be arriving at the position which
    // was assigned to new_position at the end of the previous iteration, and the current position is updated to reflect this
  static int new_position = 0;
//  if (new_position != 0) {
//    base_curr_position = new_position;
//  }

  new_position        = base_curr_position + (base_move_dir * BASE_WIDTH_INCREMENT);
  base_curr_position  = new_position;
  
  //if (Base_Is_Valid_Position(new_position)) {
  
    if (abs(new_position - base_goal_position) <= BASE_WIDTH_INCREMENT) 
    {
      Base_joint.write(new_position);
      // Goal position reached
      base_is_moving = false;
    }
    else 
    {
      Base_joint.write(new_position);
    }
    //Update_Base_Angle_Tan(new_position);
  //}
//  else {
//    Serial.print("new_position was invalid in Base_Servo_Move_Callback: ");
//    Serial.println(new_position);
//    base_is_moving = false;
//  }
}

void Prepare_Base_Servo_Move_Task(int goal_position) 
{

 // checks that 1200 < goal_position < 2100
// if (goal_position > BASE_MAX_PULSE_WIDTH || goal_position < BASE_MIN_PULSE_WIDTH) {
//   Serial.println("goal_position was invalid in call to Prepare_Base_Servo_Move_Task");
// }

 base_goal_position = goal_position;
 base_move_dir      = (base_curr_position > base_goal_position) ? -1 : 1; // -1 for CW, 1 for CCW
 base_is_moving     = true;
 base_angle_tan     = tan(Base_PWM_to_Rad(base_curr_position));           // starting value
 
}

float Base_PWM_to_Rad(int PWM_us) {
  return (((float)PWM_us - BASE_ZERO_POSITION_PULSE_WIDTH)/10)*DEG_TO_RAD;
}
float Base_Rad_to_PWM(float radians) {
 return (int)(radians*RAD_TO_DEG)*10 + BASE_ZERO_POSITION_PULSE_WIDTH;
}
bool Base_Is_Valid_Position(int position) {
 return position >= BASE_MIN_PULSE_WIDTH && position <= BASE_MAX_PULSE_WIDTH;
}
void Update_Base_Angle_Tan(int new_pos) {
 // The tangent calculation is based on the average of the starting angle and the final angle for this
     // step in the base servo's movement
   int pwm_avg = (new_pos + base_curr_position)/2;
   base_angle_tan = fabs(tan(Base_PWM_to_Rad(pwm_avg)));
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///*********************************** Linear Actuator functions ***********************************************/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Init_Linear_Actuator() {
 LA_CNTRL_SERIAL.begin(115200);
 LA_curr_position = 0;
 LA_goal_position = 0;
 LA_is_moving = false;
 LA_is_homing = false;
 block_place_mode = false;
}

void Start_LA_Homing() {
 if (base_is_moving || LA_is_moving || end_is_moving) {
   Serial.println("Could not execute homing, another motion task is currently in progress");
 }
 LA_is_homing = true;
 LA_is_moving = true;
 LA_CNTRL_SERIAL.write(SET_MODE_HOMING_MSG);
}

void Lin_Act_Move_Callback() {
  // If homing is in progress, just wait for the HOMING_COMPLETE message
  // Otherwise, read any CURR_POSITION messages received from the Uno
    // Keep only the most recent CURR_POSITION message
    // If the current position matches the goal position, set is_moving false
    // If a homing complete message is received, set curr_position to zero and is_moving/is_homing to false
  while(LA_CNTRL_SERIAL.available()) {
    // When the linear actuator is homing, the position is not being monitored so ignore all messages
      // except for HOMING_COMPLETE_MSG
    if (LA_is_homing) {
      if (LA_CNTRL_SERIAL.read() == HOMING_COMPLETE_MSG) {
        LA_curr_position = 0; 
        LA_is_homing = false;
        LA_is_moving = false;
        LA_CNTRL_SERIAL.flush();
      }
    }
    else {
      if (LA_CNTRL_SERIAL.peek() == CURR_POSITION_MSG) {
        // Only read the message now if all of the needed data has been received
        if (LA_CNTRL_SERIAL.available() >= 3) {
          // Get position info, and if it matches the goal position then the motion is complete
            // After every motion is completed, flush the serial buffer
          LA_CNTRL_SERIAL.read();
          LA_Update_Curr_Position();
          if (LA_curr_position == LA_goal_position) {
            LA_is_moving = false;
            LA_CNTRL_SERIAL.flush();
          }
        }
      }
      else {
        // The Uno should not send any messages that do not start with the CURR_POSITION_MSG
          // byte in this situation, but if it does just ignore them
        LA_CNTRL_SERIAL.read();
      }
    }
  }
}

void Prepare_LA_Move_Task(LA_Move_Command LA_cmd) {
  unsigned int goal_position = LA_cmd.goal_position;
  if (!LA_Is_Valid_Position(goal_position)) {
    Serial.println("goal_position was invalid in call to Prepare_LA_Move_Task");
    return;
  }
  LA_is_moving = true;
  switch(LA_cmd.move_mode) {
    case DEFAULT_MODE:
      LA_CNTRL_SERIAL.write(SET_MODE_DEFAULT_MSG);
      break;
    case BLOCK_PLACE_MODE:
      // Todo: send mode change message and then setup the goal velocity
      break;
    case HOMING_MODE:
      Start_LA_Homing();
      return;
      break;
    default: break;
  }
  LA_goal_position = goal_position;
  LA_Send_Goal_Position(goal_position);
  // if (LA_cmd.move_mode == BLOCK_PLACE_MODE) {
  //   block_place_mode = true;
  //   float vel_to_match = BASE_DEFAULT_VEL * (L0 + LA_Pos_mm()) * base_angle_tan;
  //   // Send goal velocity command
  //   //LA_curr_pulse_width = LA_Vel_To_Pulse_Width(vel_to_match);
  // }
  // else {
  //   //LA_Set_Accel_Parameters(LA_cmd.vel_in_mm_per_sec);
  //   block_place_mode = false;
  // }
}

// It is assumed that the next two bytes on the serial buffer store the current position information
void LA_Update_Curr_Position() {
  byte lower = LA_CNTRL_SERIAL.read();
  byte upper = LA_CNTRL_SERIAL.read();
  unsigned int position = ((unsigned int)upper << 8) | lower;
  // Reject any reported position that should not be possible
  if ((position < 0) || (position > FULL_STROKE_STEPS)) {
    return;
  }
  else {
    LA_curr_position = position;
  }
}

void LA_Send_Goal_Position(unsigned int goal_position) {
  LA_CNTRL_SERIAL.write(SET_GOAL_POSITION_MSG);
  byte goal_position_lower_7 = goal_position & 0x7F;
  byte goal_position_upper_7 = (goal_position & 0x7F80) >> 7;
  LA_CNTRL_SERIAL.write(goal_position_lower_7);
  LA_CNTRL_SERIAL.write(goal_position_upper_7);
}

//void LA_Update_Pulse_Width() {
//  // In order to cut down on unnecessary calculations, the velocity to match is only calculated on every 100 executions
//    // of this function; the velocity is still updated on every execution however
//  static unsigned long vel_update_counter = 0;
//  static unsigned int pulse_width_to_match = LA_MAX_PULSE_WIDTH;
//  vel_update_counter = LA_is_moving ? (vel_update_counter + 1) : 0;
//  if (block_place_mode) {
//    if (vel_update_counter % 100 == 0) {
//      float vel_to_match = BASE_DEFAULT_VEL * (L0 + LA_Pos_mm()) * base_angle_tan;
//      pulse_width_to_match = LA_Vel_To_Pulse_Width(vel_to_match);
//    }
//    if (abs(LA_curr_pulse_width - pulse_width_to_match) <= LA_accel_rate) {
//      LA_curr_pulse_width = pulse_width_to_match;
//    }
//    else {
//      if (LA_curr_pulse_width > pulse_width_to_match) {
//        LA_curr_pulse_width -= LA_accel_rate;
//      }
//      else if (LA_curr_pulse_width < pulse_width_to_match) {
//        LA_curr_pulse_width += LA_accel_rate;
//      }
//    }
//  }
//  else {
//    if (LA_is_accelerating) {
//      LA_curr_pulse_width -= std::min(LA_curr_pulse_width - LA_goal_pulse_width, LA_accel_rate);
//      if (LA_curr_pulse_width <= LA_goal_pulse_width) {
//        LA_is_accelerating = 0;
//      }
//    }
//    else if (LA_is_decelerating) {
//      LA_curr_pulse_width += std::min(LA_goal_pulse_width - LA_curr_pulse_width, LA_accel_rate);
//      if (LA_curr_pulse_width >= LA_goal_pulse_width) {
//        LA_is_decelerating = 0;
//      }
//    }
//  }
// // Just to be safe
//  if (LA_curr_pulse_width < LA_MIN_PULSE_WIDTH) {
//    LA_curr_pulse_width = LA_MIN_PULSE_WIDTH;
//  }
//  if (LA_curr_pulse_width > LA_MAX_PULSE_WIDTH) {
//    LA_curr_pulse_width = LA_MAX_PULSE_WIDTH;
//  }
//}

float LA_Pos_mm() {
 return (float)LA_curr_position * 0.01;
}

bool LA_Is_Valid_Position(unsigned int pos) {
 return (pos < (FULL_STROKE_STEPS - SAFETY_BUFFER_STEPS)) && (pos >= SAFETY_BUFFER_STEPS);
}

unsigned int LA_Vel_To_Pulse_Width(float vel_in_mm_per_sec) {
 return (1.0/vel_in_mm_per_sec)*5000 + 0.5;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///*********************************** End servo functions ****************************************************/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Init_End_Servo() {
  pinMode(END_SIGNAL_PIN, OUTPUT);
  End_joint.attach(END_SIGNAL_PIN);
  End_joint.write(END_ZERO_POSITION_PULSE_WIDTH);
  end_curr_position = END_ZERO_POSITION_PULSE_WIDTH;
  end_goal_position = end_curr_position;
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
  new_position = end_curr_position + (end_move_dir * END_WIDTH_INCREMENT);
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
 
 // check for 800 < goal < 2300
 if (goal_position > END_MAX_PULSE_WIDTH || goal_position < END_MIN_PULSE_WIDTH) {
   Serial.println("goal_position was invalid in call to Prepare_End_Servo_Move_Task");
 }
 end_goal_position = goal_position;
 end_move_dir = (end_curr_position > end_goal_position) ? -1 : 1; // -1 for CW, 1 for CCW
 end_is_moving = true;
}

float End_PWM_to_Rad(int PWM_us) {
  return (((float)PWM_us - END_ZERO_POSITION_PULSE_WIDTH)/10)*DEG_TO_RAD;
}
float End_Rad_to_PWM(float radians) {
  return (int)(radians*RAD_TO_DEG)*10 + END_ZERO_POSITION_PULSE_WIDTH;
}
bool End_Is_Valid_Position(int position) {
  return position >= END_MIN_PULSE_WIDTH && position <= END_MAX_PULSE_WIDTH;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///*********************************** High-level arm motion functions *****************************************/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Prepare_Arm_Motion(Arm_Move_Command cmd) {
  if (cmd.joints_to_move[0]) {
    Prepare_Base_Servo_Move_Task(cmd.base_cmd);
  }
//  if (cmd.joints_to_move[1]) {
//    Prepare_LA_Move_Task(cmd.LA_cmd);
//  }

  if (cmd.joints_to_move[2]) {
    Prepare_End_Servo_Move_Task(cmd.end_cmd);
  }
}
void Queue_Return_To_Home() {
  Arm_Move_Command go_home = GO_HOME_CMD;
  Arm_Move_Queue.enqueue(go_home); 
}
//// Bring the linear actuator back until it trips the homing limit switch
//void Queue_LA_Homing() {
//  Arm_Move_Command test_homing = HOMING_CMD;
//  Arm_Move_Queue.enqueue(test_homing);
//}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*********************************** Test functions **********************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// These functions queue specific motions in order to test functions of the arm
void Test_Base() {
  // Test range of motion for base servo
  Arm_Move_Command test_max_angle = {BASE_ONLY, BASE_MAX_PULSE_WIDTH, LA_NO_MOVE_CMD, 0};
  Arm_Move_Queue.enqueue(test_max_angle);
  Arm_Move_Command test_min_angle = {BASE_ONLY, BASE_MIN_PULSE_WIDTH, LA_NO_MOVE_CMD, 0};
  Arm_Move_Queue.enqueue(test_min_angle);
//  Queue_Return_To_Home();
}
//void Test_LA() {
//  // Test independent movement for the linear actuator
//  // Command movement at a low speed such that the linear actuator does not accelerate
//  Arm_Move_Command test_forwards_no_accel = {LA_ONLY,0,{5000,DEFAULT_MODE,LA_MAX_VEL_NO_ACCEL},0};
//  Arm_Move_Queue.enqueue(test_forwards_no_accel);
//  Arm_Move_Command test_reverse_no_accel = {LA_ONLY,0,{0,DEFAULT_MODE,LA_MAX_VEL_NO_ACCEL},0};
//  Arm_Move_Queue.enqueue(test_reverse_no_accel);
//  // Command movement at 12 mm/s, which is above the max speed that the linear actuator can achieve from 
//    // rest without taking time to accelerate
//  Arm_Move_Command test_forwards_with_accel = {LA_ONLY,0,{5000,DEFAULT_MODE,12},0};
//  Arm_Move_Queue.enqueue(test_forwards_with_accel);
//  Arm_Move_Command test_reverse_with_accel = {LA_ONLY,0,{0,DEFAULT_MODE,12},0};
//  Arm_Move_Queue.enqueue(test_reverse_with_accel);
//  // Test homing the linear actuator
//  Arm_Move_Command move_forwards = {LA_ONLY,0,{2000,DEFAULT_MODE,LA_MAX_VEL_NO_ACCEL},0};
//  Arm_Move_Queue.enqueue(move_forwards);
//  Queue_LA_Homing();
//}
void Test_End() {
  // Test range of motion for base servo
  Arm_Move_Command test_max_angle = {END_ONLY,0,LA_NO_MOVE_CMD,END_MAX_PULSE_WIDTH};
  Arm_Move_Queue.enqueue(test_max_angle);
  Arm_Move_Command test_min_angle = {END_ONLY,0,LA_NO_MOVE_CMD,END_MIN_PULSE_WIDTH};
  Arm_Move_Queue.enqueue(test_min_angle);
  Queue_Return_To_Home();
}

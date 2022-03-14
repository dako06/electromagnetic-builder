/*          Uno_LA_Controller_Final              */

/******************** Header files ********************/
// Maybe use the Timer2 library for better accuracy
#include "LA_constants.h"
#include "LA_variables.h"
#include "messages.h"
/******************** Functions for motion control ********************/
void LA_Extend();
void LA_Retract();
bool LA_Is_Valid_Position(unsigned int pos);
void LA_Toggle_Step_Pin();
void Set_Mode(LA_Move_Mode mode);
void End_of_Range_Error();
void LA_Set_Dir();
/******************** Functions for communication ********************/
void Send_Curr_Position();
void Set_Mode(LA_Move_Mode mode);
void Send_Homing_Complete();

void Send_Move_Complete() {
  Serial.write(SEND_MOVE_COMPLETE_MSG);
}

void Send_Homing_Complete() {
  Serial.write(SEND_HOMING_COMPLETE_MSG);
}


void setup() {
  /*** Initialization for stepper motor driver ***/
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  // Mode initialization (Full step)
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(STEP_PIN, LOW);
  pinMode(HOMING_SWITCH_PIN, INPUT);
  LA_Extend();
  LA_move_dir = 1;
  LA_curr_position = 0;
  LA_goal_position = 0;
  LA_curr_pulse_width = LA_ACCEL_START_PULSE_WIDTH;
  LA_goal_pulse_width = LA_ACCEL_START_PULSE_WIDTH;
  step_pin_state = LOW;
  LA_is_moving = false;
  LA_is_homing = false;
  LA_is_accelerating = false;
  LA_is_decelerating = false;
  reached_limit_switch = false;
  curr_mode = DEFAULT_MODE;
  //block_place_mode = false;
  /*** Initialization for UART ***/
  Serial.begin(115200);
  next_action_micros = 0;
  after_homing_move_cnt = 0;
}

void loop() {
  if (!LA_is_moving) {
    switch (curr_mode) {
      case DEFAULT_MODE:
        if (LA_curr_position != LA_goal_position) {
          LA_curr_pulse_width = LA_DEFAULT_PULSE_WIDTH;
          LA_Set_Dir();
          LA_is_moving = true;
        }
        break;
      case BLOCK_PLACE_MODE:
        if (LA_curr_position != LA_goal_position) {
          // Do not try to move at the goal velocity immediately if it is more than the motor can accelerate to from rest
          LA_curr_pulse_width = max(LA_goal_pulse_width,LA_ACCEL_START_PULSE_WIDTH);
          LA_Set_Dir();
          LA_is_moving = true;
        }
        break;
      case HOMING_MODE:
        LA_curr_pulse_width = LA_HOMING_PULSE_WIDTH;
        LA_is_moving = true;
        LA_is_homing = true;
        LA_Retract();
        LA_curr_position = 0;
      case ERROR_MODE:
        break;
      default: break;
    }
  }
  else if (micros() >= next_action_micros) {
    switch (curr_mode) {
      case DEFAULT_MODE:
      LA_Move();
      if (LA_curr_position == LA_goal_position) {
        digitalWrite(STEP_PIN, LOW); // Since a pulse is in progress, set the pin low before stopping
        LA_is_moving = false;
        Send_Move_Complete();
        // If the openCR gives a command to move in a different direction immediately after the previous movement is completed, do not 
          // attempt to start it immediately to avoid stalling the motor
        next_action_micros = micros() + LA_DIRECTION_CHANGE_DELAY;
      }
      else {
        // The next time when this part of the code will need to be run, unless something changes
        next_action_micros = micros() + LA_curr_pulse_width;
      }
      break;
      case BLOCK_PLACE_MODE:
        LA_Move();
        if (LA_curr_position == LA_goal_position) {
          digitalWrite(STEP_PIN, LOW); // Since a pulse is in progress, set the pin low before stopping
          LA_is_moving = false;
          LA_curr_pulse_width = LA_ACCEL_START_PULSE_WIDTH;
          Send_Move_Complete();
          // If the openCR gives a command to move in a different direction immediately after the previous movement is completed, do not 
            // attempt to start it immediately to avoid stalling the motor
          next_action_micros = micros() + LA_DIRECTION_CHANGE_DELAY;
        }
        else {
          LA_Update_Pulse_Width();
          // The next time when this part of the code will need to be run, unless something changes
          next_action_micros = micros() + LA_curr_pulse_width;
        }
        break;
      case HOMING_MODE:
        if (!reached_limit_switch) {
          if (digitalRead(HOMING_SWITCH_PIN)) {
            reached_limit_switch = true;
            LA_Extend();
            next_action_micros = micros() + LA_DIRECTION_CHANGE_DELAY;
          }
          else {
            LA_Toggle_Step_Pin();
            next_action_micros = micros() + LA_HOMING_PULSE_WIDTH;
          }
        }
        else {
          if (after_homing_move_cnt < SAFETY_BUFFER_STEPS) {
            LA_Toggle_Step_Pin();
            if (step_pin_state == HIGH) {
              after_homing_move_cnt++;
            }
            next_action_micros = micros() + LA_HOMING_PULSE_WIDTH;
          }
          else {
            // Finished homing
            after_homing_move_cnt = 0;
            LA_curr_position = 0;
            Send_Homing_Complete();
            digitalWrite(STEP_PIN, LOW);
            LA_is_moving = false;
            LA_is_homing = false;
            reached_limit_switch = false;
            curr_mode = DEFAULT_MODE;
            LA_goal_position = 0;
          }
        }
        break;
      case ERROR_MODE:
        LA_is_moving = false;
        break;
      default: break;
    }
  }
}

// Handles communication with the OpenCR
void serialEvent() {
  // Keep reading messages until the buffer is empty or an action needs to take place in the main loop
  while(Serial.available()) {
    static bool pos_recv_in_progress = false; // Flag to keep track of whether we are waiting for more bytes as part of a goal position message
    static bool vel_recv_in_progress = false; // Flag to keep track of whether we are waiting for more bytes as part of a goal velocity message
    static int inprog_value_bytes = 0; // Used for receiving values greater than 7 bits; currently only needed when receiving a goal position
    static unsigned int inprog_value = 0;
    if (pos_recv_in_progress || vel_recv_in_progress) {
      if (Serial.peek() & 0x80) { // Received a new message before all the data was received; ignore the last message and handle the new one
        //Serial.println("Too few data bytes received");
        pos_recv_in_progress = false;
        vel_recv_in_progress = false;
        inprog_value_bytes = 0;
        inprog_value = 0;
      }
      else {
        int new_bits = Serial.read();
        inprog_value |= new_bits << (7*inprog_value_bytes);
        inprog_value_bytes++;
        // If the maximum required number of data bytes have been recieved, set the new value
        if (pos_recv_in_progress && (inprog_value_bytes >= 2)) {
          LA_goal_position = inprog_value;
          pos_recv_in_progress = false;
          inprog_value_bytes = 0;
          inprog_value = 0;
        }
        if (vel_recv_in_progress && (inprog_value_bytes >= 2)) {
          LA_goal_pulse_width = inprog_value;
          vel_recv_in_progress = false;
          inprog_value_bytes = 0;
          inprog_value = 0;
        }
      }
    }
    else {
      byte recvd = Serial.read();
      switch(recvd) {
        case GOAL_POSITION_MSG: //Serial.println("Received GOAL_POSITION_MSG");
          pos_recv_in_progress = true;
          inprog_value_bytes = 0;
          break;
        case GOAL_VELOCITY_MSG: //Serial.println("Received GOAL_VELOCITY_MSG");
          vel_recv_in_progress = true;
          inprog_value_bytes = 0;
          break;
        case GET_CURR_POSITION_MSG: //Serial.println("Received GET_CURR_POSITION_MSG");
          Send_Curr_Position();
          break;
        case SET_MODE_DEFAULT_MSG: //Serial.println("Received SET_MODE_DEFAULT_MSG");
          Set_Mode(DEFAULT_MODE);
          break;
        case SET_MODE_BLOCK_PLACE_MSG: //Serial.println("Received SET_MODE_BLOCK_PLACE_MSG");
          Set_Mode(BLOCK_PLACE_MODE);
          break;
        case SET_MODE_HOMING_MSG: //Serial.println("Received SET_MODE_HOMING_MSG");
          Set_Mode(HOMING_MODE);
          break;
        default: //Serial.println("Received invalid message");
          break;
      }
    }
  }
}

void Send_Curr_Position() {
  Serial.write(SEND_CURR_POSITION_MSG);
  byte upper_byte = (LA_curr_position & 0xFF00) >> 8;
  byte lower_byte = LA_curr_position & 0xFF;
  Serial.println(lower_byte,HEX); //Serial.write(lower_byte);
  Serial.println(upper_byte,HEX); //Serial.write(upper_byte);
}

void LA_Update_Pulse_Width() {
  if (abs(LA_curr_pulse_width - LA_goal_pulse_width) <= LA_ACCEL_RATE) {
    LA_curr_pulse_width = LA_goal_pulse_width;
  }
  else {
    if (LA_curr_pulse_width > LA_goal_pulse_width) {
      LA_curr_pulse_width -= LA_ACCEL_RATE;
    }
    else if (LA_curr_pulse_width < LA_goal_pulse_width) {
      LA_curr_pulse_width += LA_ACCEL_RATE;
    }
  }
}

// Includes all the tasks that must be performed at each toggle of the step pin for default and block place modes
void LA_Move() {
  LA_Toggle_Step_Pin();
  if (step_pin_state == HIGH) {
    LA_curr_position += LA_move_dir;
  }
  if (!LA_Is_Valid_Position(LA_curr_position) || digitalRead(HOMING_SWITCH_PIN)) {
    End_of_Range_Error();
  }
}

void Set_Mode(LA_Move_Mode mode) {
  // The mode should not change while the linear actuator is moving
  if (LA_is_moving) {
    return;
  }
  curr_mode = mode;
}

void End_of_Range_Error() {
  //LA_is_moving = false;
  curr_mode = ERROR_MODE;
}

bool LA_Is_Valid_Position(unsigned int pos) {
  // Do not need to set a lower limit because the limit switch will tell if the linear actuator has retracted too far
  return (pos < (FULL_STROKE_STEPS - SAFETY_BUFFER_STEPS));
}
void LA_Toggle_Step_Pin() {
  step_pin_state = step_pin_state ? LOW : HIGH;
  digitalWrite(STEP_PIN, step_pin_state);
}
void LA_Set_Dir() {
  if (LA_goal_position >= LA_curr_position) {
    LA_Extend();
  }
  else {
    LA_Retract();
  }
}
void LA_Extend() {
  digitalWrite(DIR_PIN, HIGH);
  LA_move_dir = 1;
}
void LA_Retract() {
  digitalWrite(DIR_PIN, LOW);
  LA_move_dir = -1;
}

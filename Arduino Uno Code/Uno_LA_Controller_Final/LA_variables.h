unsigned int LA_curr_position;
unsigned int LA_goal_position;
unsigned int LA_accel_rate;
unsigned int LA_curr_pulse_width;
unsigned int LA_goal_pulse_width;
unsigned int after_homing_move_cnt;
int LA_move_dir; // 1 for forwards, -1 for reverse
bool LA_is_accelerating;
bool LA_is_decelerating;
bool step_pin_state;
bool LA_is_moving;
bool LA_is_homing;
bool reached_limit_switch;
unsigned long next_action_micros;
typedef enum {
  DEFAULT_MODE,
  BLOCK_PLACE_MODE,
  HOMING_MODE,
  ERROR_MODE
} LA_Move_Mode;
LA_Move_Mode curr_mode;

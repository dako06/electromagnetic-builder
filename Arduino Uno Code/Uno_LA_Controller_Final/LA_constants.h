// Stepper motor wires should be in this order (starting furthest from the capacitor): blue, red, black, green
#define STEP_PIN 5 // blue wire
#define DIR_PIN 4 // green wire
#define HOMING_SWITCH_PIN 7
#define STEP_SIZE_MM 0.01
#define FULL_STROKE_STEPS 10000
// Do not allow the linear actuator to get within this many steps from its limits
#define SAFETY_BUFFER_STEPS 100
//#define LA_MIN_PULSE_WIDTH 350
#define LA_MIN_PULSE_WIDTH 200
#define LA_MAX_PULSE_WIDTH 1000
#define LA_HOMING_PULSE_WIDTH 800
//#define LA_ACCEL_START_PULSE_WIDTH 625 // LA_MAX_VEL_NO_ACCEL converted to pulse width
#define LA_ACCEL_START_PULSE_WIDTH 800
#define LA_MAX_VEL_NO_ACCEL 8 // The maximum velocity that the linear actuator can begin moving to from rest, in mm/s
#define LA_DEFAULT_ACCEL_RATE 1
#define LA_DEFAULT_VEL 10
#define LA_DEFAULT_PULSE_WIDTH 625
#define LA_DIRECTION_CHANGE_DELAY 10000
#define LA_ACCEL_RATE 3

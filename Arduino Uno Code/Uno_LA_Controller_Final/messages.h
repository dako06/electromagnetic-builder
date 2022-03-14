/******************** Message definitions ********************/
/*** Incoming messages ***/
// The first byte in a message determines what information is being requested or will be sent. The upper nibble of this byte is all
  // ones. If the OpenCR is sending information, it is sent 7 bits at a time with the MSB being zero
// This message is sent in order to set the goal position for the linear actuator
#define GOAL_POSITION_MSG 0xF1
// This message is sent in order to set the goal velocity for the linear actuator in block place mode (given in terms of the microseconds per pulse) 
#define GOAL_VELOCITY_MSG 0xF2
// This message is sent in order to request the current position of the linear actuator in steps
#define GET_CURR_POSITION_MSG 0xF3
// These messages are sent to set the mode of the linear actuator
#define SET_MODE_DEFAULT_MSG 0xF4
#define SET_MODE_BLOCK_PLACE_MSG 0xF5
#define SET_MODE_HOMING_MSG 0xF6
/*** Outgoing messages ***/
// When the Uno is sending the current position of the linear actuator to the OpenCR, it sends this byte first
#define SEND_CURR_POSITION_MSG 0x01 // Not using currently
// In default or block_place mode, when the target position is reached this message is sent automatically to the OpenCR
#define SEND_MOVE_COMPLETE_MSG 0x02
// In homing mode, when the new zero position for the linear actuator is established this message is sent automatically to the OpenCR
#define SEND_HOMING_COMPLETE_MSG 0x03

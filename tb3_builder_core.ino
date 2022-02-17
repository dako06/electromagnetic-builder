#include "tb3_builder_core_config.h"
#include "RPR_manipulator_driver.hpp"

/*******************************************************************************
* Setup function
*******************************************************************************/

void setup()
{
  DEBUG_SERIAL.begin(57600);

  // Initialize ROS node handle, advertise and subscribe the topics
  nh.initNode();
  nh.getHardware()->setBaud(115200);

  nh.subscribe(cmd_vel_sub);

  // initialize subscribers for RPR manipulator and electromagnet topic's 
  nh.subscribe(la_position_sub);
  nh.subscribe(servo_position_sub);

  nh.subscribe(sound_sub);
  nh.subscribe(motor_power_sub);
  nh.subscribe(reset_sub);
  
  // builder: publisher for rpr joint states
  nh.advertise(rpr_joint_states_pub);
  nh.advertise(joint_debug_pub);

  nh.advertise(sensor_state_pub);  
  nh.advertise(version_info_pub);
  nh.advertise(imu_pub);
  nh.advertise(cmd_vel_rc100_pub);
  nh.advertise(odom_pub);
  nh.advertise(joint_states_pub);
  nh.advertise(battery_state_pub);
  nh.advertise(mag_pub);

  tf_broadcaster.init(nh);

  // Setting for Dynamixel motors
  motor_driver.init(NAME);

  // builder: intialize driver objects
  Init_Base_Servo();
  Init_Linear_Actuator();
  Init_End_Servo();

  // Setting for IMU
  sensors.init();

  // Init diagnosis
  diagnosis.init();

  // Setting for ROBOTIS RC100 remote controller and cmd_vel
  controllers.init(MAX_LINEAR_VELOCITY, MAX_ANGULAR_VELOCITY);

  // Setting for SLAM and navigation (odometry, joint states, TF)
  initOdom();

  initJointStates();

  // builder: intialize the RPR joint states message used for publishing
  initRPRJointStates();
  initJointDebug();

  // builder: joint test function
  Test_Base();
//  Test_LA();
//  Test_End();

  prev_update_time = millis();
  pinMode(LED_WORKING_CHECK, OUTPUT);
  setup_end = true;
}

/*******************************************************************************
* Loop function
*******************************************************************************/
void loop()
{
  uint32_t t = millis();
  unsigned long t_micros = micros();

  updateTime();
  updateVariable(nh.connected());
  updateTFPrefix(nh.connected());

  if ((t-tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_FREQUENCY))
  {
    updateGoalVelocity();
    motor_driver.controlMotor(WHEEL_RADIUS, WHEEL_SEPARATION, goal_velocity);
    tTime[0] = t;
    
    //TODO confirm if motor timeout is needed
    // if ((t-tTime[6]) > CONTROL_MOTOR_TIMEOUT) 
    // {
    //   motor_driver.controlMotor(WHEEL_RADIUS, WHEEL_SEPARATION, zero_velocity);
    // } 
    // else {
    //   motor_driver.controlMotor(WHEEL_RADIUS, WHEEL_SEPARATION, goal_velocity);
    // }
  }

  if ((t-tTime[1]) >= (1000 / CMD_VEL_PUBLISH_FREQUENCY))
  {
    publishCmdVelFromRC100Msg();
    tTime[1] = t;
  }

  if ((t-tTime[2]) >= (1000 / DRIVE_INFORMATION_PUBLISH_FREQUENCY))
  {
    publishSensorStateMsg();
    publishBatteryStateMsg();
    publishDriveInformation();
    tTime[2] = t;
  }

  if ((t-tTime[3]) >= (1000 / IMU_PUBLISH_FREQUENCY))
  {
    publishImuMsg();
    publishMagMsg();
    tTime[3] = t;
  }

  if ((t-tTime[4]) >= (1000 / VERSION_INFORMATION_PUBLISH_FREQUENCY))
  {
    publishVersionInfoMsg();
    tTime[4] = t;
  }

#ifdef DEBUG
  if ((t-tTime[5]) >= (1000 / DEBUG_LOG_FREQUENCY))
  {
    sendDebuglog();
    tTime[5] = t;
  }
#endif

  // If there is a move command in the queue and the arm is not currently moving,
    // start the next motion
  if (!(base_is_moving || LA_is_moving || end_is_moving)) {
    if (!Arm_Move_Queue.isEmpty()) {
      Prepare_Arm_Motion(Arm_Move_Queue.dequeue());
    }
  }

  /* builder: statement executes at 90 ms
      function call executes control of base and end servo of the RPR manipulator */ 
      // Maybe we should replace the statement on the right side of the comparison with a single integer
  if ((t-tTime[6]) >= 90) //(1000 / SERVO_CONTROL_FREQEUNCY))
  {
    servoJointControl(); 
    setRPRJointState();
    publishRPRJointState();

    tTime[6] = t;
  }

  /* builder: running at 10 micros (1000000 micros = 1s)  
      function call executes control of the linear actuator fo the RPR manipulator */
  if ((t_micros-tTimeMicros[0]) >= LA_curr_pulse_width)
  {
    LAJointControl(); // TODO jeremy: edit this function to control linear acuator every period
    tTimeMicros[0] = t_micros;
  }

  if ((t-tTime[7]) >= 50)
  {
    setJointDebugMsg();
    publishJointDebug();
    tTime[7] = t;
  }



  // Send log message after ROS connection
  sendLogMsg();

  // Receive data from RC100 
  // TODO confirm below line works instead of original impl commented out
  controllers.getRCdata(goal_velocity_from_rc100);
  // bool clicked_state = controllers.getRCdata(goal_velocity_from_rc100);
  // if (clicked_state == true)  
  //   tTime[6] = millis();

  // Check push button pressed for simple test drive
  driveTest(diagnosis.getButtonPress(3000));

  // Update the IMU unit
  sensors.updateIMU();

  // TODO
  // Update sonar data
  // sensors.updateSonar(t);

  // Start Gyro Calibration after ROS connection
  updateGyroCali(); // TODO confirm this works over below
  // updateGyroCali(nh.connected());

  // Show LED status
  diagnosis.showLedStatus(nh.connected());

  // Update Voltage
  battery_state = diagnosis.updateVoltageCheck(setup_end);

  // Call all the callbacks waiting to be called at that point in time
  nh.spinOnce();

  // Wait the serial link time to process
  waitForSerialLink(nh.connected());

} /* end software timer loop */


/************************************************************************************/
/****************** electromagnetic builder function definitions ********************/
/************************************************************************************/

/** @param linear_actuator_joint_msg - message containing coordinates for linear actuator to execute
 *  @note callback function to respond to movement commands from the remote PC  */

void LAJointCallback(const std_msgs::Float64MultiArray& linear_actuator_joint_msg)
{
  // if (is_moving == false)
  // {
  //   joint_trajectory_point = joint_trajectory_point_msg;
  //   is_moving = true;
  // }

  // TODO complete call back
//  la_goal_point = linear_actuator_joint_msg;
}

/** @param servo_msg - message containing coordinates for base and end servo to execute
 *  @note callback function to respond to movement commands from the remote PC  */

void servoJointCallback(const std_msgs::Float64MultiArray& servo_msg)
{
  // double goal_gripper_position[5] = {0.0, };
  // const double OPEN_MANIPULATOR_GRIPPER_OFFSET = -0.015f;

  // for (int index = 0; index < gripper_cnt; index++)
  //   goal_gripper_position[index] = gripper_msg.data[index] / OPEN_MANIPULATOR_GRIPPER_OFFSET;

  // manipulator_driver.writeGripperPosition(goal_gripper_position);

  // TODO complete call back
//  servo_goal_point = servo_msg;  
}

/** @note function called in software timer to handle direct actuator movement */

void LAJointControl(void)
{
  if (LA_is_moving) {
    if (LA_is_homing) {
      Homing_Callback();
    }
    else {
      Lin_Act_Move_Callback();
    }
  }
}

/** @note function called in software timer to direct base/end servo movement */

void servoJointControl(void)
{
  // base_is_moving is set true by whatever starts moving the arm, and is set
    // false inside of Base_Servo_Move_Callback
  if (base_is_moving) {
    Base_Servo_Move_Callback();
  }
  // end_is_moving is set true by whatever starts moving the arm, and is set
    // false inside of End_Servo_Move_Callback
  if (end_is_moving) {
    End_Servo_Move_Callback();
  }

}

/** @note this function intializes the joint state message for adjustment throughout execution */

void  initRPRJointStates()
{
  static char *rpr_joint_states_name[] = {"BASE_SERVO", "LINEAR_ACTUATOR", "END_SERVO"}; 
  rpr_joint_states.name = rpr_joint_states_name;
  rpr_joint_states.name_length = 3;
  rpr_joint_states.position_length = 3;
  rpr_joint_states.velocity_length = 3;
  rpr_joint_states.effort_length = 3;

  
  float rpr_joint_states_pos[RPR_JOINT_NUM] = {0.0, 0.0, 0.0};
  float rpr_joint_states_vel[RPR_JOINT_NUM] = {0.0, 0.0, 0.0};

  rpr_joint_states.position = rpr_joint_states_pos;
  rpr_joint_states.velocity = rpr_joint_states_vel;

}

/** @note this function publish the joint state message to the remote pc */

void publishRPRJointState()
{
  ros::Time stamp_now = rosNow();
  rpr_joint_states.header.stamp = stamp_now;
  rpr_joint_states_pub.publish(&rpr_joint_states);
}

/** @note this function sets the joint state message for the rpr manipulator to prepare for publishing */

void setRPRJointState()
{
 
  // prepare updates for each field
  float rpr_joint_states_pos[RPR_JOINT_NUM] = {0.0, 0.0, 0.0};
  float rpr_joint_states_vel[RPR_JOINT_NUM] = {0.0, 0.0, 0.0};
  //static float joint_states_eff[RPR_JOINT_NUM] = {0.0, 0.0};

  float base_servo_pos = static_cast<float>(base_curr_position);
  float LA_pos = static_cast<float>(LA_curr_position);
  float end_servo_pos = static_cast<float>(end_curr_position);

  // temp hard coded for test
  rpr_joint_states_pos[0] = base_servo_pos;
  rpr_joint_states_pos[1] = LA_pos;
  rpr_joint_states_pos[2] = end_servo_pos;

  rpr_joint_states_vel[0] = 0.0;
  rpr_joint_states_vel[1] = 0.0;
  rpr_joint_states_vel[2] = 0.0;

  // assign values to msg fields
  rpr_joint_states.position = rpr_joint_states_pos;
  rpr_joint_states.velocity = rpr_joint_states_vel;
  // rpr_joint_states.effort = ;

}


void initJointDebug()
{
  // intialize message to zero
  long int tmp_debug[DEBUG_LENGTH] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  joint_debug_array.data = tmp_debug;
}

void setJointDebugMsg()
{
  long int tmp_debug[DEBUG_LENGTH] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
  
  tmp_debug[0] = base_curr_position;
  tmp_debug[1] = base_goal_position;
  tmp_debug[2] = base_is_moving;
  
  tmp_debug[3] = LA_curr_position;
  tmp_debug[4] = LA_goal_position;
  tmp_debug[5] = LA_is_moving;
  
  tmp_debug[6] = end_curr_position;
  tmp_debug[7] = end_goal_position;
  tmp_debug[8] = end_is_moving;

  // assign temp values to global message 
  joint_debug_array.data = tmp_debug;

}

void publishJointDebug()
{
  ros::Time stamp_now = rosNow();
//  joint_debug_array.heBase Bader.stamp = stamp_now;
  joint_debug_pub.publish(&joint_debug_array);
}


/********* turtlebot3 waffle pi function defintions ******************/


/*******************************************************************************
* Callback function for cmd_vel msg
*******************************************************************************/
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
  goal_velocity_from_cmd[LINEAR]  = cmd_vel_msg.linear.x;
  goal_velocity_from_cmd[ANGULAR] = cmd_vel_msg.angular.z;

  goal_velocity_from_cmd[LINEAR]  = constrain(goal_velocity_from_cmd[LINEAR],  MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
  goal_velocity_from_cmd[ANGULAR] = constrain(goal_velocity_from_cmd[ANGULAR], MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
  // tTime[6] = millis(); TODO confirm this is okay being commented out
}

/*******************************************************************************
* Callback function for sound msg
*******************************************************************************/
void soundCallback(const turtlebot3_msgs::Sound& sound_msg)
{
  sensors.makeSound(sound_msg.value);
}

/*******************************************************************************
* Callback function for motor_power msg
*******************************************************************************/
void motorPowerCallback(const std_msgs::Bool& power_msg)
{
  bool dxl_power = power_msg.data;

  motor_driver.setTorque(dxl_power);
  //manipulator_driver.setTorque(dxl_power); TODO confirm this isnt needed
}

/*******************************************************************************
* Callback function for reset msg
*******************************************************************************/
void resetCallback(const std_msgs::Empty& reset_msg)
{
  char log_msg[50];

  (void)(reset_msg);

  sprintf(log_msg, "Start Calibration of Gyro");
  nh.loginfo(log_msg);

  sensors.calibrationGyro();

  sprintf(log_msg, "Calibration End");
  nh.loginfo(log_msg);

  initOdom();

  sprintf(log_msg, "Reset Odometry");
  nh.loginfo(log_msg);  
}

/*******************************************************************************
* Publish msgs (CMD Velocity data from RC100 : angular velocity, linear velocity)
*******************************************************************************/
void publishCmdVelFromRC100Msg(void)
{
  cmd_vel_rc100_msg.linear.x  = goal_velocity_from_rc100[LINEAR];
  cmd_vel_rc100_msg.angular.z = goal_velocity_from_rc100[ANGULAR];

  cmd_vel_rc100_pub.publish(&cmd_vel_rc100_msg);
}

/*******************************************************************************
* Publish msgs (IMU data: angular velocity, linear acceleration, orientation)
*******************************************************************************/
void publishImuMsg(void)
{
  imu_msg = sensors.getIMU();

  imu_msg.header.stamp    = rosNow();
  imu_msg.header.frame_id = imu_frame_id;

  imu_pub.publish(&imu_msg);
}

/*******************************************************************************
* Publish msgs (Magnetic data)
*******************************************************************************/
void publishMagMsg(void)
{
  mag_msg = sensors.getMag();

  mag_msg.header.stamp    = rosNow();
  mag_msg.header.frame_id = mag_frame_id;

  mag_pub.publish(&mag_msg);
}

/*******************************************************************************
* Publish msgs (sensor_state: bumpers, cliffs, buttons, encoders, battery)
*******************************************************************************/
void publishSensorStateMsg(void)
{
  bool dxl_comm_result = false;

  sensor_state_msg.header.stamp = rosNow();
  sensor_state_msg.battery = sensors.checkVoltage();

  dxl_comm_result = motor_driver.readEncoder(sensor_state_msg.left_encoder, sensor_state_msg.right_encoder);

  if (dxl_comm_result == true)
    updateMotorInfo(sensor_state_msg.left_encoder, sensor_state_msg.right_encoder);
  else
    return;

  sensor_state_msg.bumper = sensors.checkPushBumper();

  sensor_state_msg.cliff = sensors.getIRsensorData();

  // TODO
  // sensor_state_msg.sonar = sensors.getSonarData();

  sensor_state_msg.illumination = sensors.getIlluminationData();
  
  sensor_state_msg.button = sensors.checkPushButton();

  sensor_state_msg.torque = motor_driver.getTorque();

  sensor_state_pub.publish(&sensor_state_msg);
}

/*******************************************************************************
* Publish msgs (version info)
*******************************************************************************/
void publishVersionInfoMsg(void)
{
  version_info_msg.hardware = "0.0.0";
  version_info_msg.software = "0.0.0";
  version_info_msg.firmware = FIRMWARE_VER;

  version_info_pub.publish(&version_info_msg);
}

/*******************************************************************************
* Publish msgs (battery_state)
*******************************************************************************/
void publishBatteryStateMsg(void)
{
  battery_state_msg.header.stamp = rosNow();
  battery_state_msg.design_capacity = 1.8f; //Ah
  battery_state_msg.voltage = sensors.checkVoltage();
  battery_state_msg.percentage = (float)(battery_state_msg.voltage / 11.1f);

  if (battery_state == 0)
    battery_state_msg.present = false;
  else
    battery_state_msg.present = true;  

  battery_state_pub.publish(&battery_state_msg);
}

/*******************************************************************************
* Publish msgs (odometry, joint states, tf)
*******************************************************************************/
void publishDriveInformation(void)
{
  unsigned long time_now = millis();
  unsigned long step_time = time_now - prev_update_time;

  prev_update_time = time_now;
  ros::Time stamp_now = rosNow();

  // calculate odometry
  calcOdometry((double)(step_time * 0.001));

  // odometry
  updateOdometry();
  odom.header.stamp = stamp_now;
  odom_pub.publish(&odom);

  // odometry tf
  updateTF(odom_tf);
  odom_tf.header.stamp = stamp_now;
  tf_broadcaster.sendTransform(odom_tf);

  // joint states
  updateJointStates();
  joint_states.header.stamp = stamp_now;
  joint_states_pub.publish(&joint_states);
}

/*******************************************************************************
* Update TF Prefix
*******************************************************************************/
void updateTFPrefix(bool isConnected)
{
  static bool isChecked = false;
  char log_msg[50];

  if (isConnected)
  {
    if (isChecked == false)
    {
      nh.getParam("~tf_prefix", &get_tf_prefix);

      if (!strcmp(get_tf_prefix, ""))
      {
        sprintf(odom_header_frame_id, "odom");
        sprintf(odom_child_frame_id, "base_footprint");  

        sprintf(imu_frame_id, "imu_link");
        sprintf(mag_frame_id, "mag_link");
        sprintf(joint_state_header_frame_id, "base_link");
      }
      else
      {
        strcpy(odom_header_frame_id, get_tf_prefix);
        strcpy(odom_child_frame_id, get_tf_prefix);

        strcpy(imu_frame_id, get_tf_prefix);
        strcpy(mag_frame_id, get_tf_prefix);
        strcpy(joint_state_header_frame_id, get_tf_prefix);

        strcat(odom_header_frame_id, "/odom");
        strcat(odom_child_frame_id, "/base_footprint");

        strcat(imu_frame_id, "/imu_link");
        strcat(mag_frame_id, "/mag_link");
        strcat(joint_state_header_frame_id, "/base_link");
      }

      sprintf(log_msg, "Setup TF on Odometry [%s]", odom_header_frame_id);
      nh.loginfo(log_msg); 

      sprintf(log_msg, "Setup TF on IMU [%s]", imu_frame_id);
      nh.loginfo(log_msg); 

      sprintf(log_msg, "Setup TF on MagneticField [%s]", mag_frame_id);
      nh.loginfo(log_msg); 

      sprintf(log_msg, "Setup TF on JointState [%s]", joint_state_header_frame_id);
      nh.loginfo(log_msg); 

      isChecked = true;
    }
  }
  else
  {
    isChecked = false;
  }
}

/*******************************************************************************
* Update the odometry
*******************************************************************************/
void updateOdometry(void)
{
  odom.header.frame_id = odom_header_frame_id;
  odom.child_frame_id  = odom_child_frame_id;

  odom.pose.pose.position.x = odom_pose[0];
  odom.pose.pose.position.y = odom_pose[1];
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);

  odom.twist.twist.linear.x  = odom_vel[0];
  odom.twist.twist.angular.z = odom_vel[2];
}

/*******************************************************************************
* Update the joint states 
*******************************************************************************/
void updateJointStates(void)
{
  static float joint_states_pos[WHEEL_NUM] = {0.0, 0.0};
  static float joint_states_vel[WHEEL_NUM] = {0.0, 0.0};
  //static float joint_states_eff[WHEEL_NUM] = {0.0, 0.0};

  joint_states_pos[LEFT]  = last_rad[LEFT];
  joint_states_pos[RIGHT] = last_rad[RIGHT];

  joint_states_vel[LEFT]  = last_velocity[LEFT];
  joint_states_vel[RIGHT] = last_velocity[RIGHT];
  

  joint_states.position = joint_states_pos;
  joint_states.velocity = joint_states_vel;

}

/*******************************************************************************
* CalcUpdateulate the TF
*******************************************************************************/
void updateTF(geometry_msgs::TransformStamped& odom_tf)
{
  odom_tf.header = odom.header;
  odom_tf.child_frame_id = odom.child_frame_id;
  odom_tf.transform.translation.x = odom.pose.pose.position.x;
  odom_tf.transform.translation.y = odom.pose.pose.position.y;
  odom_tf.transform.translation.z = odom.pose.pose.position.z;
  odom_tf.transform.rotation      = odom.pose.pose.orientation;
}

/*******************************************************************************
* Update motor information
*******************************************************************************/
void updateMotorInfo(int32_t left_tick, int32_t right_tick)
{
  int32_t current_tick = 0;
  static int32_t last_tick[WHEEL_NUM] = {0, 0};
  
  if (init_encoder)
  {
    for (int index = 0; index < WHEEL_NUM; index++)
    {
      last_diff_tick[index] = 0;
      last_tick[index]      = 0;
      last_rad[index]       = 0.0;

      last_velocity[index]  = 0.0;
    }  

    last_tick[LEFT] = left_tick;
    last_tick[RIGHT] = right_tick;

    init_encoder = false;
    return;
  }

  current_tick = left_tick;

  last_diff_tick[LEFT] = current_tick - last_tick[LEFT];
  last_tick[LEFT]      = current_tick;
  last_rad[LEFT]       += TICK2RAD * (double)last_diff_tick[LEFT];

  current_tick = right_tick;

  last_diff_tick[RIGHT] = current_tick - last_tick[RIGHT];
  last_tick[RIGHT]      = current_tick;
  last_rad[RIGHT]       += TICK2RAD * (double)last_diff_tick[RIGHT];
}

/*******************************************************************************
* Calculate the odometry
*******************************************************************************/
bool calcOdometry(double diff_time)
{
  float* orientation;
  double wheel_l, wheel_r;      // rotation value of wheel [rad]
  double delta_s, theta, delta_theta;
  static double last_theta = 0.0;
  double v, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]
  double step_time;

  wheel_l = wheel_r = 0.0;
  delta_s = delta_theta = theta = 0.0;
  v = w = 0.0;
  step_time = 0.0;

  step_time = diff_time;

  if (step_time == 0)
    return false;

  wheel_l = TICK2RAD * (double)last_diff_tick[LEFT];
  wheel_r = TICK2RAD * (double)last_diff_tick[RIGHT];

  if (isnan(wheel_l))
    wheel_l = 0.0;

  if (isnan(wheel_r))
    wheel_r = 0.0;

  delta_s     = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
  // theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;  
  orientation = sensors.getOrientation();
  theta       = atan2f(orientation[1]*orientation[2] + orientation[0]*orientation[3], 
                0.5f - orientation[2]*orientation[2] - orientation[3]*orientation[3]);

  delta_theta = theta - last_theta;

  // compute odometric pose
  odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[2] += delta_theta;

  // compute odometric instantaneouse velocity

  v = delta_s / step_time;
  w = delta_theta / step_time;

  odom_vel[0] = v;
  odom_vel[1] = 0.0;
  odom_vel[2] = w;

  last_velocity[LEFT]  = wheel_l / step_time;
  last_velocity[RIGHT] = wheel_r / step_time;
  last_theta = theta;

  return true;
}


/*******************************************************************************
* Turtlebot3 test drive using push buttons
*******************************************************************************/
void driveTest(uint8_t buttons)
{
  static bool move[2] = {false, false};
  static int32_t saved_tick[2] = {0, 0};
  static double diff_encoder = 0.0;

  int32_t current_tick[2] = {0, 0};

  motor_driver.readEncoder(current_tick[LEFT], current_tick[RIGHT]);

  if (buttons & (1<<0))  
  {
    move[LINEAR] = true;
    saved_tick[RIGHT] = current_tick[RIGHT];

    diff_encoder = TEST_DISTANCE / (0.207 / 4096); // (Circumference of Wheel) / (The number of tick per revolution)
    // tTime[6] = millis(); // TODO not in openmanip, arm takes this index
  }
  else if (buttons & (1<<1))
  {
    move[ANGULAR] = true;
    saved_tick[RIGHT] = current_tick[RIGHT];

    diff_encoder = (TEST_RADIAN * TURNING_RADIUS) / (0.207 / 4096);
    // tTime[6] = millis(); TODO similar to a couple lines up
  }

  if (move[LINEAR])
  {    
    if (abs(saved_tick[RIGHT] - current_tick[RIGHT]) <= diff_encoder)
    {
      goal_velocity_from_button[LINEAR]  = 0.05;
      // tTime[6] = millis(); TODO arm uses this index
    }
    else
    {
      goal_velocity_from_button[LINEAR]  = 0.0;
      move[LINEAR] = false;
    }
  }
  else if (move[ANGULAR])
  {   
    if (abs(saved_tick[RIGHT] - current_tick[RIGHT]) <= diff_encoder)
    {
      goal_velocity_from_button[ANGULAR]= -0.7;
      // tTime[6] = millis();
    }
    else
    {
      goal_velocity_from_button[ANGULAR]  = 0.0;
      move[ANGULAR] = false;
    }
  }
}

/*******************************************************************************
* Update variable (initialization)
*******************************************************************************/
void updateVariable(bool isConnected)
{
  static bool variable_flag = false;
  
  if (isConnected)
  {
    if (variable_flag == false)
    {      
      sensors.initIMU();
      initOdom();

      variable_flag = true;
    }
  }
  else
  {
    variable_flag = false;
  }
}

/*******************************************************************************
* Wait for Serial Link
*******************************************************************************/
void waitForSerialLink(bool isConnected)
{
  static bool wait_flag = false;
  
  if (isConnected)
  {
    if (wait_flag == false)
    {      
      delay(10);

      wait_flag = true;
    }
  }
  else
  {
    wait_flag = false;
  }
}

/*******************************************************************************
* Update the base time for interpolation
*******************************************************************************/
void updateTime()
{
  current_offset = millis();
  current_time = nh.now();
}

/*******************************************************************************
* ros::Time::now() implementation
*******************************************************************************/
ros::Time rosNow()
{
  return nh.now();
}

/*******************************************************************************
* Time Interpolation function (deprecated)
*******************************************************************************/
ros::Time addMicros(ros::Time & t, uint32_t _micros)
{
  uint32_t sec, nsec;

  sec  = _micros / 1000 + t.sec;
  nsec = _micros % 1000000000 + t.nsec;

  return ros::Time(sec, nsec);
}

/*******************************************************************************
* Start Gyro Calibration
*******************************************************************************/
// void updateGyroCali(bool isConnected) // TODO this is used normal core below is for openmanip
void updateGyroCali(void)
{
  static bool isEnded = false;
  char log_msg[50];

  // (void)(isConnected); TODO not in openmanip

  if (nh.connected())
  {
    if (isEnded == false)
    {
      sprintf(log_msg, "Start Calibration of Gyro");
      nh.loginfo(log_msg);

      sensors.calibrationGyro();

      sprintf(log_msg, "Calibration End");
      nh.loginfo(log_msg);

      isEnded = true;
    }
  }
  else
  {
    isEnded = false;
  }
}

/*******************************************************************************
* Send log message
*******************************************************************************/
void sendLogMsg(void)
{
  static bool log_flag = false;
  char log_msg[100];  

  // TODO delete if okay
  // String name             = NAME;
  // String firmware_version = FIRMWARE_VER;
  // String bringup_log      = "This core(v" + firmware_version + ") is compatible with TB3 " + name;

  String firmware_version = FIRMWARE_VER;
  String bringup_log = "This core(v" + firmware_version + ") is compatible with TB3 electromagnetic-builder";

  const char* init_log_data = bringup_log.c_str();

  if (nh.connected())
  {
    if (log_flag == false)
    {      
      sprintf(log_msg, "--------------------------");
      nh.loginfo(log_msg);

      sprintf(log_msg, "Connected to OpenCR board!");
      nh.loginfo(log_msg);

      // TODO commented out in openmanip
      // sprintf(log_msg, init_log_data);
      // nh.loginfo(log_msg);

      sprintf(log_msg, "--------------------------");
      nh.loginfo(log_msg);

      log_flag = true;
    }
  }
  else
  {
    log_flag = false;
  }
}

/*******************************************************************************
* Initialization odometry data
*******************************************************************************/
void initOdom(void)
{
  init_encoder = true;

  for (int index = 0; index < 3; index++)
  {
    odom_pose[index] = 0.0;
    odom_vel[index]  = 0.0;
  }

  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;

  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  odom.pose.pose.orientation.w = 0.0;

  odom.twist.twist.linear.x  = 0.0;
  odom.twist.twist.angular.z = 0.0;
}

/*******************************************************************************
* Initialization joint states data
*******************************************************************************/
void initJointStates(void)
{
  static char *joint_states_name[] = {(char*)"wheel_left_joint", (char*)"wheel_right_joint"};
  // TODO adjus this if our arm joints are included in joint statess
  // static char *joint_states_name[] = {"wheel_left_joint", "wheel_right_joint", "joint1", "joint2", "joint3", "joint4", "gripper"};

  joint_states.header.frame_id = joint_state_header_frame_id;
  joint_states.name            = joint_states_name;

  joint_states.name_length     = WHEEL_NUM;
  joint_states.position_length = WHEEL_NUM;
  joint_states.velocity_length = WHEEL_NUM;
  joint_states.effort_length   = WHEEL_NUM;

  // TODO adjust to some variation fo this if we include the joints form the arm
  // joint_states.name_length = WHEEL_NUM + joint_cnt + gripper_cnt;
  // joint_states.position_length = WHEEL_NUM + joint_cnt + gripper_cnt;
  // joint_states.velocity_length = WHEEL_NUM + joint_cnt + gripper_cnt;
  // joint_states.effort_length   = WHEEL_NUM + joint_cnt + gripper_cnt;
}

/*******************************************************************************
* Update Goal Velocity
*******************************************************************************/
void updateGoalVelocity(void)
{
  goal_velocity[LINEAR]  = goal_velocity_from_button[LINEAR]  + goal_velocity_from_cmd[LINEAR]  + goal_velocity_from_rc100[LINEAR];
  goal_velocity[ANGULAR] = goal_velocity_from_button[ANGULAR] + goal_velocity_from_cmd[ANGULAR] + goal_velocity_from_rc100[ANGULAR];

  // TODO not in openmanip check if needed
  sensors.setLedPattern(goal_velocity[LINEAR], goal_velocity[ANGULAR]);
}

/*******************************************************************************
* map (return double)
*******************************************************************************/
// TODO used in openmanip
// double mapd(double x, double in_min, double in_max, double out_min, double out_max)
// {
//   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
// }

/*******************************************************************************
* Send Debug data
*******************************************************************************/
void sendDebuglog(void)
{
  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.println("EXTERNAL SENSORS");
  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.print("Bumper : "); DEBUG_SERIAL.println(sensors.checkPushBumper());
  DEBUG_SERIAL.print("Cliff : "); DEBUG_SERIAL.println(sensors.getIRsensorData());
  DEBUG_SERIAL.print("Sonar : "); DEBUG_SERIAL.println(sensors.getSonarData());
  DEBUG_SERIAL.print("Illumination : "); DEBUG_SERIAL.println(sensors.getIlluminationData());

  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.println("OpenCR SENSORS");
  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.print("Battery : "); DEBUG_SERIAL.println(sensors.checkVoltage());
  DEBUG_SERIAL.println("Button : " + String(sensors.checkPushButton()));

  float* quat = sensors.getOrientation();

  DEBUG_SERIAL.println("IMU : ");
  DEBUG_SERIAL.print("    w : "); DEBUG_SERIAL.println(quat[0]);
  DEBUG_SERIAL.print("    x : "); DEBUG_SERIAL.println(quat[1]);
  DEBUG_SERIAL.print("    y : "); DEBUG_SERIAL.println(quat[2]);
  DEBUG_SERIAL.print("    z : "); DEBUG_SERIAL.println(quat[3]);
  
  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.println("DYNAMIXELS");
  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.println("Torque : " + String(motor_driver.getTorque()));

  // DEBUG_SERIAL.println("---------------------------------------");
  // DEBUG_SERIAL.println("Electromagnet");
  // DEBUG_SERIAL.println("---------------------------------------");
  // TODO add state update

  // DEBUG_SERIAL.println("---------------------------------------");
  // DEBUG_SERIAL.println("");
  // DEBUG_SERIAL.println("---------------------------------------");
  // DEBUG_SERIAL.println("Torque(joint) : " + String());


  int32_t encoder[WHEEL_NUM] = {0, 0};
  motor_driver.readEncoder(encoder[LEFT], encoder[RIGHT]);
  
  DEBUG_SERIAL.println("Encoder(left) : " + String(encoder[LEFT]));
  DEBUG_SERIAL.println("Encoder(right) : " + String(encoder[RIGHT]));

  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.println("TurtleBot3");
  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.println("Odometry : ");   
  DEBUG_SERIAL.print("         x : "); DEBUG_SERIAL.println(odom_pose[0]);
  DEBUG_SERIAL.print("         y : "); DEBUG_SERIAL.println(odom_pose[1]);
  DEBUG_SERIAL.print("     theta : "); DEBUG_SERIAL.println(odom_pose[2]);
}


#ifndef TB3_BUILDER_CORE_CONFIG_H_
#define TB3_BUILDER_CORE_CONFIG_H_
#define NOETIC_SUPPORT                  // uncomment this if writing code for ROS1 Noetic

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <turtlebot3_msgs/SensorState.h>
#include <turtlebot3_msgs/Sound.h>
#include <turtlebot3_msgs/VersionInfo.h>

#include <TurtleBot3.h>

// builder: header for custom tb3 configuration
#include "tb3_builder.h"

#include <math.h>

#define FIRMWARE_VER "1.2.6"

// TODO confirm commenting this is okay
// #define CONTROL_MOTOR_TIMEOUT                  500  //ms

// builder: control frequency for RPR_manipulator joints
#define LINEAR_ACTUATOR_CONTROL_FREQEUNCY      100000   //hz using micros
#define SERVO_CONTROL_FREQEUNCY                11       //hz 

#define CONTROL_MOTOR_SPEED_FREQUENCY          30       //hz
#define IMU_PUBLISH_FREQUENCY                  200      //hz
#define CMD_VEL_PUBLISH_FREQUENCY              30       //hz
#define DRIVE_INFORMATION_PUBLISH_FREQUENCY    30       //hz
#define VERSION_INFORMATION_PUBLISH_FREQUENCY  1        //hz 
#define DEBUG_LOG_FREQUENCY                    10       //hz 


#define WHEEL_NUM                        2

#define LEFT                             0
#define RIGHT                            1

#define LINEAR                           0
#define ANGULAR                          1

#define DEG2RAD(x)                       (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                       (x * 57.2957795131)  // *180/PI

#define TICK2RAD                         0.001533981  // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f

#define TEST_DISTANCE                    0.300     // meter
#define TEST_RADIAN                      3.14      // 180 degree

// #define DEBUG                            
#define DEBUG_SERIAL                     SerialBT2


// TODO here

/**** Callback function prototypes ****/
// standard tb3
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);
void soundCallback(const turtlebot3_msgs::Sound& sound_msg);
void motorPowerCallback(const std_msgs::Bool& power_msg);
void resetCallback(const std_msgs::Empty& reset_msg);

// void jointTrajectoryPointCallback(const std_msgs::Float64MultiArray& joint_trajectory_point_msg);
// void gripperPositionCallback(const std_msgs::Float64MultiArray& pos_msg);

// builder: callback function prototypes for linear actuator and base/end servos
void LAJointCallback(const std_msgs::Float64MultiArray& LA_joint_msg);
void servoJointCallback(const std_msgs::Float64MultiArray& servo_msg);

// Function prototypes
void publishCmdVelFromRC100Msg(void);
void publishImuMsg(void);
void publishMagMsg(void);
void publishSensorStateMsg(void);
void publishVersionInfoMsg(void);
void publishBatteryStateMsg(void);
void publishDriveInformation(void);

ros::Time rosNow(void);
ros::Time addMicros(ros::Time & t, uint32_t _micros); // deprecated

void updateVariable(bool isConnected);
void updateMotorInfo(int32_t left_tick, int32_t right_tick);
void updateTime(void);
void updateOdometry(void);
void updateJoint(void);
void updateTF(geometry_msgs::TransformStamped& odom_tf);
void updateGyroCali(bool isConnected);
void updateGoalVelocity(void);
void updateTFPrefix(bool isConnected);

void initOdom(void);
void initJointStates(void);

bool calcOdometry(double diff_time);


// builder: declaration of control function for LA joint called in software timer
void LAJointControl(void); // void jointControl(void);

void sendLogMsg(void);
void waitForSerialLink(bool isConnected);

// TODO these are from openmanip stack, confirm they are needed or useful
// double mapd(double x, double in_min, double in_max, double out_min, double out_max);

/*******************************************************************************
* ROS NodeHandle
*******************************************************************************/
ros::NodeHandle nh;
ros::Time current_time;
uint32_t current_offset;

/*******************************************************************************
* ROS Parameter
*******************************************************************************/
char get_prefix[10];
char* get_tf_prefix = get_prefix;

char odom_header_frame_id[30];
char odom_child_frame_id[30];

char imu_frame_id[30];
char mag_frame_id[30];

char joint_state_header_frame_id[30];

/*******************************************************************************
* Subscriber
*******************************************************************************/
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);

ros::Subscriber<turtlebot3_msgs::Sound> sound_sub("sound", soundCallback);

ros::Subscriber<std_msgs::Bool> motor_power_sub("motor_power", motorPowerCallback);

ros::Subscriber<std_msgs::Empty> reset_sub("reset", resetCallback);


/* builder: subscribers for custom drivers 
    arg1: topic name
    arg2: callback function handle
    TODO: rename topic names
*/  
// ros::Subscriber<std_msgs::Float64MultiArray> joint_position_sub("joint_trajectory_point", jointTrajectoryPointCallback);
// ros::Subscriber<std_msgs::Float64MultiArray> gripper_position_sub("gripper_position", gripperPositionCallback);
ros::Subscriber<std_msgs::Float64MultiArray> la_position_sub("joint_trajectory_point", LAJointCallback);
ros::Subscriber<std_msgs::Float64MultiArray> servo_position_sub("gripper_position", servoJointCallback);

// TODO accompanying publishers
//   joint_trajectory_point_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("joint_trajectory_point", 10);
//   gripper_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("gripper_position", 10);

/*******************************************************************************
* Publisher
*******************************************************************************/
// Bumpers, cliffs, buttons, encoders, battery of Turtlebot3
turtlebot3_msgs::SensorState sensor_state_msg;
ros::Publisher sensor_state_pub("sensor_state", &sensor_state_msg);

// Version information of Turtlebot3
turtlebot3_msgs::VersionInfo version_info_msg;
ros::Publisher version_info_pub("firmware_version", &version_info_msg);

// IMU of Turtlebot3
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

// Command velocity of Turtlebot3 using RC100 remote controller
geometry_msgs::Twist cmd_vel_rc100_msg;
ros::Publisher cmd_vel_rc100_pub("cmd_vel_rc100", &cmd_vel_rc100_msg);

// Odometry of Turtlebot3
nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);

// Joint(Dynamixel) state of Turtlebot3
sensor_msgs::JointState joint_states;
ros::Publisher joint_states_pub("joint_states", &joint_states);

// Battey state of Turtlebot3
#if defined NOETIC_SUPPORT
sensor_msgs::BatteryStateNoetic battery_state_msg;
#else
sensor_msgs::BatteryState battery_state_msg;
#endif
ros::Publisher battery_state_pub("battery_state", &battery_state_msg);

// Magnetic field
sensor_msgs::MagneticField mag_msg;
ros::Publisher mag_pub("magnetic_field", &mag_msg);

/*******************************************************************************
* Transform Broadcaster
*******************************************************************************/
// TF of Turtlebot3
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tf_broadcaster;

/*******************************************************************************
* SoftwareTimer of Turtlebot3
*******************************************************************************/
static uint32_t tTime[10]; // TODO check that tTime doesnt overflow due to micros()

/*******************************************************************************
* Declaration for motor
*******************************************************************************/
Turtlebot3MotorDriver motor_driver;
// TODO jeremy: declare driver class objects if neccesary 

/*******************************************************************************
* Calculation for odometry
*******************************************************************************/
bool init_encoder = true;
int32_t last_diff_tick[WHEEL_NUM] = {0, 0};
double  last_rad[WHEEL_NUM]       = {0.0, 0.0};

/*******************************************************************************
* Update Joint State
*******************************************************************************/
double  last_velocity[WHEEL_NUM]  = {0.0, 0.0};

/*******************************************************************************
* Declaration for sensors
*******************************************************************************/
Turtlebot3Sensor sensors;

/*******************************************************************************
* Declaration for controllers
*******************************************************************************/
Turtlebot3Controller controllers;

// TODO make sure this is okay commented out, not included in openmanip stack
// float zero_velocity[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_button[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_cmd[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_rc100[WHEEL_NUM] = {0.0, 0.0};

/*******************************************************************************
* Declaration for diagnosis
*******************************************************************************/
Turtlebot3Diagnosis diagnosis;

/*******************************************************************************
* Declaration for SLAM and navigation
*******************************************************************************/
unsigned long prev_update_time;
float odom_pose[3];
double odom_vel[3];

/*******************************************************************************
* Declaration for Battery
*******************************************************************************/
bool setup_end        = false;
uint8_t battery_state = 0;


/*******************************************************************************
* Joint Control
*******************************************************************************/
// TODO confirm this entire section is needed or edited properly
// bool is_moving        = false; //TODO delete if unused
// std_msgs::Float64MultiArray joint_trajectory_point;
// builder: intialize float arrays used to maintain joint goal updates from callback functions
std_msgs::Float64MultiArray la_goal_point;
std_msgs::Float64MultiArray servo_goal_point;

#endif // TURTLEBOT3_BUILDER_CORE_CONFIG_H_

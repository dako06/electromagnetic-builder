// hellomynameisandreaabdulkarim

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "scan_processor.hpp"
#include "std_msgs/Float32.h"

sensor_msgs::LaserScan scanMsg;
ros::Subscriber scanSubscriber;
ros::Publisher	processed_scan_pub;

ScanProcessor scan_pro;

/* 0 degrees is directly ahead of scanner. Angle increases CW
	limit valid points to 330-359 and 0-30 degrees */


void scanCallback (sensor_msgs::LaserScan scanMessage);

int main(int argc, char **argv)
{

	std::cout << "Initializing scan processor\n"; 
	
	// initialize the ROS node
	ros::init(argc, argv, "scan_sub_cpp");
	ros::NodeHandle n;

	// set scan subscribe 
	scanSubscriber 		= n.subscribe("/scan", 10, scanCallback);
	processed_scan_pub 	= n.advertise<std_msgs::Float32>("scan_distance", 100);


	ros::spin();
}

void 
scanCallback (sensor_msgs::LaserScan scanMessage)
{
	std_msgs::Float32 dist;
	
	float sum{0};
	
	for (size_t i{0}; i <= scan_pro.max_ix; i++)
		sum += scanMessage.ranges[i];

	float mean = sum / static_cast<float>(scan_pro.max_ix);

	dist.data = mean;

	processed_scan_pub.publish(dist);

}
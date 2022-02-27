// hellomynameisandreaabdulkarim

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "scan_processor.hpp"

sensor_msgs::LaserScan scanMsg;
ros::Subscriber scanSubscriber;

ScanProcessor scan_pro;

/* 0 degrees is directly ahead of scanner. Angle increases CW
	limit valid points to 330-359 and 0-30 degrees */


void scanCallback (sensor_msgs::LaserScan scanMessage);

int main(int argc, char **argv){

	std::cout << "Initializing scan processor\n"; 
	
	//initialize the ROS node
	ros::init(argc, argv, "scan_sub_cpp");
	ros::NodeHandle n;

	// set scan subscribe 
	scanSubscriber = n.subscribe("/scan", 10, scanCallback);

	ros::spin();

}

void scanCallback (sensor_msgs::LaserScan scanMessage){
	

	std::cout << "range array size: " << scanMessage.ranges.size() << std::endl;

	for (int i{0}; i < 50 ; i++)
	{
		std::cout << "ranges[" << i << "]: " << scanMessage.ranges[i] << std::endl;
	}

	
	// cout<<"minimum range: " << getMinimumRange(scanMessage)<<endl;
    // cout<<"maximum range: " << getMaximumRange(scanMessage)<<endl;
    // cout<<"average range: " << getAverageRange(scanMessage,0,600)<<endl;
    // cout<<endl;

}
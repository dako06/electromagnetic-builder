// hellomynameisandreaabdulkarim

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "laser_scan/scan_processor.hpp"

using namespace std;

sensor_msgs::LaserScan scanMsg;
ros::Subscriber scanSubscriber;


void scanCallback (sensor_msgs::LaserScan scanMessage);

int main(int argc, char **argv){

	cout << "im in here\n"; 
	//initialize the ROS node
	ros::init(argc, argv, "scan_sub_cpp");
	ros::NodeHandle n;

	//subscribe to the laser scanner topic
	scanSubscriber = n.subscribe("/scan", 10, scanCallback);

	ros::spin();

}

void scanCallback (sensor_msgs::LaserScan scanMessage){
	scanMsg = scanMessage;

	
	
	cout << "got a message\n";
	// cout<<"minimum range: " << getMinimumRange(scanMessage)<<endl;
    // cout<<"maximum range: " << getMaximumRange(scanMessage)<<endl;
    // cout<<"average range: " << getAverageRange(scanMessage,0,600)<<endl;
    // cout<<endl;

}
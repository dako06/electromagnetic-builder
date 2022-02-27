
#ifndef SCAN_PROCESSOR_H_
#define SCAN_PROCESSOR_H_


#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"

#include "scan_utilities.h"

class ScanProcessor 
{
    public:

    // declare static functions


    private:

        // angular limits to limit forward perspective 
        std::vector <int>angular_constraints = {0, 30, 330, 359}; 




};



#endif
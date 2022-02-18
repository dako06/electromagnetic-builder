#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Int32MultiArray  

import pyexcel



class JointDebug():

    def __init__(self):

        rospy.init_node("joint_debug_node")
        self.joint_debug_sub = rospy.Subscriber("joint_debug", Int32MultiArray, self.joint_debug_callback)
        self.message = Int32MultiArray()
        self.msg_length = 9
        self.msg_field = ["BASE SERVO", "LINEAR ACTUATOR", "END SERVO"]
        self.data = list()

        try:
            pass
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")
        finally:
            # save trajectory into csv file
            np.savetxt('joint_debug.csv', np.array(self.data), fmt='%f')
        
       
    def joint_debug_callback(self, msg):
        
        self.data.append(msg.data)


if __name__ == '__main__':

    #### intialize server node and class ####
    jdebug = JointDebug()
   
    rospy.spin()
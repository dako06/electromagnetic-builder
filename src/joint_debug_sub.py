#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray  

import csv
        
class JointDebug():

    def __init__(self):

        rospy.init_node("joint_debug_node")
        self.joint_debug_sub = rospy.Subscriber("joint_debug", Int32MultiArray, self.joint_debug_callback)
        self.message = Int32MultiArray()
        self.msg_length = 9
        self.msg_field = ["BASE SERVO", "LINEAR ACTUATOR", "END SERVO"]
        
        self.values = []

    def joint_debug_callback(self, msg):
        
        self.values.apppend(msg.data) 
        



if __name__ == '__main__':

    #### intialize server node and class ####
    jdebug = JointDebug()
    rospy.spin()
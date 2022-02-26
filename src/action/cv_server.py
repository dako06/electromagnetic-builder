#!/usr/bin/env python3

import rospy
import tf

from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D
from actionlib import SimpleActionServer 

from electromagnetic_builder.msg import VisionActionFeedback
from electromagnetic_builder.msg import VisionAction
from electromagnetic_builder.msg import VisionActionResult

import numpy as np

from utilities import cv_utilities


class VisionActionServer(object):
    
    def __init__(self, name):

        #### intialize server ####
        self.action_name = name
        self.action_server = SimpleActionServer(self.action_name, VisionAction, execute_cb=self.execute_callback, auto_start = False)
        self.action_server.start()

    def execute_callback(self, goal):
        
        # intialize action messages 
        result      = VisionActionResult()
        feedback    = VisionActionFeedback()


        # check for preempt request from client 
        if self.action_server.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self.action_name)
            self.action_server.set_preempted() 

        # feedback.percent_complete = 
        self.action_server.publish_feedback(feedback) 

        rospy.loginfo('%s: Succeeded' % self.action_name)
        self.action_server.set_succeeded(result)

if __name__ == '__main__':

    #### intialize server node and class ####
    rospy.init_node('vision_action')
    nav_server = VisionActionServer(rospy.get_name())
#!/usr/bin/env python3

import rospy
from actionlib import SimpleActionServer 

from electromagnetic_builder.msg import NavigationFeedback
from electromagnetic_builder.msg import NavigationResult
from electromagnetic_builder.msg import NavigationAction

# from motion.turtlebot import Turtlebot

class NavigationActionServer(object):
    
    def __init__(self, name):


        # TODO add pub/sub
        # TODO add tb3 obj

        #### intialize server ####
        self.action_name = name
        self.action_server = SimpleActionServer(self.action_name, NavigationAction, execute_cb=self.execute_callback, auto_start = False)
        self.action_server.start()

        #### action messages ####
        self.result     = NavigationResult()
        self.feedback   = NavigationFeedback()
    
    def execute_callback(self, goal):
        
        r = rospy.Rate(10)
        # tb3 = Turtlebot()
        
        #### check for preempt request from client ####
        if self.action_server.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self.action_name)
            self.action_server.set_preempted()                  
            

        #### report feedback ####
        # rospy.loginfo('publishing feedback ...')
        # self.action_server.publish_feedback(self.feedback)      # publish feedback

        ##### execute action #####
        # tb3.run(goal)


        self.result.arrived = True


        #### return result of action ####

        # rospy.loginfo('%s: Succeeded' % self.action_name)
        self.action_server.set_succeeded(self.result)
        
if __name__ == '__main__':

    #### intialize server node and class ####

    rospy.init_node('navigation_action')
    nav_server = NavigationActionServer(rospy.get_name())
    rospy.spin()
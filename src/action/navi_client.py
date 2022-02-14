#!/usr/bin/env python3

import rospy
from actionlib import SimpleActionClient 

from electromagnetic_builder.msg import NavigationGoal
from electromagnetic_builder.msg import NavigationAction
from electromagnetic_builder.msg import NavigationResult

class NavigationClient():

    """ @note This class contains the action client for the navigation server
    """

    def __init__(self):

        # initialize client
        self.client = SimpleActionClient('navigation_action', NavigationAction)

    def requestNavigation(self, zone):

        # wait for server to prepare for goals
        self.client.wait_for_server()    

        # final_result = NavigationResult()

        # intialize goal data type
        nav_goal = NavigationGoal()

        if zone == 'blockzone':
            nav_goal.x = 1
            nav_goal.y = 1
        

        elif zone =='buildzone':
            nav_goal.x = 5
            nav_goal.y = 4

        # send goal to action server and wait for completion
        self.client.send_goal(nav_goal) 
        self.client.wait_for_result()

        # get result from server
        action_result = self.client.get_result()

        # return result to state machine
        return action_result
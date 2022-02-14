#!/usr/bin/env python3

import rospy
from actionlib import SimpleActionClient 

from electromagnetic_builder.msg import RPRManipulatorGoal
from electromagnetic_builder.msg import RPRManipulatorAction

class ManipulatorClient():

    """ @note This class contains the action client for the RPR manipulator server
    """

    def __init__(self):

        # initialize client
        self.client = SimpleActionClient('rpr_manip_action', RPRManipulatorAction)

    def requestManipulatorAction(self):

        # wait for server to prepare for goals
        self.client.wait_for_server()    

        # intialize goal data type
        manip_goal = RPRManipulatorGoal()


        # send goal to action server and wait for completion
        self.client.send_goal(manip_goal)  
        self.client.wait_for_result()

        # get result from server
        action_result = self.client.get_result()

        # return result to state machine
        return action_result
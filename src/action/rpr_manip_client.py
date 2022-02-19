#!/usr/bin/env python3

import rospy
from actionlib import SimpleActionClient 

from electromagnetic_builder.msg import RPRManipulatorGoal
from electromagnetic_builder.msg import RPRManipulatorAction

# from sensor_msgs.msg import JointState


class ManipulatorClient():

    """ @note This class contains the action client for the RPR manipulator server
    """

    def __init__(self):

        # initialize client
        rospy.init_node('rpr_client')
        self.client = SimpleActionClient('rpr_manip_action', RPRManipulatorAction)


    def requestManipulatorAction(self):

        # wait for server to prepare for goals
        self.client.wait_for_server()    

        # intialize goal data type
        manip_goal = RPRManipulatorGoal()

        manip_goal.x = 1.0
        manip_goal.y = 2.0
        manip_goal.z = 3.0

        # send goal to action server and wait for completion
        self.client.send_goal(manip_goal)  
        self.client.wait_for_result()

        # get result from server
        action_result = self.client.get_result()

        # return result to state machine
        return action_result


if __name__ == '__main__':

    #### intialize server node and class ####
    client = ManipulatorClient()

    try: 
        client.requestManipulatorAction()
    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")
    # finally:

    rospy.spin()

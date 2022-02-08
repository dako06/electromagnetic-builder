#!/usr/bin/env python3

import numpy as np

import smach
import rospy 
import sys
# import tf


# from math import atan, pi, sqrt, atan2, cos, sin
# from sensor_msgs.msg import Image
# from std_msgs.msg import Empty, String
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Twist, Pose2D

from blueprint import Blueprint


# import cv2 as cv
# import roslib



class Foreman:


    def __init__(self) -> None:

        #### ROS initialization ####

        


    
        #### Constant ####
        self.MAX_ROWS = 5
        self.MAX_COLS = 5

        #### blueprint object ####

        # file path to raw blueprint excel file
        fpath_blueprint_xlsx = '~/catkin_ws/src/foreman_controller/blueprint_raw.xlsx'

        # create blueprint object
        self.blueprint = Blueprint(fpath_blueprint_xlsx, self.MAX_ROWS, self.MAX_COLS)


"""
SMACH state machine functions
"""


class stateInitialization(smach.State):

    def __init__(self, outcomes=['build_start']):
        # state class initialization 
        
        pass

    def execute(self, userdata):
        # state execution 

        
        return 'init_complete'





class evaluateBuildStatus(smach.State):

    def __init__(self, outcomes=['build_start']):
        # state class initialization 
        
        pass

    def execute(self, userdata):
        # state execution 

        return 'build_complete'

        # return 'build_incomplete'




class navigateToBlockzone(smach.State):

    def __init__(self, outcomes=['build_complete', 'build_incomplete']):
        # state class initialization 
        
        pass

    def execute(self, userdata):
        # state execution 

        
        return 'build_complete'



def main():

    # initialize foreman state machine node
    rospy.init_node('foreman_state_machine', anonymous=True)

    # Create a SMACH state machine with state machine container outcomes
    sm = smach.StateMachine(outcomes=['system_shutdown'])

    # Open the container
    with sm:
        # Add states to the sm container
        
        # starting state
        smach.StateMachine.add('INITIALIZATION', stateInitialization(), 
                        transitions={'initialization_complete':'EVALUATE_BUILD_STATUS', })


        smach.StateMachine.add('EVALUATE_BUILD_STATUS', evaluateBuildStatus(),
                               transitions={'build_complete': 'system_shutdown',
                                    'build_incomplete':'NAVIGATE_TO_BLOCKZONE'})

        smach.StateMachine.add('NAVIGATE_TO_BLOCKZONE', navigateToBlockzone(),
                               transitions={'arrived':'ESTIMATE_BLOCK_CANDIDATE', })
                                    # 'outcome2':'outcome4'})


        smach.StateMachine.add('ESTIMATE_BLOCK_CANDIDATE', estimateBlockCandidate(),
                               transitions={'estimate_available':'POSITION_FOR_EXTRACTION', })
                                    # 'outcome2':'outcome4'})

        smach.StateMachine.add('POSITION_FOR_EXTRACTION', setExtractionPose(),
                               transitions={'extraction_pose_achieved':'BLOCK_EXTRACTION', })
                                    # 'outcome2':'outcome4'})

        smach.StateMachine.add('BLOCK_EXTRACTION', extractBlock(),
                               transitions={'block_secured':'system_shutdown', })
                                    # 'outcome2':'outcome4'})



    
    
    # execute SMACH 
    outcome = sm.execute()
    
if __name__ == '__main__':

    main()
    

    # foreman = Foreman()
    # print('Initialization of Foreman controller is complete.')
    

    # try:
    #     foreman.run()
    # except rospy.ROSInterruptException:
    #     rospy.loginfo("Action terminated.")

    # alternative to try, move call to run() function to the end of Foreman constructor call
    # try:
    #     rospy.spin()
    # except KeyboardInterrupt:
    #     print("Shutting Down")


    # class state(smach.State):

#     def __init__(self, outcomes=['outcome1', 'outcome2']):
#         # state class initialization 
        

#     def execute(self, userdata):
#         # Your state execution goes here
        
#         if :
#             return 'outcome1'
#         else: 
#             return 'outcome2'

    
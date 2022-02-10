#!/usr/bin/env python3

import numpy as np


import smach
import rospy 
import sys
# import tf
# import cv2 as cv
# import roslib


# from math import atan, pi, sqrt, atan2, cos, sin
# from sensor_msgs.msg import Image
# from std_msgs.msg import Empty, String
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Twist, Pose2D



from foreman import Foreman



# global foreman object to maintain constant features
foreman = Foreman()


"""
        SMACH state machine functions
"""


class builderSMInit(smach.State):

    """ initial state of build protocal,
            call initialization functions """

    def __init__(self):
        # state class initialization 
        smach.State.__init__(self, outcomes=['initialization_complete', 'startup_failure'])
        

    def execute(self, userdata):

        # call intialization functions

        return 'initialization_complete'



class evaluateBuildStatus(smach.State):

    """ check build progress to determine if build is complete,
            continue with build protocal while blueprint contains blocks """

    def __init__(self):
        # state class initialization 
        smach.State.__init__(self, outcomes=['build_complete', 'build_incomplete'])

    def execute(self, userdata):

        block_sum = foreman.getBlockTotal()

        if block_sum == 0:
            print('There are no blocks remaining in blueprint.\nBuild is complete.')
            return 'build_complete'
            
        else:
            
            # update current blueprint index to next block 
            foreman.setNextBlockIndex()
            print('Total blocks remaining in blueprint: %d.' % block_sum)
            return 'build_incomplete'





# class navigateToBlockzone(smach.State):

#     def __init__(self, outcomes=['build_complete', 'build_incomplete']):
#         # state class initialization 
        
#         pass

#     def execute(self, userdata):
#         # state execution 

        
#         return 'build_complete'



def main():

    # initialize foreman state machine node
    rospy.init_node('foreman_state_machine', anonymous=True)


    # Create a SMACH state machine with state machine container outcomes
    sm = smach.StateMachine(outcomes=['system_shutdown'])

    # Open the container
    with sm:

        ''' add states to the sm container '''
        
        # initial state
        smach.StateMachine.add('INIT_BUILDER', builderSMInit(), 
                        transitions={'initialization_complete':'EVALUATE_BUILD_STATUS', 
                            'startup_failure':'system_shutdown'})


        smach.StateMachine.add('EVALUATE_BUILD_STATUS', evaluateBuildStatus(),
                               transitions={'build_complete': 'system_shutdown',
                                    'build_incomplete':'system_shutdown'})

        # smach.StateMachine.add('NAVIGATE_TO_BLOCKZONE', navigateToBlockzone(),
        #                        transitions={'arrived':'ESTIMATE_BLOCK_CANDIDATE', })
        #                             # 'outcome2':'outcome4'})


        # smach.StateMachine.add('ESTIMATE_BLOCK_CANDIDATE', estimateBlockCandidate(),
        #                        transitions={'estimate_available':'POSITION_FOR_EXTRACTION', })
        #                             # 'outcome2':'outcome4'})

        # smach.StateMachine.add('POSITION_FOR_EXTRACTION', setExtractionPose(),
        #                        transitions={'extraction_pose_achieved':'BLOCK_EXTRACTION', })
        #                             # 'outcome2':'outcome4'})

        # smach.StateMachine.add('BLOCK_EXTRACTION', extractBlock(),
        #                        transitions={'block_secured':'system_shutdown', })
        #                             # 'outcome2':'outcome4'})



    
    
    # execute SMACH 
    outcome = sm.execute()
    
if __name__ == '__main__':

    main()
    

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

    
#!/usr/bin/env python3

import numpy as np
import rospy 
import smach

# import sys
# import tf
# import roslib

# from math import atan, pi, sqrt, atan2, cos, sin
# from sensor_msgs.msg import Image
# from std_msgs.msg import Empty, String
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Twist, Pose2D

from foreman import Foreman
from action import navi_client
from action import rpr_manip_client

""" global objects used throughout state machine """
foreman     = Foreman()
# nav_client  = navi_client.NavigationClient()
rpr_client  = rpr_manip_client.ManipulatorClient()


"""_____SMACH state machine functions_____"""

class initializeBuilder(smach.State):

    """ @note this is the initial state of electromangentic builder
        initialize objects used throughout build protocal then move on to main transitions """

    def __init__(self):
        # intialize state class and its outcomes   
        smach.State.__init__(self, outcomes=['initialization_complete', 'startup_failure'])

    def execute(self, userdata):
        # call intialization functions

        return 'initialization_complete'


class evaluateBuildStatus(smach.State):

    """ check build progress to determine if build is complete,
            continue with build protocal while blueprint contains blocks """

    def __init__(self):
        # intialize state class, outcomes and userdata keys passed during transitions   
        smach.State.__init__(self, outcomes=['build_complete', 'build_incomplete'],
                                    output_keys=['eval_nav_req'])

    def execute(self, userdata):

        block_sum = foreman.getBlockTotal()
        # print(block_sum)

        if block_sum == 0:
            print('There are no blocks remaining in blueprint.\nBuild is complete.')
            return 'build_complete'
            
        else:
            
            # update current blueprint index to next block 
            foreman.setNextBlockIndex()
            print("Total blocks remaining in blueprint: ", block_sum)
            
            # pass input key to navigation state
            userdata.eval_nav_req = "block_zone"

            return 'build_incomplete'


class navigateToZone(smach.State):

    def __init__(self):
        # intialize state class, outcomes and userdata keys passed during transitions   
        smach.State.__init__(self, outcomes=['arrived','navigation_failure'],
                                    input_keys=['execute_request'])

    def execute(self, userdata):

        # call action server to perform navigation based on input key
        print("executing navigation request to: %s" % userdata.execute_request)
        
        if userdata.execute_request == "block_zone" or userdata.execute_request == "build_zone":
            result = foreman.requestNavigation(userdata.execute_request)
        elif userdata.execute_request == "shimmy":
            result = foreman.requestNavigation(userdata.execute_request)
        
        if result:        
            return 'arrived'
        else: 
            return 'navigation_failure'

class extractBlock(smach.State):

    def __init__(self):
        # intialize state class and its outcomes   
        smach.State.__init__(self, outcomes=['block_secured', 'extraction_failure'])
        
    def execute(self, userdata):


        return 'block_secured'
        # return 'extraction_failure'


        


def main():

    # initialize foreman state machine node
    rospy.init_node('foreman_state_machine', anonymous=True)

    # Create a SMACH state machine with state machine container outcomes
    sm = smach.StateMachine(outcomes=['construction_complete', 'system_failure'])

    # intialize struct field to specify which zone to navigate to
    sm.userdata.nav_request = "none"

    # open the container
    with sm:

        """ add states to the sm container and specify transitions and I/O keys """
        
        # initial state
        smach.StateMachine.add('INITIALIZE_BUILDER', initializeBuilder(), 
                        transitions={'initialization_complete':'EVALUATE_BUILD_STATUS', 
                            'startup_failure':'system_failure'})

        smach.StateMachine.add('EVALUATE_BUILD_STATUS', evaluateBuildStatus(),
                               transitions={'build_incomplete':'NAVIGATE_TO_ZONE',
                                    'build_complete':'construction_complete'},
                                remapping={'eval_nav_req':'nav_request'})
        
        smach.StateMachine.add('NAVIGATE_TO_ZONE', navigateToZone(), 
                                transitions={'arrived':'EXTRACT_BLOCK',
                                            'navigation_failure':'system_failure'},
                                remapping={'execute_request':'nav_request'})


        smach.StateMachine.add('EXTRACT_BLOCK', extractBlock(),
                               transitions={'block_secured':'construction_complete', 
                                            'extraction_failure':'system_failure'})


    # execute SMACH 
    outcome = sm.execute()
    
if __name__ == '__main__':

    main()
    

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

    
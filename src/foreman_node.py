#!/usr/bin/env python3

from math import isnan
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

""" global objects used throughout state machine """
foreman = Foreman()


##########################################################
################# SMACH state functions ##################
##########################################################


class initializeBuilder(smach.State):

    """ @note this is the initial state of electromangentic builder
        initialize objects used throughout build protocal then move on to main transitions """

    def __init__(self):
        # intialize state class and its outcomes   
        smach.State.__init__(self, outcomes=['initialization_complete', 'startup_failure'])

    def execute(self, userdata):

        """ Confirm blocks were succcesfully extracted from blueprint and intial block placement is known """
        
        total_block_sum = foreman.getBlockTotal()
        # intial_block_coordinate = foreman.setNextBlockCoordinate()
        # TODO report first block placement> xy in some space or index?

        if isnan(total_block_sum) or total_block_sum==0:
            print("ERROR: Problem determining number of blocks from blueprint.")
            return 'startup_failure'
        else:
            print("Blueprint contains %d blocks to place.\nInitiating build protocal." % total_block_sum)
            return 'initialization_complete'


class evaluateBuildStatus(smach.State):

    """ This state checks if the blueprint is empty to determine if the structure is complete,
        then continues build protocal if blocks remain """

    def __init__(self):
        # intialize state class, outcomes and userdata keys passed during transitions   
        smach.State.__init__(self, outcomes=['build_complete', 'build_incomplete'],
                                    output_keys=['eval_nav_request'])

    def execute(self, userdata):

        block_count = foreman.getBlockTotal()

        if block_count == 0:
            print('There are no blocks remaining in blueprint.\nBuild is complete.')
            return 'build_complete'
            
        else:
            
            foreman.setNextBlockIndex()         # update block index to prepare for next block
            foreman.setNextBlockCoordinate()    # update current associated x,y coordinate based on block index

            # pass input key to navigation state to request movement to blockzone
            userdata.eval_nav_request = "block_zone"    

            print("Total blocks remaining in blueprint: %d" % block_count)

            return 'build_incomplete'


class navigateToZone(smach.State):

    def __init__(self):
        # intialize state class, outcomes and userdata keys passed during transitions   
        smach.State.__init__(self, outcomes=['arrived_at_blockzone', 'arrived_at_buildzone', 'navigation_failure'],
                                    input_keys=['execute_request'])

        # flag lowered when this state is reached for the first time 
        self.is_first_block = True  

    def execute(self, userdata):

        if self.is_first_block:
            self.is_first_block = False
            return 'arrived'

        # call action server to perform navigation based on input key
        if userdata.execute_request == "block_zone" or userdata.execute_request == "build_zone":
            print("Navigating to %s." % userdata.execute_request)
            result = foreman.requestNavigation(userdata.execute_request)

        if result:        
            return 'arrived'
        else: 
            return 'navigation_failure'

class positionForExtraction(smach.State):

    def __init__(self):
        # state class initialization
        smach.State.__init__(self, outcomes=['ready_for_extraction', 'extraction_positioning_failure'])
        
    def execute(self, userdata):
        # Your state execution goes here
        pass        
        # if :
        #     return 'outcome1'
        # else: 
        #     return 'outcome2'


class extractBlock(smach.State):

    def __init__(self):
        # intialize state class and its outcomes   
        smach.State.__init__(self, outcomes=['block_secured', 'extraction_failure'],
                                    input_keys=['extraction_nav_request'])
        
    def execute(self, userdata):


        return 'block_secured'
        # return 'extraction_failure'

class positionForPlacement(smach.State):

    def __init__(self):
        # state class initialization
        smach.State.__init__(self, outcomes=['ready_for_placement', 'placement_positioning_failure'])
        
    def execute(self, userdata):
        # Your state execution goes here
        pass        
        # if :
        #     return 'outcome1'
        # else: 
        #     return 'outcome2'


class placeBlock(smach.State):

    def __init__(self):
        # intialize state class and its outcomes   
        smach.State.__init__(self, outcomes=['block_placement_success', 'block_placement_failure'])
        
    def execute(self, userdata):
        pass
        # return 'block_secured'
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
                                remapping={'eval_nav_request':'nav_request'})
        
        smach.StateMachine.add('NAVIGATE_TO_ZONE', navigateToZone(), 
                                transitions={'arrived_at_blockzone':'POSITION_FOR_EXTRACTION',
                                            'arrived_at_buildzone':'POSITION_FOR_PLACEMENT',
                                            'navigation_failure':'system_failure'},
                                remapping={'execute_request':'nav_request'})

        smach.StateMachine.add('POSITION_FOR_EXTRACTION', positionForExtraction(),
                               transitions={'ready_for_extraction':'EXTRACT_BLOCK', 
                                            'extraction_positioning_failure':'system_failure'})

        smach.StateMachine.add('EXTRACT_BLOCK', extractBlock(),
                               transitions={'block_secured':'NAVIGATE_TO_ZONE', 
                                            'extraction_failure':'system_failure'},
                                remapping={'extraction_nav_request':'nav_request'})

        smach.StateMachine.add('POSITION_FOR_PLACEMENT', positionForPlacement(),
                               transitions={'ready_for_placement':'PLACE_BLOCK', 
                                            'placement_positioning_failure':'system_failure'})

        smach.StateMachine.add('PLACE_BLOCK', placeBlock(),
                               transitions={'block_placement_success':'EVALUATE_BUILD_STATUS', 
                                            'block_placement_failure':'system_failure'})


    # execute SMACH 
    outcome = sm.execute()
    
if __name__ == '__main__':

    main()
    
    # try:
    #     rospy.spin()
    # except KeyboardInterrupt:
    #     print("Shutting Down")
    # except rospy.ROSInterruptException:
    #     rospy.loginfo("Action terminated.")

# class state(smach.State):
#     def __init__(self, outcomes=['outcome1', 'outcome2']):
#         # state class initialization 
#     def execute(self, userdata):
#         # Your state execution goes here
#         if :
#             return 'outcome1'
#         else: 
#             return 'outcome2'    
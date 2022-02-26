#!/usr/bin/env python3

from math import degrees, isnan
from matplotlib.pyplot import grid
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


""" SMACH state functions """


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

        foreman.rotate(0)   # begin with rotation to 0 degrees

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
            # foreman.setNextBlockCoordinate()    # update current associated x,y coordinate based on block index

            # pass input key to navigation state to request movement to blockzone
            userdata.eval_nav_request = "block_storage"    

            print("Total blocks remaining in blueprint: %d" % block_count)

            return 'build_incomplete'


class navigateToZone(smach.State):

    def __init__(self):
        # intialize state class, outcomes and userdata keys passed during transitions   
        smach.State.__init__(self, outcomes=['at_block_storage', 'at_platform', 'navigation_failure'],
                                    input_keys=['execute_request'])


    def execute(self, userdata):

        # validate input key
        if userdata.execute_request == "platform" or userdata.execute_request == "block_storage":

            print("Navigating to %s." % userdata.execute_request)

            (p_xo, p_yo) =  foreman.grid.getCurrent()                       # get start point 
            (p_xf, p_yf) =  foreman.grid.getGoal(userdata.execute_request)  # get goal point  

            # request waypoints
            path    = foreman.grid.get_path_from_A_star((p_xo, p_yo), (p_xf, p_yf), \
                                                            foreman.grid.obstacles)
            # check that path is valid
            if (len(path) == 0):
                rospy.loginfo('A* did not succesfully find waypoints.\n')
                result = False

            else:             
                rospy.loginfo('Next path is ready.\n')
                foreman.executeTrajectory(path)   # attempt navigation

                if userdata.execute_request == "platform":        
                    return 'at_platform'

                elif userdata.execute_request == "block_storage":        
                    return 'at_block_storage'
                       
            
        else:
            print("Error: Invalid value assigned to state machine navigation key: %s\nExiting." % userdata.execute_request)
            return 'navigation_failure'


class positionForExtraction(smach.State):

    def __init__(self):
        # state class initialization
        smach.State.__init__(self, outcomes=['ready_for_extraction', 'extraction_positioning_failure'])
        
    def execute(self, userdata):

        foreman.rotate(0)   # point tb3 in general direction of blocks
        foreman.requestVisionAction('locate_block')
        

        # return 'ready_for_extraction'
        return 'extraction_positioning_failure'



class extractBlock(smach.State):

    def __init__(self):
        # intialize state class and its outcomes   
        smach.State.__init__(self, outcomes=['block_secured', 'extraction_failure'],
                                    output_keys=['extraction_nav_request'])
        
    def execute(self, userdata):


        userdata.extraction_nav_request = "platform"

        return 'extraction_failure'
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

    # initialize state machine node
    # rospy.init_node('emb_sm_node', anonymous=True)

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
                                transitions={'at_block_storage':'POSITION_FOR_EXTRACTION',
                                            'at_platform':'POSITION_FOR_PLACEMENT',
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
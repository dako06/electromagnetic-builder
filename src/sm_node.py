#!/usr/bin/env python3

from math import degrees, isnan
import numpy as np
import rospy 
import smach

# from math import atan, pi, sqrt, atan2, cos, sin
# from sensor_msgs.msg import Image
# from std_msgs.msg import Empty, String

from foreman import Foreman

""" global objects """
foreman     = Foreman()     # wrapper for lower level tasks and service requests


""" SMACH state machine operations are contained within each classes execution function """

class initializeBuilder(smach.State):

    """ initial state of electromangentic builder. Prepare objects used throughout build protocol,
        check blueprint is loaded properly, then move on to main process """

    def __init__(self):

        # intialize state class and its outcomes   
        smach.State.__init__(self, outcomes=['initialization_complete', 'startup_failure'])

    def execute(self, userdata):

        # send first gui state message
        foreman.updateGUI("block_detection")

        # read in and process raw blueprint file
        fpath_blueprint_xlsx = '~/catkin_ws/src/electromagnetic_builder/blueprint_raw.xlsx'
        
        # prepare and validate blueprint array 
        if foreman.createBlueprint(fpath_blueprint_xlsx):
            print('Blueprint array has been succesfully initialized.')
            print("Initiating build protocal")
            
            return 'initialization_complete'
        
        else:
            print("ERROR: Problem determining number of blocks from blueprint.")
            return 'startup_failure'



class evaluateBuildStatus(smach.State):

    """ This state checks if the blueprint is empty to determine if the structure is complete,
        then continues build protocal if blocks remain """

    def __init__(self):

        # intialize state class, outcomes and userdata keys passed during transitions   
        smach.State.__init__(self, outcomes=['build_complete', 'build_incomplete'])

    def execute(self, userdata):

        # update gui status
        foreman.updateGUI("block_detection")

        block_total = foreman.getBlockTotal() 

        # check progress of build based on blocks remaining
        if block_total == 0:
            print('There are no blocks remaining in blueprint.\nBuild is complete.')
            return 'build_complete'
            
        else:
   
            # update block index in blueprint array
            foreman.setBlockIndex()         

            print("Total blocks remaining in blueprint: %d" % block_total)
            print("Block position index is set.")

            return 'build_incomplete'


# class navigateToZone(smach.State):
#     def __init__(self):
#         # intialize state class, outcomes and userdata keys passed during transitions   
#         smach.State.__init__(self, outcomes=['at_block_storage', 'at_platform', 'navigation_failure'],
#                                     input_keys=['execute_request'])
#     def execute(self, userdata):

#         # validate input key
#         if userdata.execute_request == "platform" or userdata.execute_request == "block_storage":
#             print("Navigating to %s." % userdata.execute_request)
#             (p_xo, p_yo) =  foreman.grid.getCurrent()                       # get start point 
#             (p_xf, p_yf) =  foreman.grid.getGoal(userdata.execute_request)  # get goal point  

#             # request waypoints
#             path    = foreman.grid.get_path_from_A_star((p_xo, p_yo), (p_xf, p_yf), \
#                                                             foreman.grid.obstacles)
#             # check that path is valid
#             if (len(path) == 0):
#                 rospy.loginfo('A* did not succesfully find waypoints.\n')
#                 result = False
#             else:             
#                 rospy.loginfo('Next path is ready.\n')
#                 foreman.executeTrajectory(path)   # attempt navigation

#                 if userdata.execute_request == "platform":        
#                     return 'at_platform'

#                 elif userdata.execute_request == "block_storage":        
#                     return 'at_block_storage'            
#         else:
#             print("Error: Invalid value assigned to state machine navigation key: %s\nExiting." % userdata.execute_request)
#             return 'navigation_failure'

class positionForExtraction(smach.State):

    def __init__(self):
        # state class initialization
        smach.State.__init__(self, outcomes=['ready_for_extraction', 'extraction_positioning_failure'])
        
        """ 1. rotate until the block filter finds candidates
            2. identify nearest block and center it 
            3. determine its distance and confirm its within range 
            4. determine its level
            5. send estimated extraction coordinates (x,y,z) to rpr
            """

    def execute(self, userdata):

        foreman.updateGUI("block_detection")

        # find candidate blocks in the field 
        block_centered = foreman.requestVisionAction('center_candidate_block')   
        
        # return state machine transition
        if block_centered:

            print("Candidate block has been idenfitied")
            return 'ready_for_extraction'
            
        else:

            return 'extraction_positioning_failure'

        
class extractBlock(smach.State):

    def __init__(self):

        # intialize state class and its outcomes   
        smach.State.__init__(self, outcomes=['block_secured', 'extraction_failure'])
        
        """ 1. if valid coordinates are given publish rpr goal
            2. track end-effector and confirm connection with block
            3. publish commands to em
            4. publish return to home command to rpr
            5. track end-effector while confirming connection with block """
    
    def execute(self, userdata):

        # set coordinate estimate for extraction 
        # block_coordinate = foreman.setBlockCoordinate() # update current associated x,y coordinate based on block index            

        foreman.updateGUI("track_block_transport")

        # call action server to execute extraction at estimated block position
        block_extracted = foreman.requestBlockExtraction()

        if block_extracted:
            print("Block succesfully extracted")
            return 'block_secured'
        else:
            return 'extraction_failure'

# class positionForPlacement(smach.State):

#     def __init__(self):
#         # state class initialization
#         smach.State.__init__(self, outcomes=['ready_for_placement', 'placement_positioning_failure'])

#         """ 1. rotate until aligned with platform 
#             2. estimate placement coordinate based on blueprint """
        
#     def execute(self, userdata):
#         # Your state execution goes here
#         pass        
#         # if :
#         #     return 'outcome1'
#         # else: 
#         #     return 'outcome2'


# class placeBlock(smach.State):

#     def __init__(self):
#         # intialize state class and its outcomes   
#         smach.State.__init__(self, outcomes=['block_placement_success', 'block_placement_failure'])
        
#         """ 1. send goal to rpr
#             2. track rpr and conneciton to block
#             3. publish low signal to em
#             4. publish return to home to rpr """


#     def execute(self, userdata):
#         pass
#         # return 'block_secured'
#         # return 'extraction_failure'

        
def main():

    # initialize state machine node
    # rospy.init_node('emb_sm_node', anonymous=True)

    # Create a SMACH state machine with state machine container outcomes
    sm = smach.StateMachine(outcomes=['construction_complete', 'system_failure'])


    # open the container
    with sm:
 
        """ add states to the sm container and specify transitions and I/O keys """
        
        # initial state
        smach.StateMachine.add('INITIALIZE_BUILDER', initializeBuilder(), 
                        transitions={'initialization_complete':'EVALUATE_BUILD_STATUS', 
                                    'startup_failure':'system_failure'})

        smach.StateMachine.add('EVALUATE_BUILD_STATUS', evaluateBuildStatus(),
                               transitions={'build_incomplete':'POSITION_FOR_EXTRACTION',
                                            'build_complete':'construction_complete'})

        smach.StateMachine.add('POSITION_FOR_EXTRACTION', positionForExtraction(),
                               transitions={'ready_for_extraction':'EXTRACT_BLOCK', 
                                            'extraction_positioning_failure':'system_failure'})

        smach.StateMachine.add('EXTRACT_BLOCK', extractBlock(),
                               transitions={'block_secured':'construction_complete', 
                                            'extraction_failure':'system_failure'})


        # smach.StateMachine.add('NAVIGATE_TO_ZONE', navigateToZone(), 
        #                         transitions={'at_block_storage':'POSITION_FOR_EXTRACTION',
        #                                     'at_platform':'POSITION_FOR_PLACEMENT',
        #                                     'navigation_failure':'system_failure'},

        # smach.StateMachine.add('POSITION_FOR_PLACEMENT', positionForPlacement(),
        #                        transitions={'ready_for_placement':'PLACE_BLOCK', 
        #                                     'placement_positioning_failure':'system_failure'})

        # smach.StateMachine.add('PLACE_BLOCK', placeBlock(),
        #                        transitions={'block_placement_success':'EVALUATE_BUILD_STATUS', 
        #                                     'block_placement_failure':'system_failure'})


    outcome = sm.execute()  # execute SMACH 
    
if __name__ == '__main__':

    main()
    
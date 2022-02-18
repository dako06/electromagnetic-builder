#!/usr/bin/env python3

import rospy

import pandas as pd
import numpy as np
from numpy import linalg


from actionlib import SimpleActionClient 

from electromagnetic_builder.msg import NavigationGoal
from electromagnetic_builder.msg import NavigationAction
from electromagnetic_builder.msg import NavigationResult


DEBUG = 1

class Foreman:


    def __init__(self):  

        # initialize navigation client
        self.client = SimpleActionClient('navigation_action', NavigationAction)

        #### CONSTANTS ####
        self.ROW_MAX = 5
        self.COL_MAX = 5

        self.BLOCK_LENGTH = 1.5
        self.BLOCK_WIDTH = 1.5
        self.BLOCK_HEIGHT = 1.5

        # preset values for block and build zones
        self.block_zone_x = 2
        self.block_zone_y = 3
        self.build_zone_x = 5
        self.build_zone_y = 5

        self.block_center_mask =  (self.BLOCK_WIDTH/2, self.BLOCK_LENGTH/2)  

        """ iterator used to track index of current block position in blueprint array
                the starting position in the array is (0,0) and represents the physical 
                starting position in the build space which is the top left corner 
                from the origin (0, ROW_MAX*BLOCK_LEN) + offset to estimate centroid of block top
        """
        self.current_block_ix = (0,0)
        self.current_block_xy = self.setNextBlockCoordinate 

        #### blueprint object ####
        # file path to raw blueprint excel file
        self.fpath_blueprint_xlsx = '~/catkin_ws/src/electromagnetic_builder/blueprint_raw.xlsx'

        # create blueprint object
        self.blueprint = self.createBlueprint(self.fpath_blueprint_xlsx)

        if (DEBUG):
            print('Blueprint matrix has been initialized.')
            print(self.blueprint)
        
    def createBlueprint(self, fpath):

        """ construct blueprint """
        blueprint_raw = pd.read_excel(io=fpath) # read in excel file as panda data structure
        return np.array(blueprint_raw.values)   # convert to np array
 


    def getBlockTotal(self):
            total_sum = np.sum(self.blueprint)

            if (DEBUG):
                print('Total blocks: ')
                print(total_sum)

            return total_sum

    def setNextBlockIndex(self):

        # i - row index and base y-coordinate
        # j - column index and base x-coordinate
        i, j = self.current_block_ix
        
        # check if there are blocks remaining to be stacked at this position
        if self.blueprint[i, j] != 0:

            if (DEBUG):
                print("Blocks left at this index: ", self.blueprint[i, j])
            
            # dont adjust placement index
            return 

        else:

            # adjust index until nonzero element breaks loop
            while self.blueprint[i, j] == 0:

                if j+1 < self.COL_MAX:
                    j+=1  # iterate across column

                elif j+1 >= self.COL_MAX and i+1 < self.ROW_MAX:
                    i+=1    # update row 
                    j=0     # reset column

                elif j+1 >= self.COL_MAX and i+1 >= self.ROW_MAX: 
                    print('ERROR: end of Blueprint reached when setting next block index.')


            self.current_block_ix = (i,j)   # update current index
            self.setNextBlockCoordinate()   # update current x,y coordinate

            if(DEBUG):
                print("updated block index: ", self.current_block_ix)
                print("updated (x,y) coordinate: ", self.current_block_xy)

            return 


    def setNextBlockCoordinate(self):
        """ set the (x,y) coordinate corresponding to the current blueprint block index
                x - row index * block width + x offset from origin
                y - (row max - column index - 1) * block length + y offset from origin   """

        x = self.current_block_ix[1]*self.BLOCK_WIDTH + self.block_center_mask[0]
        y = (self.ROW_MAX-self.current_block_ix[0]-1)*self.BLOCK_LENGTH + self.block_center_mask[1]

        self.current_block_xy = (x, y)

    def requestNavigation(self, zone):
        """ @note request grid navigation from navigation server """

        # intialize goal and result data type
        action_result = NavigationResult()
        nav_goal = NavigationGoal()
        
        # wait for server to prepare for goals
        self.client.wait_for_server()    


        if zone == "shimmy":
            nav_goal.x = 0
            nav_goal.y = 0
            nav_goal.command = "shimmy"

        else:

            nav_goal.command = "grid_navigation"
            
            if zone == 'block_zone':
                nav_goal.x = self.block_zone_x
                nav_goal.y = self.block_zone_y
            elif zone =='build_zone':
                nav_goal.x = self.build_zone_x
                nav_goal.y = self.build_zone_y
        
        # send goal to action server and wait for completion
        self.client.send_goal(nav_goal) 
        self.client.wait_for_result()

        # get result from server
        action_result = self.client.get_result()

        # return result to state machine
        return action_result



################## EOF ################## 



        

    # camera feed
    # subscribe to raw cv image feed coming from cv node on tb3
    # self.camera_sub = rospy.Subscriber("/cv_camera/image_raw", Image, self.image_callback)

    # self.cv_bridge = CvBridge()
    # self.impro = ImageProcessor()
    # self.impipeline = ImageBuffer(10)

        #     self.BLOCK_AREA_THRESH = (2700,3100)
        # self.BLOCK_WIDTH_THRESH = (50,70)
        # self.BLOCK_HEIGHT_THRESH = (50,70)

        
    # def imgCallback(self, raw_img):

    #     # convert raw ROS image to CV image 
    #     try:
    #         cv_image = self.cv_bridge.imgmsg_to_cv2(raw_img, "bgr8")
    #     except CvBridgeError as e:
    #         print(e)

    #     self.impipeline.assignImg(cv_image.copy()) 


    # NAV
    # self.velo_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
    # self.rate = rospy.Rate(10)
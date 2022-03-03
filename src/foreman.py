#!/usr/bin/env python3

import sys


import sys
import rospy
import tf
from actionlib import SimpleActionClient 
from electromagnetic_builder.msg import RPRManipulatorGoal
from electromagnetic_builder.msg import RPRManipulatorAction
from electromagnetic_builder.msg import VisionAction
from electromagnetic_builder.msg import VisionResult
from electromagnetic_builder.msg import VisionGoal
from electromagnetic_builder.msg import GUI_State
from std_msgs.msg import Empty, Float32
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry

from math import pi, sqrt, atan2, cos, sin
import pandas as pd
import numpy as np

from utilities import controller
# from utilities import grid use in mobile application only

class Foreman:


    def __init__(self):  

        """ ROS entities """

        # initialize foreman node
        rospy.init_node('foreman_node', anonymous=True)
        rospy.loginfo("Foreman node activated: Press Ctrl + C to terminate")

        # publisher for updating GUI with current state
        self.gui_update_pub = rospy.Publisher("gui_state", GUI_State, queue_size=10)
        self.gui_state      = GUI_State(state="non_action")

        # rpr maniupulator action client
        self.rpr_client = SimpleActionClient('rpr_manip_action', RPRManipulatorAction)

        # computer vision processing client
        self.vision_client = SimpleActionClient('vision_action', VisionAction)

        # navigation items
        self.vel_pub            = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.vel                = Twist()   
        self.rate               = rospy.Rate(10)

        # odometry items
        self.odom_sub           = rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.pose               = Pose2D()

        # LDS scan processer items 
        self.processed_scan_sub = rospy.Subscriber("scan_distance", Float32, self.scan_callback)
        self.forward_distance   = np.NaN

   
        """ class members """
        self.controller = controller.Controller()   # PD controller for tracking angular velocity 
        
        # for mobile application only
        # self.grid       = grid.Grid(3, 5, 0.5)      # grid for cell decomposition navigation (0.5 m x 0.5 m) 

        """ unit converters """
        self.INCH2METER = 0.0254    # inches to meters 

        """ constants """

        # max build space is 4 rows (x) by 1 column (y) of block size
        self.BUILD_DIM = (4, 1)

        # known size of block stored in meters (length is along x-axis)
        self.BLOCK_LENGTH   = 1.5 * self.INCH2METER
        self.BLOCK_WIDTH    = 1.5 * self.INCH2METER
        self.BLOCK_HEIGHT   = 1.5 * self.INCH2METER

        # rpr manipulator offsets
        self.rpr_z_offset   = 141                       # distance of rpr z origin above ground [mm]
        self.rpr_x_offset   = 60                        # distance from rpr base servo axis of rotation to LDS
        
        # navigation 
        self.vel_ref            = 0.17  # reference velocity
        self.angular_vel_ref    = 0.1   # reference velocity
        self.time_extension     = 10    # splits T into discrete time elements 
        self.T                  = 2 

        """ member variables """

        # maintained for continuous navigation 
        self.previous_waypoint  = [0,0]
        self.previous_velocity  = [0,0]
        
        # blueprint file used to specify desired structure as scalars of height at each position
        self.blueprint  = np.array([])  

        # maintains current progress of structure being built from blueprint
        self.building   = np.array([]) 

        """ Iterator for tracking index of current block to set in blueprint array.
            The starting index in the build process is (ROW_MAX-1, 0),
             and corresponds to the furthest position physically from the robot in the build space. """
        
        self.block_ix       = (self.BUILD_DIM[0] - 1, 0)
        
        
        """ offset from measurement to center of block top (0, ROW_MAX*BLOCK_LEN) + offset to estimate centroid of block top   """
        self.block_offset   = (self.BLOCK_LENGTH/2, 0)  # assumes robot is lined up along y-axis  
        # self.block_offset   = (self.BLOCK_LENGTH/2, self.BLOCK_WIDTH/2) # used for mobile application  
        
        # estimated x,y,z coordinates of current block to place
        self.block_coordinates = (0,0,0)  


        self.cv_commands = ['locate_block']


        self.DEBUG = True   # set for verbosity during initialization
    
        if self.DEBUG:
            print("Starting block index: ", self.block_ix)



    """ blueprint functions """

    def createBlueprint(self, fpath):
        
        """ construct blueprint using excel from @param:fpath """
        
        blueprint_raw   = pd.read_excel(io=fpath)                           # read in excel file as panda data structure
        self.blueprint  = np.array(blueprint_raw.values, dtype=np.int32)    # store as np array
        
        # validate dimensions
        r, c = self.blueprint.shape

        if r > self.BUILD_DIM[0] or c > self.BUILD_DIM[1]:
            print('ERROR: Blueprint dimensions do not match dimensions of build space at initialization.')
            return False
        else:
            print('Dimensions of structure specified by blueprint: ', self.BUILD_DIM)

        # validate block count
        if self.getBlockTotal() == 0:        
            print("ERROR: Blueprint contains 0 blocks at initialization.")
            return False
        else:
            print("Blueprint contains %d blocks to set at initialization." % self.getBlockTotal())

        # intialize build progess array to size of blueprint
        self.building = np.zeros(self.blueprint.shape, dtype=np.int32)
        
        if (self.DEBUG):
                print('Blueprint specification: \n', self.blueprint)
                print('Current building progress: \n', self.building)

        return True

    def getBlockTotal(self):
        # maintain total blocks data type as int
            return np.sum(self.blueprint, dtype=np.int32)

    def setBlockIndex(self):
        """ @note update the index to point to the next block within the blueprint.
            if the current index is nonzero return since there is another block to place 
                in the same position, otherwise move along columns first then row space  """

        # i - row index and base x-coordinate
        # j - column index and base y-coordinate
        i, j = self.block_ix
        
        # return if there are blocks remaining at current position index
        if self.blueprint[i, j] != 0:
            if (self.DEBUG):
                print("Blocks remaining at current position index.", self.blueprint[i, j])

        # adjust index until nonzero element breaks loop
        while self.blueprint[i, j] == 0:

            if i-1 >= 0:
                i -= 1                      # iterate across row in reverse

            elif i-1 < 0 and j+1 < self.BUILD_DIM[1]:
                i = self.BUILD_DIM[0] - 1   # update row to max element 
                j += 1                      # update to next column 

            elif i-1 < 0 and j+1 >= self.BUILD_DIM[1]: 
                # should not be reached since this implies 
                print('ERROR: end of Blueprint reached when setting next block index.')

        # update current index
        self.block_ix = (i, j)              

        if(self.DEBUG):
            print("Updated block index: ", self.block_ix)

            

    def setBlockCoordinate(self):
        """ set the (x,y) coordinate corresponding to the current blueprint block index
                x - row index * block width + x offset from origin
                y - (row max - column index - 1) * block length + y offset from origin   """

        x = self.current_block_ix[1]*self.BLOCK_WIDTH + self.block_center_mask[0]
        y = (self.ROW_MAX-self.current_block_ix[0]-1)*self.BLOCK_LENGTH + self.block_center_mask[1]

        self.current_block_xy = (x, y)


    """ server client functions """

    def requestVisionAction(self, command):

        # validate command request
        if command not in self.cv_commands:
            rospy.logerr("Incorrect command requested by vision client")
            sys.exit("Exiting program")

        # initialize goal message 
        vis_goal    = VisionGoal()
        vis_goal.command = command          
        result      = VisionResult()
        
        self.vision_client.wait_for_server()        # wait until server is ready     
        self.vision_client.send_goal(vis_goal)      # request goal from server
        self.vision_client.wait_for_result()        # wait for execution

        # get result from server 
        result = self.vision_client.get_result() 

        # check boolean field of returned result message  
        if result.execution_status:
            
            print('Block candiate succesfully found')
            print('Objects centroid: %f %f' % (result.centroid_x, result.centroid_y))
            print('pixel area: ', result.pixel_area)
            # set next block coordinates and orientation for adjustment to block

        # return result to state machine
        return result.execution_status


    def requestBlockExtraction(self):

        # wait for server to prepare for goals
        self.rpr_client.wait_for_server()    

        # preprocess goal for rpr constraints (adjust for axis offset and convert to mm)
        manip_goal = RPRManipulatorGoal()        
        manip_goal.x = self.forward_distance * 1000.0 - self.rpr_x_offset 
        manip_goal.y = 0.0                          
        manip_goal.z = (self.BLOCK_HEIGHT * 1000.0) - self.rpr_z_offset

        # send goal to action server and wait for completion
        self.rpr_client.send_goal(manip_goal)  
        self.rpr_client.wait_for_result()

        # get result from server
        action_result = self.rpr_client.get_result()

        # return result to state machine
        return action_result


    """ navigation functions """

    def executeTrajectory(self, path):
        """ @param path: list of waypoints 
            @note traverse a path of waypoints using 3rd order polynomial time-scaling """

        path.append(path[-1])

        T = 3   # desired period to complete each segment

        # set P and D coeffecients of PD controller 
        self.controller.setPD(5,0) 

        # iterate over waypoints 
        for i in range(len(path)-1):


            # percent_complete = i/waypoint_cnt 


            # update waypoints for this segment
            current_waypoint    = path[i]       # end destination of segment
            next_waypoint       = path[i+1]     # used to smoothen transition in preparation of upcoming waypoint
                               
            # starting pose constraints
            Px_start    = self.previous_waypoint[0]     # sets starting x position 
            Py_start    = self.previous_waypoint[1]     # sets starting y position 
            Px_end      = current_waypoint[0]           # sets final x position 
            Py_end      = current_waypoint[1]           # sets final y position 

            Vx_start    = self.previous_velocity[0]     # inital velocity constraints x component
            Vy_start    = self.previous_velocity[1]     # inital velocity constraints y component
        
            # final pose constraints 
            dx          = next_waypoint[0] - Px_start   # distance in x from previous to next waypoint 
            dy          = next_waypoint[1] - Py_start   # distance in y from previous to next waypoint
            theta_f     = atan2(dy, dx)                 # angle between previous and next waypoint
            Vx_end      = self.vel_ref*cos(theta_f)     # final velocity constraint on x component
            Vy_end      = self.vel_ref*sin(theta_f)     # final velocity constraint on y component

            # get 3rd order polynomial coefficients of x and y axis for this transition
            coeff_x = self.polynomial_time_scaling_3rd_order(Px_start, Vx_start, Px_end, Vx_end, T)
            coeff_y = self.polynomial_time_scaling_3rd_order(Py_start, Vy_start, Py_end, Vy_end, T)

            # final constraints for waypoint
            # if DEBUG:
                # print("dx & dy [next-prev]: %f, %f" % (dx, dy))
                # print("estimated angle from dx and dy", theta_f)
                # print("vxf, vyf: %f, %f" % (Vx_end, Vy_end))  


            # iterate over time samples 
            for i in range(T*10):

                t = i*0.1   # update time slice 

                # print("\niteration, time sample: %d, %f" % (i, t))


                # update forward velocity 
                v_x                 = np.dot(coeff_x, [3*t**2, 2*t, 1, 0])      # velocity component along x-axis 
                v_y                 = np.dot(coeff_y, [3*t**2, 2*t, 1, 0])      # velocity component along y-axis
                theta               = atan2(v_y, v_x)   # resulting angle
                self.vel.linear.x   = sqrt(v_x**2 + v_y**2)                     # update linear velocity

                # adjust theta if discontinuity occurs
                if theta <= 0 and self.pose.theta >= 0 and self.pose.theta - theta >= pi:
                    theta += 2*pi
                elif theta > 0 and self.pose.theta < 0 and theta - self.pose.theta > pi:  
                    theta -= 2*pi

                # update controller to track theta and publish velocities 
                self.controller.setPoint(theta)                                 # set desired angle in controller      
                wz = self.controller.update(self.pose.theta)

                # print("i:\n\t theta, pose.theta, wz: %f, %f, %f" % (theta, self.pose.theta, wz))

                self.vel.angular.z = wz         # update angular velocity to be Kp*error where error = theta - self.pose.theta
                self.vel_pub.publish(self.vel)  # publish velocties
                self.rate.sleep()               # sleep until next period


            # update previous velocities and positions with current to prepare for next waypoint
            self.previous_waypoint[0] = Px_end
            self.previous_waypoint[1] = Py_end
            self.previous_velocity[0] = v_x
            self.previous_velocity[1] = v_y


        # set velocity to zero
        self.vel.linear.x   = 0
        self.vel.angular.z  = 0
        self.vel_pub.publish(self.vel)
        self.rate.sleep()
        # rospy.loginfo('%s: Waypoints succesfully found.\nThere are %d waypoints to traverse.' % (self.action_name, waypoint_cnt))


    def polynomial_time_scaling_3rd_order(self, p_start, v_start, p_end, v_end, T):
        """ @param p_start, p_end:  starting and final position for component
            @param v_start, v_end:  starting and final velocity for component
            @param T:               time in which to execute curve [0,T] [s]   
            @return a:              polynomial coefficients     """

        poly    = np.array([[0, 0, 0, 1], [T**3, T**2, T, 1], [0, 0, 1, 0], [3*T**2, 2*T, 1, 0]])
        x       = np.array([p_start, p_end, v_start, v_end])
        a       = np.dot(np.linalg.inv(poly), x)
        
        return a # a3, a2, a1, a0

    def rotate(self, goal_coordinate):

        for i in range(3):
            self.vel.angular.z = -0.05 
            self.vel_pub.publish(self.vel)
            self.rate.sleep()

        # get desired angle between cells
        # dx      =  goal_coordinate[0] - self.previous_waypoint[0]   
        # dy      =  goal_coordinate[1] - self.previous_waypoint[1]
        # theta   = atan2(dy, dx) 

        # correct theta and set controller to track angle
        # self.controller.setPD(5,0) 
        # theta = nav_utilities.theta_correction(theta)
        # self.controller.setPoint(theta)

        # # track until threshold is reached       
        # while (self.pose.theta - theta) >= 0.1:

        #     # update controller to track theta and publish velocities 
        #     self.vel.angular.z = self.controller.update(self.pose.theta) 
        #     self.vel_pub.publish(self.vel)
        #     self.rate.sleep()
        
        self.vel.angular.z = 0
        self.vel_pub.publish(self.vel)
        self.rate.sleep()

    def alignWithBlock(self, T):
        """ @param T an orientation to align with w.r.t. to some distance offset """ 
        pass


    def getBlockOrientation(self): pass


    def updateGUI(self, state_update):
        
        self.gui_state.state = state_update

        for i in range(10):
            self.gui_update_pub.publish(self.gui_state)
            self.rate.sleep()

    """ ROS callback functions """

    def odom_callback(self, msg):
        # get pose = (x, y, theta) from odometry topic
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

    def scan_callback(self, msg):
        self.forward_distance = msg.data


""" main function for testing only """
if __name__ == '__main__':
    
    foreman = Foreman()
    fpath_blueprint_xlsx = '~/catkin_ws/src/electromagnetic_builder/blueprint_raw.xlsx'
    foreman.createBlueprint(fpath_blueprint_xlsx)
    total_block_sum = foreman.getBlockTotal()
    
    block_coordinate = foreman.setNextBlockCoordinate()



    # Astar and cell decomposition navigation test
    # start       = foreman.grid.getCurrent()
    # # goal        = foreman.grid.getGoal("block_storage")
    # goal        = foreman.grid.getGoal("platform")
    # print("start: ", start)
    # print("goal: ", goal)
    # # print("obstacles: ", foreman.grid.obstacles)
    # waypoints   = foreman.grid.get_path_from_A_star(start, goal, foreman.grid.obstacles)
    # print("waypoints:", waypoints)
    # result = foreman.executeTrajectory(waypoints)



#!/usr/bin/env python3

import rospy
import tf


from math import pi, sqrt, atan2, cos, sin
import pandas as pd
import numpy as np


from actionlib import SimpleActionClient 

from electromagnetic_builder.msg import RPRManipulatorGoal
from electromagnetic_builder.msg import RPRManipulatorAction

from electromagnetic_builder.msg import VisionAction
from electromagnetic_builder.msg import VisionResult
from electromagnetic_builder.msg import VisionGoal

from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry


from utilities import grid, controller, nav_utilities


DEBUG = 1

class Foreman:


    def __init__(self):  

        """ ROS entities """

        rospy.init_node('foreman_node', anonymous=True)
        rospy.loginfo("Starting message: Press Ctrl + C to terminate")

        # intialize rpr maniupulator client
        # self.rpr_client = SimpleActionClient('rpr_manip_action', RPRManipulatorAction)

        self.vision_client = SimpleActionClient('vision_action', VisionAction)

        # navigation 
        self.vel                = Twist()   
        self.vel_pub            = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.rate               = rospy.Rate(10)

        # odometry
        self.pose               = Pose2D()
        self.odom_sub           = rospy.Subscriber("odom", Odometry, self.odom_callback)

        # reset odometry to zero
        # self.reset_pub = rospy.Publisher("mobile_base/commands/reset_odometry", Empty, queue_size=10)
        # for i in range(10):
        #     self.reset_pub.publish(Empty())
        #     self.rate.sleep()


        """ class members """
        self.grid       = grid.Grid(3, 5, 0.5)
        self.controller = controller.Controller()


        """ constants """

        # TODO move to grid
        self.ROW_MAX        = 4
        self.COL_MAX        = 4
        self.INCH2METER         = 0.0254                # inches to meters 
        self.BLOCK_LENGTH   = 1.5 * self.INCH2METER
        self.BLOCK_WIDTH    = 1.5 * self.INCH2METER
        self.BLOCK_HEIGHT   = 1.5 * self.INCH2METER

        
        # navigation 
        self.vel_ref            = 0.17   # reference velocity
        self.time_extension     = 10    # splits T into discrete time elements 
        self.T                  = 2 


        """ member variables """

        # navigation 
        self.previous_waypoint  = [0,0]
        self.previous_velocity  = [0,0]
        self.trajectory         = list()
        
        # Blueprint

        # use file path to raw blueprint excel file and create blueprint as np array
        self.fpath_blueprint_xlsx   = '~/catkin_ws/src/electromagnetic_builder/blueprint_raw.xlsx'
        self.blueprint              = self.createBlueprint(self.fpath_blueprint_xlsx)

        """ iterator used to track index of current block position in blueprint array
                the starting position in the array is (0,0) and represents the physical 
                starting position in the build space which is the top left corner 
                from the origin (0, ROW_MAX*BLOCK_LEN) + offset to estimate centroid of block top
        """

        self.current_block_ix = (0,0)
        self.block_center_mask =  (self.BLOCK_WIDTH/2, self.BLOCK_LENGTH/2)  
        self.current_block_xy = self.setNextBlockCoordinate() 


        if (DEBUG):
            print('Blueprint matrix has been initialized.')
            print(self.blueprint)

            # try:
            #     self.run()
            # except rospy.ROSInterruptException:
            #     rospy.loginfo("Action terminated.")
            # finally:
            #     pass
            # save trajectory into csv file
            # np.savetxt('trajectory.csv', np.array(self.trajectory), fmt='%f', delimiter=',')

        self.cv_commands = ['locate_block_candidate']
        # rospy.spin()

    """ blueprint functions """


    def createBlueprint(self, fpath):
        """ construct blueprint """
        blueprint_raw = pd.read_excel(io=fpath) # read in excel file as panda data structure
        return np.array(blueprint_raw.values)   # convert to np array
 

    def getBlockTotal(self):
            return np.sum(self.blueprint)

    def setNextBlockIndex(self):
        """ @note update the index to point to the next block within the blueprint.
            if the current index is nonzero return since there is another block to place 
                in the same position, otherwise move along columns first then row space  """

        # i - row index and base y-coordinate
        # j - column index and base x-coordinate
        i, j = self.current_block_ix
        
        # check if there are blocks remaining to be stacked at this position
        if self.blueprint[i, j] != 0:
            # dont adjust placement index

            if (DEBUG):
                print("Blocks left at this index: ", self.blueprint[i, j])
            
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


    """ server client functions """

    def requestVisionAction(self, command):

        # confirm valid command is requested
        if command not in self.cv_commands:
            print("Error: Incorrect Action command given for vision server")
            return False
            
        vis_goal    = VisionGoal(command=command)          # intialize goal data type
        result  = VisionResult()
        
        self.vision_client.wait_for_server()        # wait for server to prepare for goals    
        self.vision_client.send_goal(vis_goal)      # request goal from server
        self.vision_client.wait_for_result()        # wait for execution

        # get result from server [is_found, level]
        result = self.vision_client.get_result() 

        if result.is_found:

            print('Block candiate succesfully found')
            print('Objects centroid: %f %f' % (result.centroid_x, result.centroid_y))
            print('pixel area: ', result.pixel_area)
            # set next block coordinates and orientation for adjustment to block



        # return result to state machine
        return result.is_found


    def requestManipulatorAction(self):

        # wait for server to prepare for goals
        self.rpr_client.wait_for_server()    

        # intialize goal data type
        manip_goal = RPRManipulatorGoal()


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

        # get desired angle between cells
        dx      =  goal_coordinate[0] - self.previous_waypoint[0]   
        dy      =  goal_coordinate[1] - self.previous_waypoint[1]
        theta   = atan2(dy, dx) 

        # correct theta and set controller to track angle
        self.controller.setPD(5,0) 
        theta = nav_utilities.theta_correction(theta)
        self.controller.setPoint(theta)

        # track until threshold is reached       
        while (self.pose.theta - theta) >= 0.1:

            # update controller to track theta and publish velocities 
            self.vel.angular.z = self.controller.update(self.pose.theta) 
            self.vel_pub.publish(self.vel)
            self.rate.sleep()
        
        self.vel.angular.z = 0
        self.vel_pub.publish(self.vel)
        self.rate.sleep()

    def alignWithBlock(self, T):
        """ @param T an orientation to align with w.r.t. to some distance offset """ 
        pass


    def getBlockOrientation(self): pass


    """ ROS callback functions """

    def odom_callback(self, msg):
        # get pose = (x, y, theta) from odometry topic
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y



# """ temp main function for testing """
# if __name__ == '__main__':
    
    # foreman = Foreman()

    # start       = foreman.grid.getCurrent()
    # # goal        = foreman.grid.getGoal("block_storage")
    # goal        = foreman.grid.getGoal("platform")

    # print("start: ", start)
    # print("goal: ", goal)
    # # print("obstacles: ", foreman.grid.obstacles)

    
    # waypoints   = foreman.grid.get_path_from_A_star(start, goal, foreman.grid.obstacles)

    # print("waypoints:", waypoints)

    # result = foreman.executeTrajectory(waypoints)



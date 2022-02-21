#!/usr/bin/env python3

# ROS imports
import rospy
import tf
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D

# imports for action server 
from actionlib import SimpleActionServer 
from electromagnetic_builder.msg import NavigationFeedback
from electromagnetic_builder.msg import NavigationResult
from electromagnetic_builder.msg import NavigationAction

# import supporting libraries
from math import pi, sqrt, atan2, cos, sin
import numpy as np

# import supplemental classes
from controller import Controller

class NavigationActionServer(object):
    
    def __init__(self, name):

        #### intialize server ####
        self.action_name = name
        self.action_server = SimpleActionServer(self.action_name, NavigationAction, execute_cb=self.execute_callback, auto_start = False)
        self.action_server.start()

        # set up input navi publisher and prepare its message 
        self.vel = Twist()   
        self.vel_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
        self.rate = rospy.Rate(10)

        # initialize controller object
        self.controller = Controller()

        # reset odometry to zero
        # self.reset_pub = rospy.Publisher("mobile_base/commands/reset_odometry", Empty, queue_size=10)
        # for i in range(10):
        #     self.reset_pub.publish(Empty())
        #     self.rate.sleep()

        # subscribe to odometry and prepare message storage 
        self.pose = Pose2D()
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)

        # intialize variables used for grid navigation
        self.vel_ref = 0.3
        self.previous_waypoint = [0,0]
        self.previous_velocity = [0,0]
        self.trajectory = list()
        self.obstacles = []             # list of tuples encoding obstacles in grid
        # TODO init and have a way to update obstacles if needed

        
    
    def execute_callback(self, goal):
        
        # intialize action messages 
        result      = NavigationResult()
        feedback    = NavigationFeedback()
                      
            
        ##### execute action #####
    
        # prepare start and goal as a tuple representing cells in grid
        start_cell  = (0,0) # TODO have a way to retrieve start based on current position
        goal_cell   = (goal.x, goal.y)

        # get way points using A*
        waypoints       = self.get_path_from_A_star(start_cell, goal_cell, self.obstacles)
        waypoint_cnt    = len(waypoints)

        # confirm waypoints is valid for movement
        if waypoint_cnt == 0:
        
            rospy.loginfo('%s: Waypoints were not succesfully found.\nExiting server.' % self.action_name)
            result.arrived = False
            self.action_server.set_aborted()
        
        else:
            waypoints.append(waypoints[-1])
            rospy.loginfo('%s: Waypoints succesfully found.\nThere are %d waypoints to traverse.' % (self.action_name, waypoint_cnt))

	
        for i in range(waypoint_cnt-1):

            # check for preempt request from client 
            if self.action_server.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self.action_name)
                self.action_server.set_preempted() 
            
            else:

                feedback.percent_complete = i/waypoint_cnt 
                self.action_server.publish_feedback(feedback)       # publish feedback of percent of way points complete
                self.move_to_point(waypoints[i], waypoints[i+1])    # move to next way point


        # confirm action is successful then return result 
        result.arrived = True
        rospy.loginfo('%s: Succeeded' % self.action_name)
        self.action_server.set_succeeded(result)
        

    ######################################################################################
    ################################## GRID NAVIGATION ###################################
    ######################################################################################

    def get_path_from_A_star(self, start, goal, obstacles):
        """
        @param start        - integer 2-tuple of the current grid, e.g., (0, 0)
        @param goal         - integer 2-tuple of the goal grid, e.g., (5, 1)
        @param obstacles    - a list of grids marked as obstacles, e.g., [(2, -1), (2, 0), ...]
        @return path        - a list of grids connecting start to goal, e.g., [(1, 0), (1, 1), ...]
                                note that the path should contain the goal but not the start
                                e.g. path from (0, 0) to (2, 2) should be [(1, 0), (1, 1), (2, 1), (2, 2)] 
        @note               - implementation assumes a grid cell size of 0.5m to complement size of turtlebot """

        # initialize data structures
        open_list = [start]     # nodes to explore
        past_cost = {start:0}   # costs of explored nodes
        closed_list = []        # nodes explored
        cand_cost = {}          # costs for candidates/neighbors
        parent = {}             # dictionary where the key is candidate and value is current position
        path = []               # path to be outputed

        # iterate over nodes remaining to explore
        while open_list: 
            
            current = open_list.pop(0)   # assign the first node to explore as current 
            closed_list.append(current)  # list current node (first node) as explored

            # check if goal is reached
            if current == goal:  
                
                node = goal

                # reconstruct path from final node using parent dict
                while node != start:
                    path.insert(0,node)
                    node = parent[node]
                return path

            # for each neighbor around current node (connectivity determines neighbors)
            for nbr in self.neighbors(current): 

                # check if neighbor has been explored or is an obstacle
                if nbr not in closed_list and nbr not in obstacles: 
                    new_cost = past_cost[current] + 1   # cost to move in direction
                    
                    # check if neighbor has a cost value assigned to it or if is greater than new cost
                    if nbr not in past_cost or new_cost < past_cost[nbr]: 
                        past_cost[nbr] = new_cost   # assign the cost of the neighbor to dictionary
                        parent[nbr]=current         # assign the current node to the neighbor as its 'parent'
                        open_list.append(nbr)       # append this neighbor to points to be explored

            # sort based on cost
            open_list.sort(key = lambda x: past_cost[x] + self.heuristic_distance(x,goal)) 

        # return empty list if failed
        return []


    def move_to_point(self, current_waypoint, next_waypoint): 
        """ @note generate polynomial trajectory and move to current_waypoint next_waypoint is to 
            help determine the velocity to pass current_waypoint
            @param current_waypoint
            @param next_waypoint
        """
        
        self.controller.setPD(5,0) # set P and D coeffecients of PD controller 
        T = 2                   # time parametarize path with t in [0, T]

        # boundary conditions
        # position boundary
        Px_start = self.previous_waypoint[0] #sets starting x position to prev waypoint
        Py_start = self.previous_waypoint[1] #sets starting y position to prev waypoint
        Px_end = current_waypoint[0] #sets desired x position to current waypoint
        Py_end = current_waypoint[1] #sets desired y position to current waypoint
        
        #velocity boundary
        Vx_start = self.previous_velocity[0] #inital velocity constraints x component
        Vy_start = self.previous_velocity[1] #inital velocity constraints y component
        
        diff_x = next_waypoint[0] - self.previous_waypoint[0] #difference between previous and next waypoint of x
        diff_y = next_waypoint[1] - self.previous_waypoint[1] #diff. between previous and next waypoint for y

        angle = atan2(diff_y,diff_x) # angle between previous and next waypoint
        Vx_end = self.vel_ref*cos(angle) #final velocity constraints x component
        Vy_end = self.vel_ref*sin(angle) #final velocity constraints y component
 
        #3rd polynomial coefficients for x and y axis for necessary trajectory
        coeff_x = self.polynomial_time_scaling_3rd_order(Px_start, Vx_start, Px_end, Vx_end, T)
        coeff_y = self.polynomial_time_scaling_3rd_order(Py_start, Vy_start, Py_end, Vy_end, T)
        

        for i in range(T*10):
            #time scaling by 0.1
            t = i*0.1

            #desired velocity at time t
            v_x = np.dot(coeff_x,[3*t**2,2*t,1,0]) #calculate x component of v
            v_y = np.dot(coeff_y,[3*t**2,2*t,1,0]) #calculate y component of v
            self.vel.linear.x = sqrt(v_x**2+v_y**2)
       
            #use PD controller to set angle
            angle = atan2(v_y, v_x) #calculate desired angle

            #adjusting for negative angles
            if angle <= 0 and self.pose.theta >= 0 and self.pose.theta - angle >= pi:
                angle += 2*pi
            elif angle > 0 and self.pose.theta < 0 and angle - self.pose.theta > pi:  
                angle -= 2*pi
            else:
                pass    

            self.controller.setPoint(angle)    # set desired angle      
            self.vel.angular.z = self.controller.update(self.pose.theta) #update angular velocity to be Kp*error where error = angle - self.pose.theta)
            self.vel_pub.publish(self.vel)
            self.rate.sleep()

        #update previous velocities and positions with current v and positions
        self.previous_waypoint[0] = Px_end
        self.previous_waypoint[1] = Py_end
        self.previous_velocity[0] = v_x
        self.previous_velocity[1] = v_y


    def polynomial_time_scaling_3rd_order(self, p_start, v_start, p_end, v_end, T):
        # input: p,v: position and velocity of start/end point
        #        T: the desired time to complete this segment of trajectory (in second)
        # output: the coefficients of this polynomial
        poly = np.array([[0,0,0,1],[T**3,T**2,T,1],[0,0,1,0],[3*T**2,2**T,1,0]])
        x = np.array([p_start, p_end,v_start, v_end])
        a = np.dot(np.linalg.inv(poly),x)
        return a #a3, a2, a1, a0

    def neighbors(self, current):
        # define the list of 4-connectivity neighbors
        neighbors = [(1, 0), (-1, 0), (0, 1), (0, -1) ,]  # right, left, up, down
        return [ (current[0] +nbr[0], current[1] +nbr[1]) for nbr in neighbors ]

    def heuristic_distance(self, candidate, goal):
        return sqrt((candidate[0] - goal[0])**2 + (candidate[1] - goal[1])**2)


    def odom_callback(self, msg):
        # get pose = (x, y, theta) from odometry topic
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y



if __name__ == '__main__':

    #### intialize server node and class ####
    rospy.init_node('navigation_action')
    nav_server = NavigationActionServer(rospy.get_name())
    rospy.spin()
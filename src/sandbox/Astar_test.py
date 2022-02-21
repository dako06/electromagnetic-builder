import numpy as np 

class Astar():

    def __init__(self, start, obstacles):
        self.start = start
        self.obsacles = obstacles

    
    def run(self, goal):


            start = self.start 
            obstacles = self.obsacles

            waypoints = self.get_path_from_A_star(start, goal, obstacles)
            waypoints.append(waypoints[-1])
            print(waypoints)
        
            for i in range(len(waypoints)-1):
                self.move_to_point(waypoints[i], waypoints[i+1])

    def get_path_from_A_star(self, start, goal, obstacles):
        """
        @param start        - integer 2-tuple of the current grid, e.g., (0, 0)
        @param goal         - integer 2-tuple  of the goal grid, e.g., (5, 1)
        @param obstacles    - a list of grids marked as obstacles, e.g., [(2, -1), (2, 0), ...]
        @return path        - a list of grids connecting start to goal, e.g., [(1, 0), (1, 1), ...]
                                note that the path should contain the goal but not the start
                                e.g. path from (0, 0) to (2, 2) should be [(1, 0), (1, 1), (2, 1), (2, 2)]  """

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

                #reconstruct path from final node
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

class Controller:
    def __init__(self, P=0.0, D=0.0, set_point=0):
        self.Kp = P
        self.Kd = D
        self.set_point = set_point # reference (desired value)
        self.previous_error = 0

    def update(self, current_value):
        # calculate P_term and D_term
        error = self.set_point - current_value # reference - control output
        P_term = self.Kp*error
        D_term = self.Kd*(error-self.previous_error)
        self.previous_error = error
        return P_term + D_term

    def setPoint(self, set_point):
        self.set_point = set_point
        self.previous_error = 0
    
    def setPD(self, P=0.0, D=0.0):
        self.Kp = P
        self.Kd = D

if __name__ == '__main__':

    START       = (0,0)
    GOAL        = (0,3)
    OBSTACLES   = [(3,1), (1,1), 
                    (-1,-1), (-1,0), (-1,1), (-1,2), (-1,3), (-1,4), (-1, 5), 
                    (3,-1), (3,0), (3,1), (3,2), (3,3), (3,4), (3,5),
                    (0,-1), (1,-1), (2,-1), (0,5), (1,5), (2,5)] 


    star = Astar(START, OBSTACLES)



#!/usr/bin/env python3


import numpy as np
from math import pi, sqrt, atan2, cos, sin

DEBUG = False

""" conversion multipliers  """ 
INCH2METER         = 0.0254         # meter / inch 
INCHSQR2METERSQR   = INCH2METER**2  # m^2 / in^2


class Grid:

    def __init__(self, max_rows, max_columns, cell_length):  
        """ @param max_rows:    maximum column space of grid
            @param max_columns: maximum column space of grid
            @param cell_length: length and width of grid cell in meters """

        """ grid dimensions """ 
        self.grid_dimension     = (max_rows, max_columns) # max row space and max column space 
        self.cell_total         = self.grid_dimension[0] * self.grid_dimension[1]
        self.cell_length_m      = cell_length
        self.cell_length_inch  = self.cell_length_m / INCH2METER

        # length x width 
        self.grid_dim_inch  = (self.grid_dimension[0] * self.cell_length_inch, \
                                self.grid_dimension[1] * self.cell_length_inch)

        self.grid_dim_m     = (self.grid_dimension[0] * self.cell_length_m, \
                                self.grid_dimension[1] * self.cell_length_m)


        """ block dimensions """ 
        self.MAX_BLOCK_COUNT        = 25
        self.starting_length        = 5     # number of blocks lined across row and column (5x5 = 25)

        self.BLOCK_DIM_INCH         = (1.5, 1.5, 1.5)                                     # length x width x height
        self.BLOCK_AREA_INCH_SQRD   = self.BLOCK_DIM_INCH[0] * self.BLOCK_DIM_INCH[1]     # length * width 

        self.BLOCK_DIM_METER = (self.BLOCK_DIM_INCH[0]*INCH2METER, \
                                self.BLOCK_DIM_INCH[1]*INCH2METER, \
                                self.BLOCK_DIM_INCH[2]*INCH2METER)                 # length x width x height

        self.BLOCK_AREA_METER_SQRD = self.BLOCK_AREA_INCH_SQRD * INCHSQR2METERSQR  # length * width

        # max(l) x max(w) starting in block zone 
        self.STARTING_BLOCK_SPACE = (self.starting_length * self.BLOCK_DIM_METER[0], self.starting_length * self.BLOCK_DIM_METER[1])

        """ platform dimensions """ 
        self.ROW_MAX = 4
        self.COL_MAX = 4

        # built in spacing for blocks on build platform
        self.BLOCK_SPACING_METER    = 0.00936 # [m] (9.36 mm)

        # length x width
        self.PLATFORM_DIM_METER    = (self.ROW_MAX * self.BLOCK_DIM_METER[0] + (self.ROW_MAX-1) * self.BLOCK_SPACING_METER, \
                                    (self.COL_MAX * self.BLOCK_DIM_METER[1] + (self.COL_MAX-1) * self.BLOCK_SPACING_METER))


        """ navigation """

        self.start              = (0,0)
        self.previous_waypoint  = [0,0]
        self.previous_velocity  = [0,0]
        self.cost               = cell_length   # cost to travel and connectivity constant [m]
        self.obstacles          = [(0.5, 1.5), (0.5, 0.5), 
                                    (-0.5, -0.5), (-0.5, 0), (-0.5, 0.5), (-0.5, 1), (-0.5, 1.5), (-0.5, 2), (-0.5, 2.5), 
                                    (0, 2.5), (0.5, 2.5), (1, 2.5), (1.5, 2.5),
                                    (1.5, 2), (1.5, 1.5), (1.5, 1), (1.5, 0.5), (1.5, 0), (1.5, -0.5), 
                                    (1, -0.5), (0.5, -0.5), (0,-0.5)] 
        
        # static location of zones of interest
        self.block_storage_waypoint     = (0, 0.5)      # [m] 
        self.platform_waypoint          = (0, 1.5)      # [m]
        self.block_storage_coordinate   = (0.5, 0.5)    # [m] 
        self.platform_coordinate        = (0.5, 1.5)    # [m]

        # actively track current position within grid
        self.current_coordinate = self.start

        if DEBUG:
            print("Block Dimensions: ")
            print("\tl x w x h [inch]: ", self.BLOCK_DIM_INCH)
            print("\tl x w x h [meter]: ", self.BLOCK_DIM_METER)
            print("\tsquared area [inch^2]: ", self.BLOCK_AREA_INCH_SQRD)
            print("\tsquared area [meter^2]: ", self.BLOCK_AREA_METER_SQRD)
            print("\tMax starting length and width needed for blocks in block zone [meter]", self.STARTING_BLOCK_SPACE)
            
            print("Platform Dimensions: ")
            print("\tdimensions of platform [meter]: ", self.PLATFORM_DIM_METER)

            print("\nGrid Dimensions: ")
            print("\tsingle cell length [inch]: ", self.cell_length_inch)
            print("\tsingle cell length [meter]: ", self.cell_length_m)
            print("\tgrid dimension [row x col]: ", self.grid_dimension)
            print("\tcell total: ", self.cell_total)
            print("\tgrid dimensions [inch]: ", self.grid_dim_inch)
            print("\tgrid dimensions [meter]: ", self.grid_dim_m)
            print("Grid space obstacles: ", self.obstacles)
            # print("\tsquared area [inch^2]:", self.GRID_AREA_INCH_SQRD)
            # print("\tsquared area [meter^2]:", self.GRID_AREA_METER_SQRD)

    def checkGridDimensions(self):

        # check that platform dimensions fit within single cell dimensions
        if self.PLATFORM_DIM_METER[0] >= self.cell_length_m[0]:
            print("Error: Platform length exceeds single cell length")
            return False
        if self.PLATFORM_DIM_METER[1] >= self.cell_length_m[1]:
            print("Error: Platform width exceeds single cell width")
            return False

        # check that required space of blocks in block zone fit within single cell dimensions
        if self.STARTING_BLOCK_SPACE[0] >= self.cell_length_m[0] or self.STARTING_BLOCK_SPACE[1] >= self.cell_length_m[1]:
            print("Error: Total space required for blocks at start exceeds single cell dimensions")
            return False



    def get_path_from_A_star(self, start, goal, obstacles):
        """
        @param start        - integer 2-tuple of the current grid, e.g., (0, 0)
        @param goal         - integer 2-tuple  of the goal grid, e.g., (5, 1)
        @param obstacles    - a list of grids marked as obstacles, e.g., [(2, -1), (2, 0), ...]
        @return path        - a list of grids connecting start to goal, e.g., [(1, 0), (1, 1), ...]
                                note that the path should contain the goal but not the start
                                e.g. path from (0, 0) to (2, 2) should be [(1, 0), (1, 1), (2, 1), (2, 2)]  """
        
        # initialize data structures
        candidate_list  = [start]   # nodes to explore
        past_cost       = {start:0} # costs of explored nodes
        explored_list   = []        # nodes explored
        parent          = {}        # store candidate as key and current position as value 
        path            = []        # final path 

        # iterate over nodes remaining to explore
        while candidate_list: 
            
            current = candidate_list.pop(0) # pop candidate closest to goal 
            explored_list.append(current)   # add current candidate to explored

            # check if goal is reached
            if current == goal:  
                
                node = goal

                #reconstruct path from final node
                while node != start:
                    path.insert(0, node)
                    node = parent[node]
                return path

            # iterate over nbrs in connectivity list
            for nbr in self.neighbors(current, self.cost): 

                # check if neighbor has been explored or is an obstacle
                if nbr not in explored_list and nbr not in obstacles: 

                    new_cost = past_cost[current] + self.cost   # cost to move in direction
                    
                    # check if nbr has a cost value assigned to it or if its cost has improved
                    if nbr not in past_cost or new_cost < past_cost[nbr]: 
                        past_cost[nbr]  = new_cost  # update nbrs cost in dictionary
                        parent[nbr]     = current   # update nbrs parent to current node
                        candidate_list.append(nbr)  # append nbr to candidates for search

            # sort remaining candidates based on cost
            candidate_list.sort(key = lambda x: past_cost[x] + self.heuristic_distance(x,goal)) 

        # return empty list if candidates are exhausted and goal is not found
        return []


    def neighbors(self, center_node, cost):
        """ @note compute 4-connectivity list of neighbors """

        neighbors = [(cost, 0), (-cost, 0), (0, cost), (0, -cost)]  
        return [(center_node[0] + nbr[0], center_node[1] + nbr[1]) for nbr in neighbors ]

    def heuristic_distance(self, candidate, goal):
        """ @note euclidean distance as heuristic measure """
        return sqrt((candidate[0] - goal[0])**2 + (candidate[1] - goal[1])**2)


    def getGoal(self, goal_name):

        if goal_name is 'block_storage':
            goal = self.block_storage_waypoint
        elif goal_name is 'platform':
            goal = self.platform_waypoint
        else:
            print("Error: Invalid goal request")
            goal = (0,0)
        
        return goal

    def getCurrent(self):
        return self.current_coordinate

# if __name__ == '__main__':
    # grid        = Grid(3, 5, 0.5)
    # goal        = (1, 0.5)
    # waypoints   = grid.get_path_from_A_star(grid.start, goal, grid.obstacles)
    # print("waypoints:", waypoints)
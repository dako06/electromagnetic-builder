START = (0,0)
GOAL = (4,4)
OBSTACLES = [(3,3)] 


class Astar():
    
    def run(self, goal):

            """
            need way to get starting position and obstacle lists
            """
            start = (0,0)
            obstacles = (0,0)



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
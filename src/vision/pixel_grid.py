
from cmath import inf
from re import X
import numpy as np
from math import atan2, sqrt

DEBUG = True

class PixelGrid:


    def __init__(self, dim, resolution) -> None:

        """ pixel grid dimensions """

        self.max_row            = dim[0]
        self.max_col            = dim[1]
        self.min_row            = (self.max_row//resolution)
        self.min_col            = (self.max_col//resolution)
        
        self.grid_resolution        = resolution
        # self.grid                   = self.initGrid()


        """ center column window """
        self.grid_center            = ((self.max_row-1)//2, self.max_col//2)    # pixel coordinates of grid center
        self.column_window_delta    = 10                                        # expand column in both directions from center pixel

        self.center_column_window   = (self.grid_center[1] - self.column_window_delta, \
                                        self.grid_center[1] + self.column_window_delta)  
        
        # pixel reresenting half way in column space and max in row space
        self.base_pixel = ((self.max_row - 1), self.max_col // 2)

        """ kernal """
        self.kernal                 = np.array([])

        if DEBUG:   
            print("pixel grid dim: ", dim)
            print("grid_center", self.grid_center)
            print("base_pixel", self.base_pixel)
            print("center_column_window: ", self.center_column_window)



    def initGrid(self):
        base_coordinate = (self.min_row-1, self.min_col-1)
        # print('base coordinate: ', base_coordinate)
        base_cell = np.array([(0,0), (0,base_coordinate[1]), (base_coordinate[0],0), base_coordinate]) 
        # print('base_cell: ', base_cell)

        mask_row = np.array((self.min_row,0)) # updates indices each new row
        mask_col = np.array((0,self.min_col)) # updates indices each new grid col for set grid row
        
        temp_grid = []
        for i in range(self.grid_resolution):
            
            temp = []
            first_cell = base_cell + (i * mask_row)
            temp.append(first_cell)

            for j in range(1, self.grid_resolution):
                cell = first_cell + (j * mask_col)
                temp.append(cell)

            temp_grid.append(temp)
            
            # if DEBUG:
            #     t = 0
            #     for i in temp_grid:
            #         print('#%d ' % t)
            #         t+=1

            #         for j in i:
            #             print(j)
        return temp_grid

    def findNearestObject(self, compenent_list):
        """@param compenent_list: list of dictionaries containing filtered connected components
            @note identify the nearest point and return the direction w.r.t the center of image frame and the pixel coordinates """
        
        if len(compenent_list) == 0:
            return "empty", [], False

        # sort components based nearest pixel space distance 
        compenent_list.sort(key = lambda x: self.pixelDistanceHeuristic(x.get('centroid')))
        # compenent_list.sort(key = lambda x: print(x.get('centroid')))
        
        # check if objects centroid is already centered 
        nearest_comp   = compenent_list.pop(0)
        cx, cy         = nearest_comp.get("centroid")

        if (cx > self.center_column_window[0]) and (cx < self.center_column_window[1]):
            
            grid_position   = "center"
        
        else:
            
            # determine rotation direction required to center object with robots perspective 
            if (cx - self.base_pixel[1]) > 0:
                grid_position = "right"
            else:
                grid_position = "left"

        return grid_position, nearest_comp
     
        

    def pixelDistanceHeuristic(self, pixel_xy):
        """ @param pixel_xy: 2-tuple containing x which represents coordinate along column space
                                and y which represents coordinate along row space 
            @note confirm proper arithmetic operation given that row space is typically first matrix index
                    and y is typically second element in euclidean coordinates """
        dp          = (pixel_xy[1] - self.base_pixel[0], pixel_xy[0] - self.base_pixel[1])
        pixel_dist  = sqrt(dp[0]**2 + dp[1]**2)
        return pixel_dist

            

        

        





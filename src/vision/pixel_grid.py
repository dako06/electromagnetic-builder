
from cmath import inf
from re import X
import numpy as np
from math import atan2, sqrt

DEBUG = True

class PixelGrid:


    def __init__(self, dim, resolution) -> None:

        """ pixel grid dimensions """

        # take in max row and column  
        self.max_row            = dim[0]
        self.max_col            = dim[1]
        self.min_row            = (self.max_row//resolution)
        self.min_col            = (self.max_col//resolution)
        
        self.grid_resolution    = resolution
        # self.grid             = self.initGrid()


        """ center column window """

        # centroid using column-row space as x-y space
        self.centroid           = (int(self.max_col/2), int(self.max_row/2))
        
        # build scanning window using grid centroid-x with +/- offset
        self.dx                 = 50
        self.window_base_x      = self.centroid[0] - 25
        self.winow_max_x        = self.window_base_x + self.dx
        self.winow_min_x        = self.window_base_x - self.dx

        self.scanning_border = {'start_line_1' : (self.winow_min_x, 0), \
                                    'end_line_1' : (self.winow_min_x, self.max_row - 1), \
                                        'start_line_2' : (self.winow_max_x, 0), \
                                    'end_line_2':(self.winow_max_x, self.max_row - 1)}

        # pixel used to make nearness measurements,  max_x/2, max_y
        self.pixel_anchor           = (int(self.max_col // 2), self.max_row)
        self.pixel_anchor_euclidean = self.euclideanTransform(self.pixel_anchor)

        # self.grid_center            = ((self.max_row-1)//2, self.max_col//2)    # pixel coordinates of grid center
        # self.column_window_delta    = 10                                        # expand column in both directions from center pixel

        # self.center_column_window   = (self.grid_center[1] - self.column_window_delta, \
        #                                 self.grid_center[1] + self.column_window_delta)  
        

        # if DEBUG:   
        #     print("pixel grid dim: ", dim)
        #     print("grid_center", self.grid_center)
        #     print("base_pixel", self.base_pixel)
        #     print("center_column_window: ", self.center_column_window)



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
            return "unknown", []

        # sort components based nearest pixel space distance 
        compenent_list.sort(key = lambda x: self.pixelDistanceHeuristic(x.get('centroid')))
        
        # check if objects centroid is already centered 
        nearest_comp   = compenent_list.pop(0)
        cx, cy         = nearest_comp.get("centroid")   # x-axis = column space, y-axis = row space

        if (cx > self.center_column_window[0]) and (cx < self.center_column_window[1]):
            
            grid_position   = "center"
        
        else:
            
            # determine rotation direction required to center object with robots perspective 
            if (cx - self.base_pixel[1]) > 0:
                grid_position = "right"
            else:
                grid_position = "left"

        return grid_position, nearest_comp
     
        

    def pixelDistanceHeuristic(self, pixel):
        """ @param pixel: 2-tuple containing x which represents coordinate along column space
                            and y which represents coordinate along row space 
            @note confirm proper arithmetic operation given that row space is typically first matrix index
                    and y is typically second element in euclidean coordinates """

        # use pixel anchor as initial coordinate
        xo, yo      = self.pixel_anchor_euclidean
        xf, yf      = self.euclideanTransform(pixel) 
        # pixel_delta = np.linalg.norm(x=[xf - xo, yf - yo], ord=2)
        # pixel_dist  = sqrt(dp[0]**2 + dp[1]**2)
        return np.linalg.norm(x=[xf - xo, yf - yo], ord=2)

            
    def euclideanTransform(self, p):
        """ @param p 2-tuple containing x and y coordinates in pixel space (x-column, y-row) 
            @note transformation inverts the axis then applies offset equal to the pixel anchor 
                for the convenience of a right handed frame where +x is to the left and +y is up """ 
        xp, yp = p
        xo, yo = self.pixel_anchor
        return ((-1* xp) + xo, (-1 * yp) + yo)
        





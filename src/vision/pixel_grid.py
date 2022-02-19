import numpy as np

class PixelGrid:

    # non-unique object attributes here

    def __init__(self, dim, resolution) -> None:
        # unique object attributes here

        self.max_row    = dim[0]
        self.max_col    = dim[1]
        self.min_row    = (self.max_row//resolution)
        self.min_col    = (self.max_col//resolution)
        self.grid_resolution  = resolution
        self.grid       = self.initGrid()
        self.base_pixel = (self.max_row-1, self.max_col//2)

        self.center_window = (self.base_pixel[1]-self.grid_resolution, self.base_pixel[1]+self.grid_resolution)  

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
        
        return temp_grid
        # t = 0
        # for i in temp_grid:
        #     print('#%d ' % t)
        #     t+=1

        #     for j in i:
        #         print(j)






# from pandas.core.indexing import convert_to_index_sliceable
import pandas as pd
import numpy as np


class Blueprint:

    def __init__(self, fpath, max_row, max_col):

        self.max_row = max_row  # max number of elements across the x-axis
        self.max_col = max_col  # max number of elements across the y-axis
        self.current_block = ((0,0), 0)

        #### construct blueprint ####
        blueprint_raw = pd.read_excel(io=fpath)         # read in excel file as panda data structure
        self.blueprint = np.array(blueprint_raw.values) # convert to np array
        print('Bluerprint matrix has been initialized.')
        # print('Total blocks to place: %d.' % self.getBlockTotal())



    # def getBlockTotal(self):
    #     sum = 0
    #     for row in self.blueprint:
    #         for i in row:
    #             sum+=i
    #     return sum


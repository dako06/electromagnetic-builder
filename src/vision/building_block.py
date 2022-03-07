#!/usr/bin/env python3

import numpy as np 

class Block:

    def __init__(self, width, height, area, centroid, p_o):

        self.pixel_dim      = {'width':width, 'height':height, 'area':area} 
        self.centroid       = centroid  # centroid stored as (x:col, y:row)
        self.pixel_o        = p_o       # starting pixel stored as (x:col, y:row)
        # self.label_matrix   = []

        """ {'p_min':, 'p_max':, 'width':,'height':, 'centroid':, 
            'area', 'label_id'} """
        self.metal_blobs    = []
        
        self.feature_array  = []
        self.updateFeatureArray()

    def addMetalBlob(self, blob_list):

        """ @param blob_list dictionary list containing features of the blob associated
                    with the block """
        
        self.metal_blobs = blob_list

    def updateBlock(self, block):
        """ update this blocks features during correspondence """       
        self.pixel_dim      = block.pixel_dim
        self.centroid       = block.centroid
        self.pixel_o        = block.pixel_o 
        self.metal_blobs    = block.metal_blobs
        self.updateFeatureArray() 
 

    def updateFeatureArray(self):
        """ update so that elements in order are:
            top left corner, top right corner,
            bottom left corner, bottom right corner, and centroid """
        x, y    = self.pixel_o
        w       = self.pixel_dim.get('width') 
        h       = self.pixel_dim.get('height') 
        self.feature_array = np.array([(x, y), (x+w, y), (x, y+h), (x+w, y+h), self.centroid])



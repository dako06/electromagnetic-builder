
class Block():

    def __init__(self, width, height, area, centroid, p_o) -> None:

        self.pixel_dim      = {'width':width, 'height':height, 'area':area} 
        self.centroid       = centroid
        self.pixel_o        = p_o
        self.label_matrix   = []

        """ {'p_min':, 'p_max':, 'width':,'height':, 'centroid':, 
            'area', 'label_id'} """
        self.metal_blobs    = []

    def addMetalBlob(self, blob_list):

        """ @param blob_list dictionary list containing features of the blob associated
                    with the block """
        
        self.metal_blobs = blob_list




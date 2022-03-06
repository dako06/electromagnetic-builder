#!/usr/bin/env python3

import cv2 as cv
import numpy as np
from block import Block

from pixel_grid import PixelGrid

class ImageProcessor:

    def __init__(self) -> None:

        # HSV color filters with lower and upper thresholds  
        self.block_filter_HSV  = {'lower':np.array([100, 90, 20], dtype = "uint8"), \
                                    'upper':np.array([120, 255, 255],  dtype = "uint8")}


        self.emag_filter_HSV   = {'lower':np.array([155, 100, 20], dtype = "uint8"), \
                                'upper':np.array([170, 255, 255],  dtype = "uint8")}

        self.pix_grid = PixelGrid((480, 640), 25)

        """ thresholds for filtering objects in pixel space
                (element 0 is min and element 1 is max)     """
        # single block 
        self.block_thresh = {'width':(45,175), 'height':(45,140), 'area':(4000,20000)}
        self.metal_thresh = {'width':(20,50), 'height':(10,50), 'area':(200,2000)}

        self.block_open_param = {'erosion':2, 'dilation':3}
        self.metal_open_param = {'erosion':3, 'dilation':4}

        # block and electromagnet connection 
        self.block_emag_thresh = {'width':(50,90), 'height':(50,90), 'area':(2400,3100)}


    def getConnectedComponents(self, img_binary, connectivity):
        """ @param binary - binary thresholded image
            @param connectivity - 4 is diamond neighboorhood, 8 is square neighboorhood
            @param filter - 0 return all components, 1- filter based on constraint
            @return 4tuple - (count, label, stats, centroids)
                            count - number of components
                            label - same dim as image, each element consists of an int indicating the component it belongs to
                            stats - nx5 matrix containing stats
                            centroids - nx2 matrix containing centroid
            @note returns all connected components (first component is always the background) """

        if (connectivity != 4 and connectivity != 8):
            connectivity = 8

        # features to return (numLabels, labels, stats, centroids)
        return cv.connectedComponentsWithStats(img_binary, connectivity, cv.CV_32S)

    def filterComponents(self, comp_tuple, threshold):
        """ @param comp_tuple:      4-tuple containing connected components and there statistics
            @param threshold:       threshold used for filtering as a dict
            @note accessing stats:  x = stats[i, cv2.CC_STAT_LEFT]      - starting x coordinate of the component
	                                y = stats[i, cv2.CC_STAT_TOP]       - starting y coordinate of the component
	                                w = stats[i, cv2.CC_STAT_WIDTH]     - width of the component
	                                h = stats[i, cv2.CC_STAT_HEIGHT]    - height of the component
	                                area = stats[i, cv2.CC_STAT_AREA]   - area (in pixels)  """
        
        # get upper and lower bounds from threshold dictionary
        (min_width, max_width)      = threshold.get('width')
        (min_height, max_height)    = threshold.get('height')
        (min_area, max_area)        = threshold.get('area')

        # unpack connected component tuple 
        (label_count, _, stats, centroids) = comp_tuple

        # store inlier indices
        inlier_labels = []

        # iterate over number of components 
        for ix in range(0, label_count):

            w, h    = stats[ix, cv.CC_STAT_WIDTH], stats[ix, cv.CC_STAT_HEIGHT]
            area    = stats[ix, cv.CC_STAT_AREA]

            # check that component satisfies thresholds
            if (w >= min_width and w <= max_width) and (h >= min_height and h <= max_height) and (area >= min_area and area <= max_area): 
                
                inlier_labels.append(ix) # add inlier index

        return inlier_labels



    """ tracking/detection functions """

    def trackBlockTransport(self, src_img):

        lower_mask = self.filterColor(src_img, self.block_filter_HSV)
        upper_mask = self.filterColor(src_img, self.emag_filter_HSV)
        connection_mask = lower_mask + upper_mask 

        opened_connection_mask = self.compoundOpenImage(connection_mask)

        return opened_connection_mask

    def matchComponents(self, comp_p, inlier_ix_p, comp_s, inlier_ix_s):

        # return if there are not sufficient components
        if len(inlier_ix_p) == 0 or len(inlier_ix_s) == 0:
            return []

        block_list   = []    # list of running block objects to return   
        matched_ix   = []    # list of associated indices to prevent over-processing
        
        # extract component containers 
        (_, label_matrix_p, stats_p, centroids_p) = comp_p  # primary blobs
        (_, label_matrix_s, stats_s, centroids_s) = comp_s  # sub blobs
        
        # iterate over each inlier label to access inlier primary components
        for ix_p in inlier_ix_p:

            # construct pixel space geofence for filtering sub-blobs
            w_p, h_p        = stats_p[ix_p, cv.CC_STAT_WIDTH], stats_p[ix_p, cv.CC_STAT_HEIGHT]
            min_xp, min_yp  = stats_p[ix_p, cv.CC_STAT_LEFT], stats_p[ix_p, cv.CC_STAT_TOP]
            pixel_geofence  = {'min_coordinate': (min_xp, min_yp), 'max_coordinate': (min_xp + w_p, min_yp + h_p)}

            # store all associations to this primary blob
            association_list = []
            
            # compare non-associated sub-blobs to current primary
            for ix_s in inlier_ix_s:

                # skip if label has already been associated
                if ix_s in matched_ix:
                    continue  

                # get pixel features used for matching test
                cX_s, cY_s  = centroids_s[ix_s]
                w_s, h_s    = stats_s[ix_s, cv.CC_STAT_WIDTH], stats_s[ix_s, cv.CC_STAT_HEIGHT]
                x_s, y_s    = stats_s[ix_s, cv.CC_STAT_LEFT], stats_s[ix_s, cv.CC_STAT_TOP]

                # check that sub blob is within primary blob
                if self.isBlobInBlob(pixel_geofence, (cX_s, cY_s), (w_s/2, h_s/2)):

                    blob_dict = {'p_min': (x_s, y_s), 'p_max': (x_s+w_s, y_s+h_s), 'width':w_s, 'height':h_s, \
                                'centroid':(cX_s, cY_s), 'area':stats_s[ix_s, cv.CC_STAT_AREA], 'label_id':ix_s}
                    
                    association_list.append(blob_dict)  # add index to associations for this primary iteration 
                    matched_ix.append(ix_s)          # add sub blob index to known associations

            # if number of associations is reasonable then keep label composition
            if len(association_list) > 0 and len(association_list) <= 3:

                label_matrix_p + label_matrix_s

                # construct block and add to list
                block = Block(width=w_p, height=h_p, area=stats_p[ix_p, cv.CC_STAT_AREA], centroid=centroids_p[ix_p], p_o=(min_xp, min_yp))
                block.addMetalBlob(association_list)    # add composition to success list  
                block_list.append(block)                # update associated metal indices  
        
        return block_list
        # get the first blob within the primary object set
        # prime_blob = (label_mat_p == ix_p).astype("uint8") * 255
        # sub_blob = (label_mat_s == ix_s).astype("uint8") * 255
        # mask = cv.bitwise_or(mask, componentMask)
        # self.displayImg("component mask from label", comp_mask, 'img')  

    def isBlobInBlob(self, pixel_GF, centroid, radius):

        # get min and max pixel geofence coordinates
        min_x, min_y    = pixel_GF.get('min_coordinate')
        max_x, max_y    = pixel_GF.get('max_coordinate')
        
        # get centroid to test position within blob 
        (cx, cy)        = centroid
        dx, dy          = radius

        # print("centroid: (%f, %f)" % (cx, cy))
        # print("geofence points:")
        # print("1. (%f, %f) \t2. (%f, %f)" % (min_x, min_y, max_x, min_y))
        # print("3. (%f, %f) \t4. (%f, %f)" % (min_x, max_y, max_x, max_y))

        # test secondary centroid position with respect to geofence 
        if (cx > min_x and cy > min_y) and (cx < max_x and cy > min_y) and (cx > min_x and cy < max_y) and (cx < max_x and cy < max_y):
            centroid_test = True
        else: 
            centroid_test = False

        # if cx + dx < max_x and cx - dx > min_x and cy + dy < max_y and cy - dy > min_y:   
        #     boundary_test = True
        # else: 
        #     boundary_test = False

        return centroid_test #and boundary_test)

    def labelBlocks(self, src_img, block_list):
        """ take a list of block objects and mark the source image with the block positions """

        block_num = 1

        for block in block_list:

            cx, cy      = block.centroid
            x, y        = block.pixel_o
            width       = block.pixel_dim.get('width')
            height      = block.pixel_dim.get('height')

            cv.circle(src_img, (int(cx), int(cy)), 3, (0, 175, 0), -1)
            cv.rectangle(src_img, (x, y), (x + width, y + height), (0, 175, 0), 3)
            # cv.putText(src_img, str(block_num), (int(cx),int(cy)), cv.FONT_HERSHEY_SIMPLEX, 1,(0,255,0), 1, cv.LINE_AA)
            
            for metal in block.metal_blobs:            
                cv.rectangle(src_img, metal.get('p_min'), metal.get('p_max'),  (0, 0, 255), 3)
            
            block_num += 1

            # use in the future for block tracker
            # comp_mask = (label_matrix == ix).astype("uint8") * 255
        return src_img




    def detectBlocks(self, src_img):

        # apply color filter to get block mask and then inverse to get metal mask
        block_mask  = self.filterColor(img_bgr=src_img, filter=self.block_filter_HSV)
        metal_mask  = cv.bitwise_not(block_mask)

        # open both masks with different structuring elements
        opened_block_mask   = self.compoundOpenImage(src_img=block_mask, param_dict=self.block_open_param)
        opened_metal_mask   = self.compoundOpenImage(src_img=metal_mask, param_dict=self.metal_open_param)  

        # get connected components within masks
        block_comps = self.getConnectedComponents(img_binary=opened_block_mask, connectivity=8)  
        metal_comps = self.getConnectedComponents(img_binary=opened_metal_mask, connectivity=8) 
        
        # filter components and retrieve inlier labels
        block_label_list = self.filterComponents(comp_tuple=block_comps, threshold=self.block_thresh)
        metal_label_list = self.filterComponents(comp_tuple=metal_comps, threshold=self.metal_thresh)

        # make blue block and metal associations
        block_obj_list = self.matchComponents(comp_p=block_comps, inlier_ix_p=block_label_list, \
                                                comp_s=metal_comps, inlier_ix_s=metal_label_list)
            
        return block_obj_list




    def filterColor(self, img_bgr, filter):
        """
        Find indices of image within range of lower and upper bound in HSV color range.
        @param img_bgr  - BGR image 
        @param filter   - lower and upper bound of HSV range as a dict
        @return: binary mask describing inbound indices """
        lower = filter.get('lower') 
        upper = filter.get('upper')

        return cv.inRange(cv.cvtColor(img_bgr, cv.COLOR_BGR2HSV), lower, upper)


    def applyErosion(self, kernal_size, src, shape):
 
        if (shape=="rectangle"):
            erosion_shape = cv.MORPH_RECT 
        elif (shape=="cross"):
            erosion_shape = cv.MORPH_CROSS 
        elif (shape=="ellipse"):   
            erosion_shape = cv.MORPH_ELLIPSE

        element = cv.getStructuringElement(erosion_shape, (2 * kernal_size + 1, 2 * kernal_size + 1),
                                       (kernal_size, kernal_size))

        return cv.erode(src, element)

    def applyDilation(self, kernal_size, src, shape):

        if (shape=="rectangle"):
            dilation_shape = cv.MORPH_RECT 
        elif (shape=="cross"):
            dilation_shape = cv.MORPH_CROSS 
        elif (shape=="ellipse"):   
            dilation_shape = cv.MORPH_ELLIPSE

        element = cv.getStructuringElement(dilation_shape, (2 * kernal_size + 1, 2 * kernal_size + 1),
                                       (kernal_size, kernal_size))

        return cv.dilate(src, element)

    def compoundOpenImage(self, src_img, param_dict):

        # extract kernal scaler from open parameter dictionary
        erosion_kernal  = param_dict.get('erosion')
        dilation_kernal = param_dict.get('dilation')

        img_erosion     = self.applyErosion(erosion_kernal, src_img, "cross")
        opened_img      = self.applyDilation(dilation_kernal, img_erosion, "rectangle")
        
        return opened_img

    def displayImg(self, window_name, img, save_as):
        """ @param title    - name given to image in cv window
            @param img      - image object to display
            @param save_as  - (string) name to save image as
            @note press s key during display to save image or any other key to close image """

        # display and wait for key
        cv.imshow(window_name, img)     
        key = cv.waitKey(0)      
               
        # save image if requested
        if key==ord("s"):
            cv.imwrite(save_as, img)    
        

    def viewComponents(self,  src_img, components, inlier_ix, verbose):

        (label_count, label_matrix, stats, centroids) = components

        if len(inlier_ix) == 0:
            iter = np.arange(0, label_count, 1)
        else:
            iter = inlier_ix

        for ix in iter:

            w, h    = stats[ix, cv.CC_STAT_WIDTH], stats[ix, cv.CC_STAT_HEIGHT]
            x, y    = stats[ix, cv.CC_STAT_LEFT], stats[ix, cv.CC_STAT_TOP]
            cx, cy  = centroids[ix]

            if verbose:
                print("\n\nLABEL INDEX: %d" % ix)
                print("width, height, area: %f, %f, %f"  % (w, h, stats[ix, cv.CC_STAT_AREA]))
                print("x, y, centroid: %f, %f, (%f, %f)" % (x, y, cx, cy))

            comp_mask = (label_matrix == ix).astype("uint8") * 255
            cv.circle(src_img, (int(cx), int(cy)), 4, (0, 0, 255), -1)
            cv.rectangle(src_img, (x, y), (x + w, y + h), (0, 255, 0), 3)
            cv.putText(src_img, str(ix), (int(cx),int(cy)), cv.FONT_HERSHEY_SIMPLEX, 1,(255,0,0), 2, cv.LINE_AA)
            
            self.displayImg("source image with bounded labels", src_img, 'img')
            self.displayImg("component mask", comp_mask, 'img')
        
        cv.destroyAllWindows()


    def drawDetectionWindow(self, src_img):
        
        labeled_img = src_img.copy()
        min_1 = self.pix_grid.scanning_border.get('start_line_1')
        max_1 = self.pix_grid.scanning_border.get('end_line_1')
        min_2 = self.pix_grid.scanning_border.get('start_line_2')
        max_2 = self.pix_grid.scanning_border.get('end_line_2')
        
        # Draw a diagonal blue line with thickness of 5 px
        cv.line(labeled_img, min_1, max_1, (0,255,255), 1)
        cv.line(labeled_img, min_2, max_2, (0,255,255), 1)

        return labeled_img

   

    def displayImgProperties(self, img):
        print("Image Properties:")
        print("\tRows, columns, channels: ", img.shape)
        print("\tTotal element count: ", img.size)
        print("\tThe data type used to represent each pixel is: ", img.dtype)

    def convert_bgr2rgb(self, img_bgr):
        return cv.cvtColor(img_bgr, cv.COLOR_BGR2RGB)

    def convert_bgr2grayscale(self, img_bgr):
        return cv.cvtColor(img_bgr, cv.COLOR_BGR2GRAY)



    def applyThreshold(self, img_gray, thresh_type):
        """
        @param img_gray(grayscale image)
        @param thresh_type(string) - "adaptive" or "simple" thresholding
        """
        epsilon = 127   # threshold value applied to pixels
        max_val = 255   # max value assigned to pixels above threshold
        blocksize = 115 # size of neighbourhood area used
        c = 2           # constant subtracted from mean of neighbourhood 
        
        thresh = 0
        if thresh_type == "adaptive":

            # if an image has different lighting conditions in different areas
            # use when illumitaion varies throughout image
            thresh_img = cv.adaptiveThreshold(img_gray, max_val, cv.ADAPTIVE_THRESH_GAUSSIAN_C, 
                cv.THRESH_BINARY_INV, blocksize, c)

        elif thresh_type == "simple":

            # returns threshold used, and image after threshold
            thresh, thresh_img = cv.threshold(img_gray, epsilon, max_val, cv.THRESH_BINARY_INV)

        return thresh_img


    def getContours(self, bin_img):
        contours, hierarchy = cv.findContours(bin_img, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        return contours




        
    # def draw_contour(self, bin_img, rgb_img, contours):
    #     ix = -1                 # all contours
    #     color = (150,250,150)   # color of contour
    #     thick = 1           # thickness of line

    #     black_image = np.zeros([bin_img.shape[0], bin_img.shape[1],3],'uint8')

    #     # draw contours on image
    #     for c in contours:
    #         area = cv.contourArea(c)
    #         perimeter= cv.arcLength(c, True)
    #         ((x, y), radius) = cv.minEnclosingCircle(c)
    #         if (area>100):
    #             cv.drawContours(rgb_img, [c], ix, color, thick)
    #             cv.drawContours(black_image, [c], ix, color, thick)
    #             cx, cy = self.get_contour_center(c)
    #             cv.circle(rgb_img, (cx,cy),(int)(radius),(0,0,255),1)
    #             cv.circle(black_image, (cx,cy),(int)(radius),(0,0,255),1)
    #             cv.circle(black_image, (cx,cy),5,(150,150,255),-1)
    #             print ("Area: {}, Perimeter: {}".format(area, perimeter))
    #     print ("number of contours: {}".format(len(contours)))
    #     cv.imshow("RGB Image Contours",rgb_img)
    #     cv.imshow("Black Image Contours",black_image)
    # #     cv.imwrite('test_2.jpg', img)
    # #     cv.imshow(img_name, img)

    # def get_contour_center(self, contour):
    #     M = cv.moments(contour)
    #     cx=-1
    #     cy=-1
    #     if (M['m00']!=0):
    #         cx= int(M['m10']/M['m00'])
    #         cy= int(M['m01']/M['m00'])
    #     return cx, cy    

    # def draw_ball_contour(self, binary_image, rgb_image, contours):
    #     black_image = np.zeros([binary_image.shape[0], binary_image.shape[1],3],'uint8')
        
    #     for c in contours:
    #         area = cv.contourArea(c)
    #         perimeter= cv.arcLength(c, True)
    #         ((x, y), radius) = cv.minEnclosingCircle(c)
    #         if (area>3000):
    #             cv.drawContours(rgb_image, [c], -1, (150,250,150), 1)
    #             cv.drawContours(black_image, [c], -1, (150,250,150), 1)
    #             cx, cy = self.get_contour_center(c)
    #             cv.circle(rgb_image, (cx,cy),(int)(radius),(0,0,255),1)
    #             cv.circle(black_image, (cx,cy),(int)(radius),(0,0,255),1)
    #             cv.circle(black_image, (cx,cy),5,(150,150,255),-1)
    #             #print ("Area: {}, Perimeter: {}".format(area, perimeter))
    #     #print ("number of contours: {}".format(len(contours)))
    #     cv.imshow("RGB Image Contours",rgb_image)

    #     cv.imshow("Black Image Contours",black_image)


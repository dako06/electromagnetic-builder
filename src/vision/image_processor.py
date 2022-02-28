#!/usr/bin/env python3

import cv2 as cv
import numpy as np

class ImageProcessor:

    def __init__(self) -> None:

        # HSV color lower and upper thresholds  
        self.block_filter_HSV           = np.array(([110, 100, 20], [120, 255, 255]), dtype = "uint8")      
        self.electromagnet_filter_HSV   = np.array(([155, 100, 20], [170, 255, 255]), dtype = "uint8")           
        # self.electromagnet_filter_HSV   = np.array(([170, 80, 20], [180, 255, 255]), dtype = "uint8")   
        
        
        self.color_buildsite            = np.array(([165, 90, 20], [170, 200, 255]), dtype = "uint8")  

        # threshold used for filtering single block during pixel component analysis (element 0 is min and element 1 is max)
        self.block_pixel_thresh = {'width':(50,90), 'height':(50,90), 'area':(2400,3100)}


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

        (numLabels, labels, stats, centroids) = cv.connectedComponentsWithStats(img_binary, connectivity, cv.CV_32S)
        # print("number of components: ", numLabels)

        return (numLabels, labels, stats, centroids)
        
  
    def filterComponents(self, comp_tuple, threshold):
        """ @param comp_tuple:      4-tuple containing connected components and there statistics
            @param threshold:       threshold used for filtering as a dict
            @note accessing stats:  x = stats[i, cv2.CC_STAT_LEFT]      - starting x coordinate of the component
	                                y = stats[i, cv2.CC_STAT_TOP]       - starting y coordinate of the component
	                                w = stats[i, cv2.CC_STAT_WIDTH]     - width of the component
	                                h = stats[i, cv2.CC_STAT_HEIGHT]    - height of the component
	                                area = stats[i, cv2.CC_STAT_AREA]   - area (in pixels)  """
        
        # get boundaries from dictionary
        (min_width, max_width)      = threshold.get('width')
        (min_height, max_height)    = threshold.get('height')
        (min_area, max_area)        = threshold.get('area')
        print("threshold: ", threshold)

        # unpack connected component tuple 
        (label_count, label_matrix, stats, centroids) = comp_tuple

        comp_list = []

        # iterate over number of components (hard coded to ignore background)
        for i in range(1, label_count):

            w = stats[i, cv.CC_STAT_WIDTH]
            h = stats[i, cv.CC_STAT_HEIGHT]
            area = stats[i, cv.CC_STAT_AREA]
            print("width, height, area: %f, %f, %f" % (w,h,area)) # debug
            

            # check that component satisfies thresholds
            if (w > min_width and w < max_width) and (h > min_height and h < max_height) and (area > min_area and area < max_area): 

                comp_dim = {'xy': (stats[i, cv.CC_STAT_LEFT], stats[i, cv.CC_STAT_TOP]), \
                            'wh':(w,h), 'centroid':(centroids[i]), 'area':area, 'label_id':i}

                comp_list.append(comp_dim)

                # print("x, y : ", comp_dim.get('xy'))                
                # print("w, h : ", comp_dim.get('wh')) 
                # print("centroid : ", comp_dim.get('centroid')) 

        return comp_list, label_matrix  


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


    def compoundOpenImage(self, src_img):

        img_erosion     = self.applyErosion(4, src_img, "cross")
        opened_img      = self.applyDilation(3, img_erosion, "rectangle")
        return opened_img

    """ tracking/detection functions """

    def trackBlockTransport(self, src_img):

        lower_mask = self.filterColor(src_img, self.block_filter_HSV[0], self.block_filter_HSV[1])
        upper_mask = self.filterColor(src_img, self.electromagnet_filter_HSV[0], self.electromagnet_filter_HSV[1])
        connection_mask = lower_mask + upper_mask 

        opened_connection_mask = self.compoundOpenImage(connection_mask)

        return opened_connection_mask



    def detectBlocks(self, src_img):

        ret_img = src_img.copy()

        block_mask = self.filterColor(src_img, self.block_filter_HSV[0], self.block_filter_HSV[1])
        opened_block_mask = self.compoundOpenImage(block_mask)

        comp_list = self.getConnectedComponents(opened_block_mask, connectivity=8)
        filtered_comp_list, label_matrix = self.filterComponents(comp_list, self.block_pixel_thresh)

        for c in filtered_comp_list:

            (x,y)       = c.get('xy')
            (w,h)       = c.get('wh')
            (cX, cY)    = c.get('centroid')
            ix          = c.get('label_id')
            print("x,y: %f, %f" % (x,y))
            print("cX,cY: %f, %f" % (cX,cY))
            print("w,h: %f, %f" % (w,h))

            component_mask = (label_matrix == ix).astype("uint8") * 255
            cv.circle(ret_img, (int(cX), int(cY)), 4, (0, 0, 255), -1)
            cv.rectangle(ret_img, (x, y), (x + w, y + h), (0, 255, 0), 3)
        # bounding_rect = cv.rectangle(src_img, (x, y), (x + w, y + h), (0, 255, 0), 3)

        return ret_img


    def filterColor(self, img_bgr, lower, upper):
        """
        Find indices of image within range of lower and upper bound in HSV color range.
        @param img_bgr - BGR image 
        @param lower - lower bound of HSV range
        @param upper - upper bound of HSV range
        @return: binary mask describing inbound indices """

        return cv.inRange(cv.cvtColor(img_bgr, cv.COLOR_BGR2HSV), lower, upper)


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
        
        # cv.destroyAllWindows()

    def displayImgProperties(self, img):
        print("Image Properties:")
        print("\tRows, columns, channels: ", img.shape)
        print("\tTotal element count: ", img.size)
        print("\tThe data type used to represent each pixel is: ", img.dtype)

    def convert_bgr2rgb(self, img_bgr):
        return cv.cvtColor(img_bgr, cv.COLOR_BGR2RGB)

    def convert_bgr2grayscale(self, img_bgr):
        return cv.cvtColor(img_bgr, cv.COLOR_BGR2GRAY)


    ######## UNUSED ######## 


    def findCorner(self, img_gray):
        gray = np.float32(img_gray)
        return cv.cornerHarris(gray,2,3,0.04)


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


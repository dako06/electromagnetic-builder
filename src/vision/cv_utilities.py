#!/usr/bin/env python3

import cv2 as cv
import numpy as np

class ImageProcessor:


    def __init__(self) -> None:

        # HSV color lower and upper thresholds  
        self.block_filter_HSV = np.array(([110, 100, 20], [125, 255, 255]), dtype = "uint8")      
        self.color_workspace = np.array(([170, 80, 20], [180, 255, 255]), dtype = "uint8")   
        self.color_buildsite = np.array(([165, 90, 20], [170, 200, 255]), dtype = "uint8")  

        # threshold used for filtering single block during pixel component analysis (element 0 is min and element 1 is max)
        self.block_pixel_thresh = {'width':(50,70), 'height':(50,70), 'area':(2700,3100)}


    def compoundOpenImage(self, src):

        img_erosion     = self.applyErosion(4, src, "cross")
        opened_img      = self.applyDilation(2, img_erosion, "rectangle")
        return opened_img

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

        # unpack connected component tuple 
        (label_count, label_matrix, stats, centroids) = comp_tuple

        # iterate over number of components (hard coded to ignore background)
        for i in range(1, label_count):

            w = stats[i, cv.CC_STAT_WIDTH]
            h = stats[i, cv.CC_STAT_HEIGHT]
            area = stats[i, cv.CC_STAT_AREA]

            if (w < min_width or w > max_width):
                continue
            if (h < min_height or h > max_height): 
                continue                 
            if (area < min_area or area > max_area): 
                continue 

            x = stats[i, cv.CC_STAT_LEFT]
            y = stats[i, cv.CC_STAT_TOP]

            # store this components x and y in stats matrix
            xy_tup = (stats[i, cv.CC_STAT_LEFT], stats[i, cv.CC_STAT_TOP])

            wh_tup = (w, h)                 # store width and height

            (c_x, c_y) = (centroids[i]) # store this components centroid

            component_mask = (label_matrix == i).astype("uint8") * 255

            # cv.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 3)
            # cv.circle(img, (int(cX), int(cY)), 4, (0, 0, 255), -1)
                
            print("x, y :", xy_tup)                
            print("w, h :", wh_tup) 
            print("centroid : %f, %f" % (c_x, c_y)) 
            print("component_mask :", component_mask) 
                

            return [xy_tup, wh_tup, (c_x, c_y)], component_mask  


    def filterColor(self, img_bgr, lower, upper):
        """
        Find indices of image within range of lower and upper bound in HSV color range.
        @param img_bgr - BGR image 
        @param lower - lower bound of HSV range
        @param upper - upper bound of HSV range
        @return: binary mask describing inbound indices """

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



    def findBlock(self, src):

        mask = self.filterColor(src, self.color_block[0], self.color_block[1])
        dst_erode = self.applyErosion(kernal_size=4, src=mask)
        dst_dilate = self.applyDilation(kernal_size=5, src=dst_erode)
        comp = self.getConnectedComponents(mask, connectivity=8)
        x, y, w, h, cX, cY = self.filterComponents(comp)

        bounding_rect = cv.rectangle(src, (x, y), (x + w, y + h), (0, 255, 0), 3)



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


    def convert_bgr2rgb(self, img_bgr):
        return cv.cvtColor(img_bgr, cv.COLOR_BGR2RGB)

    def convert_bgr2grayscale(self, img_bgr):
        return cv.cvtColor(img_bgr, cv.COLOR_BGR2GRAY)


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


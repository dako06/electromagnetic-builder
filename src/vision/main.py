#!/usr/bin/env python3

import cv2 as cv
import numpy as np
import sys
import time
from matplotlib import pyplot as plt

from cv_utilities import ImageProcessor 
 
image_name = "block.png"        # get test image 
img_bgr = cv.imread(image_name) # load test image as BGR 
img = img_bgr.copy()            # copy 

if img_bgr is None:
    sys.exit("Could not read the image.") # exit if issue opening image

# main visualization window 
main_window = 'Main Window'
cv.namedWindow(main_window, cv.WINDOW_AUTOSIZE)

# instantiate classes
# grid        = PixelGrid(img_bgr.shape, 4)
img_pro       = ImageProcessor()
# impipeline  = ImageContainer(10)

# display  properties of original image
# img_pro.displayImgProperties(img_bgr)                                   
# img_pro.displayImg(main_window, img_bgr, 'original_img')   

# starting timer
to = time.time()

# filter image based on HSV color masks 
# mask1 = img_pro.filterColor(img_bgr, img_pro.color_workspace[0], img_pro.color_workspace[1])
mask2 = img_pro.filterColor(img_bgr, img_pro.block_filter_HSV[0], img_pro.block_filter_HSV[1])
# mask3 = img_pro.filterColor(img_bgr, img_pro.color_buildsite[0], img_pro.color_buildsite[1])
mask = mask2

# display properties and image of color mask in binary space
# img_pro.displayImgProperties(mask)            
# img_pro.displayImg(main_window, mask, 'raw_color_mask')  


img_erosion = img_pro.applyErosion(kernal_size=4, src=mask, shape="cross")
# img_pro.displayImgProperties(img_erosion)            
# img_pro.displayImg(main_window, img_erosion, 'eroded_raw_mask') 

img_dilation = img_pro.applyDilation(kernal_size=2, src=img_erosion, shape="rectangle")
# img_pro.displayImg(main_window, img_dilation, 'opened_image.png') 


# # binary mask & original image 
# result = cv.bitwise_and(img_bgr, img_bgr, mask=img_dil)
# img_pro.displayImg('Bitwise and of mask and original', result, 'result_img')  # preview loaded image 


comp = img_pro.getConnectedComponents(mask, connectivity=8)
img_pro.filterComponents(comp, img)

# inline testing of erosion and dilation
# dilation_erosion_test.main_dial(img_ero)

# # ending time
tf = time.time()
print(f"Runtime of the program is {tf-to}")



# #### plot results ####
# plt.subplot(2, 1, 1)
# plt.imshow(mask, cmap="gray")
# plt.title("Color mask")
# plt.subplot(2, 1, 2)
# plt.imshow(result)
# plt.title("Original image after mask")
# plt.show()














# (numLabels, labels, stats, centroids) = comp
# for i in range(0, numLabels):
    
#     if i == 0:
#         text = "examining component {}/{} (background)".format(i + 1, numLabels)
#     # otherwise, we are examining an actual connected component
#     else:
#         text = "examining component {}/{}".format( i + 1, numLabels)
#     # print a status message update for the current connected
#     # component
#     print("[INFO] {}".format(text))
#     # extract the connected component statistics and centroid for
#     # the current label
#     x = stats[i, cv.CC_STAT_LEFT]
#     y = stats[i, cv.CC_STAT_TOP]
#     w = stats[i, cv.CC_STAT_WIDTH]
#     h = stats[i, cv.CC_STAT_HEIGHT]
#     area = stats[i, cv.CC_STAT_AREA]
#     (cX, cY) = centroids[i]
    
#     print("\twidth and height: %d, %d\n" % (w, h))
#     cv.rectangle(original_img, (x, y), (x + w, y + h), (0, 255, 0), 3)
#     cv.circle(original_img, (int(cX), int(cY)), 4, (0, 0, 255), -1)

# 	# construct a mask for the current connected component by
# 	# finding a pixels in the labels array that have the current
# 	# connected component ID
#     componentMask = (labels == i).astype("uint8") * 255
# 	# show our output image and connected component mask
#     cv.imshow("Output", original_img)
#     cv.imshow("Connected Component", componentMask)
#     cv.waitKey(0)






#!/usr/bin/env python3

import cv2 as cv
import numpy as np
import sys
import time
from matplotlib import pyplot as plt

# from survey_workspace.src.camera.cv_utilities import ImageContainer, ImageProcessor
# from survey_workspace.src.camera.pixel_grid import PixelGrid

# import survey_workspace.src.testing.dilation_erosion_test as dilation_erosion_test

#### test image ####
image_name = "test1.png"   # test image 
img_bgr = cv.imread(image_name)     # load test image as BGR 
original_img = img_bgr.copy()       # copy for visualization 
if img_bgr is None:
    sys.exit("Could not read the image.") # exit if issue opening image

#### main testing window ####
main_window = 'Main Window'
cv.namedWindow(main_window)


#### Classes being tested/used ####
grid        = PixelGrid(img_bgr.shape, 4)
impro       = ImageProcessor()
impipeline  = ImageContainer(10)

# impro.displayImgProperties(img_bgr)                           # preview details
impro.displayImg('Image seen by TB3', original_img, 'OG_img')   # preview loaded image 

# starting timer
to = time.time()

#### color masks ####
mask1 = impro.filterColor(img_bgr, impro.color_workspace[0], impro.color_workspace[1])
mask2 = impro.filterColor(img_bgr, impro.color_block[0], impro.color_block[1])
mask3 = impro.filterColor(img_bgr, impro.color_buildsite[0], impro.color_buildsite[1])
mask = mask1

# impro.displayImgProperties(mask)            # preview details
# impro.displayImg('Mask', mask, 'mask_img')  # preview loaded image 


img_ero = impro.applyErosion(kernal_size=2, src=mask, shape="cross")
# impro.displayImg('MASK EROSION', img_ero, 'Mask') 

img_dil = impro.applyDilation(kernal_size=2, src=img_ero, shape="rectangle")
# impro.displayImg('MASK DILATION AFTER EROSION', img_dil, 'Mask') 


# binary mask & original image 
result = cv.bitwise_and(img_bgr, img_bgr, mask=img_dil)
impro.displayImg('Bitwise and of mask and original', result, 'result_img')  # preview loaded image 


# apply erosion with kernal size n = 4 in 2n+1
# comp = impro.getConnectedComponents(mask, connectivity=8)
# impro.filterComponents(comp, original_img)

# inline testing of erosion and dilation
# dilation_erosion_test.main_dial(img_ero)

# ending time
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






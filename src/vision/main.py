#!/usr/bin/env python3
import cv2 as cv
import numpy as np
import sys
import time
from matplotlib import pyplot as plt
from image_processor import ImageProcessor
from image_buffer import ImageBuffer 
from pixel_grid import PixelGrid

TEST = 2

""" setup window """
main_window = 'Main Window'
cv.namedWindow(main_window, cv.WINDOW_AUTOSIZE)

""" classes """
img_pro     = ImageProcessor()
img_buff    = ImageBuffer(5)

""" image properties """
# img_pro.displayImgProperties(img_bgr)                                   
# img_pro.displayImg(main_window, img_bgr, 'original_img')   

# filter image based on HSV color masks 
# mask1 = img_pro.filterColor(img_bgr, img_pro.color_workspace[0], img_pro.color_workspace[1])
# mask3 = img_pro.filterColor(img_bgr, img_pro.color_buildsite[0], img_pro.color_buildsite[1])

# display properties and image of color mask in binary space
# img_pro.displayImgProperties(mask)            
# img_pro.displayImg(main_window, mask, 'raw_color_mask')  

# img_erosion = img_pro.applyErosion(kernal_size=4, src=mask, shape="cross")
# img_pro.displayImgProperties(img_erosion)            
# img_pro.displayImg(main_window, img_erosion, 'eroded_raw_mask') 

# img_dilation = img_pro.applyDilation(kernal_size=2, src=img_erosion, shape="rectangle")



if TEST == 0:
    
    """ block recognition test """

    image_name  = "test_3.png"         
    img_bgr     = cv.imread(image_name)  
    img         = img_bgr.copy()             

    if img_bgr is None:
        sys.exit("Could not read the image.") # exit if issue opening image

    to                          = time.time()
    mask                        = img_pro.filterColor(img_bgr, img_pro.block_filter_HSV[0], img_pro.block_filter_HSV[1])
    opened_mask                 = img_pro.compoundOpenImage(mask)
    # img_pro.displayImg(main_window, opened_mask, 'opened_image.png') 
    comp                        = img_pro.getConnectedComponents(opened_mask, connectivity=8)
    comp_list, label_matrix     = img_pro.filterComponents(comp, img_pro.block_pixel_thresh)
    # print("number of remaining components: ", len(comp_list))
    tf                          = time.time()
    print(f"Runtime of the program is {tf-to}")

    for d in comp_list:

        (x,y)       = d.get('xy')
        (w,h)       = d.get('wh')
        (cX, cY)    = d.get('centroid')
        ix          = d.get('label_id')
        component_mask = (label_matrix == ix).astype("uint8") * 255
        cv.circle(img, (int(cX), int(cY)), 4, (0, 0, 255), -1)
        cv.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 3)
        img_pro.displayImg(main_window ,img, "final.png")
        img_pro.displayImg(main_window ,component_mask, "comp.png")


if TEST == 1:
    
    """     image buffer test   """

    img_cnt = 11
    for i in range(img_cnt):

        test_img_name   = "test_" + str(i) + ".png"
        img_test        = cv.imread(test_img_name)
        
        if img_test is None:
            sys.exit("Could not read the image.") # exit if issue opening image  
        
        img_copy        = img_test.copy() 
        grey_img        = img_pro.convert_bgr2grayscale(img_test) 
        img_buff.assignImg(img_copy)
        
        print("bgr to grey conversion: ", i)
        img_pro.displayImg(main_window , grey_img, "final.png")

    for j in range(img_buff.getItemCount()):

        is_img, img = img_buff.getImg()

        if is_img:
            print("recovered valid image")
            img_pro.displayImg(main_window , img, "final.png")
        else:
            print("unable to recover valid image")

if TEST == 2:

    """ pixel tracker """
    image_name  = "test_3.png"         
    img_bgr     = cv.imread(image_name)  
    img         = img_bgr.copy()  

    pix_grid = PixelGrid(img_bgr.shape, 4)

    to                          = time.time()
    mask                        = img_pro.filterColor(img_bgr, img_pro.block_filter_HSV[0], img_pro.block_filter_HSV[1])
    opened_mask                 = img_pro.compoundOpenImage(mask)
    # img_pro.displayImg(main_window, opened_mask, 'opened_image.png') 
    comp                        = img_pro.getConnectedComponents(opened_mask, connectivity=8)
    comp_list, label_matrix     = img_pro.filterComponents(comp, img_pro.block_pixel_thresh)
    # print("number of remaining components: ", len(comp_list))


    # find nearest object to turtlebot
    obj_dir, nearest_comp, is_centered  = pix_grid.findNearestObject(comp_list) 

    print("direction: ", obj_dir)
    print("nearest component", nearest_comp)
    print("is it centered?", is_centered)
    tf                          = time.time()



# # binary mask & original image 
# result = cv.bitwise_and(img_bgr, img_bgr, mask=img_dil)
# img_pro.displayImg('Bitwise and of mask and original', result, 'result_img')  # preview loaded image 

# inline testing of erosion and dilation
# dilation_erosion_test.main_dial(img_ero)

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






#!/usr/bin/env python3
from cProfile import label
from copy import copy
from statistics import median
from turtle import color, width
from matplotlib import scale
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import cv2 as cv
import time
import sys

from image_processor import ImageProcessor
from image_buffer import ImageBuffer 
from pixel_grid import PixelGrid


main_window = 'Main Window'
cv.namedWindow(main_window, cv.WINDOW_AUTOSIZE)

img_pro     = ImageProcessor()
img_buff    = ImageBuffer(10)

TEST = 4

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

    img_pro.displayImgProperties(img_bgr)                                   
          

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
        print("x,y: %f, %f" % (x,y))
        print("cX,cY: %f, %f" % (cX,cY))
        print("w,h: %f, %f" % (w,h))

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

    """ find nearest object """
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
    obj_position, nearest_comp = pix_grid.findNearestObject(comp_list) 

    tf = time.time()
    
    print("position of object in pixel space: ", obj_position)
    print("nearest component: ", nearest_comp)

if TEST == 3:

    img_cnt = 18

    for i in range(img_cnt):

        print("Image number: ", i)

        name = "./block_test/block_test_" + str(i) + ".png"
        img = cv.imread(name)

 


if TEST == 4:

    distance    = [0.08, 0.10, 0.125, 0.15, 0.175, 0.20, 0.225, 0.25, 0.275, 0.30] 
    columns     = ['distance', 'x', 'y', 'cent_x', 'cent_y', 'width', 'height', 'area']

    index         = len(distance)
    sample_shape  = (1, len(columns))

    data = np.zeros(shape=sample_shape)
    time_cost = []

    view_filtered = True

    # iterate over reach distance 
    for i in range(0, index):

        # get each sample at the given distance
        for j in range(2):

            if j == 0:
                sub_name = "a"
            elif j == 1:
                sub_name = "b"
    
            # print("Image %s" % (str(i) + sub_name))    
            path = "./distance_tests/label_test_" + str(i) + sub_name + ".png"
            img = cv.imread(path)

            if img is None:
                print("path: ", path)
                sys.exit("Could not read the image.") # exit if issue opening image

            img_pro.displayImg(main_window, img, 'img')      

            # copies used for processing visualization
            img_copy_1 = img.copy()
            img_copy_2 = img.copy()
            img_copy_3 = img.copy()
            img_copy_4 = img.copy()
            img_copy_5 = img.copy()

            # start time to measure analysis
            

            # # apply color filter  
            # block_mask = img_pro.filterColor(img_copy_1, img_pro.block_filter_HSV)               
            # img_pro.displayImg("raw blue mask", block_mask, 'img')    

            
            # """ process blue filtered blobs """

            # # open blue mask
            # opened_blue = img_pro.compoundOpenImage(block_mask, img_pro.block_open_param)
            # # img_pro.displayImg("opened blue mask", opened_blue, 'img')      

            # blue_comps = img_pro.getConnectedComponents(opened_blue, connectivity=8)  
            # blue_inliers = img_pro.filterComponents(blue_comps, img_pro.block_thresh)
            # # print("number of blue inlier indices found: ", len(blue_inliers))            
            # # img_pro.viewComponents(img_copy_4, blue_comps, blue_inliers, verbose=True)

            # """ process inverse """
            # # img_pro.displayImg("opened inverse", opened_inverse, 'img')      
            # inverse         = cv.bitwise_not(block_mask)
            # opened_inverse  = img_pro.compoundOpenImage(inverse, img_pro.metal_open_param)  

            # metal_comps   = img_pro.getConnectedComponents(opened_inverse, connectivity=8) 
            # metal_inliers = img_pro.filterComponents(metal_comps, img_pro.metal_thresh)
            # # print("number of metal inlier indices found: ", len(metal_inliers))            
            # # img_pro.viewComponents(img_copy_2, metal_comps, metal_inliers, verbose=True)

            to = time.time() 
            block_list = img_pro.detectBlocks(img)
            tf = time.time()
            print('%d labels took %f s to process' % (len(block_list), (tf-to)))

            labeled_img = img_pro.labelBlocks(src_img=img, block_list=block_list)
            window_img = img_pro.drawDetectionWindow(labeled_img)            
            
            nearest_block, pixel_vector, obj_position = img_pro.getBlockTarget(block_list) 
            
            if pixel_vector[0] == 0 and pixel_vector[1] == 0:
                print("no pixel vector returned")
            else:  
                p1 = img_pro.pix_grid.pixel_anchor
                cv.line(window_img, p1, pixel_vector, (0,255,255), 1)


            img_pro.displayImg("labeled image with window", window_img, 'img')




            # # filtered_list = inverse_filtered
            # filtered_list = filtered_blue

            # # comps = inverse_comps
            # comps = comps_blue

            # """ show filtered or non-filtered results """
            # if view_filtered:

            #     print("number of filtered elements: ", len(filtered_list))
            #     for c in filtered_list:

            #         (x,y)       = c.get('xy')
            #         (w,h)       = c.get('wh')
            #         (cX, cY)    = c.get('centroid')
            #         ix          = c.get('label_id')
            #         area        = c.get('area')

            #         # component_mask = (label_matrix == ix).astype("uint8") * 255

            #         sample = np.array([distance[i], x, y, cX, cY, w, h, area])
            #         data = np.concatenate((data, sample.reshape(sample_shape)),axis=0)

            
            # else:

            #     # unpack un-filtered components 
            #     (label_count, label_matrix, stats, centroids) = comps

            #     # ignore back ground
            #     for k in range(1, label_count):

            #         cX, cY  = centroids[k]
            #         sample = np.array([distance[i], stats[k, cv.CC_STAT_LEFT], stats[k, cv.CC_STAT_TOP], \
            #           cX, cY, stats[k, cv.CC_STAT_WIDTH], stats[k, cv.CC_STAT_HEIGHT], stats[k, cv.CC_STAT_AREA]])
            #         data = np.concatenate((data, sample.reshape(sample_shape)),axis=0)




            # time_cost.append(tf-to)

    
    # df = pd.DataFrame(data=data[1:,:], columns=columns)
    
    # """ plot results """

    # y = df['y'].values
    # x = df['x'].values
    # d = df['distance'].values
    # w = df['width'].values
    # h = df['height'].values
    # area = df['area'].values
    # cy = df['cent_y'].values
    # cx = df['cent_x'].values

    # b = 30
    # index = len(y)

    # print("Total number of labels resulting from connected component analysis: %d" % index)

    # """ estimated features """

    # median_width    = median(w)
    # mean_width      = np.mean(w)
    # median_height   = median(h)
    # mean_height     = np.mean(h)

    # res_med_width   = w - median_width
    # res_mean_width  = w - mean_width
    # res_med_height  = h - median_height
    # res_mean_height = h - mean_height

    # stretch = w/h
    # strain = h/w
  


    # """ plot time cost """
    # # figt, axt = plt.subplots()
    # # t_ix = np.arange(0, len(time_cost), 1)
    # # axt.plot(t_ix, time_cost)
    # # axt.set_title('Time Cost vs distance')

    # """ feature analysis in pixel area """
    # fig, ax = plt.subplots(2,2)

    # ax[0, 0].hist(stretch, bins=100)
    # ax[0, 0].set_title('w/h ratio')
    # ax[0, 0].grid()

    # ax[0, 1].scatter(stretch, np.negative(cy), color='red', s=5)
    # ax[0, 1].set(xlabel='w/h ratio', ylabel='row index',title='centroid-y vs. w/h ratio')
    # ax[0, 1].grid()

    # ax[1, 0].hist(strain, bins=75)
    # ax[1, 0].set_title('h/w ratio')
    # ax[1, 0].grid()

    # ax[1, 1].scatter(d, stretch, color='red', s=5, label='w/h')
    # ax[1, 1].scatter(d, strain, color='blue', s=5, label='h/w')
    # ax[1, 1].set(xlabel='[m]', ylabel='ratio',title='ratio vs. distance')    
    # ax[1, 1].grid()
    # ax[1, 1].legend()

    # plt.show()



    # # """ analysis of pixel area """
    # fig, ax = plt.subplots(1,3)
    
    # ax[0].hist(area, bins=50)
    # ax[0].set_title('pixel area distribution')


    # ax[1].scatter(area, np.negative(y), color='red', s=5)
    # ax[1].set_title('row vs. pixel area')
 
    # ax[2].scatter(d, area, color='red', s=5)
    # ax[2].set_title('area vs. distance [m]')

    # plt.show()


    # # """ analysis of pixel width """
    # fig1, ax1 = plt.subplots(2,2)
    
    # ax1[0, 0].hist(w, bins=100)
    # ax1[0, 0].set_title('pixel width distribution')

    # ax1[0, 1].scatter(w, np.negative(cy), color='red', s=5)
    # ax1[0, 1].set_title('row centroid vs. pixel width')

    # ax1[1, 0].scatter(cx, w, color='red', s=5)
    # ax1[1, 0].set_title('pixel width vs. column centroid')

    # ax1[1, 1].scatter(d, w, color='red', s=5)
    # ax1[1, 1].set_title('pixel width vs. distance [m]')

    # plt.show()


    # fig, ax = plt.subplots()
    # ax.scatter(w, h, color='blue', s=5)
    # ax.scatter(median_width, median_height, color='green', s=5, label='median')
    # ax.scatter(mean_width, mean_height, color='red', s=5, label='mean')
    # ax.set(xlabel='width', ylabel='height',title='pixel height vs. pixel width') 
    # ax.grid() 
    # ax.legend()
    # plt.show()



    # # """ analysis of pixel height """
    # fig2, ax2 = plt.subplots(1,3)
    
    # ax2[0].hist(h, bins=100)
    # ax2[0].set_title('pixel height distribution')

    # ax2[1].scatter(h, np.negative(cy), color='red', s=5)
    # ax2[1].set(xlabel='height', ylabel='row centroid',title='row centroid vs. pixel height')
    # ax2[1].grid()    

    # ax2[2].scatter(d, h, color='red', s=5)
    # ax2[2].set(xlabel='[m]', ylabel='height',title='pixel height vs. distance ')    
    # ax2[2].grid()    
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






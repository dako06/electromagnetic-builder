#!/usr/bin/env python3

from re import T
import rospy
from sensor_msgs.msg import Image
from electromagnetic_builder.msg import GUI_State

import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError

from vision import image_processor
from vision import building_block


class GUI:

    def __init__(self) -> None:

        rospy.init_node('gui_node', anonymous=True)
        self.image_sub = rospy.Subscriber("/camera/image", Image, self.gui_callback)

        # subscribe to active state of build process 
        self.gui_state_sub  = rospy.Subscriber("gui_state", GUI_State, self.gui_state_callback)
        self.gui_state      = GUI_State(state="block_detection",target_acquired=False)

        # cv bridge for converting raw ROS image
        self.cv_bridge  = CvBridge()
        self.rate       = rospy.Rate(100)
        
        # window maintained throughout GUI duration
        self.gui_name   = 'Electromagnetic Builder Vision'
        self.refresh    = 10
        cv.namedWindow(self.gui_name, cv.WINDOW_AUTOSIZE)
        
        self.img_pro    = image_processor.ImageProcessor()

        self.target_set = False
        self.target_block       = building_block.Block(0,0,0,(0,0),(0,0))
        self.prev_target_block  = building_block.Block(0,0,0,(0,0),(0,0))

        self.valid_states = ["track_target_block", "block_detection", "track_block_transport", "debug"]

    def gui_callback(self, img):

    
        # convert from raw ROS image to opencv image  
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            print(e)

        # update state variable
        # state = "track_target_block"
        state = self.gui_state.state
        
        # process image  
        if state == "block_detection":

            block_list                          = self.img_pro.detectBlocks(src_img=cv_image)
            sorted_blocks, block_target_specs   = self.img_pro.getBlockTarget(block_list)
            
            # label block borders and centroids
            if len(sorted_blocks) != 0:
                disp_img = self.drawBlocks(src_img=cv_image, block_list=sorted_blocks)
            else:
                disp_img = cv_image
                    
        elif state == "track_target_block" and self.target_set:
            
            # mark the target block with a red cross hair and draw window used to center target block
            disp_img    = self.markTargetBlock(cv_image, self.target_block, (0,0,255))
            disp_img    = self.drawDetectionWindow(disp_img)
            
            # to enable visualization of block movement
            # disp_img = self.markTargetBlock(disp_img, self.prev_target_block, (193, 182, 255))
            
        elif state == "track_block_transport":
            is_detected, pixel_features = self.img_pro.trackBlockTransport(cv_image)
            
            if is_detected:
                disp_img = self.drawBlockConnectionWindow(cv_image, pixel_features)
            else:
                disp_img = cv_image

        elif state == "debug":
            
            block_mask          = self.img_pro.filterColor(img_bgr=cv_image, filter=self.img_pro.block_filter_HSV)
            opened_block_mask   = self.img_pro.compoundOpenImage(src_img=block_mask, param_dict=self.img_pro.block_open_param)
            # block_comps         = self.img_pro.getConnectedComponents(img_binary=opened_block_mask, connectivity=8)  
            # block_label_list    = self.filterComponents(comp_tuple=block_comps, threshold=self.block_thresh)
            disp_img = opened_block_mask

        else:
            disp_img = cv_image

        # refresh window
        # cv.resize(disp_img, (840, 680))
        cv.imshow(self.gui_name, disp_img)
        cv.waitKey(self.refresh)
        self.rate.sleep()


    def drawBlocks(self, src_img, block_list):
        """ take a list of block objects and mark the source image with the block positions """

        for block in block_list:

            cx, cy      = block.centroid
            x, y        = block.pixel_o
            width       = block.pixel_dim.get('width')
            height      = block.pixel_dim.get('height')

            # draw bounding square of blocks
            cv.rectangle(src_img, (x, y), (x + width, y + height), (0, 200, 0), 2)
            
            # draw centroid and its numerical value
            cv.circle(src_img, (int(cx), int(cy)), 4, (0, 0, 255), -1)            
            centroid_txt = "(" + str(round(cx, 1)) + ", "+ str(round(cy, 1))  + ")"
            cv.putText(src_img, centroid_txt, (int(cx-35),int(cy-20)), cv.FONT_HERSHEY_SIMPLEX, 0.4,(0,0,255), 1, cv.LINE_AA)
            
            # draw each metal blob within the block
            for metal in block.metal_blobs:            
                cv.rectangle(src_img, metal.get('p_min'), metal.get('p_max'),  (200, 0, 0), 2)

        return src_img

    def drawBlockConnectionWindow(self, src_img, pixel_features):
        
        x,y           = pixel_features.get('po')
        width, height = pixel_features.get('blob_dim')
        cv.rectangle(src_img, (x, y), (x + width, y + height), (226, 43, 138), 2)
        return src_img

    def drawDetectionWindow(self, src_img):
        """ draw yellow boundaries used for centering a block for extraction """
        
        min_1 = self.img_pro.pix_grid.scanning_border.get('start_line_1')
        max_1 = self.img_pro.pix_grid.scanning_border.get('end_line_1')
        min_2 = self.img_pro.pix_grid.scanning_border.get('start_line_2')
        max_2 = self.img_pro.pix_grid.scanning_border.get('end_line_2')
        
        cv.line(src_img, min_1, max_1, (0,255,255), 2)
        cv.line(src_img, min_2, max_2, (0,255,255), 2)

        return src_img

    def markTargetBlock(self, src_img, target_block, color_tuple):

        w       = target_block.pixel_dim.get('width')  
        h       = target_block.pixel_dim.get('height')  
        cx, cy  = target_block.centroid
        x,y     = target_block.pixel_o 

        if w > h:
            radius = int(w*0.60)
        else: 
            radius = int(h*0.60)

        line_1_start    = (int(cx), int(cy + radius))
        line_1_end      = (int(cx), int(cy - radius))
        line_2_start    = (int(cx - radius), int(cy))
        line_2_end      = (int(cx + radius), int(cy))

        cv.circle(src_img, (int(cx), int(cy)), radius, color_tuple, 2)
        
        # set cross hairs
        cv.line(src_img, line_1_start, line_1_end, (0,0,255), 1)
        cv.line(src_img, line_2_start, line_2_end, (0,0,255), 1)
        
        centroid_txt = "(" + str(round(cx, 1)) + ", "+ str(round(cy, 1))  + ")"
        
        cv.putText(src_img, centroid_txt, (int(cx),int(cy)), cv.FONT_HERSHEY_SIMPLEX, 0.4,(0,0,255), 1, cv.LINE_AA)
    
        # write target block label
        label_coordinate = (int(cx), int(cy - h//2 + 10))
        cv.putText(src_img, "Target Block", label_coordinate, cv.FONT_HERSHEY_SIMPLEX, 0.4,(0,0,255), 1, cv.LINE_AA)
        return src_img


    def gui_state_callback(self, msg):
        # update gui state based on sm node transitions
        
        prev_state              = msg.state
        self.gui_state.state    = msg.state

        if prev_state != msg.state:
            rospy.loginfo('State updated to: %s' % msg.state)

        if msg.target_acquired and msg.state == "track_target_block":

            cx,cy   = msg.target_centroid_x, msg.target_centroid_y
            x,y     = msg.target_xo, msg.target_yo 
            w       = msg.target_width
            h       = msg.target_height

            self.prev_target_block.updateBlock(self.target_block)
            temp_block = building_block.Block(w, h, 0, (cx,cy), (x,y))
            self.target_block.updateBlock(temp_block)

            self.target_set = True

        if not msg.target_acquired:
            self.target_set = False



if __name__ == '__main__':

    gui = GUI()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down GUI")
    finally:
        cv.destroyAllWindows()

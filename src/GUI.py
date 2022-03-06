#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from electromagnetic_builder.msg import GUI_State

import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError

from vision import image_processor


class GUI:

    def __init__(self) -> None:

        rospy.init_node('gui_node', anonymous=True)
        self.image_sub = rospy.Subscriber("/camera/image", Image, self.gui_callback)

        # subscribe to active state of build process 
        self.gui_state_sub      = rospy.Subscriber("gui_state", GUI_State, self.gui_state_callback)
        self.gui_state          = GUI_State()
        self.gui_state.state    = "non_action"

        # cv bridge for converting raw ROS image
        self.cv_bridge  = CvBridge()
        self.rate       = rospy.Rate(100)
        
        # window maintained throughout GUI duration
        self.gui_name   = 'Main Window'
        self.refresh    = 10
        cv.namedWindow(self.gui_name, cv.WINDOW_AUTOSIZE)
        
        self.img_pro    = image_processor.ImageProcessor()

    def gui_callback(self, img):

        # convert from raw ROS image to opencv image  
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            print(e)

        # update state variable
        state = "block_detection"
        # state = self.gui_state.state
        
        # process image  
        if state == "block_detection":

            block_list  = self.img_pro.detectBlocks(src_img=cv_image)
            block_img   = self.img_pro.labelBlocks(src_img=cv_image, block_list=block_list)
            disp_img    = self.img_pro.drawDetectionWindow(block_img)
                    
            nearest_block, pixel_vector, obj_position = self.img_pro.getBlockTarget(block_list) 
            
            if pixel_vector[0] == 0 and pixel_vector[1] == 0:
                print("no pixel vector returned")
            else:  
                p1 = self.img_pro.pix_grid.pixel_anchor
                cv.line(disp_img, p1, pixel_vector, (255,0,255), 1)


        elif state == "track_block_transport":
            disp_img = self.img_pro.trackBlockTransport(cv_image)
        
        elif state == "non_action":
            disp_img = cv_image

        # self.img_pro.displayImg("test_window", cv_image, "label_test_9a.png")
        # self.img_pro.displayImg("test_window", cv_image, "label_test_9b.png")

        # refresh window
        cv.imshow(self.gui_name, disp_img)
        cv.waitKey(self.refresh)
        self.rate.sleep()     


    def gui_state_callback(self, msg):
        # update gui state based on sm node transitions
        self.gui_state.state = msg.state

if __name__ == '__main__':

    gui = GUI()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down GUI")
    finally:
        cv.destroyAllWindows()

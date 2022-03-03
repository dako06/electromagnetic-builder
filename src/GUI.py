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
        state = "track_block_transport"
        # state = self.gui_state.state
        
        # process image  
        if state == "block_detection":
            img = self.img_pro.detectBlocks(cv_image)

        elif state == "track_block_transport":
            img = self.img_pro.trackBlockTransport(cv_image)
        
        elif state == "non_action":
            img = cv_image

        # refresh window
        cv.imshow(self.gui_name, img)
        cv.waitKey(self.refresh)
        self.rate.sleep()     


    def gui_state_callback(self, msg):
        # update gui state based on sm node transitions
        self.current_state.state = msg.state

if __name__ == '__main__':

    gui = GUI()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down GUI")
    finally:
        cv.destroyAllWindows()

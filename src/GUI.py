#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from electromagnetic_builder.msg import gui_state


import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError

from vision import image_processor


class GUI:

    def __init__(self) -> None:


        rospy.init_node('gui_node', anonymous=True)
        self.image_sub = rospy.Subscriber("/camera/image", Image, self.gui_callback)

        self.gui_state_sub = rospy.Subscriber("gui_state", gui_state, self.gui_state_callback)
        self.current_state = gui_state()
        self.current_state = "non_action"

        self.cv_bridge  = CvBridge()
        self.rate       = rospy.Rate(100)
        
        self.gui_name   = 'Main Window'
        self.refresh    = 10
        
        self.img_pro    = image_processor.ImageProcessor()

        cv.namedWindow(self.gui_name, cv.WINDOW_AUTOSIZE)


    def gui_callback(self, img):

        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            print(e)

        # state = "track_block_transport"
        state = "block_detection"
        # self.current_state
        
        if state == "block_detection":
            img = self.img_pro.detectBlocks(cv_image)

        elif state == "track_block_transport":
            img = self.img_pro.trackBlockTransport(cv_image)
        
        elif state == "non_action":
            img = cv_image


    
        cv.imshow(self.gui_name, img)
        cv.waitKey(self.refresh)
        self.rate.sleep()     


    def gui_state_callback(self, msg):

        self.current_state = msg.state

if __name__ == '__main__':

    gui = GUI()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
    finally:
        cv.destroyAllWindows()

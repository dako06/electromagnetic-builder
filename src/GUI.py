#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image

import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError


class GUI:

    def __init__(self) -> None:


        rospy.init_node('gui_node', anonymous=True)
        self.image_sub = rospy.Subscriber("/cv_camera/image_raw", Image, self.gui_callback)

        self.cv_bridge  = CvBridge()
        self.rate       = rospy.Rate(10)
        
        self.gui_name   = 'Main Window'
        
        cv.namedWindow(self.gui_name, cv.WINDOW_AUTOSIZE)


    def gui_callback(self, img):

        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv.imshow(self.gui_name, cv_image)
        self.rate.sleep()     




if __name__ == '__main__':

    gui = GUI()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")

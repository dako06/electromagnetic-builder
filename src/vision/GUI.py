#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image


def update_callback(img):
    pass

if __name__ == '__main__':
    rospy.init_node('GUI_node', anonymous=True)
    image_sub = rospy.Subscriber("tbd", Image, update_callback)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")

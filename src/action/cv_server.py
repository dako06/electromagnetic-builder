#!/usr/bin/env python3

import rospy
import tf

from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D
from actionlib import SimpleActionServer 
from sensor_msgs.msg import Image

from electromagnetic_builder.msg import VisionActionFeedback
from electromagnetic_builder.msg import VisionAction
from electromagnetic_builder.msg import VisionActionResult

import numpy as np
from math import pi
from cv_bridge import CvBridge, CvBridgeError

from utilities import decision_switch, controller, nav_utilities
from vision import image_buffer, image_processor

class VisionActionServer(object):
    
    def __init__(self, name):

        #### intialize server ####
        self.action_name = name
        self.action_server = SimpleActionServer(self.action_name, VisionAction, execute_cb=self.execute_callback, auto_start = False)
        self.action_server.start()

        # navigation 
        self.vel                = Twist()   
        self.vel_pub            = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.rate               = rospy.Rate(10)
        self.lin_vel_ref        = 0.17
        self.scan_vel_ref       = 0.1

        # odometry
        self.pose               = Pose2D()
        self.odom_sub           = rospy.Subscriber("odom", Odometry, self.odom_callback)

        # subscribe to raw image feed
        self.camera_sub = rospy.Subscriber("/camera/image", Image, self.image_callback)
        self.cv_bridge = CvBridge()

        # support objects for image processing
        self.controller = controller.Controller()
        self.img_pro    = image_processor.ImageProcessor()
        self.img_buffer = image_buffer.ImageBuffer(5)


        self.cmd_dict   = ["locate_block"]
        self.dswitch         = decision_switch.DecisionBlock(self.cmd_dict)

    def execute_callback(self, goal):
        
        # intialize action messages 
        result      = VisionActionResult()
        feedback    = VisionActionFeedback()

        rospy.loginfo('%s: Executing command %s' % (self.action_name, goal.command))
        
        if goal.command == "locate_block":
            rospy.loginfo('%s: Scanning field for object' % self.action_name)
            scan_outcome = self.fieldScan(self.impro.block_filter_HSV)


        # check for preempt request from client 
        # if self.action_server.is_preempt_requested():
        #     rospy.loginfo('%s: Preempted' % self.action_name)
        #     self.action_server.set_preempted() 
        # # feedback.percent_complete = 
        # self.action_server.publish_feedback(feedback) 

        rospy.loginfo('%s: Succeeded' % self.action_name)
        self.action_server.set_succeeded(result)


    def fieldScan(self, filter):
        """ @param filter: use filter to rotate and scan field for object """

        theta_sum, theta_prev, theta_i = 0, 0, 0
        
        pixel_coordinates   = []

        while len(pixel_coordinates) == 0 and theta_sum < 2*pi:
            
            is_img, img = self.img_buffer.getImg()

            # skip if image is not available
            if not is_img:
                continue
            
            # self.img_pro.displayImg("temp", img, "temp.png")

            
            mask                    = self.img_pro.filterColor(img, filter[0], filter[1])                   # apply color filter to mask
            opened_mask             = self.img_pro.compoundOpenImage(mask)                                  # open image to remove outliers
            comp                    = self.img_pro.getConnectedComponents(opened_mask, connectivity=8)      # get connected components
            comp_list, label_matrix = self.img_pro.filterComponents(comp, self.img_pro.block_pixel_thresh)  # filter outlier components

            

            # update theta for angle loop condition 
            theta_i     = self.pose.theta
            theta_sum  += (theta_i-theta_prev)
            theta_prev  = theta_i

            # publish angular velocity
            self.vel.angular.z = self.scan_vel_ref
            self.vel_pub.publish(self.vel)
            self.rate.sleep()
                                

    """ callback functions """

    def image_callback(self, img):
        
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv_image_copy = cv_image.copy()
        
        if cv_image_copy is None:
            return
        else:
            self.img_buffer.assignImg(img.copy()) 

    def odom_callback(self, msg):
        # get pose = (x, y, theta) from odometry topic
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y




if __name__ == '__main__':

    #### intialize server node and class ####
    rospy.init_node('vision_action')
    nav_server = VisionActionServer(rospy.get_name())
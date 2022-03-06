#!/usr/bin/env python3

import rospy
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D
from actionlib import SimpleActionServer 
from sensor_msgs.msg import Image
from electromagnetic_builder.msg import VisionFeedback, VisionAction, VisionResult

import numpy as np
from math import pi
from cv_bridge import CvBridge, CvBridgeError

from utilities import controller, nav_utilities
from vision import image_buffer, image_processor, pixel_grid

class VisionActionServer(object):
    
    def __init__(self):

        # initialize ROS node 
        rospy.init_node('vision_action')

        # intialize server 
        self.action_name = rospy.get_name()
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
        self.img_buffer = image_buffer.ImageBuffer(10)
        self.pix_grid   = pixel_grid.PixelGrid((480,640), 4)


    def execute_callback(self, goal):
        
        # intialize action messages 
        result      = VisionResult()
        feedback    = VisionFeedback()

        rospy.loginfo('%s: Executing command %s' % (self.action_name, goal.command))
        
        if goal.command == "locate_block":

            rospy.loginfo('%s: Scanning field for object' % self.action_name)

            execution_status, obj_comp_stats = self.fieldScan(self.img_pro.block_filter_HSV)
            result.execution_status = execution_status

            # TODO check if its stacked and return level 


        # return server outcome
        if result.execution_status:
            rospy.loginfo('%s: Succeeded' % self.action_name)
            self.action_server.set_succeeded(result)
        
        else:
            rospy.loginfo('%s: Failed' % self.action_name)
            self.action_server.set_aborted(result)

        # check for preempt request from client 
        # if self.action_server.is_preempt_requested():
        #     rospy.loginfo('%s: Preempted' % self.action_name)
        #     self.action_server.set_preempted() 
        # # feedback.percent_complete = 
        # self.action_server.publish_feedback(feedback) 


    def fieldScan(self, filter):
        """ @param filter: filter used as identification during field scan 
            @note this function commands the robot to scan the field until an object is
                identified by the filter and its centered with the forward direction of the robot """

        theta_sum, theta_prev, theta_i = 0, 0, 0
        
        obj_position = "unknown"

        while obj_position != "center" and theta_sum < 2*pi and not rospy.is_shutdown():
            
            is_img, img = self.img_buffer.popImg()

            # skip if image is not available
            if not is_img:
                rospy.loginfo('image is invalid')
                continue
 
            # apply color filter to image
            mask = self.img_pro.filterColor(img, filter[0], filter[1])                   
            
            # apply compound open to image to remove outliers
            opened_mask = self.img_pro.compoundOpenImage(mask)                                  
            
            # get connected components
            comp = self.img_pro.getConnectedComponents(opened_mask, connectivity=8)      
            
            # filter outlier components
            comp_list, label_matrix = self.img_pro.filterComponents(comp, self.img_pro.block_thresh)  

            # find nearest connected component to base pixel
            obj_position, nearest_comp  = self.pix_grid.findNearestObject(comp_list) 

            print("obj position:", obj_position)
            # set angular scan velocity direction
            if obj_position == "right": 
                self.vel.angular.z = -1 * self.scan_vel_ref
            elif obj_position == "left" or obj_position == "unknown":
                self.vel.angular.z = self.scan_vel_ref
            elif obj_position == "center":
                self.vel.angular.z = 0

            print("self.vel.angular.z:", self.vel.angular.z)

            # update theta for angle loop condition 
            theta_i     = self.pose.theta
            theta_sum  += abs(theta_i-theta_prev)
            theta_prev  = theta_i

            # publish angular velocity
            self.vel_pub.publish(self.vel)
            self.rate.sleep()
        
        # return success indicator and component
        if len(nearest_comp) == 0 or obj_position != "center":
            return False, []
        else:
            return True, nearest_comp
        
                                

    """ callback functions """

    def image_callback(self, img):
        
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(img, "bgr8")
            self.img_buffer.pushImg(cv_image)
        
        except CvBridgeError as e:
            print(e)



    def odom_callback(self, msg):
        # get pose = (x, y, theta) from odometry topic
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y


if __name__ == '__main__':

    # intialize server node and class 
    cv_server = VisionActionServer()
    rospy.spin()
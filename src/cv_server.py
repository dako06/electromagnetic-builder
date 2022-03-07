#!/usr/bin/env python3

import rospy
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D
from actionlib import SimpleActionServer 
from sensor_msgs.msg import Image
from electromagnetic_builder.msg import VisionFeedback, VisionAction, VisionResult
from electromagnetic_builder.msg import GUI_State



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

        self.gui_update_pub = rospy.Publisher("gui_state", GUI_State, queue_size=10)
        self.gui_state      = GUI_State(state="block_detection")

        # navigation 
        self.vel                = Twist()   
        self.vel_pub            = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.rate               = rospy.Rate(10)
        self.lin_vel_ref        = 0.17
        self.scan_vel_ref       = 0.08

        # odometry
        self.pose               = Pose2D()
        self.odom_sub           = rospy.Subscriber("odom", Odometry, self.odom_callback)

        # subscribe to raw image feed
        self.camera_sub = rospy.Subscriber("/camera/image", Image, self.image_callback)
        self.cv_bridge = CvBridge()

        # support objects for image processing
        self.controller = controller.Controller()
        self.img_pro    = image_processor.ImageProcessor()
        self.img_buffer = image_buffer.ImageBuffer(15)

        self.block_trajectory       = {}
    
        self.epsilon                = 0.05
    
        self.alpha_wz               = 0         # scalar for controlling angular velocity as a function of dx
        self.extraction_goal        = (0,0)

        self.target_block           = None

        # self.block_list             = []

    def execute_callback(self, goal):
        
        # intialize action messages 
        result      = VisionResult()
        feedback    = VisionFeedback()

        rospy.loginfo('%s: Executing command %s' % (self.action_name, goal.command))

        
        """ execute requested command """
        if goal.command == "center_candidate_block":
            
            # scan field, find nearest block, track until robot is in position
            rospy.loginfo('%s: Scanning field for target block' % self.action_name)

            # sets the target block and the specifications to meet 
            if self.setTargetBlock():

                cx, cy = self.target_block.centroid
                rospy.loginfo('%s: Target block acquired. Centroid of target is (%s, %s)'% (self.action_name, int(round(cx)), int(round(cy))))
                
                # build geofence and check if target block is already within window
                min_pt          = self.img_pro.pix_grid.scanning_border.get('start_line_1')
                max_pt          = self.img_pro.pix_grid.scanning_border.get('end_line_2')
                pixel_geofence  = {'min_coordinate': min_pt, 'max_coordinate':max_pt}

                if self.img_pro.isBlobInBlob(pixel_geofence, self.target_block.centroid):
                    rospy.loginfo('%s: Target block is already within extraction window' % self.action_name)
                    result.execution_status = True
                
                else:
                
                    rospy.loginfo('%s: Attempting to position block within extraction window' % self.action_name)
                    # attempt to center target block with extraction window
                    is_centered = self.centerTargetBlock()
                    result.execution_status = is_centered

            else:
                result.execution_status = False
                rospy.loginfo('%s: Failed to find target block within 360 degrees range of view' % self.action_name)
             
        else:

            result.execution_status = False
            rospy.loginfo('%s: Invalid command requested' % self.action_name)


        """ update server with result """
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



    def setTargetBlock(self):
        """@note this function commands the robot to scan the field until an object is identified 
            by the filter and its centered with the forward direction of the robot """
        
        theta_sum, theta_prev, theta_i = 0, 0, 0
        
        sorted_blocks = []

        # run until a distance vector is found
        while theta_sum < 2*pi and not rospy.is_shutdown():
            
            # pop image from image buffer
            is_img, img = self.img_buffer.popImg()

            if not is_img:
                
                # halt scan if image is not available
                rospy.loginfo('Image unavailable, temporarily halting scan')
                self.vel.angular.z = 0
            
            else:
            
                block_list                      = self.img_pro.detectBlocks(src_img=img)                # find blocks in field of view
                sorted_blocks, block_trajectory = self.img_pro.getBlockTarget(block_list=block_list)    # sort based on pixel distance and get nearest
    
                # set angular scan velocity, default to rotate positive around z axis
                self.vel.angular.z = self.scan_vel_ref

                # update theta for angle loop condition 
                theta_i     = self.pose.theta
                theta_sum  += abs(theta_i-theta_prev)
                theta_prev  = theta_i
                rospy.loginfo('Total scan: %s radians', str(round(theta_sum,2)))

            if len(sorted_blocks) != 0:
                break

            # publish angular velocity
            self.vel_pub.publish(self.vel)
            self.updateGUIMsg("block_detection")
            self.rate.sleep()

        # stop rotation
        self.vel.angular.z = 0
        self.vel_pub.publish(self.vel)
        self.rate.sleep()


        # return success indicator and component
        if len(sorted_blocks) != 0:  
            self.block_list         = sorted_blocks
            self.block_trajectory   = block_trajectory
            self.target_block       = sorted_blocks[0]
            self.updateGUIMsg("track_target_block")
            return True
        
        else:
            return False

    
    def centerTargetBlock(self):

        # track the inverse of the slope until dx/dy is very small 
        po          = self.block_trajectory.get('p_o')
        pb          = self.block_trajectory.get('p_f')
        inv_slope   =  (pb[0]- po[0]) / (-1 * (pb[1]- po[1]))

        rospy.loginfo('Tracking inverse slope initially set to: %s', str(round(inv_slope,4)))


        # initialize theta accumulator
        theta_sum, theta_prev, theta_i = 0, 0, 0

        # guess used during correspondence, rotating right means -wz and correspondence shifts to the left over time 
        direction   = self.block_trajectory.get('direction')
        
        """ set multiplier for sign of angular rotation 
             -1 rotation towards right of the screen 
             1 rotation towards left of the screen """ 
        wz_sign = direction     
    
        # set threshold for successful termination
        max_slope = self.epsilon
        min_slope = -1 * self.epsilon
        
        # iterate until shutdown or epsilon > slope > -epsilon
        while not (inv_slope < max_slope and inv_slope > min_slope) and theta_sum < 2*pi and not rospy.is_shutdown():
            
            # pop image from image buffer
            is_img, img = self.img_buffer.popImg()

            if not is_img:
                # halt scan if image is not available
                rospy.loginfo('Image unavailable, temporarily halting scan')
                self.vel.angular.z = 0
            
            else:
                
                # find blocks in field of view
                block_list = self.img_pro.detectBlocks(src_img=img) 

                # check if blocks were found for this iteration
                if len(block_list) != 0 :

                    # search list for correspondence to target block
                    is_matched, corresp_block = self.img_pro.getBlockCorrespondence(block_list=block_list, target_block=self.target_block, direction=direction)

                    # update features of the target block
                    if is_matched:
                        self.target_block.updateBlock(corresp_block)

                        # update inverse slope condition
                        cx, cy = self.target_block.centroid
                        inv_slope = (cx - po[0]) / (-1 * (cy- po[1]))
                        rospy.loginfo('Block correspondence succesful, inverse slope updated: %s', str(round(inv_slope,4)))


                # set angular scan velocity
                self.vel.angular.z = wz_sign * self.scan_vel_ref

                # update theta for angle loop condition 
                theta_i     = self.pose.theta
                theta_sum  += abs(theta_i-theta_prev)
                theta_prev  = theta_i
                rospy.loginfo('Total scan: %s radians', str(round(theta_sum,2)))

            
        
            # keep loop at defined rate
            self.vel_pub.publish(self.vel)
            self.updateGUIMsg("track_target_block")
            self.rate.sleep()


        # stop rotation
        self.vel.angular.z = 0
        self.vel_pub.publish(self.vel)
        self.rate.sleep()

        # build geofence and check if centroid is within window
        min_pt          = self.img_pro.pix_grid.scanning_border.get('start_line_1')
        max_pt          = self.img_pro.pix_grid.scanning_border.get('end_line_2')
        pixel_geofence  = {'min_coordinate': min_pt, 'max_coordinate':max_pt}

        if self.img_pro.isBlobInBlob(pixel_geofence, self.target_block.centroid):
            # UPDATE GUI STATE
            rospy.loginfo('Block confirmed to be within extraction window')
            return True
        else:
            return False
    

        
    def updateGUIMsg(self, state):

        self.gui_state.state = state
        if state == "track_target_block":

            cx,cy                               = self.target_block.centroid
            x,y                                 = self.target_block.pixel_o
            self.gui_state.target_width         = int(self.target_block.pixel_dim.get('width'))
            self.gui_state.target_height        = int(self.target_block.pixel_dim.get('height'))
            self.gui_state.target_centroid_x    = cx
            self.gui_state.target_centroid_y    = cy 
            self.gui_state.target_xo            = int(x)
            self.gui_state.target_yo            = int(y) 
            self.gui_state.target_acquired      = True

            # publish update to GUI
            self.gui_update_pub.publish(self.gui_state)

        elif state == "block_detection":
            
            self.gui_state.target_acquired = False
            
        # publish update to GUI
        self.gui_update_pub.publish(self.gui_state)
                                

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
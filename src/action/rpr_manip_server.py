#!/usr/bin/env python3

import rospy
import numpy as np
import sys

from actionlib import SimpleActionServer 

from electromagnetic_builder.msg import RPRManipulatorFeedback, RPRManipulatorGoal
from electromagnetic_builder.msg import RPRManipulatorResult
from electromagnetic_builder.msg import RPRManipulatorAction

from std_msgs.msg import Int32MultiArray, Float64MultiArray


class ManipulatorActionServer(object):
    

    def __init__(self):

        # initialize ROS node 
        rospy.init_node('rpr_manip_action')

        # intialize server 
        self.action_name    = rospy.get_name()
        self.action_server  = SimpleActionServer(self.action_name, RPRManipulatorAction, execute_cb=self.execute_callback, auto_start = False)
        self.action_server.start()

        # intialize command publisher for RPR module on openCR
        self.rpr_command_pub    = rospy.Publisher("rpr_joint_trajectory", Float64MultiArray, queue_size=100)
        self.goal               = RPRManipulatorGoal()
        self.rate               = rospy.Rate(100)
        
        self.x_offset = .05 # approximately 5 cm offset from LDS sensor to the base servo along x-axis         
        self.z_offset = .05 # needs to be set

        # joint debug sub
        self.joint_debug_sub    = rospy.Subscriber("joint_debug", Int32MultiArray, self.joint_debug_callback)
        self.msg_field          = ["base_servo", "linear_actuator", "end_servo"]
        self.msg_length         = 9
        self.joint_data   = {'base_servo_current':0, 'base_servo_goal':0, 'base_is_moving':0, \
                                    'linear_actuator_current':0, 'linear_actuator_goal':0, 'linear_actuator_is_moving':0, \
                                        'end_servo_current':0, 'end_servo_goal':0, 'end_is_moving':0}

        self.joint_limits       = {'min_x':0, 'max_x':0, 'min_z':0, 'max_z':0}

        # used as terminating loop condition to determine success
        percent_goal            = 0.99
        self.goal_percentage    = np.array([percent_goal, percent_goal, percent_goal], dtype=np.float32)


    def execute_callback(self, rpr_goal):

        # validate command recieved [mm]
        if rpr_goal.x > self.joint_limits.get('min_x') and rpr_goal.x < self.joint_limits.get('max_x') \
            and rpr_goal.z > self.joint_limits.get('min_z') and rpr_goal.z < self.joint_limits.get('max_z'):
            
            self.goal = rpr_goal
            rospy.loginfo('%s recieved action request for goal coordinates %d, %d, %d [mm]' % (self.action_name, rpr_goal.x, rpr_goal.y, rpr_goal.z))
        
        else:
             self.action_server.set_aborted("Error: Coordinates given are out of bounds")

        # intialize action messages
        result = RPRManipulatorResult()
        rpr_fb = RPRManipulatorFeedback()

        # intialize loop variable
        joint_status_bool   = np.array([False, False, False])
        
        # iterate until all three joints have reached a terminating condition of percentage to goal complete
        while not np.all(joint_status_bool):

            # check for preempt request from client 
            if self.action_server.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self.action_name)
                self.action_server.set_preempted()   
            
            """ execute action """ 
            goal_array      = [rpr_goal.x, rpr_goal.y, rpr_goal.z]
            current_goal    = Float64MultiArray(data=goal_array)
            rospy.loginfo('%s publishing goal %d, %d, %d.' % (self.action_name, goal_array[0], goal_array[1], goal_array[2]))

            self.rpr_command_pub.publish(current_goal)

            """ report feedback """
            fb_array                            = self.getFeedbackArray()
            rpr_fb.base_pct_complete            = fb_array[0]
            rpr_fb.linear_actuator_pct_complete = fb_array[1]
            rpr_fb.end_pct_complete             = fb_array[2]
            self.action_server.publish_feedback(feedback=rpr_fb)

            """ update loop variable """
            # check which joints have reached the completion threshold
            joint_status_bool = np.greater_equal(fb_array, self.goal_percentage)
        
        # return result of action 
        result.complete = True
        rospy.loginfo('%s action succedded, exiting.' % self.action_name)
        self.action_server.set_succeeded(result)


    def joint_debug_callback(self, msg):

        # update debug dictionary 
        self.joint_data['base_servo_current']           = (msg.data[0])
        self.joint_data['base_servo_goal']            = (msg.data[1])
        self.joint_data['base_is_moving']             = (msg.data[2])
        self.joint_data['linear_actuator_current']      = (msg.data[3])
        self.joint_data['linear_actuator_goal']       = (msg.data[4])
        self.joint_data['linear_actuator_is_moving']  = (msg.data[5])
        self.joint_data['end_servo_current']            = (msg.data[6])
        self.joint_data['end_servo_goal']             = (msg.data[7])
        self.joint_data['end_is_moving']              = (msg.data[8])

    def getFeedbackArray(self):

        feedback_array = np.array([ self.joint_data.get('base_servo_current') / self.joint_data.get('base_servo_goal'), \
                                    self.joint_data.get('linear_actuator_current') / self.joint_data.get('linear_actuator_goal'), \
                                    self.joint_data.get('end_servo_current') / self.joint_data.get('end_servo_goal')])
        return feedback_array



if __name__ == '__main__':

    # intialize server node and class 
    rpr_server = ManipulatorActionServer()
    rospy.spin()

    # try:
    # except rospy.ROSInterruptException:
    #     rospy.loginfo("Action terminated.")
    # finally:
    #     pass
    #     # save trajectory into csv file
        # np.savetxt('joint_debug.csv', np.array(rpr_server.joint_data), fmt='%f')
    
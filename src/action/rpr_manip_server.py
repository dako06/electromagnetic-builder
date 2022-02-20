#!/usr/bin/env python3


import rospy
from actionlib import SimpleActionServer 

from electromagnetic_builder.msg import RPRManipulatorFeedback
from electromagnetic_builder.msg import RPRManipulatorResult
from electromagnetic_builder.msg import RPRManipulatorAction

from std_msgs.msg import Int32MultiArray


class ManipulatorActionServer(object):
    

    def __init__(self, name):

        #### intialize server ####
        self.action_name = name
        self.action_server = SimpleActionServer(self.action_name, RPRManipulatorAction, execute_cb=self.execute_callback, auto_start = False)
        self.action_server.start()

        # publish commands to RPR module on openCR
        self.rpr_command_pub = rospy.Publisher("rpr_joint_trajectory", Int32MultiArray, queue_size=10)


        #### action messages ####
        self.result     = RPRManipulatorResult()
        self.feedback   = RPRManipulatorFeedback()

        # rpr goal command
        # self.rpr_command = Int32MultiArray()


       

    def execute_callback(self, goal):
        
        r = rospy.Rate(10)

        #### check for preempt request from client ####
        if self.action_server.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self.action_name)
            self.action_server.set_preempted()   

    
        #### report feedback ####
        rospy.loginfo('%s recieved action request' % self.action_name)
      

        ##### execute action #####
        tmp = [1, 2, 3]
        tmp_array = Int32MultiArray(data=tmp)
        self.rpr_command_pub.publish(tmp_array)
        rospy.loginfo('%s publishing goal %d, %d, %d.' % (self.action_name, tmp[0], tmp[1], tmp[2]))


        self.result.complete = True

        #### return result of action ####

        rospy.loginfo('%s action succedded, exiting.' % self.action_name)
        self.action_server.set_succeeded(self.result)


if __name__ == '__main__':

    #### intialize server node and class ####
    rospy.init_node('rpr_manip_action')
    rpr_server = ManipulatorActionServer(rospy.get_name())
    rospy.spin()
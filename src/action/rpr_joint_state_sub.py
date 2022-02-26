#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState  
        
class RPRJointState():

    def __init__(self):

        rospy.init_node("rpr_joint_state_node")
        self.rpr_sub = rospy.Subscriber("rpr_joint_state", JointState, self.rpr_joint_state_callback)
        self.message = JointState()

    def rpr_joint_state_callback(self, msg):
        
        self.message = msg

        print("Recieved joint status update from %s" % msg.name)
        print("Joint positions are: ")
        i = 0
        for joint in msg.position:
            print("Joint %d: %f" % (i, joint))


if __name__ == '__main__':

    #### intialize server node and class ####
    rpr = RPRJointState()
    rospy.spin()
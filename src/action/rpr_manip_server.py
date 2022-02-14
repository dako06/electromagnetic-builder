import rospy
from actionlib import SimpleActionServer 

from electromagnetic_builder.msg import RPRManipulatorFeedback
from electromagnetic_builder.msg import RPRManipulatorResult
from electromagnetic_builder.msg import RPRManipulatorAction


class ManipulatorActionServer(object):
    

    def __init__(self, name):

        # TODO add pub/sub

        #### intialize server ####
        self.action_name = name
        self.action_server = SimpleActionServer(self.action_name, RPRManipulatorAction, execute_cb=self.execute_callback, auto_start = False)
        self.action_server.start()

        #### action messages ####
        self.result     = RPRManipulatorResult()
        self.feedback   = RPRManipulatorFeedback()

    def execute_callback(self, goal):
        
        r = rospy.Rate(10)

        #### check for preempt request from client ####
        if self.action_server.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self.action_name)
            self.action_server.set_preempted()   


        #### report feedback ####
      

        ##### execute action #####


        self.result.complete = True

        #### return result of action ####


        self.action_server.set_succeeded(self.result)


if __name__ == '__main__':

    #### intialize server node and class ####
    rospy.init_node('rpr_manip_action')
    rpr_server = ManipulatorActionServer(rospy.get_name())
    rospy.spin()
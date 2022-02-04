import sys
import rospy
import numpy as np
from interbotixarm.msg import *
from interbotixarm.srv import *

class ServiceHelper:
    def __init__(self, robot_model):
        self.name = "all"                           #Description of joint group
        self.commands = [0, 0, 0, 0, 0, 0, 0]       #Description of joint positions
        self.robot_model = robot_model
        self.srv_group = rospy.Publisher('/kiosk/arm/service', JointGroupCommand, queue_size=1)
        rospy.Service("/" + self.robot_model + "/custom/desired_pose", DesiredPose, self.robot_srv_go_to_pose)
        rospy.Service("/" + self.robot_model + "/custom/sleep", SleepPose, self.robot_srv_go_to_sleep)

    def robot_srv_go_to_sleep(self, req):
        if(req.sleep):
            print("true")
            self.commands = [0, -1.85, 1.55, 0, 0.8, 0, 0]
            joint_commands = JointGroupCommand(self.name, self.commands)
            self.srv_group.publish(joint_commands)
        return SleepPoseResponse()

    def robot_srv_go_to_pose(self, req):
        if(req.enable):
            if(req.cmd): self.commands = req.cmd
            joint_commands =  JointGroupCommand(self.name, self.commands)
            self.srv_group.publish(joint_commands)
        return DesiredPoseResponse()

def main(arg):
    rospy.init_node('robot_service_helper')
    ServiceHelper(arg[0])
    rospy.spin()

if __name__=="__main__":
    main(sys.argv[1:])

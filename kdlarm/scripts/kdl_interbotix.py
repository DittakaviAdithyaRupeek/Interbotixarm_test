#!/usr/bin/python

import rospy
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from std_msgs.msg import Bool, Float64MultiArray

class kdlInterbotixArm:
    def __init__(self):
        self.bot = InterbotixManipulatorXS("wx250s", "arm", "gripper", init_node=False)
        self.pitch_down = True

        self.js_b1_topic = '/kiosk/arm/js/b1'
        self.js_b2_topic = '/kiosk/arm/js/b2'
        self.js_b3_topic = '/kiosk/arm/js/b3'
        self.js_b4_topic = '/kiosk/arm/js/b4'
        self.js_b5_topic = '/kiosk/arm/js/b5'
        self.js_b6_topic = '/kiosk/arm/js/b6'

        self.kdl_js_sub = rospy.Subscriber('/kdl/joint_states', Float64MultiArray, self.kdl_joints_callback, queue_size=10)
        self.js_b1_sub = rospy.Subscriber(self.js_b1_topic, Bool, self.js_b1_callback, queue_size=1)
        self.js_b2_sub = rospy.Subscriber(self.js_b2_topic, Bool, self.js_b2_callback, queue_size=1)
        self.js_b3_sub = rospy.Subscriber(self.js_b3_topic, Bool, self.js_b3_callback, queue_size=1)
        self.js_b5_sub = rospy.Subscriber(self.js_b5_topic, Bool, self.js_b5_callback, queue_size=1)
        self.js_b6_sub = rospy.Subscriber(self.js_b6_topic, Bool, self.js_b6_callback, queue_size=1)

        self.initialize_arm()

        rospy.loginfo("Joystick Control Node Initialized !")

    def initialize_arm(self):
        self.bot.arm.go_to_sleep_pose()
        self.bot.arm.go_to_home_pose()

    def kdl_joints_callback(self, data):
        self.bot.arm.set_joint_positions(list(data.data))

    def js_b1_callback(self, data):
        if data.data:
            self.bot.gripper.close()

    def js_b2_callback(self, data):
        if data.data:
            self.bot.gripper.open()

    def js_b3_callback(self, data):
        if data.data:
            if self.pitch_down:
                self.bot.arm.set_single_joint_position('wrist_angle', position=1.5)
                self.pitch_down = False
            else:
                self.bot.arm.set_single_joint_position('wrist_angle', position=0)
                self.pitch_down = True
        rospy.sleep(1)

    def js_b5_callback(self, data):
        if data.data:
            self.bot.arm.set_ee_cartesian_trajectory(roll=0.1)

    def js_b6_callback(self, data):
        if data.data:
            self.bot.arm.set_ee_cartesian_trajectory(roll=-0.1)


if __name__ == '__main__':
    rospy.init_node('kdl_interbotix_bridge', anonymous=False)

    # create the arm instance
    arm = kdlInterbotixArm()

    while not rospy.is_shutdown():
        rospy.spin()

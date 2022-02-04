#!/usr/bin/env python3

# This script moves the arm based on control input received from a joystick
# Joint positions are calculated via reverse kinematics from target 3D position
# To get started, open a terminal and type 'roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=wx250s'

import time
import json
import rospy
import subprocess
import datetime
from enum import Enum
from collections import OrderedDict
from auxiliary import AirtableLogger, LogitechC270Camera
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Bool, Int8
from interbotixarm.msg import *
from interbotix_xs_modules.arm import InterbotixManipulatorXS


class Modes(Enum):
	DELTA = 1
	ABSOL = 2

class Waypoint:
	def __init__(self):
		self.name = None
		self.joint_states = []
		self.time = None
		self.joystick = None
		self.log = None

class Trajectory:
	def __init__(self, trajectory_file):
		self.waypoints = []
		self.current_wp_index = 0
		self.trajectory_file = trajectory_file

	def gen_trajectory_from_json(self):
		json_object = json.load(open(self.trajectory_file), object_pairs_hook=OrderedDict)

		for key in json_object:
			wp = Waypoint()
			wp.name = str(json_object[key]["name"])
			wp.joint_states = eval(json_object[key]["joint_states"])
			wp.time = eval(json_object[key]["time"])
			wp.log = eval(json_object[key]["log"])
			wp.joystick = eval(json_object[key]["joystick"])
			self.waypoints.append(wp)
		
		rospy.loginfo("Loaded %d waypoints from %s" % (len(self.waypoints), self.trajectory_file))

	def get_current_waypoint(self):
		return self.waypoints[self.current_wp_index]

	def get_next_waypoint(self):
		if self.current_wp_index < (len(self.waypoints) - 1):
			self.current_wp_index += 1
			return self.waypoints[self.current_wp_index]
		else:
			rospy.loginfo("Reached the end of trajectory !")
			return None
	
	def get_previous_waypoint(self):
		if self.current_wp_index > 0:
			self.current_wp_index -= 1
			return self.waypoints[self.current_wp_index]
		else:
			rospy.loginfo("Reached the beginning of trajectory !")
			return None
			
class KioskArm:
	def __init__(self, traj_file=None, arm_limit=0.5, js_limit=327, mode=Modes.DELTA.value):
		self.js_limit = js_limit                                # maximum value +/- per axis received from the joystick
		self.arm_limit = arm_limit                              # maximum reach for all axes in meters
		self.js_scale = self.arm_limit / self.js_limit          # meters of displacement per unit joystick input
		self.mode = mode                                        # operating mode of the robotic arm
		self.trajectory = Trajectory(trajectory_file=traj_file)
		self.trajectory.gen_trajectory_from_json()
		self.last_waypoint_successfull = False
		self.top_image_captured = False
		self.front_image_captured = False
		self.camera = LogitechC270Camera(device=0)
		self.sd_front_cmd = ["python3", "/home/rupeek/test/Stone-Deduction/main.py", "-i", "/home/rupeek/front.png"]
		self.sd_front_log = open('/home/rupeek/front_log.txt', 'w+')
		self.sd_top_cmd = ["python3", "/home/rupeek/test/Stone-Deduction/main.py", "-i", "/home/rupeek/top.png"]
		self.sd_top_log = open('/home/rupeek/top_log.txt', 'w+')
		self.wp = Waypoint()
		self.wp_start_time = 0
		self.wp_end_time = 0
		self.airtable_logger = AirtableLogger(config_file_json='/home/rupeek/armws/src/remoteAssayingMachine/interbotixarm/config/vault_airtable.json')
		self.js_pose = Point(0, 0, 0)                           # cartesian pose computed from joystick values
		self.js_value = Point(0, 0, 0)                          # raw joystick values along each axes
		self.arm_pose = Point(0, 0, 0)                          # current end effector pose (3DoF)
		self.wp_pose = Point(0, 0, 0)							# end effector pose (3DoF) at active waypoint
		self.diff = Point(0, 0, 0)
		self.waypoints = []

		self.js_b1_topic = '/kiosk/arm/js/b1'
		self.js_b2_topic = '/kiosk/arm/js/b2'
		self.js_b3_topic = '/kiosk/arm/js/b3'
		self.js_b4_topic = '/kiosk/arm/js/b4'
		self.js_b5_topic = '/kiosk/arm/js/b5'
		self.js_b6_topic = '/kiosk/arm/js/b6'
		self.js_b7_topic = '/kiosk/arm/js/b7'
		self.js_b8_topic = '/kiosk/arm/js/b8'
		self.arm_mode_topic = '/kiosk/arm/mode'
		self.js_pose_topic = '/kiosk/arm/js/axes'
		self.arm_srv_topic = '/kiosk/arm/service'

		self.js_b1_sub = rospy.Subscriber(self.js_b1_topic, Bool, self.js_b1_callback, queue_size=1)
		self.js_b2_sub = rospy.Subscriber(self.js_b2_topic, Bool, self.js_b2_callback, queue_size=1)
		self.js_b3_sub = rospy.Subscriber(self.js_b3_topic, Bool, self.js_b3_callback, queue_size=1)
		self.js_b4_sub = rospy.Subscriber(self.js_b4_topic, Bool, self.js_b4_callback, queue_size=1)
		self.js_b5_sub = rospy.Subscriber(self.js_b5_topic, Bool, self.js_b5_callback, queue_size=1)
		self.js_b6_sub = rospy.Subscriber(self.js_b6_topic, Bool, self.js_b6_callback, queue_size=1)
		self.js_b7_sub = rospy.Subscriber(self.js_b7_topic, Bool, self.js_b7_callback, queue_size=1)
		self.js_b8_sub = rospy.Subscriber(self.js_b8_topic, Bool, self.js_b8_callback, queue_size=1)
		self.js_pose_sub = rospy.Subscriber(self.js_pose_topic, Twist, self.js_pose_callback, queue_size=1)         # subscriber for 3DoF (Cartesian) target pose
		self.arm_mode_sub = rospy.Subscriber(self.arm_mode_topic, Int8, self.arm_mode_callback, queue_size=1)       # subscriber for operating mode value
		self.arm_srv_sub = rospy.Subscriber(self.arm_srv_topic, JointGroupCommand, self.arm_srv_callback, queue_size=1) # subscriber for communicating with services
    
		self.bot = InterbotixManipulatorXS("wx250s", "arm", "gripper", init_node=False)
		self.initialize()

	def initialize(self):
		self.bot.arm.go_to_sleep_pose()
		self.bot.arm.go_to_home_pose()
		self.bot.gripper.set_pressure(0.5)
		self.update_arm_pose()
		rospy.loginfo("Joystick Control Node Initialized !")
		self.wp.name = 'Initialize'
		self.wp.log = True
		self.wp_end_time = time.time()
		self.wp_start_time = time.time()
		self.airtable_logger.set_field('TimeStamp', str(datetime.datetime.now()))

	def update_arm_pose(self):
		T_SB = self.bot.arm.get_ee_pose()
		self.arm_pose.x = T_SB[0][3]
		self.arm_pose.y = T_SB[1][3]
		self.arm_pose.z = T_SB[2][3]
		
	def update_waypoint_pose(self):
		T_SB = self.bot.arm.get_ee_pose()
		self.wp_pose.x = T_SB[0][3]
		self.wp_pose.y = T_SB[1][3]
		self.wp_pose.z = T_SB[2][3]
		
	def arm_mode_callback(self, data):
		if data.data != self.mode:
			self.mode = data.data
			self.bot.arm.go_to_home_pose()

	def arm_srv_callback(self, data):
		# data from subscriber and act accordingly
		if data.name=="all":
			joint_positions = data.cmd
			self.bot.arm.set_joint_positions(joint_positions[0:-1])
			self.bot.gripper.close()
		elif data.name=="arm":
			joint_positions = data.cmd
			self.bot.arm.set_joint_positions(joint_positions)
		else:
			rospy.loginfo("Invalid type of request through manual service")


	def js_pose_callback(self, data):
		# populate data into variables
		self.js_value.x = data.linear.x
		self.js_value.y = data.linear.y
		self.js_value.z = data.linear.z

		# calculate js pose in cartesian values
		self.js_pose.x = self.js_value.x * self.js_scale
		self.js_pose.y = self.js_value.y * self.js_scale
		self.js_pose.z = self.js_value.z * self.js_scale

		# check operating mode and execute motion command
		if self.wp.joystick:
			if (abs(self.wp_pose.x - self.arm_pose.x) < 0.1) and (abs(self.wp_pose.y - self.arm_pose.y) < 0.1) and (abs(self.wp_pose.z - self.arm_pose.z) < 0.1):
				if self.mode == Modes.DELTA.value: 
					self.diff.x = self.js_value.x/100 if self.js_value.x else 0
					self.diff.y = self.js_value.y/100 if self.js_value.y else 0
					self.diff.z = self.js_value.z/100 if self.js_value.z else 0
					rospy.loginfo("Joystick Control -> %f %f %f" % (self.diff.x, self.diff.y, self.diff.z))
					self.bot.arm.set_ee_cartesian_trajectory(x=-self.diff.y, y=self.diff.x, z=self.diff.z, moving_time=0.5)
			elif self.mode == Modes.ABSOL.value:
				self.bot.arm.set_ee_pose_components(x=self.js_pose.x, y=self.js_pose.y, z=self.js_pose.z)
			else:
				pass
			self.update_arm_pose()
		else:
			rospy.loginfo("Joystick disabled at this waypoint !")

	def js_b1_callback(self, data): # A-Button on JS
		if data.data:
			rospy.loginfo("Actuating Gripper, please wait...")
			self.bot.gripper.close()
			rospy.loginfo("Gripper Actuation Finished!")

	def js_b2_callback(self, data): # B-Button on JS
		if data.data:
			rospy.loginfo("Actuating Gripper, please wait...")
			self.bot.gripper.open() 
			rospy.loginfo("Gripper Actuation Finished!")

	def js_b3_callback(self, data): # X-Button on JS
		if data.data:
			pick_joint_positions = [0, 0, 0, 0, 1.57, 0]
			self.bot.arm.set_joint_positions(pick_joint_positions)

	def js_b4_callback(self, data): # Y-Button on JS
		if data.data:
			self.bot.arm.go_to_home_pose(moving_time=2)

	def js_b5_callback(self, data): # LB-Button on JS
		if data.data:
			self.bot.arm.set_ee_cartesian_trajectory(roll=0.2)

	def js_b6_callback(self, data): # RB-Button on JS
		if data.data:
			self.bot.arm.set_ee_cartesian_trajectory(roll=-0.2)

	def js_b7_callback(self, data): # LT-Button on JS
		if data.data:
			# get the waypoint to go to from trajectory sampler
			if self.last_waypoint_successfull:
				wp = self.trajectory.get_next_waypoint()
			else:
				wp = self.trajectory.get_current_waypoint()
			# execute motion to received waypoint or do nothing if EOT
			if wp == None:
				return
			else:
				# waypoint received from trajectory sampler, lets go to that waypoint
				rospy.loginfo("Going to waypoint %s" % wp.name)
				ret = self.bot.arm.set_joint_positions(wp.joint_states, moving_time=wp.time)
				if ret == False:
					rospy.loginfo("Could not set joint positions")
					self.last_waypoint_successfull = False
				else:
					rospy.loginfo("Waypoint reached")
					# calculate time spent at previous waypoint and log
					self.wp_end_time = time.time()
					if self.wp.log:
						self.airtable_logger.add_to_field(self.wp.name, self.wp_end_time - self.wp_start_time)
						self.airtable_logger.add_to_field('total_time', self.wp_end_time - self.wp_start_time)
					self.wp_start_time = time.time()
					self.wp = wp

					# internal states update
					self.update_arm_pose()
					self.update_waypoint_pose()
					self.last_waypoint_successfull = True

			 		# check if waypoints have additional tasks
					if (self.wp.name == 'front_image' and not self.front_image_captured):
						rospy.loginfo("Capturing Image, Please Wait....")
						suc = self.camera.capture_image(save=True, name='/home/rupeek/front.png', show=False)
						if suc:
							rospy.loginfo("Front Image captured successfully ! Stone Deduction Started !")
							self.front_image_captured = True
							process = subprocess.Popen(self.sd_front_cmd, shell=False, stdout=self.sd_front_log, stderr=subprocess.PIPE)
					elif (self.wp.name == 'top_image' and not self.top_image_captured):
						rospy.loginfo("Capturing Image, Please Wait....")
						suc = self.camera.capture_image(save=True, name='/home/rupeek/top.png', show=False)
						if suc:
							rospy.loginfo("Top Image captured successfully ! Stone Deduction Started !")
							self.top_image_captured = True
							process = subprocess.Popen(self.sd_top_cmd, shell=False, stdout=self.sd_top_log, stderr=subprocess.PIPE)
					elif self.wp.name == 'home':
						# calculate the operator joystick timings
						js_1 = self.airtable_logger.get_field('pickup_from_tray') - 6
						self.airtable_logger.set_field('js_1', js_1)
						js_2 = self.airtable_logger.get_field('pickup_from_wm') - 2
						self.airtable_logger.set_field('js_2', js_2)
						js_3 = self.airtable_logger.get_field('pickup_from_wm_2') - 4
						self.airtable_logger.set_field('js_3', js_3)
						js_4 = self.airtable_logger.get_field('pickup_from_tray_2') - 2
						self.airtable_logger.set_field('js_4', js_4)

						# calculate independent task timings
						task_1 = self.airtable_logger.get_field('Initialize') + self.airtable_logger.get_field('top_of_tray') + self.airtable_logger.get_field('pickup_from_tray') + self.airtable_logger.get_field('clearing')
						self.airtable_logger.set_field('task_1', task_1)
						task_2 = self.airtable_logger.get_field('top_of_wm') + self.airtable_logger.get_field('pickup_from_wm') + self.airtable_logger.get_field('top_of_wm_2') + self.airtable_logger.get_field('clearing_2')
						self.airtable_logger.set_field('task_2', task_2)
						task_3 = self.airtable_logger.get_field('front_image') + self.airtable_logger.get_field('clearing_3') + self.airtable_logger.get_field('top_image')
						self.airtable_logger.set_field('task_3', task_3)
						task_4 = self.airtable_logger.get_field('top_of_wm_3') + self.airtable_logger.get_field('pickup_from_wm_2') + self.airtable_logger.get_field('top_of_wm_4')
						self.airtable_logger.set_field('task_4', task_4)
						task_5 = self.airtable_logger.get_field('top_of_tray_2') + self.airtable_logger.get_field('pickup_from_tray_2') + self.airtable_logger.get_field('top_of_tray_3')
						self.airtable_logger.set_field('task_5', task_5)
						
						# log everything
						self.airtable_logger.log()
					else:
						pass

	def js_b8_callback(self, data): # RT-Button on JS
		if data.data:
			# get the waypoint to go to from trajectory sampler
			if self.last_waypoint_successfull:
				wp = self.trajectory.get_previous_waypoint()
			else:
				wp = self.trajectory.get_current_waypoint()

			# execute motion to received waypoint or do nothing if EOT
			if wp == None:
				return
			else:
				rospy.loginfo("Going to waypoint %s" % wp.name)
				ret = self.bot.arm.set_joint_positions(wp.joint_states, moving_time=8)
				if ret == False:
					rospy.loginfo("Could not set joint positions")
					self.last_waypoint_successfull = False
				else:
					rospy.loginfo("Waypoint reached")
					self.update_arm_pose()
					self.update_waypoint_pose()
					self.last_waypoint_successfull = True

if __name__ == '__main__':
		rospy.init_node('arm_joystick_controller', anonymous=False)

		# create the arm instance
		traj_file = '/home/rupeek/armws/src/remoteAssayingMachine/interbotixarm/config/vault_trajectory.json'
		
		arm = KioskArm(traj_file=traj_file)

		###### main loop  ######
		while not rospy.is_shutdown():
			rospy.spin()

#!/usr/bin/env python
import tf
import os
import sys
import time
import rospy
import select
import datetime
import actionlib
import numpy as np
import termios, tty
import pandas as pd
import message_filters
import moveit_msgs.msg
import moveit_commander

from scipy import signal
from std_msgs.msg import Int16MultiArray
from xela_server.msg import XStream
from sensor_msgs.msg import JointState
from pynput.keyboard import Key, Listener
from franka_gripper.msg import HomingAction
from franka_gripper.msg import MoveAction, MoveActionGoal, GraspAction, GraspActionGoal

class RobotReader(object):
	def __init__(self):
		super(RobotReader, self).__init__()
		rospy.init_node('data_collection_client', anonymous=True, disable_signals=False)
		self.settings = termios.tcgetattr(sys.stdin)
		# self.rate_hz = 1000
		# rate = rospy.Rate(self.rate_hz)

		self.robot_state = moveit_commander.RobotCommander()
		self.group = moveit_commander.MoveGroupCommander("panda_arm")
		self.group.set_end_effector_link("panda_hand")
		self.listenertf = tf.TransformListener()
		self.gripper = GripperClient()
		self.grip_speed = None
		self.grip_position = None

		# self.gripper.move_action(self.grip_speed, self.grip_position)
		# state = self.gripper.homing_action()
		# self.gripper.grasp_action(0.06, 0.2, 20.0)

		while raw_input("press enter to start saving data, or type ctrl c then n to not: ") != "n":
			self.stop = False
			self.xelaSensor1 = []
			self.xelaSensor2 = []
			self.robot_states = []
			self.proximitySensorFormated = []
			self.listener = Listener(on_press=self.start_collection)
			self.listener.start()
			print(self.stop)
			self.xela_sub = message_filters.Subscriber('/xServTopic', XStream)
			self.prox_sub = message_filters.Subscriber('/proximityShadow/raw', Int16MultiArray)
			self.robot_sub = message_filters.Subscriber('/joint_states', JointState)
			subscribers = [self.robot_sub, self.xela_sub, self.prox_sub]
			self.start_time = datetime.datetime.now()
			print(self.start_time)
			self.prev_i = 0
			self.i = 1
			self.index__ = 0
			self.ts = message_filters.ApproximateTimeSynchronizer(subscribers, queue_size=50, slop=0.05, allow_headerless=True)
			self.ts.registerCallback(self.read_robot_data)
			t0 = time.time()
			while not rospy.is_shutdown() and self.stop is False:
				self.i += 1
				# rate.sleep()
			t1 = time.time()
			self.stop = False
			self.stop_time = datetime.datetime.now()
			print(self.stop_time)

			self.rate = (len(self.xelaSensor1)) / (t1-t0) 

			print("\n Stopped the data collection \n now saving the stored data")
			self.listener.stop()
			self.save_data()

	def print_stats(self):
		print("robot_states: ", len(self.robot_states))
		print("xelaSensor1: ", len(self.xelaSensor1))
		print("xelaSensor2: ", len(self.xelaSensor2))
		print("proximitySensorFormated: ", len(self.proximitySensorFormated))

	def check_keyboard(self, key_timeout):
		if self.getKey(key_timeout):
			print("herer")
			self.stop = True
			self.robot_sub.unregister()
			self.xela_sub.unregister()
			self.prox_sub.unregister()

	def read_robot_data(self, robot_joint_data, xela_data, prox_data):
		if self.stop == False and self.i != self.prev_i:
			self.prev_i = self.i
			self.index__ +=1
			ee_state = self.group.get_current_pose().pose
			self.robot_states.append([robot_joint_data, ee_state])
			self.xelaSensor1.append(xela_data.data[0])
			self.xelaSensor2.append(xela_data.data[1])
			self.proximitySensorFormated.append(prox_data.data[3])

	def start_collection(self, key):
		print("herer")
		if key == Key.esc:
			self.stop = True
			self.listener.stop()
			self.robot_sub.unregister()
			self.xela_sub.unregister()
			self.prox_sub.unregister()

	def format_data_for_saving(self):
		print("Formating the data")

		self.robot_states_formated = []
		self.xelaSensor1Formatted = []
		self.xelaSensor2Formatted = []

		for data_sample_index in range(len(self.xelaSensor1)):
			robot_joint_data = self.robot_states[data_sample_index][0]
			ee_state = self.robot_states[data_sample_index][1]
			self.robot_states_formated.append(list(robot_joint_data.position) + list(robot_joint_data.velocity) + list(robot_joint_data.effort) + 
												[ee_state.position.x, ee_state.position.y, ee_state.position.z,
												 ee_state.orientation.x, ee_state.orientation.y, ee_state.orientation.z, ee_state.orientation.w])

			Sensor1_data = self.xelaSensor1[data_sample_index]
			Sensor2_data = self.xelaSensor2[data_sample_index]
			Sensor1_vector = np.zeros((1, 48))
			Sensor2_vector = np.zeros((1, 48))

			xela_vector = np.zeros((1, 96))
			for index, i in enumerate(range(0, 48, 3)):
				Sensor1_vector[0, i]     = Sensor1_data.xyz[index].x
				Sensor1_vector[0, i + 1] = Sensor1_data.xyz[index].y
				Sensor1_vector[0, i + 2] = Sensor1_data.xyz[index].z
				Sensor2_vector[0, i]     = Sensor2_data.xyz[index].x
				Sensor2_vector[0, i + 1] = Sensor2_data.xyz[index].y
				Sensor2_vector[0, i + 2] = Sensor2_data.xyz[index].z

			self.xelaSensor1Formatted.append(Sensor1_vector)
			self.xelaSensor2Formatted.append(Sensor2_vector)

	def save_data(self):

		self.format_data_for_saving()

		self.xelaSensor1Formatted = np.array(self.xelaSensor1Formatted)
		self.xelaSensor1Formatted = np.reshape(self.xelaSensor1Formatted, (self.xelaSensor1Formatted.shape[0], 48))

		self.xelaSensor2Formatted = np.array(self.xelaSensor2Formatted)
		self.xelaSensor2Formatted = np.reshape(self.xelaSensor2Formatted, (self.xelaSensor2Formatted.shape[0], 48))

		self.proximitySensorFormated = np.array(self.proximitySensorFormated)
		self.proximitySensorFormated = np.reshape(self.proximitySensorFormated, (self.proximitySensorFormated.shape[0], 1))

		downsample_ratio = self.xelaSensor1Formatted.shape[0]

		print("xelaSensor1Formatted length: ", self.xelaSensor1Formatted.shape)
		print("xelaSensor2Formatted length: ", self.xelaSensor2Formatted.shape)
		print("proximitySensorFormated length: ", self.proximitySensorFormated.shape)
		print("robot_states_formated; ", np.asarray(self.robot_states_formated).shape)
		print("rate: ", self.rate)

		T1 = pd.DataFrame(self.xelaSensor1Formatted)
		T2 = pd.DataFrame(self.xelaSensor2Formatted)
		T3 = pd.DataFrame(self.proximitySensorFormated)
		T4 = pd.DataFrame(self.robot_states_formated)

		# create new folder for this experiment:
		folder = str('/home/kiyanoush/will_data_collection/data_collection_001/data_sample_' + datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S'))
		mydir = os.mkdir(folder)

		xela_Sensor_col = ['txl1_x', 'txl1_y', 'txl1_z', 'txl2_x', 'txl2_y', 'txl2_z','txl3_x', 'txl3_y', 'txl3_z','txl4_x', 'txl4_y', 'txl4_z','txl5_x', 'txl5_y', 'txl5_z','txl6_x', 'txl6_y', 'txl6_z',
		'txl7_x', 'txl7_y', 'txl7_z','txl8_x', 'txl8_y', 'txl8_z','txl9_x', 'txl9_y', 'txl9_z','txl10_x', 'txl10_y', 'txl10_z','txl11_x', 'txl11_y', 'txl11_z','txl12_x', 'txl12_y', 'txl12_z',
		'txl13_x', 'txl13_y', 'txl13_z','txl14_x', 'txl14_y', 'txl14_z','txl15_x', 'txl15_y', 'txl15_z','txl16_x', 'txl16_y', 'txl16_z']

		robot_states_col = ["position_panda_joint1", "position_panda_joint2", "position_panda_joint3", "position_panda_joint4", "position_panda_joint5", "position_panda_joint6", "position_panda_joint7", "position_panda_finger_joint1", "position_panda_finger_joint2",
		"velocity_panda_joint1", "velocity_panda_joint2", "velocity_panda_joint3", "velocity_panda_joint4", "velocity_panda_joint5", "velocity_panda_joint6", "velocity_panda_joint7", "velocity_panda_finger_joint1", "velocity_panda_finger_joint2",
		"effort_panda_joint1", "panda_joint2", "effort_panda_joint3", "effort_panda_joint4", "panda_joint5", "effort_panda_joint6", "effort_panda_joint7", "effort_panda_finger_joint1", "effort_panda_finger_joint2",
		"ee_state_position_x", "ee_state_position_y", "ee_state_position_z", "ee_state_orientation_x", "ee_state_orientation_y", "ee_state_orientation_z", "ee_state_orientation_w"]

		proximitySensor_col = ['tip_proximity']

		T1.to_csv(folder + '/xela_sensor1.csv', header=xela_Sensor_col, index=False)
		T2.to_csv(folder + '/xela_sensor2.csv', header=xela_Sensor_col, index=False)
		T3.to_csv(folder + '/proximity.csv', header=proximitySensor_col, index=False)
		T4.to_csv(folder + '/robot_state.csv', header=robot_states_col, index=False)

		# Create meta data
		save = raw_input("save meta file? 'n' to not")
		if save != 'n':
			meta_data = ['object_type', 'video', 'slipping', 'dropped', 'notes/comments']
			meta_data_ans = []
			for info in meta_data:
				value = raw_input(str("please enter the " + info))
				meta_data_ans.append(value)
			meta_data.extend(('sensor_type', 'frequency_hz', 'gripper_position', 'gripper_speed', 'start_time', 'stop_time'))
			meta_data_ans.extend(('xela_2fingers_noglove', str(self.rate), str(self.grip_position), str(self.grip_speed), str(self.start_time), str(self.stop_time)))
			meta_data_ans = np.array([meta_data_ans])
			T5 = pd.DataFrame(meta_data_ans)
			T5.to_csv(folder + '/meta_data.csv', header=meta_data, index=False)


class GripperClient():
	def __init__(self):
		self.robot_state = moveit_commander.RobotCommander()
		self.group = moveit_commander.MoveGroupCommander("hand")
		self.listener = tf.TransformListener()
		self.scene = moveit_commander.PlanningSceneInterface()

	def current_state(self):
		joint_state = self.group.get_current_joint_values()
		return joint_state

	def homing_action(self):
		client = actionlib.SimpleActionClient("/franka_gripper/homing", HomingAction)
		client.wait_for_server()
		client.send_goal(True)
		homing_done = client.wait_for_result()
		print("#### homing gripper ...")
		return homing_done

	def grasp_action(self, width, speed, force):
		client = actionlib.SimpleActionClient("/franka_gripper/grasp", GraspAction)
		client.wait_for_server()
		goal = GraspActionGoal()
		goal.goal.width = width
		goal.goal.speed = speed
		goal.goal.force = force
		print(GraspActionGoal())

		client.send_goal(goal.goal)
		move_done = client.wait_for_result()

		print(move_done)

	def move_action(self, w, s):
		client = actionlib.SimpleActionClient("/franka_gripper/move", MoveAction)
		client.wait_for_server()
		goal = MoveActionGoal
		goal.width = w
		goal.speed = s
		client.send_goal(goal)
		print("#### moving gripper to set width=", goal.width, "set speed=", goal.speed)
		move_done = client.wait_for_result()

		return move_done

if __name__ == "__main__":
	robot_reader = RobotReader()

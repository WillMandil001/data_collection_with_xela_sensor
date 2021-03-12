#!/usr/bin/env python
import tf
import os
import sys
import rospy
import datetime
import keyboard
import numpy as np
import pandas as pd
import message_filters
import moveit_msgs.msg
import moveit_commander

from scipy import signal
from std_msgs.msg import Int16MultiArray
from xela_server.msg import XStream
from sensor_msgs.msg import JointState
from pynput.keyboard import Key, Listener


class RobotReader(object):
	def __init__(self):
		# super(RobotReader, self).__init__()
		# self.robot_state = moveit_commander.RobotCommander()
		# self.group = moveit_commander.MoveGroupCommander("panda_arm")
		# self.group.set_end_effector_link("panda_hand")
		# self.listener = tf.TransformListener()

		rospy.init_node('data_collection_client', anonymous=True, disable_signals=False)
		rate = rospy.Rate(1000)
	
		while raw_input("press enter to start saving data, or type ctrl c then n to not: ") != "n":
			self.stop = False
			self.xelaSensor1 = []
			self.xelaSensor2 = []
			self.robot_states = []
			self.proximitySensor = []
			self.listener = Listener(on_press=self.start_collection)
			self.listener.start()
			print("here1223")
			print(self.stop)
			print(rospy.is_shutdown())
			self.xela_sub = message_filters.Subscriber('/xServTopic', XStream)
			self.prox_sub = message_filters.Subscriber('/proximityShadow/raw', Int16MultiArray)
			self.robot_sub = message_filters.Subscriber('/joint_states', JointState)
			print(datetime.datetime.now())
			self.prev_i = 0
			self.i = 1
			while not rospy.is_shutdown() and self.stop is False:
				self.ts = message_filters.ApproximateTimeSynchronizer([self.robot_sub, self.xela_sub, self.prox_sub], queue_size=1, slop=0.01, allow_headerless=True)
				self.ts.registerCallback(self.read_robot_data)
				self.i += 1
				rate.sleep()
			# rospy.signal_shutdown("reason is to stop collecting data")
			print(datetime.datetime.now())
			print("\n Stopped the data collection \n now saving the stored data")
			self.listener.stop()
			self.save_data()
			self.stop = False

	def read_robot_data(self, robot_joint_data, xela_data, prox_data):
		# print("Stop = False")
		if self.stop == False and self.i != self.prev_i:
			self.prev_i = self.i
			self.robot_states.append(np.asarray([robot_joint_data.position, robot_joint_data.velocity, robot_joint_data.effort]).flatten())
			self.proximitySensor.append(prox_data.data[3])

			Sensor1_data = xela_data.data[0]
			Sensor2_data = xela_data.data[1]
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

			self.xelaSensor1.append(Sensor1_vector)
			self.xelaSensor2.append(Sensor2_vector)

	def start_collection(self, key):
		print("herer")
		if key == Key.esc:
			self.stop = True
			self.listener.stop()
			self.robot_sub.unregister()
			self.xela_sub.unregister()
			self.prox_sub.unregister()

	def save_data(self):
		self.xelaSensor1
		self.xelaSensor2
		self.robot_states
		self.proximitySensor
		self.xelaSensor1 = np.array(self.xelaSensor1)
		self.xelaSensor1 = np.reshape(self.xelaSensor1, (self.xelaSensor1.shape[0], 48))

		self.xelaSensor2 = np.array(self.xelaSensor2)
		self.xelaSensor2 = np.reshape(self.xelaSensor2, (self.xelaSensor2.shape[0], 48))

		self.proximitySensor = np.array(self.proximitySensor)
		self.proximitySensor = np.reshape(self.proximitySensor, (self.proximitySensor.shape[0], 1))

		downsample_ratio = self.xelaSensor1.shape[0]

		print("xelaSensor1 length: ", self.xelaSensor1.shape)
		print("xelaSensor2 length: ", self.xelaSensor2.shape)
		print("proximitySensor length: ", self.proximitySensor.shape)
		print("robot_states; ", np.asarray(self.robot_states).shape)

		T1 = pd.DataFrame(self.xelaSensor1)
		T2 = pd.DataFrame(self.xelaSensor2)
		T3 = pd.DataFrame(self.proximitySensor)
		T4 = pd.DataFrame(self.robot_states)

		# create new folder for this experiment:
		folder = str('/home/kiyanoush/will_data_collection/data_collection_001/data_sample_' + datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S'))
		mydir = os.mkdir(folder)

		xela_Sensor_col = ['txl1_x', 'txl1_y', 'txl1_z', 'txl2_x', 'txl2_y', 'txl2_z','txl3_x', 'txl3_y', 'txl3_z','txl4_x', 'txl4_y', 'txl4_z','txl5_x', 'txl5_y', 'txl5_z','txl6_x', 'txl6_y', 'txl6_z',
		'txl7_x', 'txl7_y', 'txl7_z','txl8_x', 'txl8_y', 'txl8_z','txl9_x', 'txl9_y', 'txl9_z','txl10_x', 'txl10_y', 'txl10_z','txl11_x', 'txl11_y', 'txl11_z','txl12_x', 'txl12_y', 'txl12_z',
		'txl13_x', 'txl13_y', 'txl13_z','txl14_x', 'txl14_y', 'txl14_z','txl15_x', 'txl15_y', 'txl15_z','txl16_x', 'txl16_y', 'txl16_z']

		robot_states_col = ["position_panda_joint1", "position_panda_joint2", "position_panda_joint3", "position_panda_joint4", "position_panda_joint5", "position_panda_joint6", "position_panda_joint7", "position_panda_finger_joint1", "position_panda_finger_joint2",
		"velocity_panda_joint1", "velocity_panda_joint2", "velocity_panda_joint3", "velocity_panda_joint4", "velocity_panda_joint5", "velocity_panda_joint6", "velocity_panda_joint7", "velocity_panda_finger_joint1", "velocity_panda_finger_joint2",
		"effort_panda_joint1", "panda_joint2", "effort_panda_joint3", "effort_panda_joint4", "panda_joint5", "effort_panda_joint6", "effort_panda_joint7", "effort_panda_finger_joint1", "effort_panda_finger_joint2"]

		proximitySensor_col = ['tip_proximity']

		T1.to_csv(folder + '/xelaSensor1_1.csv', header=xela_Sensor_col, index=False)
		T2.to_csv(folder + '/xelaSensor2_1.csv', header=xela_Sensor_col, index=False)
		T3.to_csv(folder + '/proximity_1.csv', header=proximitySensor_col, index=False)
		T4.to_csv(folder + '/robot_data.csv', header=robot_states_col, index=False)

	# def current_pose(self):
	# 	return self.group.get_current_pose().pose


if __name__ == "__main__":
	robot_reader = RobotReader()

#!/usr/bin/env python
import rospy
import message_filters
from xela_server.msg import XStream
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import JointState
import sys
import pandas as pd
import numpy as np
from scipy import signal
import keyboard
from pynput.keyboard import Key, Listener


class RobotReader():
	def __init__(self):
		rospy.init_node('data_collection_client', anonymous=True, disable_signals=True)

		self.xelaSensor1 = []
		self.xelaSensor2 = []
		self.robot_states = []
		self.proximitySensor = []
	
		while raw_input("press enter to start saving data, or type n to not: ") != "n":
			self.stop = False
			self.listener = Listener(on_press=self.start_collection)
			self.listener.start()
			while not rospy.is_shutdown() and self.stop is False:
				# xela_sub = rospy.Subscriber('/xServTopic', XStream, sub_cb)
				# prox_sub = rospy.Subscriber('/proximityShadow/raw', Int16MultiArray, prox_cb)
				self.robot_sub = message_filters.Subscriber('/joint_states', JointState)
				ts = message_filters.TimeSynchronizer([self.robot_sub], queue_size=50)
				ts.registerCallback(self.callback)


			print("\n Stopped the data collection \n now saving the stored data")

	def prox_cb(self, data):
		self.proximitySensor.append(data.data[3])
		print("Hi_proximity")

	def read_joint_state(self, data):
		print("hi joint_state")

	def read_tactile_data(self, tactile_data):
		Sensor1_data = tactile_data.data[0]
		Sensor2_data = tactile_data.data[1]
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
		if key == Key.esc:
			self.stop = True
			self.listener.stop()

	def callback(self, data1):
		if self.stop == True:
			self.robot_sub.unregister()

if __name__ == "__main__":
	robot_reader = RobotReader()



		# xelaSensor1 = np.array(xelaSensor1)
		# xelaSensor1 = np.reshape(xelaSensor1, (xelaSensor1.shape[0], 48))

		# xelaSensor2 = np.array(xelaSensor2)
		# xelaSensor2 = np.reshape(xelaSensor2, (xelaSensor2.shape[0], 48))

		# proximitySensor = np.array(proximitySensor)
		# proximitySensor = np.reshape(proximitySensor, (proximitySensor.shape[0], 1))

		# downsample_ratio = xelaSensor1.shape[0]
		# proximityDownsampled = signal.resample(proximitySensor, downsample_ratio)

		# print(xelaSensor1.shape)
		# print(xelaSensor2.shape)
		# print(proximitySensor.shape)
		# print(proximityDownsampled.shape)

		# T1 = pd.DataFrame(xelaSensor1)
		# T2 = pd.DataFrame(xelaSensor2)
		# T3 = pd.DataFrame(proximitySensor)
		# T4 = pd.DataFrame(proximityDownsampled)


		# xela_Sensor_col = ['txl1_x', 'txl1_y', 'txl1_z', 'txl2_x', 'txl2_y', 'txl2_z','txl3_x', 'txl3_y', 'txl3_z','txl4_x', 'txl4_y', 'txl4_z','txl5_x', 'txl5_y', 'txl5_z','txl6_x', 'txl6_y', 'txl6_z',
		# 'txl7_x', 'txl7_y', 'txl7_z','txl8_x', 'txl8_y', 'txl8_z','txl9_x', 'txl9_y', 'txl9_z','txl10_x', 'txl10_y', 'txl10_z','txl11_x', 'txl11_y', 'txl11_z','txl12_x', 'txl12_y', 'txl12_z',
		# 'txl13_x', 'txl13_y', 'txl13_z','txl14_x', 'txl14_y', 'txl14_z','txl15_x', 'txl15_y', 'txl15_z','txl16_x', 'txl16_y', 'txl16_z']

		# proximitySensor_col = ['tip_proximity']

		# T1.to_csv('/home/kiyanoush/Desktop/xela_validation/xelaSensor1_1.csv', header=xela_Sensor_col, index=False)
		# T2.to_csv('/home/kiyanoush/Desktop/xela_validation/xelaSensor2_1.csv', header=xela_Sensor_col, index=False)
		# T3.to_csv('/home/kiyanoush/Desktop/xela_validation/proximity_1.csv', header=proximitySensor_col, index=False)
		# T4.to_csv('/home/kiyanoush/Desktop/xela_validation/proximitydownsampled_1.csv', header=proximitySensor_col, index=False)

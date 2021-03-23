#!/usr/bin/env python
import tf
import os
import sys
import rospy
import select
import datetime
import keyboard
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
		self.rate_hz = 100
		rate = rospy.Rate(self.rate_hz)

		while raw_input("press enter to start saving data, or type ctrl c then n to not: ") != "n":
			self.stop = False
			self.xelaSensor1 = []
			self.xelaSensor2 = []
			self.robot_states = []
			self.proximitySensor = []
			self.listener = Listener(on_press=self.start_collection)
			self.listener.start()


			self.xela_sub = rospy.Subscriber('/xServTopic', XStream, self.callbackxela_sub)
			self.prox_sub = rospy.Subscriber('/proximityShadow/raw', Int16MultiArray, self.callbackprox_sub)
			self.robot_sub = rospy.Subscriber('/joint_states', JointState, self.callbackrobot_sub)

			print(datetime.datetime.now())
			self.prev_i = 0
			self.i = 1
			self.index__ = 0
			self.collected_xela = False
			self.collected_prox = False
			self.collected_robot = False
			while not rospy.is_shutdown() and self.stop is False:
				self.i += 1
				rate.sleep()
			self.listener.stop()
			self.stop = False
			print(datetime.datetime.now())
			print("\n Stopped the data collection \n now saving the stored data")
			self.save_data()

	def callbackxela_sub(self, data):
		self.xelaSensor1.append(data.data[0])
		self.xelaSensor2.append(data.data[1])

	def callbackprox_sub(self, data):
		self.proximitySensor.append(data.data[3])

	def callbackrobot_sub(self, data):
		self.index__ +=1
		self.robot_states.append(list(data.position) + list(data.velocity) + list(data.effort))

	def start_collection(self, key):
		print("herer")
		if key == Key.esc:
			self.stop = True
			self.listener.stop()
			self.robot_sub.unregister()
			self.xela_sub.unregister()
			self.prox_sub.unregister()

	def save_data(self):
		print("xelaSensor1 length: ", len(self.xelaSensor1))
		print("xelaSensor2 length: ", len(self.xelaSensor2))
		print("proximitySensor length: ", len(self.proximitySensor))
		print("robot_states; ", self.index__)
		print("robot_states; ", len(self.robot_states))


if __name__ == "__main__":
	robot_reader = RobotReader()

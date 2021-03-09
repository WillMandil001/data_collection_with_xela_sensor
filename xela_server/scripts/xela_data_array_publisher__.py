#!/usr/bin/env python
import rospy 
from xela_server.msg import XStream
from std_msgs.msg import Int16MultiArray
import sys
import pandas as pd
import numpy as np
from scipy import signal
from python.CppPythonSocket.server import Server

import time
#======================================================
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from pylab import rcParams

from std_msgs.msg import Float32MultiArray


from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split
from sklearn.metrics import confusion_matrix, precision_recall_curve
from sklearn.metrics import recall_score, classification_report, auc, roc_curve
from sklearn.metrics import precision_recall_fscore_support, f1_score

from numpy.random import seed
seed(7)

SEED = 123 #used to help randomly select the data points
DATA_SPLIT_PCT = 0.2

rcParams['figure.figsize'] = 8, 6
LABELS = ["non_slip","slip"]


# function for making suitable input for lstm
def temporalize(X, y, lookback):
	output_X = []
	output_y = []
	for i in range(len(X)-lookback-1):
		t = []
		for j in range(1,lookback+1):
			# Gather past records upto the lookback period
			t.append(X[[(i+j+1)], :])
		output_X.append(t)
		output_y.append(y[i+lookback+1])
	return output_X, output_y


def flatten(X):
	flattened_X = np.empty((X.shape[0], X.shape[2]))  # sample x features array.
	for i in range(X.shape[0]):
		flattened_X[i] = X[i, (X.shape[1]-1), :]
	return(flattened_X)

def scale(X, scaler):
	for i in range(X.shape[0]):
		X[i, :, :] = scaler.transform(X[i, :, :])

	return X

def descale(X, scaler):
	for i in range(X.shape[0]):
		X[i, :, :] = scaler.inverse_transform(X[i, :, :])
		
	return X


#========================================================================================================================
class format_xela_data():
	def __init__(self):
		print(">> xela_data_publsiher initialised")

	def read_tactile_data(self):
		xela_vector = np.zeros((1, 96))
		for index, i in enumerate(range(0, 48, 3)):
			xela_vector[0, i] = self.Sensor1_data.xyz[index].x
			xela_vector[0, i + 1] = self.Sensor1_data.xyz[index].y
			xela_vector[0, i + 2] = self.Sensor1_data.xyz[index].z
			xela_vector[0, 48 + i] = self.Sensor2_data.xyz[index].x
			xela_vector[0, 48 + i + 1] = self.Sensor2_data.xyz[index].y
			xela_vector[0, 48 + i + 2] = self.Sensor2_data.xyz[index].z
		return xela_vector

	def sub_cb(self, tactile_data):
			self.Sensor1_data = tactile_data.data[0]
			self.Sensor2_data = tactile_data.data[1]
			xela_vector = self.read_tactile_data()
			xela_data.append(xela_vector)
			if len(xela_data) > 20:
				start = time.time()
				xela_seq = np.array(xela_data[-20:])
				xela_seq = np.squeeze(xela_seq)
				xela_seq = scaler_tactile.transform(xela_seq)

				# publish data:
				msg = Float32MultiArray()
				msg.data = xela_seq.flatten().astype(np.float32).tolist()
				xela_data_pub.publish(msg)
				xela_data.pop(0)

if __name__ == "__main__":
	mf = format_xela_data()
	proximitySensor = []
	
	time_flag=0
	stop_flag = 0
	#=========================================================================================================================
	x0 = 0.58
	z0 = 0.18
	xela_data = []
	robot_data = []
	action_data = []

	lookback = 20
	# load dataset
	tactile = np.load('/home/kiyanoush/catkin_ws/src/xela_server/data_load/xela_numeric.npy')
	robot = np.load('/home/kiyanoush/catkin_ws/src/xela_server/data_load/new_robot_data_26993.npy')

	#========================================================================================================================
	# Initialize a scaler using the training data.
	scaler_tactile = StandardScaler().fit(tactile)
	scaler_robot = StandardScaler().fit(robot)
	scaler_action = StandardScaler().fit(robot)

	print(scaler_tactile.mean_)
	print("scaler mean: ", scaler_robot.mean_)
	#=========================================================================================================================

	rospy.init_node('xela_dat_publisher', anonymous=True, disable_signals=True)
	rate = rospy.Rate(100)
	xela_data_pub = rospy.Publisher("xela_data_array", Float32MultiArray, queue_size=1)
	while not rospy.is_shutdown():
		time_flag = 0
		xela_Sub = rospy.Subscriber('/xServTopic', XStream, mf.sub_cb)
		rospy.spin()
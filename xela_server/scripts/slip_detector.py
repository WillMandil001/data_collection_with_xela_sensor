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

import tensorflow as tf
from keras import optimizers, Sequential
from keras.models import Model
from keras.utils import plot_model
from keras.layers import Dense, LSTM, RepeatVector, TimeDistributed, Input, concatenate
from keras.callbacks import ModelCheckpoint, TensorBoard
from tcn import compiled_tcn, TCN, tcn_full_summary
import datetime

from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split
from sklearn.metrics import confusion_matrix, precision_recall_curve
from sklearn.metrics import recall_score, classification_report, auc, roc_curve
from sklearn.metrics import precision_recall_fscore_support, f1_score

from numpy.random import seed
seed(7)
tf.random.set_seed(11)

SEED = 123 #used to help randomly select the data points
DATA_SPLIT_PCT = 0.2

rcParams['figure.figsize'] = 8, 6
LABELS = ["non_slip","slip"]

#=====================================================


		
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


# This function is to replace the trained weights on GPU which have nan values by their numpy equivalent when using cpu for model.predict
def model_refresh_without_nan(models):
	import numpy as np
	valid_weights = []
	for l in models.get_weights():
		if np.isnan(l).any():
			valid_weights.append(np.nan_to_num(l))
			#print("!!!!!", l)
		else:
			valid_weights.append(l)
	models.set_weights(valid_weights)

def build_robot_trajectory(VMAX , T, scalar_robot):
	time_steps = 6
	ts = 0.01
	x = []
	z = []
	vx = []
	vz = []
	time_steps_list = [] 
	for j in range(time_steps + 1):
		for time_step in np.linspace(j*T, (j+1)*T, num=int(T/ts)+1, endpoint=False):
			time_steps_list.append(time_step)
			if j == 0:
				x.append(x0)
				z.append(z0 + 0.5 * a * (time_step**2))
				vx.append(0)
				vz.append(a * time_step)

			if j == 1:
				x.append(x0)
				z.append((z0 + 0.5 * a * (T**2)) + V_max*(time_step - T))
				vx.append(0)
				vz.append(V_max)

			if j == 2:
				x.append(x0)
				z.append((z0 + 0.5 * a * (T**2)) + V_max*T + (-0.5 * a * (time_step-2*T)**2 + V_max*(time_step-2*T)))
				vx.append(0)
				vz.append(V_max - a * (time_step-2*T))

			if j == 3:
				x.append(x0 + 0.5 * a * ((time_step-3*T)**2))
				z.append((z0 + 0.5 * a * (T**2)) + V_max*T + (-0.5 * a * T**2 + V_max*T))
				vx.append(a * (time_step - 3*T))
				vz.append(0)

			if j == 4:
				x.append((x0 + 0.5 * a * (T**2)) + V_max*(time_step - 4*T))
				z.append((z0 + 0.5 * a * (T**2)) + V_max*T + (-0.5 * a * T**2 + V_max*T))
				vx.append(V_max)
				vz.append(0)
			if j == 5:
				x.append((x0 + 0.5 * a * (T**2)) + V_max*T + (-0.5 * a * (time_step-5*T)**2 + V_max*(time_step-5*T)))
				z.append((z0 + 0.5 * a * (T**2)) + V_max*T + (-0.5 * a * T**2 + V_max*T))
				vx.append(V_max - a * (time_step-5*T))
				vz.append(0)
			if j == 6:
				x.append(x0 + 0.5 * a * (T**2) + V_max*T + (-0.5 * a * T**2 + V_max*T))
				z.append(z0 + 0.5 * a * (T**2) + V_max*T + (-0.5 * a * T**2 + V_max*T))
				vx.append(0)
				vz.append(0)


	x = np.array(x)
	z = np.array(z)
	vx = np.array(vx)
	vz = np.array(vz)
	x = np.reshape(x, (x.shape[0], 1))
	z = np.reshape(z, (z.shape[0], 1))
	vx = np.reshape(vx, (vx.shape[0], 1))
	vz = np.reshape(vz, (vz.shape[0], 1))
	robot_vec = np.concatenate([x, z, vx, vz], axis=1)
	robot_vec = scaler_robot.transform(robot_vec)
	# print(robot_vec.shape)
	# robot_vec, y = temporalize(robot_vec, np.zeros((robot_vec.shape[0], 1)), lookback=20)
	# robot_vec = np.array(robot_vec)
	# print(robot_vec.shape)
	#robot_vec = scale(robot_vec, scalar_robot)

	#robot_vec = scale(np.expand_dims(robot_vec, axis=1), scalar_robot)		
	time_steps_list = np.around(time_steps_list, decimals=2)

	return robot_vec, time_steps_list

#========================================================================================================================
class my_functions():
	def __init__(self):
		self.time_step = 0
		self.time_flag = 0
		self.TCN_model = TCN_model
		_ = self.TCN_model.predict([np.zeros((1,20,96)), np.zeros((1,20,4)), np.zeros((1,20,4))])
		self.lstm_autoencoder = lstm_autoencoder
		_ = self.lstm_autoencoder.predict([np.zeros((1,10,96))])

	def sub_cb(self, tactile_data):
		if self.time_flag==0:
			print("---------------------------")
			self.start_time = time.time()
		else:
			print("+++++++++++++++++++++++++++++", self.time_step)
			self.time_step +=1
			self.loop_start_time = time.time()
			#================================================================================================================================================================
			# reshape xela array to [time_seq, 96]
			xela_seq = np.expand_dims(np.asarray(tactile_data.data).reshape(20, 96), axis=0)
			#================================================================================================================================================================
			## create state and action vectors:
			current_time = time.time() - self.start_time
			current_time_index = np.where(np.isclose(time_steps_list, np.around(current_time, decimals=2)))[0][0]
			if current_time_index >= 20:
				robot_seq = np.expand_dims(np.asarray(robot_traj[current_time_index-20:current_time_index]), axis=0)
				action_seq = np.expand_dims(np.asarray(robot_traj[current_time_index:current_time_index+20]), axis=0)
				

				tactile_save.append(np.asarray(tactile_data.data).reshape(20, 96)[-1])
				robot_save.append(np.squeeze(np.asarray(robot_seq))[-1])
				action_save.append(np.squeeze(np.asarray(action_seq))[-1])

				# print("after2: ", time.time() - start)
				#tactile_predicted = self.TCN_model([np.expand_dims(np.squeeze(xela_seq), axis=0), np.expand_dims(np.squeeze(robot_seq), axis=0),
				# np.expand_dims(np.squeeze(action_seq), axis=0)])
				print(flatten(xela_seq)[0 , :20])
				#tactile_predicted = self.TCN_model.predict([xela_seq, robot_seq, action_seq])
				print("=======")
				#print(flatten(tactile_predicted)[0][:20])
				#tactile_predicted = self.TCN_model.predict([np.expand_dims(np.squeeze(xela_seq), axis=0), np.expand_dims(np.squeeze(robot_seq),
				# axis=0), np.expand_dims(np.squeeze(action_seq), axis=0)])
				
				#tactile_predicted = descale(tactile_predicted, scaler_tactile)
				# print("after3: ", time.time() - start)
				
				#tactile_predicted = tactile_predicted[:, 0:10, :]
				# xela_Sub.unregister()
				#print(tactile_predicted.shape)
				# print("after4: ", time.time() - start)
				#tactile_AE_reconstructed = self.lstm_autoencoder.predict(np.asarray(tactile_predicted))
				
				#tactile_AE_reconstructed = self.lstm_autoencoder(np.asarray(tactile_predicted))
				#print("reconstruction shape: ", tactile_AE_reconstructed.shape)
				# print("after5: ", time.time() - start)

				#mse = np.mean(np.power(flatten(tactile_predicted) - flatten(tactile_AE_reconstructed), 2), axis=1)
				#threshold_fixed = 0.3
				#print(mse)
				#pred_y = [1 if e > threshold_fixed else 0 for e in mse]
				# #print(pred_y)
				# if 1 in pred_y:
				# 	print("slippppppppppppppp")
				# 	## return STOP ROBOT!
				# 	self.stop_robot()
					# print("after6: ", time.time() - start)

		self.time_flag = 1

	def stop_robot(self):
		pass

	def prox_cb(self, data):
		proximitySensor.append(data.data[3])
		#prox_pub.publish(data.data[3])
		print("Hi_proximity")



if __name__ == "__main__":


	# Load model and predict for test/validation data
	TCN_model = tf.keras.models.load_model('/home/kiyanoush/Desktop/load/updated/final_TCN_truerobotaction.h5', custom_objects={'TCN': TCN})
	model_refresh_without_nan(TCN_model)
	lstm_autoencoder = tf.keras.models.load_model('/home/kiyanoush/Desktop/load/updated/lstm_autoencoder_classifier_final.h5')

	mf = my_functions()

	print('hi')
	proximitySensor = []
	

	print('hi')
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

	V_max = 0.2
	T = 0.75
	a = V_max / T
	robot_traj, time_steps_list = build_robot_trajectory(V_max, T, scaler_robot)
	print(robot_traj.shape)
	#=========================================================================================================================
	tactile_save = []
	robot_save = []
	action_save = []

	rospy.init_node('listener', anonymous=True, disable_signals=True)
	rate = rospy.Rate(100)
	#prox_pub = rospy.Publisher("proximity", Int32, queue_size=1)
	server = Server("127.0.0.1", 5002)
	while not rospy.is_shutdown():

		message = server.receive()
		print(message)
		
		if message == "Object_Grasped!":
			print("message received!!")
			time_flag = 0
			xela_Sub = rospy.Subscriber('/xela_data_array', Float32MultiArray, mf.sub_cb)
			#prox_Sub = rospy.Subscriber('/proximityShadow/raw', Int16MultiArray, mf.prox_cb)
			while True:
				print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1")
				message = server.receive()
				print(message)
				if message == "Stop_data_collection!":
					rospy.signal_shutdown("Stop message recieved")
					break
		rospy.spin()



	tactile_save = np.array(tactile_save)
	#tactile_save = np.reshape(tactile_save, (tactile_save.shape[0], 96))
	T = pd.DataFrame(tactile_save)
	T.to_csv('/home/kiyanoush/Desktop/tactile_save.csv', header=None, index=False)

	
	T = pd.DataFrame(np.array(robot_save))
	T.to_csv('/home/kiyanoush/Desktop/robot_save.csv', header=None, index=False)
	

	T = pd.DataFrame(np.array(action_save))
	T.to_csv('/home/kiyanoush/Desktop/action_save.csv', header=None, index=False)

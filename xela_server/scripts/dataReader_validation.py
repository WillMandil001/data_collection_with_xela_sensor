#!/usr/bin/env python
import rospy 
from xela_server.msg import XStream
from std_msgs.msg import Int16MultiArray
import sys
import pandas as pd
import numpy as np
from scipy import signal
from python.CppPythonSocket.server import Server


class Listen():
    
    def __init__(self):
        
        rospy.Subscriber('/xServTopic', XStream, self.callback)

    def callback(self, tactile_data):
       
		Sensor1_data = tactile_data.data[0]
		Sensor2_data = tactile_data.data[1]

		Sensor1_vector = np.zeros((1, 48))
		Sensor2_vector = np.zeros((1, 48))

	######################################################################
		Sensor1_vector[0, 0] = Sensor1_data.xyz[0].x
		Sensor1_vector[0, 1] = Sensor1_data.xyz[0].y
		Sensor1_vector[0, 2] = Sensor1_data.xyz[0].z

		Sensor1_vector[0, 3] = Sensor1_data.xyz[1].x
		Sensor1_vector[0, 4] = Sensor1_data.xyz[1].y
		Sensor1_vector[0, 5] = Sensor1_data.xyz[1].z

		Sensor1_vector[0, 6] = Sensor1_data.xyz[2].x
		Sensor1_vector[0, 7] = Sensor1_data.xyz[2].y
		Sensor1_vector[0, 8] = Sensor1_data.xyz[2].z

		Sensor1_vector[0, 9] = Sensor1_data.xyz[3].x
		Sensor1_vector[0, 10] = Sensor1_data.xyz[3].y
		Sensor1_vector[0, 11] = Sensor1_data.xyz[3].z

		Sensor1_vector[0, 12] = Sensor1_data.xyz[4].x
		Sensor1_vector[0, 13] = Sensor1_data.xyz[4].y
		Sensor1_vector[0, 14] = Sensor1_data.xyz[4].z

		Sensor1_vector[0, 15] = Sensor1_data.xyz[5].x
		Sensor1_vector[0, 16] = Sensor1_data.xyz[5].y
		Sensor1_vector[0, 17] = Sensor1_data.xyz[5].z

		Sensor1_vector[0, 18] = Sensor1_data.xyz[6].x
		Sensor1_vector[0, 19] = Sensor1_data.xyz[6].y
		Sensor1_vector[0, 20] = Sensor1_data.xyz[6].z

		Sensor1_vector[0, 21] = Sensor1_data.xyz[7].x
		Sensor1_vector[0, 22] = Sensor1_data.xyz[7].y
		Sensor1_vector[0, 23] = Sensor1_data.xyz[7].z

		Sensor1_vector[0, 24] = Sensor1_data.xyz[8].x
		Sensor1_vector[0, 25] = Sensor1_data.xyz[8].y
		Sensor1_vector[0, 26] = Sensor1_data.xyz[8].z

		Sensor1_vector[0, 27] = Sensor1_data.xyz[9].x
		Sensor1_vector[0, 28] = Sensor1_data.xyz[9].y
		Sensor1_vector[0, 29] = Sensor1_data.xyz[9].z

		Sensor1_vector[0, 30] = Sensor1_data.xyz[10].x
		Sensor1_vector[0, 31] = Sensor1_data.xyz[10].y
		Sensor1_vector[0, 32] = Sensor1_data.xyz[10].z

		Sensor1_vector[0, 33] = Sensor1_data.xyz[11].x
		Sensor1_vector[0, 34] = Sensor1_data.xyz[11].y
		Sensor1_vector[0, 35] = Sensor1_data.xyz[11].z

		Sensor1_vector[0, 36] = Sensor1_data.xyz[12].x
		Sensor1_vector[0, 37] = Sensor1_data.xyz[12].y
		Sensor1_vector[0, 38] = Sensor1_data.xyz[12].z

		Sensor1_vector[0, 39] = Sensor1_data.xyz[13].x
		Sensor1_vector[0, 40] = Sensor1_data.xyz[13].y
		Sensor1_vector[0, 41] = Sensor1_data.xyz[13].z

		Sensor1_vector[0, 42] = Sensor1_data.xyz[14].x
		Sensor1_vector[0, 43] = Sensor1_data.xyz[14].y
		Sensor1_vector[0, 44] = Sensor1_data.xyz[14].z

		Sensor1_vector[0, 45] = Sensor1_data.xyz[15].x
		Sensor1_vector[0, 46] = Sensor1_data.xyz[15].y
		Sensor1_vector[0, 47] = Sensor1_data.xyz[15].z
	################################################################

		Sensor2_vector[0, 0] = Sensor2_data.xyz[0].x
		Sensor2_vector[0, 1] = Sensor2_data.xyz[0].y
		Sensor2_vector[0, 2] = Sensor2_data.xyz[0].z

		Sensor2_vector[0, 3] = Sensor2_data.xyz[1].x
		Sensor2_vector[0, 4] = Sensor2_data.xyz[1].y
		Sensor2_vector[0, 5] = Sensor2_data.xyz[1].z

		Sensor2_vector[0, 6] = Sensor2_data.xyz[2].x
		Sensor2_vector[0, 7] = Sensor2_data.xyz[2].y
		Sensor2_vector[0, 8] = Sensor2_data.xyz[2].z

		Sensor2_vector[0, 9] = Sensor2_data.xyz[3].x
		Sensor2_vector[0, 10] = Sensor2_data.xyz[3].y
		Sensor2_vector[0, 11] = Sensor2_data.xyz[3].z

		Sensor2_vector[0, 12] = Sensor2_data.xyz[4].x
		Sensor2_vector[0, 13] = Sensor2_data.xyz[4].y
		Sensor2_vector[0, 14] = Sensor2_data.xyz[4].z

		Sensor2_vector[0, 15] = Sensor2_data.xyz[5].x
		Sensor2_vector[0, 16] = Sensor2_data.xyz[5].y
		Sensor2_vector[0, 17] = Sensor2_data.xyz[5].z

		Sensor2_vector[0, 18] = Sensor2_data.xyz[6].x
		Sensor2_vector[0, 19] = Sensor2_data.xyz[6].y
		Sensor2_vector[0, 20] = Sensor2_data.xyz[6].z

		Sensor2_vector[0, 21] = Sensor2_data.xyz[7].x
		Sensor2_vector[0, 22] = Sensor2_data.xyz[7].y
		Sensor2_vector[0, 23] = Sensor2_data.xyz[7].z

		Sensor2_vector[0, 24] = Sensor2_data.xyz[8].x
		Sensor2_vector[0, 25] = Sensor2_data.xyz[8].y
		Sensor2_vector[0, 26] = Sensor2_data.xyz[8].z

		Sensor2_vector[0, 27] = Sensor2_data.xyz[9].x
		Sensor2_vector[0, 28] = Sensor2_data.xyz[9].y
		Sensor2_vector[0, 29] = Sensor2_data.xyz[9].z

		Sensor2_vector[0, 30] = Sensor2_data.xyz[10].x
		Sensor2_vector[0, 31] = Sensor2_data.xyz[10].y
		Sensor2_vector[0, 32] = Sensor2_data.xyz[10].z

		Sensor2_vector[0, 33] = Sensor2_data.xyz[11].x
		Sensor2_vector[0, 34] = Sensor2_data.xyz[11].y
		Sensor2_vector[0, 35] = Sensor2_data.xyz[11].z

		Sensor2_vector[0, 36] = Sensor2_data.xyz[12].x
		Sensor2_vector[0, 37] = Sensor2_data.xyz[12].y
		Sensor2_vector[0, 38] = Sensor2_data.xyz[12].z

		Sensor2_vector[0, 39] = Sensor2_data.xyz[13].x
		Sensor2_vector[0, 40] = Sensor2_data.xyz[13].y
		Sensor2_vector[0, 41] = Sensor2_data.xyz[13].z

		Sensor2_vector[0, 42] = Sensor2_data.xyz[14].x
		Sensor2_vector[0, 43] = Sensor2_data.xyz[14].y
		Sensor2_vector[0, 44] = Sensor2_data.xyz[14].z

		Sensor2_vector[0, 45] = Sensor2_data.xyz[15].x
		Sensor2_vector[0, 46] = Sensor2_data.xyz[15].y
		Sensor2_vector[0, 47] = Sensor2_data.xyz[15].z
	################################################################

		xelaSensor1.append(Sensor1_vector)
		xelaSensor2.append(Sensor2_vector)
		print("Hi_xela")

if __name__ == '__main__':

	xelaSensor1 = []
	xelaSensor2 = []
	rospy.init_node('listener', anonymous=True)

	l = Listen()
	rospy.spin()

	xelaSensor1 = np.array(xelaSensor1)
	xelaSensor1 = np.reshape(xelaSensor1, (xelaSensor1.shape[0], 48))

	xelaSensor2 = np.array(xelaSensor2)
	xelaSensor2 = np.reshape(xelaSensor2, (xelaSensor2.shape[0], 48))

	T1 = pd.DataFrame(xelaSensor1)
	T2 = pd.DataFrame(xelaSensor2)

	xela_Sensor_col = ['txl1_x', 'txl1_y', 'txl1_z', 'txl2_x', 'txl2_y', 'txl2_z','txl3_x', 'txl3_y', 'txl3_z','txl4_x', 'txl4_y', 'txl4_z','txl5_x', 'txl5_y', 'txl5_z','txl6_x', 'txl6_y', 'txl6_z',
	'txl7_x', 'txl7_y', 'txl7_z','txl8_x', 'txl8_y', 'txl8_z','txl9_x', 'txl9_y', 'txl9_z','txl10_x', 'txl10_y', 'txl10_z','txl11_x', 'txl11_y', 'txl11_z','txl12_x', 'txl12_y', 'txl12_z',
	'txl13_x', 'txl13_y', 'txl13_z','txl14_x', 'txl14_y', 'txl14_z','txl15_x', 'txl15_y', 'txl15_z','txl16_x', 'txl16_y', 'txl16_z']


	T1.to_csv('/home/kiyanoush/Desktop/xela_validation/xelaSensor1_spin.csv', header=xela_Sensor_col, index=False)
	T2.to_csv('/home/kiyanoush/Desktop/xela_validation/xelaSensor2_spin.csv', header=xela_Sensor_col, index=False)
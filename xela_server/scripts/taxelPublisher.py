#!/usr/bin/env python

# import sys
# import os
# import cv2
# import copy
# import numpy as np

# import rospy
# import rospkg
# rospack = rospkg.RosPack()
# import actionlib
# from xela_ros.msg import *
# import xela_ros.srv

# taxel_rows = 4
# taxel_cols = 4
# taxel_num = taxel_rows*taxel_cols

# class XelaSensorClient(object):
#   def __init__(self):
#     self._base = [0] * taxel_num * 3
#     self._data = [0] * taxel_num * 3
#     self
#     # self._sub_data = rospy.Subscriber("data_topic", XelaSensorStamped, self.data_callback)
#     # self._sub_base = rospy.Subscriber("base_topic", XelaSensorStamped, self.base_callback)
#     self._pub = rospy.Publisher("sensor_xela", sensor, queue_size=10)

#   # def calibrate(self, sample_num, log_filename):
#   #   calibrate_action_client = actionlib.SimpleActionClient('calibrate', xela_ros.msg.CalibrateAction)
#   #   calibrate_action_client.wait_for_server()
#   #   goal = xela_ros.msg.CalibrateGoal()
#   #   goal.sample_num = sample_num
#   #   goal.log_filename = log_filename
#   #   calibrate_action_client.send_goal(goal)
#   #   calibrate_action_client.wait_for_result()

#   # def data_callback(self, msg_in):
#   #   self._data = copy.deepcopy(msg_in.data)
  
#   # def base_callback(self, msg_in):
#   #   self._base = copy.deepcopy(msg_in.data)

#   # @property
#   # def data(self):
#   #   return self._data
  
#   # @property
#   # def base(self):
#   #   return self._base

# class XelaSensorDemo(XelaSensorClient):
#   def __init__(self):
#     super(XelaSensorDemo, self).__init__()
#     # Run calibration and get baseline of the sensor (if necessary)
#     # data_dir = os.path.join(rospack.get_path("xela_ros"), "data")
#     # self.calibrate(sample_num=100, log_filename=os.path.join(data_dir, "calibration_log.csv"))

#     # Constants
#     width  = 800 # width of the image
#     height = 800 # height of the image
#     margin = 160 # margin of the taxel in the image
#     pitch  = 160 # pitch between taxels in the image
#     scale  = 25  # scale from the sensor data to the image
#     tz     = 12  # default size of the circle
#     color  = (0,255,0,255)

#     # Open window to display state of taxels on the board
#     img = np.zeros((height,width,3), np.uint8)
#     cv2.namedWindow('xela-sensor', cv2.WINDOW_NORMAL)

#     while not rospy.is_shutdown():
#       # Read new data from the sensor 
#       diff = np.array(self.data) - np.array(self.base)
#       dx = diff.reshape((taxel_num,3))[:,0] / scale
#       dy = diff.reshape((taxel_num,3))[:,1] / scale
#       dz = diff.reshape((taxel_num,3))[:,2] / scale
#       # Init image and control variable
#       img[:]=(0,0,0)
#       k = 0
#       # Init variables for sensor coordinates
#       x_s = 800
#       y_s = 160
#       # Init normal force
#       f_n = 0
#       # Init tangential force
#       f_t = np.zeros(2)
#       # Init center of pressure array and help variables
#       center_of_pressure = np.zeros(2)
#       Cij = np.zeros(2)
#       SUM_zC = np.zeros(2)
#       # Init variables for pressure and pressure area
#       p = 0
#       pressure_area = 0

#       for j in range(taxel_rows):
#         for i in range(taxel_cols):
#           # Convert output data to sensor coordinates
#           x = np.clip(width -margin-i*pitch-dx[k], 0, width)
#           y = np.clip(height-margin-j*pitch-dy[k], 0, height)
#           z = np.clip(                  tz+dz[k], 0, 100)
#           # Draw sensor circles
#           color = (255-int(z/100 * 255), 210, 255-int(z/100 * 255))
#           cv2.circle(img, (int(x), int(y)), int(z), color, -1)
#           #Control variable
#           k = k+1
#           #Help variable for sensor x-Coordinate
#           x_s = x_s-160
#           # Add up tangential vector coordinates to f_t
#           f_t[0] += (x-x_s)
#           f_t[1] += (y-y_s)
#           #Add all normal forces together
#           f_n += z
#           # Add up all areas of pressure if that area receives a normal force
#           if z >= 12:
#             pressure_area += 0.0047*0.0047
#           # Append coordinates to array
#           Cij[0] = x
#           Cij[1] = y
#           SUM_zC += z*Cij
#         #Init helping variables for sensor coordinates with new values
#         x_s = 800
#         y_s = 160 + ((j+1)*160)
#       # Calculate center of pressure
#       center_of_pressure = SUM_zC/f_n
#       # Visualize center of pressure as a white circle
#       cv2.circle(img, (int(center_of_pressure[0]), int(center_of_pressure[1])), 10, (255,255,255,255), -1)
#       #Visualize total tangential force with an arrow
#       cv2.arrowedLine(img, (int(center_of_pressure[0]), int(center_of_pressure[1])), (int(f_t[0]+center_of_pressure[0]), int(f_t[1]+center_of_pressure[1])), (255,100,0,255), 15)
#       # Display sensor image
#       cv2.imshow("xela-sensor", img)
#       # Publish sensor data
#       self._pub.publish(f_n, f_t, center_of_pressure)
#       #Stop criteria
#       if cv2.waitKey(1) > 0:
#         break

#   def __del__(self):
#     cv2.destroyAllWindows()

# if __name__ == '__main__':
#   rospy.init_node('xela_demo', anonymous=True, log_level=rospy.DEBUG)
#   node = XelaSensorDemo()
#   rospy.spin()



import rospy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Int32
from xela_server.msg import XStream

class ListenAndPublish():

    def __init__(self):
        rospy.Subscriber("/xServTopic", XStream, self.callback)
        self.pub_sensor1 = rospy.Publisher("sensor1normal", Int32, queue_size=1)
        self.pub_sensor2 = rospy.Publisher("sensor2normal", Int32, queue_size=1)


    def callback(self, data):
        #print(data.data[0].xyz[12].z)
        sensor1normal = data.data[0].xyz[12].z
        
        # sensor1_normal_mean = np.mean([data.data[0].xyz[0].z, data.data[0].xyz[1].z, data.data[0].xyz[2].z, data.data[0].xyz[3].z, data.data[0].xyz[4].z, data.data[0].xyz[5].z,
        #     data.data[0].xyz[6].z, data.data[0].xyz[7].z, data.data[0].xyz[8].z, data.data[0].xyz[9].z, data.data[0].xyz[10].z, data.data[0].xyz[11].z, data.data[0].xyz[12].z, 
        #     data.data[0].xyz[13].z, data.data[0].xyz[14].z, data.data[0].xyz[15].z])
        # sensor2_normal_mean = np.mean([data.data[1].xyz[0].z, data.data[1].xyz[1].z, data.data[1].xyz[2].z, data.data[1].xyz[3].z, data.data[1].xyz[4].z, data.data[1].xyz[5].z,
        #     data.data[1].xyz[6].z, data.data[1].xyz[7].z, data.data[1].xyz[8].z, data.data[1].xyz[9].z, data.data[1].xyz[10].z, data.data[1].xyz[11].z, data.data[1].xyz[12].z, 
        #     data.data[1].xyz[13].z, data.data[1].xyz[14].z, data.data[1].xyz[15].z])

        self.pub_sensor1.publish(sensor1normal)
        # self.pub_sensor2.publish(sensor2_normal_mean)

        #rospy.loginfo(rospy.get_caller_id() + "I heard %s (message %d)", data.data, self.count)
        #self.pub.publish(self.count)


if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)

    l = ListenAndPublish()
    rospy.spin()

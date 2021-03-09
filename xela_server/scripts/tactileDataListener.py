#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Int32
from xela_server.msg import XStream

class ListenAndPublish():

    def __init__(self):
        rospy.Subscriber("/xServTopic", XStream, self.callback)
        self.S1_normalZ = list()
        self.S1_shearX = list()
        self.S1_shearY = list()

        self.S2_normalZ = list()
        self.S2_shearX = list()
        self.S2_shearY = list()


    def callback(self, data):


        self.sensr1_vector = [data.data[0].xyz[0:16]]

        self.S1_normalZ.append(data.data[0].xyz[0:16])
        #self.S1_shearX.append(data.data[0].xyz[0:16])
        #self.S1_shearY.append(data.data[0].xyz[0:16])

        self.S2_normalZ.append(data.data[1].xyz[0:16])
        #self.S2_shearX.append(data.data[1].xyz[0:16])
        #self.S2_shearY.append(data.data[1].xyz[0:16])



if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)

    l = ListenAndPublish()

    rospy.spin()
    if rospy.is_shutdown():
        print(np.shape(np.array(l.S1_normalZ)[0]))
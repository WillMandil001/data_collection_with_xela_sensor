#!/usr/bin/env python
import rospy
import numpy as np
from scipy.interpolate import interp2d
import matplotlib.pyplot as plt
from xela_server.msg import XStream




class ListenAndPublish():

    def __init__(self):
        # PLOT AXIS
        self.n = 24
        self.m = 24
        self.y = np.linspace(0, 3, 4)
        self.x = np.linspace(0, 3, 4)
        self.y2 = np.linspace(0, 3, self.n)
        self.x2 = np.linspace(0, 3, self.n)

        self.XX, self.YY = np.meshgrid(self.x, self.y)
        self.XX = np.reshape(self.XX, -1)
        self.YY = np.flip(np.reshape(self.YY, -1), 0)
        self.XX2, self.YY2 = np.meshgrid(self.x2, self.y2)

        self.comp_vect = np.ones(3 * 16)
        self.comp_vect[0:16] = 70
        self.comp_vect[16:32] = 30
        self.comp_vect[32::] = 30

        self.j = 0
        self.Z0 = np.ones(16 * 3)
        self.ZZ = np.ones(16 * 3)

        self.Z2 = np.zeros(self.n*self.m)
        self.X2 = np.zeros(self.n*self.m)
        self.Y2 = np.zeros(self.n*self.m)

        self.Z = self.ZZ[0:16]
        self.X = self.ZZ[16:32]
        self.Y = self.ZZ[32::]

        self.f_Z = interp2d(self.XX, self.YY, self.Z, kind='linear')
        self.Z2 = self.f_Z(self.x2, self.y2)

        self.f_X = interp2d(self.XX, self.YY, self.X, kind='linear')
        self.X2 = self.f_X(self.x2, self.y2)

        self.f_Y = interp2d(self.XX, self.YY, self.Y, kind='linear')
        self.Y2 = self.f_Y(self.x2, self.y2)


        self.fig, self.ax_lst = plt.subplots(figsize=(13, 3), ncols=3)
        self.ax_lst = self.ax_lst.ravel()
        plt.show(block=False)


        self.im1 = self.ax_lst[0].imshow(self.Z2, interpolation='sinc',
                              origin='bottom',
                              vmin=0,
                              vmax=2500,
                              cmap='Spectral')

        self.im2 = self.ax_lst[1].imshow(self.X2, interpolation='bicubic',
                              origin='bottom',
                              aspect='equal',
                              vmin=0,
                              vmax=700,
                              cmap='Spectral')

        self.im3 = self.ax_lst[2].imshow(self.Z2, interpolation='bicubic',
                              origin='bottom',
                              aspect='equal',
                              vmin=0,
                              vmax=700,
                              cmap='Spectral')

        self.ax_lst[0].title.set_text('Normal Force')
        self.ax_lst[1].title.set_text('Shear Force along x')
        self.ax_lst[2].title.set_text('Shear Force along y')
        self.fig.colorbar(self.im1, ax=self.ax_lst[0], shrink=0.5)
        self.fig.colorbar(self.im2, ax=self.ax_lst[1], shrink=0.5)
        self.fig.colorbar(self.im3, ax=self.ax_lst[2],shrink=0.5)

        rospy.Subscriber("/xServTopic", XStream, self.callback)
        self.count = 0

    def callback(self, data):
        sensor = data
        """
        if self.j == 0:
            for i in range(16):
                self.Z0[i] = np.array(sensor.data[0].xyz[i].z)
                self.Z0[i + 16] = np.array(sensor.data[0].xyz[i].x)
                self.Z0[i + 32] = np.array(sensor.data[0].xyz[i].y)
            self.j=+1

        for i in range(16):
            self.ZZ[i] = np.array(sensor.data[0].xyz[i].z)
            self.ZZ[i + 16] = np.array(sensor.data[0].xyz[i].x)
            self.ZZ[i + 32] = np.array(sensor.data[0].xyz[i].y)

        self.ZZ = (self.ZZ - self.Z0) * (abs(self.ZZ - self.Z0) > self.comp_vect)
        self.Z = self.ZZ[0:16]
        self.X = self.ZZ[16:32]
        self.Y = self.ZZ[32::]


        self.f_Z = interp2d(self.XX, self.YY, self.Z, kind='linear')
        self.Z2 = self.f_Z(self.x2, self.y2)

        self.f_X = interp2d(self.XX, self.YY, self.X, kind='linear')
        self.X2 = self.f_X(self.x2, self.y2)

        self.f_Y = interp2d(self.XX, self.YY, self.Y, kind='linear')
        self.Y2 = self.f_Y(self.x2, self.y2)

        self.im1.set_data(self.Z2)
        self.im2.set_data(self.X2)
        self.im3.set_data(self.Y2)

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

        self.Z2 = np.zeros(self.n * self.m)
        self.X2 = np.zeros(self.n * self.m)
        self.Y2 = np.zeros(self.n * self.m)

        self.Z = np.zeros(16)
        self.X = np.zeros(16)
        self.Y = np.zeros(16)
        """

        

if __name__ == '__main__':

    rospy.init_node('listener', anonymous=True)
    l = ListenAndPublish()

    # "spin()" is a special method in ROS, which just keeps this
    # programm running and handling all incoming event (on subscribers)
    # to be processed. 
    rospy.spin()





"""

def receive_sensor(msg):
    global sensor
    sensor = msg


if __name__ == "__main__":

    sensor = None

    client = roslibpy.Ros(host='127.0.0.1', port=9090)
    client.run()
    sub_data = roslibpy.Topic(client, '/xServTopic', 'xela_server/XStream')

# PLOT AXIS
    n = 24
    m = 36
    y = np.linspace(0, 3, 4)
    x = np.linspace(0, 5, 6)
    y2 = np.linspace(0, 3, n)
    x2 = np.linspace(0, 5, m)

    XX, YY = np.meshgrid(x, y)
    XX = np.reshape(XX, -1)
    YY = np.flip(np.reshape(YY, -1), 0)
    XX2, YY2 = np.meshgrid(x2, y2)

    comp_vect = np.ones(3 * 24)
    comp_vect[0:24] = 70
    comp_vect[24:48] = 30
    comp_vect[48::] = 30

    j = 0
    n = 0
    Z0 = np.ones(24 * 3)
    ZZ = np.ones(24 * 3)

    Z2 = np.zeros(n*m)
    X2 = np.zeros(n*m)
    Y2 = np.zeros(n*m)

    Z = ZZ[0:24]
    X = ZZ[24:48]
    Y = ZZ[48::]

    f_Z = interp2d(XX, YY, Z, kind='linear')
    Z2 = f_Z(x2, y2)

    f_X = interp2d(XX, YY, X, kind='linear')
    X2 = f_X(x2, y2)

    f_Y = interp2d(XX, YY, Y, kind='linear')
    Y2 = f_Y(x2, y2)


    fig, ax_lst = plt.subplots(figsize=(13, 3), ncols=3)
    ax_lst = ax_lst.ravel()
    plt.show(block=False)


    im1 = ax_lst[0].imshow(Z2, interpolation='sinc',
                          origin='bottom',
                          vmin=0,
                          vmax=2500,
                          cmap='Spectral')

    im2 = ax_lst[1].imshow(X2, interpolation='bicubic',
                          origin='bottom',
                          aspect='equal',
                          vmin=0,
                          vmax=700,
                          cmap='Spectral')

    im3 = ax_lst[2].imshow(Z2, interpolation='bicubic',
                          origin='bottom',
                          aspect='equal',
                          vmin=0,
                          vmax=700,
                          cmap='Spectral')

    ax_lst[0].title.set_text('Normal Force')
    ax_lst[1].title.set_text('Shear Force along x')
    ax_lst[2].title.set_text('Shear Force along y')
    fig.colorbar(im1, ax=ax_lst[0], shrink=0.5)
    fig.colorbar(im2, ax=ax_lst[1], shrink=0.5)
    fig.colorbar(im3, ax=ax_lst[2],shrink=0.5)

    while True:

        sub_data.subscribe(receive_sensor)

        if sensor!=None:


            if j == 0:
                for i in range(24):
                    Z0[i] = np.array(sensor['data'][0]['xyz'][i]['z'])
                    Z0[i + 24] = np.array(sensor['data'][0]['xyz'][i]['x'])
                    Z0[i + 48] = np.array(sensor['data'][0]['xyz'][i]['y'])
                j=+1

            for i in range(24):
                ZZ[i] = np.array(sensor['data'][0]['xyz'][i]['z'])
                ZZ[i + 24] = np.array(sensor['data'][0]['xyz'][i]['x'])
                ZZ[i + 48] = np.array(sensor['data'][0]['xyz'][i]['y'])

            ZZ = (ZZ - Z0) * (abs(ZZ - Z0) > comp_vect)
            Z = ZZ[0:24]
            X = ZZ[24:48]
            Y = ZZ[48::]


            f_Z = interp2d(XX, YY, Z, kind='linear')
            Z2 = f_Z(x2, y2)

            f_X = interp2d(XX, YY, X, kind='linear')
            X2 = f_X(x2, y2)

            f_Y = interp2d(XX, YY, Y, kind='linear')
            Y2 = f_Y(x2, y2)



            im1.set_data(Z2)
            im2.set_data(X2)
            im3.set_data(Y2)

            fig.canvas.draw_idle()
            fig.canvas.flush_events()

            Z2 = np.zeros(n * m)
            X2 = np.zeros(n * m)
            Y2 = np.zeros(n * m)

            Z = np.zeros(24)
            X = np.zeros(24)
            Y = np.zeros(24)

            plt.savefig("Plot/Graph" + str(n) + ".png", format="PNG")
            n += 1
"""
import numpy as np
import pandas as pd
import cv2


def rot90(m, k=1, axes=(0,1)):
    axes = tuple(axes)
    if len(axes) != 2:
        raise ValueError("len(axes) must be 2.")

    m = np.asanyarray(m)

    if axes[0] == axes[1] or np.absolute(axes[0] - axes[1]) == m.ndim:
        raise ValueError("Axes must be different.")

    if (axes[0] >= m.ndim or axes[0] < -m.ndim
        or axes[1] >= m.ndim or axes[1] < -m.ndim):
        raise ValueError("Axes={} out of range for array of ndim={}."
            .format(axes, m.ndim))

    k %= 4

    if k == 0:
        return m[:]
    if k == 2:
        return flip(flip(m, axes[0]), axes[1])

    axes_list = np.arange(0, m.ndim)
    (axes_list[axes[0]], axes_list[axes[1]]) = (axes_list[axes[1]],
                                                axes_list[axes[0]])

    if k == 1:
        return np.transpose(flip(m,axes[1]), axes_list)
    else:
        # k == 3
        return flip(np.transpose(m, axes_list), axes[1])

def flip(m, axis):
    if not hasattr(m, 'ndim'):
        m = asarray(m)
    indexer = [slice(None)] * m.ndim
    try:
        indexer[axis] = slice(None, None, -1)
    except IndexError:
        raise ValueError("axis=%i is invalid for the %i-dimensional input array"
                         % (axis, m.ndim))
    return m[tuple(indexer)]
#===================================================================================================


class tactile_viz():

# /home/kiyanoush/Desktop/xela_validation/xelaSensor1_1
# /home/kiyanoush/Desktop/xela_validation/xelaSensor2_1
    def tester(self):
        tactile_sensor_files1 = "/home/kiyanoush/Desktop/xela_validation/xelaSensor1_1.csv"  #  xelaSensor1_bottomup   xelaSensor1_left2right.csv"
        tactile_sensor_files2 = "/home/kiyanoush/Desktop/xela_validation/xelaSensor2_1.csv"
        # tactile_sensor_files3 = "/home/kiyanoush/slip_detection_franka/Dataset/xela_validation/xelaSensor1_topdown.csv"
        # tactile_sensor_files4 = "/home/kiyanoush/slip_detection_franka/Dataset/xela_validation/xelaSensor1_bottomup.csv"
        # tactile_sensor_files5 = "/home/kiyanoush/slip_detection_franka/Dataset/xela_validation/xelaSensor1_spin.csv"

        images_new_sample1 = np.asarray(pd.read_csv(tactile_sensor_files1, header=None))[1:]
        images_new_sample2 = np.asarray(pd.read_csv(tactile_sensor_files2, header=None))[1:]
        # images_new_sample3 = np.asarray(pd.read_csv(tactile_sensor_files3, header=None))[1:]
        # images_new_sample4 = np.asarray(pd.read_csv(tactile_sensor_files4, header=None))[1:]
        # images_new_sample5 = np.asarray(pd.read_csv(tactile_sensor_files5, header=None))[1:]
        image_set = [images_new_sample1, images_new_sample2]#, images_new_sample3, images_new_sample4, images_new_sample5]
        colors = [(255, 0, 0), (0, 255, 0)]#, (0, 0, 255), (255, 255, 0), (0, 255, 255)]


        while 1:
            for index, (color, images) in enumerate(zip(colors, image_set)):
                for image in images:
                    base  = image.astype(np.float32)
                    image = image_set[index][0].astype(np.float32)
                    self.visualise_time_sequence(image, base, color)
    

    def visualise_time_sequence(self, data, base, color):
        width  = 160  # 800 # width of the image
        height = 160  # 800 # height of the image
        margin = 30  # 160 # margin of the taxel in the image
        scale  = 40  # 15
        scale_normal  = 500  # 15
        radius = 3  # 15

        img = np.zeros((height,width,3), np.uint8)
        cv2.namedWindow('xela-sensor', cv2.WINDOW_NORMAL)


        diff = np.array(data) - np.array(base)
        diff = diff.reshape(4,4,3)
        diff = diff.T.reshape(3,4,4)
        dx = rot90((flip(diff[0], axis=0) / scale), k=3, axes=(0,1)).flatten()
        dy = rot90((flip(diff[1], axis=0) / scale), k=3, axes=(0,1)).flatten()
        dz = rot90((flip(diff[2], axis=0) / scale_normal), k=3, axes=(0,1)).flatten()
 

        image_positions = []
        for x in range(margin, 5*margin, margin):
            for y in range(margin, 5*margin, margin):
                image_positions.append([y,x])


        for xx, yy ,zz, image_position in zip(dx, dy, dz, image_positions):
            z = radius # + (abs(xx))
            x = image_position[0] + int(-xx) # int(zz)
            y = image_position[1] + int(-yy) # int(yy)
            #color = (255-int(z/100 * 255), 210, 255-int(z/100 * 255))  # Draw sensor circles
            gray = cv2.circle(img, (x, y), int(z), color=color, thickness=-1)  # Draw sensor circles
            #gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        cv2.imshow("xela-sensor", gray)   # Display sensor image
        key = cv2.waitKey(40)
        if key == 27:
            cv2.destroyAllWindows()




if __name__ == '__main__':
    v = tactile_viz()
    v.tester()

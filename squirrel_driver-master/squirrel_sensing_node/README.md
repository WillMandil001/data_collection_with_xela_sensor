# Squirrel sensing node
This package contains topic to read tactile sensor data and proximity sensor data via ROS.

# Setup
In order to launch this both proximity and tactile sensors arduino's boards need to be connected.</br>
The proximity board needs to be set as device **ttyACM0**, while the tactile board needs to be set as device **ttyACM1**.
Those ports can be set in the launch file [here](https://github.com/shadow-robot/squirrel_driver/blob/qmul/squirrel_sensing_node/launch/launch_with_rosserial.launch#L8). </br>

# Launch file
In order to launch the node run the following command:

```
roslaunch squirrel_sensing_node launch_with_rosserial.launch
```

# Topics
To read the values reported by the tactile sensors run the following command in the terminal:

```
rostopic echo /tactileShadow/calibrated
```

To read the values reported by the proximity sensors, run the following command in the terminal:

```
rostopic echo /proximityShadow/calibrated
```

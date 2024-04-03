# SLIMKIT ROS package

ROS node and test application for slamkit

## How to build rplidar ros package

   1) Clone this project to your catkin's workspace src folder
   2) Running catkin_make to build slamkitNode and slamkitNodeClient

## How to run slamkit ros package

There're two ways to run slamkit ros package

### I. Run slamkit node and test node 

```bash
roslaunch slamkit_ros test_slamkit.lanuch
```

### II. Run rplidar node only

```bash
roslaunch slamkit_ros slamkit_usb.launch
```

If want to run test node, in another terminal, run the following command:

```bash
rosrun slamkit_ros slamkitNodeClient
```

You should see slamkit's topic result in the console.

## slamkit frame

Use sensor_msgs/Imu.msg

# SLIMKIT ROS package

ROS node and test application for slamkit

## Dependency

Slamkit use libusb-1.0 to comunicate with host currently, before build slamkit ros package, make sure libusb-1.0 has been installed.

## How to build slamkit ros package

   1) Clone this project to your catkin's workspace src folder
   2) Running catkin_make to build slamkitNode and slamkitNodeClient

## How to run slamkit ros package

There're two ways to run slamkit ros package

### I. Run slamkit node and test node 

```bash
roslaunch slamkit_ros test_slamkit.lanuch
```

### II. Run slamkit node only

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

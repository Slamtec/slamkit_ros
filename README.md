# SLIMKIT ROS package

ROS node and test application for slamkit

## Dependency
This project need the following dependency:

1. libusb-1.0-0-dev
   sudo apt-get install libusb-1.0-0-dev
2. imu-tools
   sudo apt-get install ros-\<dist\>-imu-tools
   \<dist> is the distrbution, such as "noetic" or "melodic", and not include "<>"

## How to build slamkit ros package

1. make sure your system have installed ROS
   If you don't familar with ROS, please visit <https://wiki.ros.org> firstly, and follow the "Getting Started" and "Tutorials".
2. Clone or copy this project to your catkin's workspace src folder
   . If not catkin's workspace or src folder, please create them first, such as the following shell command.

   ```bash
   mkdir slamkit_ws
   cd slamkit_ws
   mkdir src
   cd src
   git clone <slamkit_ros repo>
   ```

3. Go to the catkin's workspace and running catkin_make to build slamkitNode and slamkitNodeClient
   
   ```bash
   cd slamkit_ws
   catkin_make
   source devel/setup.bash
   ```

## How to run slamkit ros package

1. Please plug in the SLAMKIT device, and use lsusb cmd to check the device is detected

   ```bash
   yuan@yuan ~/works/slamkit_ros_ws $ lsusb
   // other usb device

   //slamkit device
   Bus 003 Device 011: ID fccf:f100 SLAMTEC SLAMWARELC

   ```

2. Run the shell script to add udev rule, the script is in the scripts/ folder

   ```bash
      cd scripts
      ./add_udev.sh
   ```


### I. Run slamkit node only

```bash
//only run slamkitNode
roslaunch slamkit_ros slamkit_usb.launch
```

The slamkitNode will publis 3 topics: 

```bash
yuan@yuan ~ $ rostopic list
/imu/data_raw
/imu/mag
/imu/processed_yaw
/rosout
/rosout_agg
```

topic:

- imu/data_raw (sensor_msgs/Imu)
   Message containing raw IMU data, including angular velocities and linear accelerations.
   acc raw data in m/s^2, and gyro in rad/s.

- imu/mag (sensor_msgs/MagneticField)
   Magnetic data in Tesla.

- imu/processed_yaw (geometry_msgs::Vector3Stamped)
   Yaw data which processed in slamkit hardware device, in degree.

### II. Run slamkit node and imu filter node

```bash
roslaunch slamkit_usb_imu_filter.launch
```

The complementary_filter_node in imu tools will start at the same time, and the topics are:

- imu/data (sensor_msgs/Imu)
   The fused Imu message, containing the orientation.
- imu/rpy/filtered (geometry_msgs/Vector3)
   Debug only: The roll, pitch and yaw angles corresponding to the orientation published on the imu_data topic. (only published when publish_debug_topics == true)
- imu/steady_state (std_msgs/Bool)
   Debug only: Whether we are in the steady state when doing bias estimation. (only published when ~publish_debug_topics == true)

### III. View in Rviz

```bash
roslaunch slamkit_ros test_slamkit.launch
```

The test_slamkit.launch will start the following nodes:

```bash
    complementary_filter_node (imu_complementary_filter/complementary_filter_node)
    rviz (rviz/rviz)
    slamkitNode (slamkit_ros/slamkitNode)
    slamkitNodeClient (slamkit_ros/slamkitNodeClient)
```

The rviz will use /imu/data which published from complementary_filter_node and display int window.

If want to view the roll/pich/yaw data, start another teminal and echo the following topic:

- view roll/pich/yaw in rad

```bash
yuan@yuan ~ $ rostopic echo /imu/rpy/filtered 
header: 
  seq: 116510
  stamp: 
    secs: 1712653629
    nsecs: 982544281
  frame_id: "imu"
vector: 
  x: -0.5118724746572548
  y: -0.851032829630307
  z: 0.24633383804987355
---
```

- view roll/pich/yaw in degree

```bash
yuan@yuan ~ $ rostopic echo /imu/angles_degree 
header: 
  seq: 134377
  stamp: 
    secs: 1712653708
    nsecs: 150230549
  frame_id: "angle_degree"
vector: 
  x: -29.359611349189777
  y: -48.705977758347366
  z: 14.224317662550938
---
```

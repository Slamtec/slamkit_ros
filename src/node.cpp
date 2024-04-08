/*
 *  SLAMKIT ROS NODE
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2016 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <ros/ros.h>
//#include <tf/transform_broadcaster.h>
//#include <tf2/LinearMath/Quaternion.h>
//#include <tf2/LinearMath/Matrix3x3.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <tf2_ros/transform_broadcaster.h>
//#include <geometry_msgs/TransformStamped.h>
//#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <sl_slamkit.h>

#define DEG2RAD(x) ((x)*M_PI / 180.0)
#define SHIFT15BITS 32768.00
#define SCALE_FACTOR 16384.00
using namespace sl;



// ------------------------- Part3 Raw IMU -----------------
void publish_imu(ros::Publisher* pub, const sl_imu_raw_data_t& imu_data, std::string& frame_id)
{
    
    // according to datasheet,sensitivity scale factor is 16384, +-2g
    double acc_x = imu_data.acc_x / SHIFT15BITS * 2 * 9.8;
    double acc_y = imu_data.acc_y / SHIFT15BITS * 2 * 9.8;
    double acc_z = imu_data.acc_z / SHIFT15BITS * 2 * 9.8;
    
    //double acc_x_test = static_cast<double>(imu_data.acc_x);

    double gyro_x = imu_data.gyro_x / SHIFT15BITS * 500 / 180 * 3.1415926;
    double gyro_y = imu_data.gyro_y / SHIFT15BITS * 500 / 180 * 3.1415926;
    double gyro_z = imu_data.gyro_z / SHIFT15BITS * 500 / 180 * 3.1415926;

    double mag_x = imu_data.mag_x / 65536.0;
    double mag_y = imu_data.mag_y / 65536.0;
    double mag_z = imu_data.mag_z / 65536.0;

    // ROS_INFO("Received velocity message: raw_acc_x: %d, acc_x_test: %d, acc_y: %d, acc_z: %d", acc_x, acc_y, acc_y, acc_z);
    // ROS_INFO("Received velocity message: gyro_x: %d, gyro_x: %d, gyro_x: %d", imu_data.gyro_x , imu_data.gyro_y , imu_data.gyro_z);
    // ROS_INFO("Received velocity message: acc_x: %d, acc_x: %d, acc_x: %d", imu_data.acc_x , imu_data.acc_y , imu_data.acc_z);
    sensor_msgs::Imu Imu;
    
    Imu.header.stamp = ros::Time::now();
    Imu.header.frame_id = frame_id;
    Imu.linear_acceleration.x =  acc_x;
    Imu.linear_acceleration.y =  acc_y;
    Imu.linear_acceleration.z =  acc_z;

    Imu.angular_velocity.x = gyro_x;
    Imu.angular_velocity.y = gyro_y;
    Imu.angular_velocity.z = gyro_z;

    pub->publish(Imu);
}


//***************************************** Main Function ****************************************************8
int main(int argc, char * argv[])
{
    // initialize
    ros::init(argc, argv, "slamkit_node");   

    // define parameters
    std::string channel_type;
    std::string frame_id;

    int usb_venderId_slamkit;
    int usb_productId_slamkit;
    int usb_interfaceId_slamkit;
    int usb_txEndpoint_slamkit;
    int usb_rxEndpoint_slamkit;
    
    // Initialize Publisher
    ros::NodeHandle nh;
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 100);
    

    // get param
    ros::NodeHandle nh_private("~"); 
    nh_private.param<std::string>("channel_type", channel_type, "usb");

    // slamkit
    nh_private.param<int>("usb_venderId_slamkit", usb_venderId_slamkit, 64719);
    nh_private.param<int>("usb_productId_slamkit", usb_productId_slamkit, 61696);
    nh_private.param<int>("usb_interfaceId_slamkit", usb_interfaceId_slamkit, 3);
    nh_private.param<int>("usb_txEndpoint_slamkit", usb_txEndpoint_slamkit, 5);
    nh_private.param<int>("usb_rxEndpoint_slamkit", usb_rxEndpoint_slamkit, 5);
    nh_private.param<std::string>("frame_id", frame_id, "imu");

    // echo slamkit version info 
    int ver_major = SL_SLAMKIT_SDK_VERSION_MAJOR;
    int ver_minor = SL_SLAMKIT_SDK_VERSION_MINOR;
    int ver_patch = SL_SLAMKIT_SDK_VERSION_PATCH;   
    ROS_INFO("slamkit running on ROS package slamkit_ros, SDK Version:%d.%d.%d",ver_major,ver_minor,ver_patch);

    sl_result  op_result;

    std::shared_ptr<ISlamkitDriver> slamkit_drv = createSlamkitDriver();

    // usb communication
    if (channel_type == "usb")
    {
        // SLAMKIT usb channel connect
        auto _channel = createUSBChannel(usb_venderId_slamkit, usb_productId_slamkit, usb_interfaceId_slamkit, usb_txEndpoint_slamkit, usb_rxEndpoint_slamkit);
        if (SL_IS_FAIL((slamkit_drv)->connect(_channel)))
        {
            ROS_ERROR("Error, cannot connect to slamkit.");
            return -1;
        }
    }
    else
    {
        ROS_ERROR("Error, channel not support yet, please use usb channel.");
        return -1;
    }

    // define parameters
    sl_imu_raw_data_t imu_data;

    // main loop
    ros::Rate rate(200);  // loop rate
    while (ros::ok())
    {
        // 3. Publish IMU Raw topic
        op_result = slamkit_drv->getImuRawData(imu_data);
        if (SL_IS_FAIL(op_result))
        {
            ROS_ERROR("can not get Imu Raw Data.\n");
        }
        publish_imu(&imu_pub, imu_data, frame_id);
        
        ros::spinOnce();  
        rate.sleep();     
    }

    slamkit_drv->disconnect();
    return 0;
}

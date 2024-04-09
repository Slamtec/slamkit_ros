/*
 * Copyright (c) 2014, RoboPeak
 * All rights reserved.
 *
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
/*
 *  
 *  RPlidar ROS Node client test app
 *
 *  Copyright 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 * 
 */


#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <geometry_msgs/Vector3Stamped.h>

#define MY_PI   (3.141592654)

ros::Publisher degree_pub;

void imuCallback(const geometry_msgs::Vector3Stamped::ConstPtr& angle_rad)
{
    //ROS_INFO("I heard a slamkit %s[%u]:", imu->header.frame_id.c_str(), imu->header.seq);

    //ROS_INFO("I heard a slamkit %s[%u]:", angle_rad->header.frame_id.c_str(), angle_rad->header.seq);


    double angle_degree_roll = angle_rad->vector.x * 180.0 / MY_PI;
    double angle_degree_pitch = angle_rad->vector.y * 180.0 / MY_PI;
    double angle_degree_yaw = angle_rad->vector.z * 180.0 / MY_PI;


    geometry_msgs::Vector3Stamped angle;
    
    angle.header.stamp = ros::Time::now();
    angle.header.frame_id = "angle_degree";
    angle.vector.x =  angle_degree_roll;
    angle.vector.y =  angle_degree_pitch;
    angle.vector.z =  angle_degree_yaw;

    degree_pub.publish(angle);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "slamkit_node_client");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<geometry_msgs::Vector3Stamped>("imu/rpy/filtered", 100, imuCallback);

    degree_pub = nh.advertise<geometry_msgs::Vector3Stamped>("imu/angles_degree", 100);

    ros::spin();
    return 0;
}

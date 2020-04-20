/******************************************************************
 * Filename: encoder.cpp
 * Version: v1.0
 * Author: Meng Peng
 * Copyright(C): chuhang.tech
 * Date: 2020-04-15
 * Description: publish car speed to the topic car_speed
 * 
 ******************************************************************/

#include <ros/ros.h>
#include <std_msgs/Float32.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "encoder");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Float32>("car_speed", 1);

    // define message
    std_msgs::Float32 flt_msg;
    //set message data
    flt_msg.data = 20.1;

    // set loop iteration frequence
    ros::Rate loop_rate(100);
    while (nh.ok())
    {
        // output the published message
        ROS_INFO("car speed: %f kmh", flt_msg.data);

        // publish message 
        pub.publish(flt_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
/******************************************************************
 * Filename: controller_parking_start.cpp
 * Version: v1.0
 * Author: Meng Peng
 * Date: 2020-04-15
 * Description: publish message to the topic parking_enable to 
 * start autoparking
 * 
 ******************************************************************/

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "autopark/autoparking.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_parking_start");
    ros::NodeHandle nh;
    ros::Publisher pub_enable = nh.advertise<std_msgs::Bool>("parking_enable", 1);

    ros::Duration(1).sleep();   // wait initialization of publishers

    // define message
    std_msgs::Bool msg_parking_enable;
    // set message
    msg_parking_enable.data = true;     // to start autoparking

    // set loop rate: 10 Hz
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        if (pub_enable.getNumSubscribers() > 0)
        {
            // publish message to all subscribers of topic "parking_enable"
            pub_enable.publish(msg_parking_enable);

            ROS_INFO("autoparking enabled");

            // close this node
            ros::shutdown();
        }

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
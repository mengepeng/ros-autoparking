/******************************************************************
 * Filename: controller_parking_out.cpp
 * Version: v1.0
 * Author: Meng Peng
 * Date: 2020-06-01
 * Description: publish message of topic parking_out_enable to 
 * enable parking out function
 * 
 ******************************************************************/

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "autopark/autoparking.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_parking_out");
    ros::NodeHandle nh;
    ros::Publisher pub_enable = nh.advertise<std_msgs::Bool>("parking_out_enable", 1);

    ros::Duration(1).sleep();   // wait for initialization of publisher

    // define message
    std_msgs::Bool msg_parking_out_enable;
    // set message
    msg_parking_out_enable.data = true;    // to enable parking out

    // set loop rate: 10 Hz
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        if (pub_enable.getNumSubscribers() > 0)
        {
            // publish message to all subscribers of topic "parking_out"
            pub_enable.publish(msg_parking_out_enable);

            ROS_INFO("parking out enabled");

            // close this node
            ros::shutdown();
        }

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
/******************************************************************
 * Filename: controller_parking.cpp
 * Version: v1.0
 * Author: Meng Peng
 * Date: 2020-04-15
 * Description: publish message to the topic parking_enable to 
 * start search parking space
 * 
 ******************************************************************/

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include "autopark/autoparking.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_parking");
    ros::NodeHandle nh;
    ros::Publisher pub_enable = nh.advertise<std_msgs::Bool>("parking_enable", 1);
    ros::Publisher pub_move = nh.advertise<std_msgs::Float32>("cmd_move", 1);

    ros::Duration(1).sleep();   // wait 1 second for creating communication

    // define message
    std_msgs::Bool msg_parking_enable;
    std_msgs::Float32 msg_move;
    // set message
    msg_parking_enable.data = true;
    msg_move.data = speed_search_parking;

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        if (pub_enable.getNumSubscribers() > 0 && pub_move.getNumSubscribers() > 0)
        {
            // output the published message
            ROS_INFO("parking enable: %d", msg_parking_enable.data);
            ROS_INFO("cmd_move: %f", msg_move.data);

            // publish message 
            pub_enable.publish(msg_parking_enable);
            pub_move.publish(msg_move);

            ros::spinOnce();

            // close this node
            ros::shutdown();    // this node run only once at the beginning
        }

        loop_rate.sleep();
    }

    return 0;
}
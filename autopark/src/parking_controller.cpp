/******************************************************************
 * Filename: parking_controller.cpp
 * Version: v1.0
 * Author: Meng Peng
 * Copyright(C): chuhang.tech
 * Date: 2020-04-15
 * Description: publish "start" message to the topic parking_state
 * 
 ******************************************************************/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "parking_controller");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("parking_state", 1);

    // define message
    std_msgs::String str_msg;
    //set message data
    std::stringstream ss;
    ss << "start";
    str_msg.data = ss.str();

    while (ros::ok())
    {
        if (pub.getNumSubscribers() > 0)
        {
            // output the published message
            ROS_INFO("%s", str_msg.data.c_str());
            // publish message 
            pub.publish(str_msg);

            ros::spinOnce();
            // close this node
            ros::shutdown();
        }
    }

    return 0;
}
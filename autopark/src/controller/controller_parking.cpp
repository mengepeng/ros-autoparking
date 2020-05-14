/******************************************************************
 * Filename: controller_parking.cpp
 * Version: v1.0
 * Author: Meng Peng
 * Date: 2020-04-15
 * Description: publish message to the topic parking_enable to 
 * start or stop search parking space
 * 
 ******************************************************************/

#include <sstream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_parking");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("parking_enable", 1);
    ros::Duration(1).sleep();   // wait 1 second for creating communication

    // define message
    std_msgs::Bool msg_parking_enable;
    // set message
    msg_parking_enable.data = true;

    while (ros::ok())
    {
        if (pub.getNumSubscribers() > 0)
        {
            // output the published message
            ROS_INFO("parking enable: %d", msg_parking_enable.data);
            // publish message 
            pub.publish(msg_parking_enable);

            ros::spinOnce();

            // close this node
            ros::shutdown();    // this node run only once at the beginning
        }
    }

    return 0;
}
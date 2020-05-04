/******************************************************************
 * Filename: controller_parking.cpp
 * Version: v1.0
 * Author: Meng Peng
 * Date: 2020-04-15
 * Description: publish "start" message to the topic parking_state
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

    // define message
    std_msgs::Bool msg_enable;
    // set message
    msg_enable.data = true;

    while (ros::ok())
    {
        if (pub.getNumSubscribers() > 0)
        {
            // output the published message
            ROS_INFO("parking enable: %d", msg_enable.data);
            // publish message 
            pub.publish(msg_enable);

            ros::spinOnce();
            // shutdown this node
            ros::shutdown();    // this node only run once at begin
        }
    }

    return 0;
}
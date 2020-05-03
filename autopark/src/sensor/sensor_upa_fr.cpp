/******************************************************************
 * Filename: sensor_upa_fr.cpp
 * Version: v1.0
 * Author: Meng Peng
 * Date: 2020-04-18
 * Description: publish sensor message to topic upa_fr
 * 
 ******************************************************************/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Range.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor_upa_fr");
    ros::NodeHandle nh;

    ros::Publisher pub_upa_fr = nh.advertise<sensor_msgs::Range>("upa_fr", 1);

    // define sensor message
    sensor_msgs::Range msg_upa_fr;

    // values depend on upa
    msg_upa_fr.header.frame_id = "upa_fr";
    msg_upa_fr.radiation_type = sensor_msgs::Range::ULTRASOUND;
    msg_upa_fr.field_of_view = 2;
    msg_upa_fr.min_range = 0.1;
    msg_upa_fr.max_range = 3;

    ros::Rate loop_rate(50);
    while (nh.ok())
    {
        // set current values
        msg_upa_fr.header.stamp = ros::Time::now();
        msg_upa_fr.range = 1;   // fake
        // output range of published message
        ROS_INFO("upa_fr: range = %f", msg_upa_fr.range);

        // publish the sensor message
        pub_upa_fr.publish(msg_upa_fr);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
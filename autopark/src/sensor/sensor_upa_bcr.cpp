/******************************************************************
 * Filename: sensor_upa_bcr.cpp
 * Version: v1.0
 * Author: Meng Peng
 * Date: 2020-04-18
 * Description: publish sensor message to topic upa_bcr
 * 
 ******************************************************************/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Range.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor_upa_bcr");
    ros::NodeHandle nh;

    ros::Publisher pub_upa_bcr = nh.advertise<sensor_msgs::Range>("upa_bcr", 1);

    // define sensor message
    sensor_msgs::Range msg_upa_bcr;

    // values depend on upa
    msg_upa_bcr.header.frame_id = "upa_bcr";
    msg_upa_bcr.radiation_type = sensor_msgs::Range::ULTRASOUND;
    msg_upa_bcr.field_of_view = 2;
    msg_upa_bcr.min_range = 0.1;
    msg_upa_bcr.max_range = 3;

    ros::Rate loop_rate(50);
    while (nh.ok())
    {
        // set current values
        msg_upa_bcr.header.stamp = ros::Time::now();
        msg_upa_bcr.range = 1;   // fake
        // output range of published message
        ROS_INFO("upa_bcr: range = %f", msg_upa_bcr.range);

        // publish the sensor message
        pub_upa_bcr.publish(msg_upa_bcr);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
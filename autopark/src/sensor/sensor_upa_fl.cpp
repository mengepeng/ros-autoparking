/******************************************************************
 * Filename: sensor_upa_fl.cpp
 * Version: v1.0
 * Author: Meng Peng
 * Date: 2020-04-18
 * Description: publish sensor message to topic upa_fl
 * 
 ******************************************************************/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Range.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor_upa_fl");
    ros::NodeHandle nh;

    ros::Publisher pub_upa_fl = nh.advertise<sensor_msgs::Range>("upa_fl", 1);

    // define sensor message
    sensor_msgs::Range msg_upa_fl;

    // values depend on upa
    msg_upa_fl.header.frame_id = "upa_fl";
    msg_upa_fl.radiation_type = sensor_msgs::Range::ULTRASOUND;
    msg_upa_fl.field_of_view = 2;
    msg_upa_fl.min_range = 0.1;
    msg_upa_fl.max_range = 3;

    ros::Rate loop_rate(50);
    while (nh.ok())
    {
        // set current values
        msg_upa_fl.header.stamp = ros::Time::now();
        msg_upa_fl.range = 1;   // fake
        // output range of published message
        ROS_INFO("upa_fl: range = %f", msg_upa_fl.range);

        // publish the sensor message
        pub_upa_fl.publish(msg_upa_fl);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
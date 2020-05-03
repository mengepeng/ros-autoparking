/******************************************************************
 * Filename: sensor_upa_fcr.cpp
 * Version: v1.0
 * Author: Meng Peng
 * Date: 2020-04-18
 * Description: publish sensor message to topic upa_fcr
 * 
 ******************************************************************/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Range.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor_upa_fcr");
    ros::NodeHandle nh;

    ros::Publisher pub_upa_fcr = nh.advertise<sensor_msgs::Range>("upa_fcr", 1);

    // define sensor message
    sensor_msgs::Range msg_upa_fcr;

    // values depend on upa
    msg_upa_fcr.header.frame_id = "upa_fcr";
    msg_upa_fcr.radiation_type = sensor_msgs::Range::ULTRASOUND;
    msg_upa_fcr.field_of_view = 2;
    msg_upa_fcr.min_range = 0.1;
    msg_upa_fcr.max_range = 3;

    // set loop iteration frequence
    ros::Rate loop_rate(50);
    while (nh.ok())
    {
        // set current values
        msg_upa_fcr.header.stamp = ros::Time::now();
        msg_upa_fcr.range = 1;   // fake
        // output range of published message
        ROS_INFO("upa_fcr: range = %f", msg_upa_fcr.range);

        // publish the sensor message
        pub_upa_fcr.publish(msg_upa_fcr);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
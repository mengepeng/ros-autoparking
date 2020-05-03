/******************************************************************
 * Filename: sensor_upa_br.cpp
 * Version: v1.0
 * Author: Meng Peng
 * Date: 2020-04-18
 * Description: publish sensor message to topic upa_br
 * 
 ******************************************************************/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Range.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor_upa_br");
    ros::NodeHandle nh;

    ros::Publisher pub_upa_br = nh.advertise<sensor_msgs::Range>("upa_br", 1);

    // define sensor message
    sensor_msgs::Range msg_upa_br;

    // values depend on upa
    msg_upa_br.header.frame_id = "upa_br";
    msg_upa_br.radiation_type = sensor_msgs::Range::ULTRASOUND;
    msg_upa_br.field_of_view = 2;
    msg_upa_br.min_range = 0.1;
    msg_upa_br.max_range = 3;

    ros::Rate loop_rate(50);
    while (nh.ok())
    {
        // set current values
        msg_upa_br.header.stamp = ros::Time::now();
        msg_upa_br.range = 1;   // fake
        // output range of published message
        ROS_INFO("upa_br: range = %f", msg_upa_br.range);

        // publish the sensor message
        pub_upa_br.publish(msg_upa_br);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
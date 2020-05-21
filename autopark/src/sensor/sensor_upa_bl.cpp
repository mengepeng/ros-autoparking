/******************************************************************
 * Filename: sensor_upa_bl.cpp
 * Version: v1.0
 * Author: Meng Peng
 * Date: 2020-04-18
 * Description: publish sensor message to topic upa_bl
 * 
 ******************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/Range.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor_upa_bl");
    ros::NodeHandle nh;

    ros::Publisher pub_upa_bl = nh.advertise<sensor_msgs::Range>("upa_bl", 1);

    // define sensor message
    sensor_msgs::Range msg_upa_bl;

    // values depend on upa
    msg_upa_bl.header.frame_id = "upa_bl";
    msg_upa_bl.radiation_type = sensor_msgs::Range::ULTRASOUND;
    msg_upa_bl.field_of_view = 2;
    msg_upa_bl.min_range = 0.1;
    msg_upa_bl.max_range = 3;

    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        // set current values
        msg_upa_bl.header.stamp = ros::Time::now();
        msg_upa_bl.range = 1;   // fake
        // output range of published message
        ROS_INFO("upa_bl: range = %f", msg_upa_bl.range);

        // publish the sensor message
        pub_upa_bl.publish(msg_upa_bl);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
/******************************************************************
 * Filename: sensor_upa_bcl.cpp
 * Version: v1.0
 * Author: Meng Peng
 * Date: 2020-04-18
 * Description: publish sensor message to topic upa_bcl
 * 
 ******************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/Range.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor_upa_bcl");
    ros::NodeHandle nh;

    ros::Publisher pub_upa_bcl = nh.advertise<sensor_msgs::Range>("upa_bcl", 1);

    // define sensor message
    sensor_msgs::Range msg_upa_bcl;

    // values depend on upa
    msg_upa_bcl.header.frame_id = "upa_bcl";
    msg_upa_bcl.radiation_type = sensor_msgs::Range::ULTRASOUND;
    msg_upa_bcl.field_of_view = 2;
    msg_upa_bcl.min_range = 0.1;
    msg_upa_bcl.max_range = 3;

    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        // set current values
        msg_upa_bcl.header.stamp = ros::Time::now();
        msg_upa_bcl.range = 1;   // fake
        // output range of published message
        ROS_INFO("upa_bcl: range = %f", msg_upa_bcl.range);

        // publish the sensor message
        pub_upa_bcl.publish(msg_upa_bcl);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
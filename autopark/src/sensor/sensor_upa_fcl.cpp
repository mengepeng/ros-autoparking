/******************************************************************
 * Filename: sensor_upa_fcl.cpp
 * Version: v1.0
 * Author: Meng Peng
 * Date: 2020-04-18
 * Description: publish sensor message to topic upa_fcl
 * 
 ******************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/Range.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor_upa_fcl");
    ros::NodeHandle nh;

    ros::Publisher pub_upa_fcl = nh.advertise<sensor_msgs::Range>("upa_fcl", 1);

    // define sensor message
    sensor_msgs::Range msg_upa_fcl;

    // values depend on upa
    msg_upa_fcl.header.frame_id = "upa_fcl";
    msg_upa_fcl.radiation_type = sensor_msgs::Range::ULTRASOUND;
    msg_upa_fcl.field_of_view = 2;
    msg_upa_fcl.min_range = 0.1;
    msg_upa_fcl.max_range = 3;

    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        // set current values
        msg_upa_fcl.header.stamp = ros::Time::now();
        msg_upa_fcl.range = 1;   // fake
        // output range of published message
        ROS_INFO("upa_fcl: range = %f", msg_upa_fcl.range);

        // publish the sensor message
        pub_upa_fcl.publish(msg_upa_fcl);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
/******************************************************************
 * Filename: sensor_apa_rf.cpp
 * Version: v1.0
 * Author: Meng Peng
 * Date: 2020-04-18
 * Description: publish sensor message to topic apa_rf 
 * 
 ******************************************************************/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Range.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor_apa_rf");
    ros::NodeHandle nh;

    // create and initialize publisher, set queue_size 1 to ensure real time data
    ros::Publisher pub_apa_rf = nh.advertise<sensor_msgs::Range>("apa_rf", 1);

    // define sensor message
    sensor_msgs::Range msg_apa_rf;

    // values depend on apa
    msg_apa_rf.header.frame_id = "apa_rb";
    msg_apa_rf.radiation_type = sensor_msgs::Range::ULTRASOUND;
    msg_apa_rf.field_of_view = 1;
    msg_apa_rf.min_range = 0.2;
    msg_apa_rf.max_range = 7;

    ros::Rate loop_rate(20);
    while (nh.ok())
    {
        // set current values
        msg_apa_rf.header.stamp = ros::Time::now();
        msg_apa_rf.range = 1;   // fake

        // output range of published message
        ROS_INFO("apa_rf: range = %f", msg_apa_rf.range);

        // publish the sensor message
        pub_apa_rf.publish(msg_apa_rf);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
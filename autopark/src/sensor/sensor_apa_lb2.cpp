/******************************************************************
 * Filename: sensor_apa_lb2.cpp
 * Version: v1.0
 * Author: Meng Peng
 * Date: 2020-05-18
 * Description: publish sensor message to topic apa_lb2
 * apa_lb2 is a apa sensor near the rear of car and closed to apa_lb
 * 
 ******************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/Range.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor_apa_lb2");
    ros::NodeHandle nh;

    // create and initialize publisher, set queue_size 1 to ensure real time data
    ros::Publisher pub_apa_lb2 = nh.advertise<sensor_msgs::Range>("apa_lb2", 1);

    // define sensor message
    sensor_msgs::Range msg_apa_lb2;

    // values depend on apa
    msg_apa_lb2.header.frame_id = "apa_lb2";
    msg_apa_lb2.radiation_type = sensor_msgs::Range::ULTRASOUND;
    msg_apa_lb2.field_of_view = 1;
    msg_apa_lb2.min_range = 0.2;
    msg_apa_lb2.max_range = 7;

    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        // set current values
        msg_apa_lb2.header.stamp = ros::Time::now();
        msg_apa_lb2.range = 1;   // fake

        // output range of published message
        ROS_INFO("apa_lb2: range = %f", msg_apa_lb2.range);

        // publish the sensor message
        pub_apa_lb2.publish(msg_apa_lb2);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
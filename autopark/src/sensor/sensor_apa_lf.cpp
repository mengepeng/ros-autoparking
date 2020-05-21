/******************************************************************
 * Filename: sensor_apa_lf.cpp
 * Version: v1.0
 * Author: Meng Peng
 * Date: 2020-04-18
 * Description: publish sensor message to topic apa_lf 
 * 
 ******************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/Range.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor_apa_lf");
    ros::NodeHandle nh;

    // create and initialize publisher, set queue_size 1 to ensure real time data
    ros::Publisher pub_apa_lf = nh.advertise<sensor_msgs::Range>("apa_lf", 1);

    // define sensor message
    sensor_msgs::Range msg_apa_lf;

    // values depend on apa
    msg_apa_lf.header.frame_id = "apa_lf";
    msg_apa_lf.radiation_type = sensor_msgs::Range::ULTRASOUND;
    msg_apa_lf.field_of_view = 1;
    msg_apa_lf.min_range = 0.2;
    msg_apa_lf.max_range = 7;

    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        // set curret values
        msg_apa_lf.header.stamp = ros::Time::now();
        msg_apa_lf.range = 2;   // fake

        // output range of published message
        ROS_INFO("apa_lf: range = %f", msg_apa_lf.range);

        // publish the sensor message
        pub_apa_lf.publish(msg_apa_lf);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
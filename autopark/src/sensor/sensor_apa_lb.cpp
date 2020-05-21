/******************************************************************
 * Filename: sensor_apa_lb.cpp
 * Version: v1.0
 * Author: Meng Peng
 * Date: 2020-04-18
 * Description: publish sensor message to topic apa_lb 
 * 
 ******************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/Range.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor_apa_lb");
    ros::NodeHandle nh;

    // create and initialize publisher, set queue_size 1 to ensure real time data
    ros::Publisher pub_apa_lb = nh.advertise<sensor_msgs::Range>("apa_lb", 1);

    // define sensor message
    sensor_msgs::Range msg_apa_lb;

    // values depend on apa
    msg_apa_lb.header.frame_id = "apa_lb";
    msg_apa_lb.radiation_type = sensor_msgs::Range::ULTRASOUND;
    msg_apa_lb.field_of_view = 1;
    msg_apa_lb.min_range = 0.2;
    msg_apa_lb.max_range = 7;

    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        // set current values
        msg_apa_lb.header.stamp = ros::Time::now();
        msg_apa_lb.range = 1;   // fake

        // output range of published message
        ROS_INFO("apa_lb: range = %f", msg_apa_lb.range);

        // publish the sensor message
        pub_apa_lb.publish(msg_apa_lb);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
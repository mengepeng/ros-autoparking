/******************************************************************
 * Filename: sensor_encoder.cpp
 * Version: v1.0
 * Author: Meng Peng
 * Date: 2020-04-17
 * Description: publish car speed to topic car_speed
 * 
 ******************************************************************/

#include <ros/ros.h>
#include <std_msgs/Float32.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "encoder");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Float32>("car_speed", 1);

    // define message
    std_msgs::Float32 flt_msg;
    //set message data
    flt_msg.data = 5;   // 5 m/s = 18 km/h

    // set loop rate 100 Hz, this rate should not smaller than publish rate of upas
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        // output the published message
        ROS_INFO("car speed: %f m/s", flt_msg.data);

        // publish message 
        pub.publish(flt_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
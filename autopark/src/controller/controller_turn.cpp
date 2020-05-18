/******************************************************************
 * Filename: controller_turn.cpp
 * Version: v1.0
 * Author: Meng Peng
 * Date: 2020-05-16
 * Description: subscribe message from topic cmd_turn, then turn
 * 
 ******************************************************************/

#include <ros/ros.h>
#include <std_msgs/Float32.h>

// global variable to update message of topic cmd_turn
float turn_angle;

// steering control function
void do_turn(float angle)
{
    /*control program
    * angle = 0: turn straight
    * angle > 0: turn left
    * angle < 0: turn right
    */
}

// callback funktion of subscriber
void callback_cmd_turn(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("turn_angle: %f", msg->data);
    turn_angle = msg->data;

    do_turn(turn_angle);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_turn");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<std_msgs::Float32>("cmd_turn", 1, callback_cmd_turn);

    // set loop rate 10 Hz
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
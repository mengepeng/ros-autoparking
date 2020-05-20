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

using namespace std;

// global variable to update message of topic cmd_turn
static float turn_angle;

// steering control function
void do_turn()
{
    /*control program
    * turn_angle = 0: turn straight
    * turn_angle > 0: turn left
    * turn_angle < 0: turn right
    */
}

// callback funktion of subscriber
void callback_cmd_turn(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("turn_angle: %f", msg->data);
    turn_angle = msg->data;

    do_turn();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_turn");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<std_msgs::Float32>("cmd_turn", 1, callback_cmd_turn);

    // process callbacks in loop
    ros::spin();

    return 0;
}
/******************************************************************
 * Filename: controller_move.cpp
 * Version: v1.0
 * Author: Meng Peng
 * Date: 2020-05-16
 * Description: subscribe message from topic cmd_move, then move
 * 
 ******************************************************************/

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

using namespace std;


// global variables to update message of topic cmd_move
static float move_speed;                // move speed
static bool forward_state = true;       // forward state: default enabled
static bool backward_state = true;      // backward state: default enabled


// control motor with messages from "cmd_move", "forward_enable", "backward_enable" 
void do_move()
{
    /*control program
    * move_speed = 0: stop
    * move_speed > 0: forward with the speed |speed|
    * move_speed < 0: backward with the speed |speed|
    * 
    * forward_state = true: forward enabled
    * forward_state = false: forward disabled
    * 
    * backward_state = true: backward enabled
    * backward_state = false: backward disabled
    */
    if (forward_state && move_speed > 0)
    {
        ROS_INFO("move forward");
        // do move forward
    }
    else if (backward_state && move_speed < 0)
    {
        ROS_INFO("move backward");
        // do move backward
    }
    else
    {
        ROS_INFO("stop");
        // do stop
    }
}


// callback for "cmd_move"
void callback_cmd_move(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("move_speed: %f", msg->data);
    move_speed = msg->data;

    do_move();
}

// callback for "forward_enable"
void callback_forward_enable(const std_msgs::Bool::ConstPtr& msg)
{
    ROS_INFO("forward_enable: %d", msg->data);
    forward_state = msg->data;
}

// callback for "backward_enable"
void callback_backward_enable(const std_msgs::Bool::ConstPtr& msg)
{
    ROS_INFO("backward_enable: %d", msg->data);
    backward_state = msg->data;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_move");
    ros::NodeHandle nh;

    // define subscriber for topics "cmd_move", "forward_enable", "backward_enable"
    ros::Subscriber sub1 = nh.subscribe<std_msgs::Float32>("cmd_move", 1, callback_cmd_move);
    ros::Subscriber sub2 = nh.subscribe<std_msgs::Bool>("forward_enable", 1, callback_forward_enable);
    ros::Subscriber sub3 = nh.subscribe<std_msgs::Bool>("backward_enable", 1, callback_backward_enable);

    // process callbacks in loop
    ros::spin();

    return 0;
}
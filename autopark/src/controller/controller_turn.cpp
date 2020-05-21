/******************************************************************
 * Filename: controller_turn.cpp
 * Version: v1.0
 * Author: Meng Peng
 * Date: 2020-04-16
 * Description: subscribe message from topic cmd_turn, then turn
 * 
 ******************************************************************/

#include <ros/ros.h>
#include <std_msgs/Char.h>

using namespace std;

// global variable to update message from topic cmd_turn
static char turn_angle;


// function to check subscribed message from topic cmd_turn and do turn
void do_turn()
{
    switch (turn_angle)
    {
    case 'l':
        ROS_INFO("turn left once");     // turn wheel 1° to the left (steering rotate maybe 18°)
        /* code for real controller */
        break;
    case 'r':
        ROS_INFO("turn right once");    // turn wheel 1° to the right
        /* code */
        break;
    case 'L':
        ROS_INFO("turn full left");     // turn wheel to the full left position
        /* code */
        break;
    case 'R':
        ROS_INFO("turn full right");    // turn wheel to the full right position
        /* code */
        break;
    case 'D':
        ROS_INFO("turn direct");        // keep wheel direct (default-position)
        /* code */
        break;
    default:
        break;
    }
}

// callback of "cmd_turn"
void callback_cmd_turn(const std_msgs::Char::ConstPtr& msg)
{
    ROS_INFO("cmd_turn: %i", msg->data);
    turn_angle = msg->data;

    do_turn();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_turn");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<std_msgs::Char>("cmd_turn", 1, callback_cmd_turn);

    // process callbacks in loop
    ros::spin();

    return 0;
}
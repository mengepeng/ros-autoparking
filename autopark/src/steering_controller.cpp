/******************************************************************
 * Filename: steering_controller.cpp
 * Version: v1.0
 * Author: Meng Peng
 * Copyright(C): chuhang.tech
 * Date: 2020-04-18
 * Description: subscribe message from topic cmd_turn, then turn
 * 
 ******************************************************************/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

// global variable to update message from topic cmd_turn
std::string cmd_turn = "";

// callback funktion of subscriber
void cmd_turn_callback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("cmd_turn: %s", msg->data.c_str());
    std::stringstream ss;
    ss << msg->data.c_str();
    ss >> cmd_turn;
}

// function to control the car
void do_turn(char cc)
{
    switch (cc)
    {
    case 'l':
        /* code */
        break;
    case 'r':
        /* code */
        break;
    case 'L':
        /* code */
        break;
    case 'R':
        /* code */
        break;
    case 'D':
        /* code */
        break;
    default:
        break;
    }
}

// function to check subscribed message from topic cmd_turn
void check_cmd_turn(std::string cmd_str)
{
    ROS_INFO("check cmd_turn: %s", cmd_str.c_str());
    if(cmd_str == "left")
    {
        ROS_INFO("turn left once");
        do_turn('l');   // turn wheel 1° to the left (steering rotate maybe 18°)
    }
    if(cmd_str == "right")
    {
        ROS_INFO("turn right once");
        do_turn('r');   // turn wheel 1° to the right
    }
    if(cmd_str == "full-left")
    {
        ROS_INFO("turn full left");
        do_turn('L');   // turn wheel to the full left position
    }
    if(cmd_str == "full-right")
    {
        ROS_INFO("turn full right");
        do_turn('R');   // turn wheel to the full right position
    }
    if(cmd_str == "direct")
    {
        ROS_INFO("turn direct");
        do_turn('D');   // keep wheel direct (default-position)
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "steering_controller");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<std_msgs::String>("cmd_turn", 1, cmd_turn_callback);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();

        if (cmd_turn != "")
        {
            check_cmd_turn(cmd_turn);
            cmd_turn = "";
        }

        loop_rate.sleep();
    }

    return 0;
}
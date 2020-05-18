/******************************************************************
 * Filename: controller_move.cpp
 * Version: v1.0
 * Author: Meng Peng
 * Date: 2020-05-16
 * Description: subscribe message from topic cmd_move, then move
 * 
 ******************************************************************/

#include <ros/ros.h>
#include <std_msgs/Float32.h>

// global variable to update message of topic cmd_move
float move_speed;

// motor control function
void do_move(float speed)
{
    /*control program
    * speed = 0: stop
    * speed > 0: forward with the speed |speed|
    * speed < 0: backward with the speed |speed|
    */
}

// callback funktion of subscriber
void callback_cmd_move(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("move_speed: %f", msg->data);
    move_speed = msg->data;

    do_move(move_speed);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_move");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<std_msgs::Float32>("cmd_move", 1, callback_cmd_move);

    // set loop rate 10 Hz
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
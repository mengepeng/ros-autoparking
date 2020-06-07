/******************************************************************
 * Filename: controller_parking.cpp
 * Version: v1.0
 * Author: Meng Peng
 * Date: 2020-04-15
 * Description: subscribe message from topics parking_enable and 
 * search_done to start or stop searching parking space
 * 
 ******************************************************************/

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include "autopark/autoparking.h"

using namespace std;

static bool parking_enable = false;     // flag of parking enable
static bool search_done = false;        // flag of searching parking space done
static bool trigger_search = false;     // flag of search trigger


// callback of sub_parking_enable
void callback_parking_enable(const std_msgs::Bool::ConstPtr& msg)
{
    ROS_INFO("call callback of parking_enable: %d", msg->data);
    parking_enable = msg->data;
}

// callback of sub_search_done
void callback_search_done(const std_msgs::Bool::ConstPtr& msg)
{
    ROS_INFO("call callback of search_done: %d", msg->data);
    search_done = msg->data;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_search_parking");
    ros::NodeHandle nh;

    // create and initialize subscribers
    ros::Subscriber sub_parking_enable = nh.subscribe<std_msgs::Bool>("parking_enable", 1, \
    callback_parking_enable);

    ros::Subscriber sub_search_done = nh.subscribe<std_msgs::Bool>("search_done", 1, \
    callback_search_done);

    // create and initialize publicher
    ros::Publisher pub_move = nh.advertise<std_msgs::Float32>("cmd_move", 10);

    // define message
    std_msgs::Float32 msg_move;
    // set message
    msg_move.data = speed_search_parking;

    // set loop rate: 20Hz
    ros::Rate loop_rate(20);

    while (ros::ok())
    {
        ROS_INFO_STREAM_ONCE("node search_parking_space is running");
        if (parking_enable)
        {
            if (!trigger_search)
            {
                ROS_INFO("search parking space started");
                trigger_search = true;
            }
            else
            {
                if (!search_done)
                {
                    // publish message to topic "cmd_move"
                    pub_move.publish(msg_move);
                }
                else
                {
                    ROS_INFO("search parking space finished");
                    // reset
                    parking_enable = false;
                    search_done = false;
                    trigger_search = false;
                }
            }
        }
        else
        {
            if (trigger_search)
            {
                ROS_INFO("search parking space disabled");

                // reset
                parking_enable = false;
                search_done = false;
                trigger_search = false;
            }
        }

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
/******************************************************************
 * Filename: search_parking_space.h
 * Version: v1.0
 * Author: Meng Peng
 * Date: 2020-04-22
 * Description: class for search parking space
 * 
 ******************************************************************/

#ifndef SEARCH_PARKING_SPACE_H_
#define SEARCH_PARKING_SPACE_H_

#include <queue>
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>
#include <sstream>
#include <stdio.h>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Range.h>


class SearchParkingSpace
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_apa_;
    ros::Subscriber sub_car_speed_;
    ros::Publisher pub_parking_space_;

    sensor_msgs::Range msg_apa_;
    std_msgs::Float32 msg_car_speed_;
    std_msgs::Header msg_parking_space_;

    std::queue<sensor_msgs::Range> que_apa_;
    std::vector<sensor_msgs::Range> vec_turnpoint_;

public:
    SearchParkingSpace(ros::NodeHandle* nodehandle);
    void callback_apa(const sensor_msgs::Range::ConstPtr& msg);
    void callback_car_speed(const std_msgs::Float32::ConstPtr& msg);
    void check_parking_space();
    ~SearchParkingSpace();
};

#endif
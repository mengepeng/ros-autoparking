/******************************************************************
 * Filename: choose_parking_space.h
 * Version: v1.0
 * Author: Meng Peng
 * Date: 2020-05-03
 * Description: class for choose parking space
 * 
 ******************************************************************/

#ifndef CHOOSE_PARKING_SPACE_H_
#define CHOOSE_PARKING_SPACE_H_

#include <queue>
#include <cmath>
#include <algorithm>
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>


class ChooseParkingSpace
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_car_speed_;
    ros::Subscriber sub_parking_space_lf_;
    ros::Subscriber sub_parking_space_lb_;
    ros::Subscriber sub_parking_space_rf_;
    ros::Subscriber sub_parking_space_rb_;
    ros::Publisher pub_parking_start_;

    std_msgs::Float32 msg_car_speed_;
    std_msgs::Header msg_parking_space_;
    std_msgs::Header msg_parking_space_lf_;
    std_msgs::Header msg_parking_space_lb_;
    std_msgs::Header msg_parking_space_rf_;
    std_msgs::Header msg_parking_space_rb_;
    std::queue<std_msgs::Header> que_parking_space_lf_;
    std::queue<std_msgs::Header> que_parking_space_rf_;


public:
    ChooseParkingSpace();
    void callback_car_speed(const std_msgs::Float32::ConstPtr& msg);
    void callback_parking_space_lf(const std_msgs::Header::ConstPtr& msg);
    void callback_parking_space_lb(const std_msgs::Header::ConstPtr& msg);
    void callback_parking_space_rf(const std_msgs::Header::ConstPtr& msg);
    void callback_parking_space_rb(const std_msgs::Header::ConstPtr& msg);
    void choose_parking_space();
    ~ChooseParkingSpace();
};

#endif
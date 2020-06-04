/******************************************************************
 * Filename: parking_out.h
 * Version: v1.0
 * Author: Meng Peng
 * Date: 2020-06-01
 * Description: declare class for getting out of parking space
 * 
 ******************************************************************/

#ifndef PARKING_OUT_H_
#define PARKING_OUT_H_

#include <cmath>
#include <algorithm>
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Char.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Range.h>


class ParkingOut
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_car_speed_;
    ros::Subscriber sub_apa_lf_;
    ros::Subscriber sub_apa_lb_;
    ros::Subscriber sub_apa_rf_;
    ros::Subscriber sub_apa_rb_;
    ros::Subscriber sub_upa_fl_;
    ros::Subscriber sub_upa_fcl_;
    ros::Subscriber sub_upa_fcr_;
    ros::Subscriber sub_upa_fr_;
    ros::Subscriber sub_upa_bl_;
    ros::Subscriber sub_upa_bcl_;
    ros::Subscriber sub_upa_bcr_;
    ros::Subscriber sub_upa_br_;

    ros::Publisher pub_move_;
    ros::Publisher pub_turn_;

    std_msgs::Float32 msg_car_speed_;
    sensor_msgs::Range msg_apa_lf_;
    sensor_msgs::Range msg_apa_lb_;
    sensor_msgs::Range msg_apa_rf_;
    sensor_msgs::Range msg_apa_rb_;
    sensor_msgs::Range msg_upa_fl_;
    sensor_msgs::Range msg_upa_fcl_;
    sensor_msgs::Range msg_upa_fcr_;
    sensor_msgs::Range msg_upa_fr_;
    sensor_msgs::Range msg_upa_bl_;
    sensor_msgs::Range msg_upa_bcl_;
    sensor_msgs::Range msg_upa_bcr_;
    sensor_msgs::Range msg_upa_br_;

    std_msgs::Float32 msg_cmd_move_;
    std_msgs::Char msg_cmd_turn_;

public:
    ParkingOut(ros::NodeHandle* nodehandle);

    void callback_car_speed(const std_msgs::Float32::ConstPtr& msg);

    void callback_apa_lf(const sensor_msgs::Range::ConstPtr& msg);
    void callback_apa_lb(const sensor_msgs::Range::ConstPtr& msg);
    void callback_apa_rf(const sensor_msgs::Range::ConstPtr& msg);
    void callback_apa_rb(const sensor_msgs::Range::ConstPtr& msg);

    void callback_upa_fl(const sensor_msgs::Range::ConstPtr& msg);
    void callback_upa_fcl(const sensor_msgs::Range::ConstPtr& msg);
    void callback_upa_fcr(const sensor_msgs::Range::ConstPtr& msg);
    void callback_upa_fr(const sensor_msgs::Range::ConstPtr& msg);

    void callback_upa_bl(const sensor_msgs::Range::ConstPtr& msg);
    void callback_upa_bcl(const sensor_msgs::Range::ConstPtr& msg);
    void callback_upa_bcr(const sensor_msgs::Range::ConstPtr& msg);
    void callback_upa_br(const sensor_msgs::Range::ConstPtr& msg);

    void parking_out_left_perpendicular();
    void parking_out_left_parallel();
    void parking_out_right_perpendicular();
    void parking_out_right_parallel();

    ~ParkingOut();
};

#endif
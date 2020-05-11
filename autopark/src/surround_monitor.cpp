/******************************************************************
 * Filename: surround_monitor.cpp
 * Version: v1.0
 * Author: Meng Peng
 * Date: 2020-05-08
 * Description: surround monitor with ultrasonic sensors for AEB
 * 
 ******************************************************************/

#include "autopark/autoparking.h"
#include "autopark/surround_monitor.h"

using namespace std;


// CONSTRUCTOR: called when this object is created to set up subscribers and publishers
SurroundMonitor::SurroundMonitor()
{
    ROS_INFO("call constructor in surround_monitor");

    sub_car_speed_ = nh_.subscribe<std_msgs::Float32>("car_speed", 1, \
    &SurroundMonitor::callback_car_speed, this);

    sub_apa_lf_ = nh_.subscribe<sensor_msgs::Range>("apa_lf", 1, \
    &SurroundMonitor::callback_apa_lf, this);

    sub_apa_lb_ = nh_.subscribe<sensor_msgs::Range>("apa_lb", 1, \
    &SurroundMonitor::callback_apa_lb, this);

    sub_apa_rf_ = nh_.subscribe<sensor_msgs::Range>("apa_rf", 1, \
    &SurroundMonitor::callback_apa_rf, this);

    sub_apa_rb_ = nh_.subscribe<sensor_msgs::Range>("apa_rb", 1, \
    &SurroundMonitor::callback_apa_rb, this);

    sub_upa_fl_ = nh_.subscribe<sensor_msgs::Range>("upa_fl", 1, \
    &SurroundMonitor::callback_upa_fl, this);

    sub_upa_fcl_ = nh_.subscribe<sensor_msgs::Range>("upa_fcl", 1, \
    &SurroundMonitor::callback_upa_fcl, this);

    sub_upa_fcr_ = nh_.subscribe<sensor_msgs::Range>("upa_fcr", 1, \
    &SurroundMonitor::callback_upa_fcr, this);

    sub_upa_fr_ = nh_.subscribe<sensor_msgs::Range>("upa_fr", 1, \
    &SurroundMonitor::callback_upa_fr, this);

    sub_upa_bl_ = nh_.subscribe<sensor_msgs::Range>("upa_bl", 1, \
    &SurroundMonitor::callback_upa_bl, this);

    sub_upa_bcl_ = nh_.subscribe<sensor_msgs::Range>("upa_bcl", 1, \
    &SurroundMonitor::callback_upa_bcl, this);

    sub_upa_bcr_ = nh_.subscribe<sensor_msgs::Range>("upa_bcr", 1, \
    &SurroundMonitor::callback_upa_bcr, this);

    sub_upa_br_ = nh_.subscribe<sensor_msgs::Range>("upa_br", 1, \
    &SurroundMonitor::callback_upa_br, this);

    pub_motor_ = nh_.advertise<std_msgs::String>("cmd_move", 1);
}

// DESTRUCTOR: called when this object is deleted to release memory 
SurroundMonitor::~SurroundMonitor(void)
{
    ROS_INFO("call destructor in surround_monitor");
}

// callbacks from custom callback queue
// callback of sub_car_speed
void SurroundMonitor::callback_car_speed(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("call callback of car_speed: speed=%f", msg->data);
    msg_car_speed_.data = msg->data;

    // brakedistance = 0.02v + v²/(2µg) = 0.02v + v²/15.68, (µ=0.8, g=9.8 m/s²)
    brake_distance = 0.02 * fabs(msg_car_speed_.data) + pow(fabs(msg_car_speed_.data), 2) / 15.68;
}

// callback of sub_apa_lf_
void SurroundMonitor::callback_apa_lf(const sensor_msgs::Range::ConstPtr& msg)
{
    ROS_INFO("call callback of apa_lf: range=%f", msg->range);
    msg_apa_lf_.header.stamp = msg->header.stamp;
    msg_apa_lf_.header.frame_id = msg->header.frame_id;
    msg_apa_lf_.range = msg->range;

    if (msg_apa_lf_.range < brake_distance_s)
    {
        msg_cmd_move_.data = "stop";
        pub_motor_.publish(msg_cmd_move_);
    }
}

// callback of sub_apa_lb_
void SurroundMonitor::callback_apa_lb(const sensor_msgs::Range::ConstPtr& msg)
{
    ROS_INFO("call callback of apa_lb: range=%f", msg->range);
    msg_apa_lb_.header.stamp = msg->header.stamp;
    msg_apa_lb_.header.frame_id = msg->header.frame_id;
    msg_apa_lb_.range = msg->range;

    if (msg_apa_lb_.range < brake_distance_s)
    {
        msg_cmd_move_.data = "stop";
        pub_motor_.publish(msg_cmd_move_);
    }
}

// callback of sub_apa_rf_
void SurroundMonitor::callback_apa_rf(const sensor_msgs::Range::ConstPtr& msg)
{
    ROS_INFO("call callback of apa_rf: range=%f", msg->range);
    msg_apa_rf_.header.stamp = msg->header.stamp;
    msg_apa_rf_.header.frame_id = msg->header.frame_id;
    msg_apa_rf_.range = msg->range;

    if (msg_apa_rf_.range < brake_distance_s)
    {
        msg_cmd_move_.data = "stop";
        pub_motor_.publish(msg_cmd_move_);
    }
}

// callback of sub_apa_rb_
void SurroundMonitor::callback_apa_rb(const sensor_msgs::Range::ConstPtr& msg)
{
    ROS_INFO("call callback of apa_rb: range=%f", msg->range);
    msg_apa_rb_.header.stamp = msg->header.stamp;
    msg_apa_rb_.header.frame_id = msg->header.frame_id;
    msg_apa_rb_.range = msg->range;

    if (msg_apa_rb_.range < brake_distance_s)
    {
        msg_cmd_move_.data = "stop";
        pub_motor_.publish(msg_cmd_move_);
    }
}

// callback of sub_upa_fl_
void SurroundMonitor::callback_upa_fl(const sensor_msgs::Range::ConstPtr& msg)
{
    ROS_INFO("call callback of upa_fl: range=%f", msg->range);
    msg_upa_fl_.header.stamp = msg->header.stamp;
    msg_upa_fl_.header.frame_id = msg->header.frame_id;
    msg_upa_fl_.range = msg->range;

    if (msg_car_speed_.data > 0)
    {
        if (msg_upa_fl_.range < brake_distance)
        {
            msg_cmd_move_.data = "stop";
            pub_motor_.publish(msg_cmd_move_);
        }
    }
}

// callback of sub_upa_fcl_
void SurroundMonitor::callback_upa_fcl(const sensor_msgs::Range::ConstPtr& msg)
{
    ROS_INFO("call callback of upa_fcl: range=%f", msg->range);
    msg_upa_fcl_.header.stamp = msg->header.stamp;
    msg_upa_fcl_.header.frame_id = msg->header.frame_id;
    msg_upa_fcl_.range = msg->range;

    if (msg_car_speed_.data > 0)
    {
        if (msg_upa_fcl_.range < brake_distance)
        {
            msg_cmd_move_.data = "stop";
            pub_motor_.publish(msg_cmd_move_);
        }
    }
}

// callback of sub_upa_fcr_
void SurroundMonitor::callback_upa_fcr(const sensor_msgs::Range::ConstPtr& msg)
{
    ROS_INFO("call callback of upa_fcr: range=%f", msg->range);
    msg_upa_fcr_.header.stamp = msg->header.stamp;
    msg_upa_fcr_.header.frame_id = msg->header.frame_id;
    msg_upa_fcr_.range = msg->range;

    if (msg_car_speed_.data > 0)
    {
        if (msg_upa_fcr_.range < brake_distance)
        {
            msg_cmd_move_.data = "stop";
            pub_motor_.publish(msg_cmd_move_);
        }
    }
}

// callback of sub_upa_fr_
void SurroundMonitor::callback_upa_fr(const sensor_msgs::Range::ConstPtr& msg)
{
    ROS_INFO("call callback of upa_fr: range=%f", msg->range);
    msg_upa_fr_.header.stamp = msg->header.stamp;
    msg_upa_fr_.header.frame_id = msg->header.frame_id;
    msg_upa_fr_.range = msg->range;

    if (msg_car_speed_.data > 0)
    {
        if (msg_upa_fr_.range < brake_distance)
        {
            msg_cmd_move_.data = "stop";
            pub_motor_.publish(msg_cmd_move_);
        }
    }
}


// callback of sub_upa_bl_
void SurroundMonitor::callback_upa_bl(const sensor_msgs::Range::ConstPtr& msg)
{
    ROS_INFO("call callback of upa_bl: range=%f", msg->range);
    msg_upa_bl_.header.stamp = msg->header.stamp;
    msg_upa_bl_.header.frame_id = msg->header.frame_id;
    msg_upa_bl_.range = msg->range;

    if (msg_car_speed_.data < 0)
    {
        if (msg_upa_bl_.range < brake_distance)
        {
            msg_cmd_move_.data = "stop";
            pub_motor_.publish(msg_cmd_move_);
        }
    }
}

// callback of sub_upa_bcl_
void SurroundMonitor::callback_upa_bcl(const sensor_msgs::Range::ConstPtr& msg)
{
    ROS_INFO("call callback of upa_bcl: range=%f", msg->range);
    msg_upa_bcl_.header.stamp = msg->header.stamp;
    msg_upa_bcl_.header.frame_id = msg->header.frame_id;
    msg_upa_bcl_.range = msg->range;

    if (msg_car_speed_.data < 0)
    {
        if (msg_upa_bcl_.range < brake_distance)
        {
            msg_cmd_move_.data = "stop";
            pub_motor_.publish(msg_cmd_move_);
        }
    }
}

// callback of sub_upa_bcr_
void SurroundMonitor::callback_upa_bcr(const sensor_msgs::Range::ConstPtr& msg)
{
    ROS_INFO("call callback of upa_bcr: range=%f", msg->range);
    msg_upa_bcr_.header.stamp = msg->header.stamp;
    msg_upa_bcr_.header.frame_id = msg->header.frame_id;
    msg_upa_bcr_.range = msg->range;

    if (msg_car_speed_.data < 0)
    {
        if (msg_upa_fcr_.range < brake_distance)
        {
            msg_cmd_move_.data = "stop";
            pub_motor_.publish(msg_cmd_move_);
        }
    }
}

// callback of sub_upa_br_
void SurroundMonitor::callback_upa_br(const sensor_msgs::Range::ConstPtr& msg)
{
    ROS_INFO("call callback of upa_br: range=%f", msg->range);
    msg_upa_br_.header.stamp = msg->header.stamp;
    msg_upa_br_.header.frame_id = msg->header.frame_id;
    msg_upa_br_.range = msg->range;

    if (msg_car_speed_.data < 0)
    {
        if (msg_upa_br_.range < brake_distance)
        {
            msg_cmd_move_.data = "stop";
            pub_motor_.publish(msg_cmd_move_);
        }
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "surround_monitor");

    // instantiating an object of class SurroundMonitor
    SurroundMonitor SurroundMonitor_obj;

    // use 13 threads for 13 callbacks
    ros::AsyncSpinner spinner(13);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
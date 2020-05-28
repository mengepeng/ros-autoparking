/******************************************************************
 * Filename: surround_monitor.cpp
 * Version: v1.0
 * Author: Meng Peng
 * Date: 2020-05-15
 * Description: surround monitor with ultrasonic sensors for AEB
 * 
 ******************************************************************/

#include "autopark/autoparking.h"
#include "autopark/surround_monitor.h"

using namespace std;

static float brake_distance = 0;                // real brake distance
static float move_speed_old = 0;                // save move original speed before stoping

static bool stop_trigger_forward = false;       // flag of stop trigger for forward
static bool stop_trigger_backward = false;      // flag of stop trigger for backward
static bool stop_trigger_enable = false;        // flag of stop trigger state

static bool trigger_check = false;              // flag to enable check function 
static bool trigger_spinner = false;            // flag to enable spinners

boost::shared_ptr<ros::AsyncSpinner> sp_spinner;   // create a shared_ptr for AsyncSpinner object


// CONSTRUCTOR: called when this object is created to set up subscribers and publishers
SurroundMonitor::SurroundMonitor()
{
    ROS_INFO("call constructor in surround_monitor");

    sub_car_speed_ = nh_.subscribe<std_msgs::Float32>("car_speed", 1, \
    &SurroundMonitor::callback_car_speed, this);

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

    pub_move_ = nh_.advertise<std_msgs::Float32>("cmd_move", 1);

    pub_forward_ = nh_.advertise<std_msgs::Bool>("forward_enable", 1);

    pub_backward_ = nh_.advertise<std_msgs::Bool>("backward_enable", 1);
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

    // brakedistance = 0.1v + v²/(2µg) = 0.02v + v²/15.68, (µ=0.8, g=9.8 m/s²)
    brake_distance = 0.1 * fabs(msg_car_speed_.data) + pow(fabs(msg_car_speed_.data), 2) / 15.68;

    // check all car speed and sensor ranges
    check_signals();
}

// callback of sub_upa_fl_
void SurroundMonitor::callback_upa_fl(const sensor_msgs::Range::ConstPtr& msg)
{
    ROS_INFO("call callback of upa_fl: range=%f", msg->range);
    msg_upa_fl_.header.stamp = msg->header.stamp;
    msg_upa_fl_.header.frame_id = msg->header.frame_id;
    msg_upa_fl_.range = msg->range;
}

// callback of sub_upa_fcl_
void SurroundMonitor::callback_upa_fcl(const sensor_msgs::Range::ConstPtr& msg)
{
    ROS_INFO("call callback of upa_fcl: range=%f", msg->range);
    msg_upa_fcl_.header.stamp = msg->header.stamp;
    msg_upa_fcl_.header.frame_id = msg->header.frame_id;
    msg_upa_fcl_.range = msg->range;
}

// callback of sub_upa_fcr_
void SurroundMonitor::callback_upa_fcr(const sensor_msgs::Range::ConstPtr& msg)
{
    ROS_INFO("call callback of upa_fcr: range=%f", msg->range);
    msg_upa_fcr_.header.stamp = msg->header.stamp;
    msg_upa_fcr_.header.frame_id = msg->header.frame_id;
    msg_upa_fcr_.range = msg->range;
}

// callback of sub_upa_fr_
void SurroundMonitor::callback_upa_fr(const sensor_msgs::Range::ConstPtr& msg)
{
    ROS_INFO("call callback of upa_fr: range=%f", msg->range);
    msg_upa_fr_.header.stamp = msg->header.stamp;
    msg_upa_fr_.header.frame_id = msg->header.frame_id;
    msg_upa_fr_.range = msg->range;
}

// callback of sub_upa_bl_
void SurroundMonitor::callback_upa_bl(const sensor_msgs::Range::ConstPtr& msg)
{
    ROS_INFO("call callback of upa_bl: range=%f", msg->range);
    msg_upa_bl_.header.stamp = msg->header.stamp;
    msg_upa_bl_.header.frame_id = msg->header.frame_id;
    msg_upa_bl_.range = msg->range;
}

// callback of sub_upa_bcl_
void SurroundMonitor::callback_upa_bcl(const sensor_msgs::Range::ConstPtr& msg)
{
    ROS_INFO("call callback of upa_bcl: range=%f", msg->range);
    msg_upa_bcl_.header.stamp = msg->header.stamp;
    msg_upa_bcl_.header.frame_id = msg->header.frame_id;
    msg_upa_bcl_.range = msg->range;
}

// callback of sub_upa_bcr_
void SurroundMonitor::callback_upa_bcr(const sensor_msgs::Range::ConstPtr& msg)
{
    ROS_INFO("call callback of upa_bcr: range=%f", msg->range);
    msg_upa_bcr_.header.stamp = msg->header.stamp;
    msg_upa_bcr_.header.frame_id = msg->header.frame_id;
    msg_upa_bcr_.range = msg->range;
}

// callback of sub_upa_br_
void SurroundMonitor::callback_upa_br(const sensor_msgs::Range::ConstPtr& msg)
{
    ROS_INFO("call callback of upa_br: range=%f", msg->range);
    msg_upa_br_.header.stamp = msg->header.stamp;
    msg_upa_br_.header.frame_id = msg->header.frame_id;
    msg_upa_br_.range = msg->range;
}


// function for stopping the car when it is too close to object
void SurroundMonitor::check_signals()
{
    // if car moves forward
    if (msg_car_speed_.data > 0)
    {
        // the distance between car and object at front is smaller than brake distance
        if (msg_upa_fl_.range < brake_distance \
        || msg_upa_fcl_.range < brake_distance \
        || msg_upa_fcr_.range < brake_distance \
        || msg_upa_fr_.range < brake_distance)
        {
            if (stop_trigger_forward == false)
            {
                // save current speed
                move_speed_old = msg_car_speed_.data;
            }
            // set stop_trigger_forward
            stop_trigger_forward = true;
            // set stop_trigger_enable
            stop_trigger_enable = true;

            // stop
            msg_cmd_move_.data = 0;
            pub_move_.publish(msg_cmd_move_);
        }
    }

    // if car moves backward
    else if (msg_car_speed_.data < 0)
    {
        // the distance between car and object at back is smaller than brake distance
        if (msg_upa_bl_.range < brake_distance \
        || msg_upa_bcl_.range < brake_distance \
        || msg_upa_bcr_.range < brake_distance \
        || msg_upa_br_.range < brake_distance)
        {
            if (stop_trigger_backward == false)
            {
                // save current speed
                move_speed_old = msg_car_speed_.data;
            }
            // set stop_trigger_backward
            stop_trigger_backward = true;
            // set stop_trigger_enable
            stop_trigger_enable = true;

            // stop
            msg_cmd_move_.data = 0;
            pub_move_.publish(msg_cmd_move_);
        }
    }

    // if car stops
    else
    {
        // the distance between car and object at front is larger than default brake distance
        if (msg_upa_fl_.range > brake_distance_default \
        && msg_upa_fcl_.range > brake_distance_default \
        && msg_upa_fcr_.range > brake_distance_default \
        && msg_upa_fr_.range > brake_distance_default)
        {
            // reset stop_trigger_forward
            stop_trigger_forward = false;

            // enable moving forward
            msg_cmd_forward_.data = true;
            pub_forward_.publish(msg_cmd_forward_);

            if (stop_trigger_enable)
            {
                // cancel stop, move forward again with original speed
                msg_cmd_move_.data = move_speed_old;
                pub_move_.publish(msg_cmd_move_);

                // reset move_speed_old
                move_speed_old = 0;
                // reset stop_trigger_enable
                stop_trigger_enable = false;
            }
        }
        else
        {
            // disable moving forward
            msg_cmd_forward_.data = false;
            pub_forward_.publish(msg_cmd_forward_);
        }

        // the distance between car and object at back is larger than default brake distance
        if (msg_upa_bl_.range > brake_distance_default \
        && msg_upa_bcl_.range > brake_distance_default \
        && msg_upa_bcr_.range > brake_distance_default \
        && msg_upa_br_.range > brake_distance_default)
        {
            // reset stop_trigger_backward
            stop_trigger_backward = false;

            // enable moving backward
            msg_cmd_backward_.data = true;
            pub_forward_.publish(msg_cmd_backward_);

            if (stop_trigger_enable)
            {
                // cancel stop, move backward again with original speed
                msg_cmd_move_.data = move_speed_old;
                pub_move_.publish(msg_cmd_move_);

                // reset move_speed_old
                move_speed_old = 0;
                // reset stop_trigger_enable
                stop_trigger_enable = false;
            }
        }
        else
        {
            // disable moving backward
            msg_cmd_backward_.data = false;
            pub_backward_.publish(msg_cmd_backward_);
        }
    }

    // if move direction is changed
    if (msg_car_speed_.data * move_speed_old < 0)
    {
        // reset stop triggers
        stop_trigger_forward = false;
        stop_trigger_backward = false;
        stop_trigger_enable = false;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "surround_monitor");

    // instantiating an object of class ChooseParkingSpace
    SurroundMonitor SurroundMonitor_obj;

    // use 9 threads for 9 callbacks
    ros::AsyncSpinner spinner(9);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
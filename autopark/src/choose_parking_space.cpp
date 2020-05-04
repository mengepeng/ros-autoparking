/******************************************************************
 * Filename: choose_parking_space.cpp
 * Version: v1.0
 * Author: Meng Peng
 * Date: 2020-05-03
 * Description: choose parking space according to message from the 
 * topics of parking_space
 * 
 ******************************************************************/

#include "autopark/autoparking.h"
#include "autopark/choose_parking_space.h"

using namespace std;

//  x   x   x   x     x   x   x   x
// high four bits for left side, low four bits for left side
// 7 bit: not used
// 6 bit: parking space on the left side
// 5 bit: parking type is parallel parking
// 4 bit: parking type is perpendicular parking
// 3 bit: not used
// 2 bit: parking space on the right side
// 1 bit: parking type is parallel parking
// 0 bit: parking type is perpendicular parking
#define SPACE_LEFT_PARALLEL         0x96    // 0 1 1 0  0 0 0 0
#define SPACE_LEFT_PERPENDICULAR    0x80    // 0 1 0 1  0 0 0 0
#define SPACE_RIGHT_PARALLEL        0x06    // 0 0 0 0  0 1 1 0
#define SPACE_RIGHT_PERPENDICULAR   0x05    // 0 0 0 0  0 1 0 1


// CONSTRUCTOR: called when this object is created to set up subscribers and publishers
ChooseParkingSpace::ChooseParkingSpace()
{
    ROS_INFO("call constructor in choose_parking_space");

    sub_car_speed_ = nh_.subscribe<std_msgs::Float32>("car_speed", 1, \
    &ChooseParkingSpace::callback_car_speed, this);

    sub_parking_space_lf_ = nh_.subscribe<std_msgs::Header>("parking_space_lf", 1, \
    &ChooseParkingSpace::callback_parking_space_lf, this);

    sub_parking_space_lb_ = nh_.subscribe<std_msgs::Header>("parking_space_lb", 1, \
    &ChooseParkingSpace::callback_parking_space_lb, this);

    sub_parking_space_rf_ = nh_.subscribe<std_msgs::Header>("parking_space_rf", 1, \
    &ChooseParkingSpace::callback_parking_space_rf, this);

    sub_parking_space_rb_ = nh_.subscribe<std_msgs::Header>("parking_space_rb", 1, \
    &ChooseParkingSpace::callback_parking_space_rb, this);

    pub_parking_start_ = nh_.advertise<std_msgs::Header>("parking_space", 1);

    // initialize:
    msg_parking_space_.seq = 0;            // 0 0 0 0  0 0 0 0
}

// DESTRUCTOR: called when this object is deleted to release memory 
ChooseParkingSpace::~ChooseParkingSpace(void)
{
    ROS_INFO("call destructor in choose_parking_space");
}

// callbacks from custom callback queue
// callback of sub_car_speed_
void ChooseParkingSpace::callback_car_speed(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("call callback of car_speed: speed=%f", msg->data);
    msg_car_speed_.data = msg->data;
}

// callback of sub_parking_space_lf_
void ChooseParkingSpace::callback_parking_space_lf(const std_msgs::Header::ConstPtr& msg)
{
    ROS_INFO("call callback_parking_space_lf: %d", msg->seq);

    msg_parking_space_lf_.seq = msg->seq;
    msg_parking_space_lf_.stamp = msg->stamp;
    que_parking_space_lf_.push(msg_parking_space_lf_);
}

// callback of sub_parking_space_lb_
void ChooseParkingSpace::callback_parking_space_lb(const std_msgs::Header::ConstPtr& msg)
{
    ROS_INFO("call callback_parking_space_lb: %d", msg->seq);

    msg_parking_space_lb_.seq = msg->seq;
    msg_parking_space_lb_.stamp = msg->stamp;
    if (!que_parking_space_lf_.empty())
    {
        // time duration messages from parking_space_lf and parking_space_lb
        double time_diff, distance_fb;
        time_diff = (msg_parking_space_lb_.stamp - que_parking_space_lf_.front().stamp).toSec();
        // car moved distance
        distance_fb = msg_car_speed_.data * time_diff;

        // check parking space
        if (abs(distance_fb - distance_apa) < apa_width)
        {
            msg_parking_space_.seq = que_parking_space_lf_.front().seq & msg_parking_space_lb_.seq;

            // choose parking space
            choose_parking_space();
        }
        // delete first message in que_parking_space_lf_
        que_parking_space_lf_.pop();
    }
}

// callback of sub_parking_space_rf_
void ChooseParkingSpace::callback_parking_space_rf(const std_msgs::Header::ConstPtr& msg)
{
    ROS_INFO("call callback_parking_space_rf: %d", msg->seq);

    msg_parking_space_rf_.seq = msg->seq;
    msg_parking_space_rf_.stamp = msg->stamp;
    que_parking_space_rf_.push(msg_parking_space_rf_);
}

// callback of sub_parking_space_rb_
void ChooseParkingSpace::callback_parking_space_rb(const std_msgs::Header::ConstPtr& msg)
{
    ROS_INFO("call callback_parking_space_rb: %d", msg->seq);

    msg_parking_space_rb_.seq = msg->seq;
    msg_parking_space_rb_.stamp = msg->stamp;
    if (!que_parking_space_rf_.empty())
    {
        // time duration messages from parking_space_lf and parking_space_lb
        double time_diff, distance_fb;
        time_diff = (msg_parking_space_rb_.stamp - que_parking_space_rf_.front().stamp).toSec();
        // car moved distance
        distance_fb = msg_car_speed_.data * time_diff;

        // check parking space
        if (abs(distance_fb - distance_apa) < apa_width)
        {
            msg_parking_space_.seq = que_parking_space_rf_.front().seq & msg_parking_space_rb_.seq;

            // choose parking
            choose_parking_space();
        }
        // delete first message in que_parking_space_rf_
        que_parking_space_rf_.pop();
    }
}

// choose a parking space: parking space on the right side has priority
void ChooseParkingSpace::choose_parking_space()
{
    ROS_INFO("call choose function");
    // parallel parking space on the right side
    if (msg_parking_space_.seq & SPACE_RIGHT_PARALLEL == SPACE_RIGHT_PARALLEL)
    {
        msg_parking_space_.seq = SPACE_RIGHT_PARALLEL;
        msg_parking_space_.stamp = ros::Time::now();
        pub_parking_start_.publish(msg_parking_space_);
    }
    // perpendicular parking space on the right side 
    else if (msg_parking_space_.seq & SPACE_RIGHT_PERPENDICULAR == SPACE_RIGHT_PERPENDICULAR)
    {
        msg_parking_space_.seq = SPACE_RIGHT_PERPENDICULAR;
        msg_parking_space_.stamp = ros::Time::now();
        pub_parking_start_.publish(msg_parking_space_);
    }
    // parallel parking space on the left side
    else if (msg_parking_space_.seq & SPACE_LEFT_PARALLEL == SPACE_LEFT_PARALLEL)
    {
        msg_parking_space_.seq = SPACE_LEFT_PARALLEL;
        msg_parking_space_.stamp = ros::Time::now();
        pub_parking_start_.publish(msg_parking_space_);
    }
    // perpendicular parking space on the left side
    else if (msg_parking_space_.seq & SPACE_LEFT_PERPENDICULAR == SPACE_LEFT_PERPENDICULAR)
    {
        msg_parking_space_.seq = SPACE_LEFT_PERPENDICULAR;
        msg_parking_space_.stamp = ros::Time::now();
        pub_parking_start_.publish(msg_parking_space_);
    }
    else
    {
        // do nothing
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "choose_parking_space");

    // instantiating an object of class ChooseParkingSpace
    ChooseParkingSpace ChooseParkingSpace_obj;

    // use 5 threads for 5 callbacks
    ros::AsyncSpinner spinner(5);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
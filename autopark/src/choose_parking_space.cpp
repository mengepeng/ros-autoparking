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

static bool parking_enable = false;     // flag of parking enable
static bool search_done = false;        // flag of searching parking space done
static bool trigger_spinner = false;    // flag of spinners enable

boost::shared_ptr<ros::AsyncSpinner> sp_spinner;   // create a shared_ptr for AsyncSpinner object

// define struct of Times for stop situation during check and choose parking space
struct Times
{
    ros::Time begin;
    ros::Time end;
    double duration;
    bool trigger;
} time_left, time_right;    // define two objects for left and right side


// callback from global callback queue
// callback of sub_parking_enable
void callback_parking_enable(const std_msgs::Bool::ConstPtr& msg)
{
    ROS_INFO("call callback of parking_enable: %d", msg->data);
    parking_enable = msg->data;
}


// CONSTRUCTOR: called when this object is created to set up subscribers and publishers
ChooseParkingSpace::ChooseParkingSpace(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
    ROS_INFO("call constructor in choose_parking_space");

    sub_parking_space_lf_ = nh_.subscribe<std_msgs::Header>("parking_space_lf", 1, \
    &ChooseParkingSpace::callback_parking_space_lf, this);

    sub_parking_space_lb_ = nh_.subscribe<std_msgs::Header>("parking_space_lb", 1, \
    &ChooseParkingSpace::callback_parking_space_lb, this);

    sub_parking_space_rf_ = nh_.subscribe<std_msgs::Header>("parking_space_rf", 1, \
    &ChooseParkingSpace::callback_parking_space_rf, this);

    sub_parking_space_rb_ = nh_.subscribe<std_msgs::Header>("parking_space_rb", 1, \
    &ChooseParkingSpace::callback_parking_space_rb, this);

    pub_parking_space_ = nh_.advertise<std_msgs::Header>("parking_space", 1);
    pub_search_done_ = nh_.advertise<std_msgs::Bool>("search_done", 1);

    // initialize:
    msg_parking_space_.seq = 0;             // 0 0 0 0  0 0 0 0
    msg_search_done_.data = true;           // to stop searching parking space
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

    if (!que_parking_space_lf_.empty())
    {
        if (msg_car_speed_.data == 0 && !time_left.trigger)
        {
            time_left.begin = ros::Time::now();
            time_left.trigger = true;
        }
        if (msg_car_speed_.data != 0 && time_left.trigger)
        {
            time_left.end = ros::Time::now();
            time_left.trigger = false;

            time_left.duration += (time_left.end - time_left.begin).toSec();
        }
    }
    if (!que_parking_space_rf_.empty())
    {
        if (msg_car_speed_.data == 0 && !time_right.trigger)
        {
            time_right.begin = ros::Time::now();
            time_right.trigger = true;
        }
        if (msg_car_speed_.data != 0 && time_right.trigger)
        {
            time_right.end = ros::Time::now();
            time_right.trigger = false;

            time_right.duration += (time_right.end - time_right.begin).toSec();
        }
    }
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
        double time_diff = (msg_parking_space_lb_.stamp - que_parking_space_lf_.front().stamp).toSec();
        // car moved distance
        double distance_fb = msg_car_speed_.data * (time_diff - time_left.duration);
        // reset time duration for stop
        time_left.duration = 0;

        // check parking space
        if (fabs(distance_fb - distance_apa) < apa_width)
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
        double time_diff = (msg_parking_space_rb_.stamp - que_parking_space_rf_.front().stamp).toSec();
        // car moved distance
        double distance_fb = msg_car_speed_.data * (time_diff - time_right.duration);
        // reset time duration for stop
        time_right.duration = 0;

        // check parking space
        if (fabs(distance_fb - distance_apa) < apa_width)
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
    ROS_INFO("call choose parking space function");

    // parallel parking space on the right side
    if (msg_parking_space_.seq & SPACE_RIGHT_PARALLEL == SPACE_RIGHT_PARALLEL)
    {
        msg_parking_space_.seq = SPACE_RIGHT_PARALLEL;
        msg_parking_space_.stamp = ros::Time::now();
        pub_parking_space_.publish(msg_parking_space_);
        pub_search_done_.publish(msg_search_done_);

        // save parking space info via global variable parking_space
        parking_space = SPACE_RIGHT_PARALLEL;

        search_done = true;     // to stop choosing parking space
    }
    // perpendicular parking space on the right side 
    else if (msg_parking_space_.seq & SPACE_RIGHT_PERPENDICULAR == SPACE_RIGHT_PERPENDICULAR)
    {
        msg_parking_space_.seq = SPACE_RIGHT_PERPENDICULAR;
        msg_parking_space_.stamp = ros::Time::now();
        pub_parking_space_.publish(msg_parking_space_);
        pub_search_done_.publish(msg_search_done_);

        // save parking space info via global variable parking_space
        parking_space = SPACE_RIGHT_PERPENDICULAR;

        search_done = true;
    }
    // parallel parking space on the left side
    else if (msg_parking_space_.seq & SPACE_LEFT_PARALLEL == SPACE_LEFT_PARALLEL)
    {
        msg_parking_space_.seq = SPACE_LEFT_PARALLEL;
        msg_parking_space_.stamp = ros::Time::now();
        pub_parking_space_.publish(msg_parking_space_);
        pub_search_done_.publish(msg_search_done_);

        // save parking space info via global variable parking_space
        parking_space = SPACE_LEFT_PARALLEL;

        search_done = true;
    }
    // perpendicular parking space on the left side
    else if (msg_parking_space_.seq & SPACE_LEFT_PERPENDICULAR == SPACE_LEFT_PERPENDICULAR)
    {
        msg_parking_space_.seq = SPACE_LEFT_PERPENDICULAR;
        msg_parking_space_.stamp = ros::Time::now();
        pub_parking_space_.publish(msg_parking_space_);
        pub_search_done_.publish(msg_search_done_);

        // save parking space info via global variable parking_space
        parking_space = SPACE_LEFT_PERPENDICULAR;

        search_done = true;
    }
    else
    {
        // do nothing
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "choose_parking_space");

    // create a node handle, use global callback queue
    ros::NodeHandle nh;
    //create a node handle for class, use custom callback queue
    ros::NodeHandle nh_c;
    // create custom callback queue
    ros::CallbackQueue callback_queue;
    // set custom callback queue
    nh_c.setCallbackQueue(&callback_queue);

    ros::Subscriber sub_parking_enable = nh.subscribe<std_msgs::Bool>("parking_enable", 1, \
    callback_parking_enable);

    // instantiating an object of class ChooseParkingSpace
    ChooseParkingSpace ChooseParkingSpace_obj(&nh_c);  // pass nh_c to class constructor

    // create AsyncSpinner, run it on all available cores to process custom callback queue
    sp_spinner.reset(new ros::AsyncSpinner(0, &callback_queue));

    // initialize Times: 
    time_left.duration = 0;
    time_left.trigger = false;

    time_right.duration = 0;
    time_right.trigger = false;

    // set loop rate
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ROS_INFO_STREAM_ONCE("node choose_parking_space is running");
        if (parking_enable)
        {
            if (!trigger_spinner)
            {
                ROS_INFO("choose parking space enabled");

                // clear old callbacks in custom callback queue
                callback_queue.clear();
                // start spinners for custom callback queue
                sp_spinner->start();
                ROS_INFO("spinners start");

                trigger_spinner = true;
            }
            else
            {
                if (search_done)
                {
                    ROS_INFO("choose parking space finished");

                    // stop spinners for custom callback queue
                    sp_spinner->stop();
                    ROS_INFO("spinners stop");

                    // reset
                    parking_enable = false;
                    search_done = false;
                    trigger_spinner = false;
                }
            }
        }
        else
        {
            if (trigger_spinner)
            {
                ROS_INFO("choose parking space disabled");

                // stop spinners for custom callback queue
                sp_spinner->stop();
                ROS_INFO("spinners stop");

                // reset
                parking_enable = false;
                search_done = false;
                trigger_spinner = false;
            }
        }

        // process message in global callback queue: "parking_enable"
        ros::spinOnce();

        loop_rate.sleep();
    }

    // release AsyncSpinner object
    sp_spinner.reset();

    // wait for ROS threads to terminate
    ros::waitForShutdown();

    return 0;
}
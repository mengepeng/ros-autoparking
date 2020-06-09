/******************************************************************
 * Filename: parking_out.cpp
 * Version: v1.0
 * Author: Meng Peng
 * Date: 2020-06-01
 * Description: main program for getting out of parking space inkl. 
 * perpendicular or parallel parking space on both sides
 * 
 ******************************************************************/

#include "autopark/autoparking.h"
#include "autopark/parking_out.h"

using namespace std;

static bool parking_out_enable = false;         // flag of parking out enable
static bool parking_out_finished = false;       // flag of parking finished
static bool trigger_spinner = false;            // flag of spinners enable

static double moved_distance = 0;
static ros::Time time_begin, time_end;

boost::shared_ptr<ros::AsyncSpinner> sp_spinner;   // create a shared_ptr for AsyncSpinner object


// callback from global callback queue
// callback of sub_parking_out_enable
void callback_parking_out_enable(const std_msgs::Bool::ConstPtr& msg)
{
    ROS_INFO("call callback of parking_out_enable: %d", msg->data);
    parking_out_enable = msg->data;
}


// CONSTRUCTOR: called when this object is created to set up subscribers and publishers
ParkingOut::ParkingOut(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
    ROS_INFO("call constructor in parking_out");

    sub_car_speed_ = nh_.subscribe<std_msgs::Float32>("car_speed", 1, \
    &ParkingOut::callback_car_speed, this);

    sub_apa_lf_ = nh_.subscribe<sensor_msgs::Range>("apa_lf", 1, \
    &ParkingOut::callback_apa_lf, this);

    sub_apa_lb_ = nh_.subscribe<sensor_msgs::Range>("apa_lb", 1, \
    &ParkingOut::callback_apa_lb, this);

    sub_apa_rf_ = nh_.subscribe<sensor_msgs::Range>("apa_rf", 1, \
    &ParkingOut::callback_apa_rf, this);

    sub_apa_rb_ = nh_.subscribe<sensor_msgs::Range>("apa_rb", 1, \
    &ParkingOut::callback_apa_rb, this);

    sub_upa_fl_ = nh_.subscribe<sensor_msgs::Range>("upa_fl", 1, \
    &ParkingOut::callback_upa_fl, this);

    sub_upa_fcl_ = nh_.subscribe<sensor_msgs::Range>("upa_fcl", 1, \
    &ParkingOut::callback_upa_fcl, this);

    sub_upa_fcr_ = nh_.subscribe<sensor_msgs::Range>("upa_fcr", 1, \
    &ParkingOut::callback_upa_fcr, this);

    sub_upa_fr_ = nh_.subscribe<sensor_msgs::Range>("upa_fr", 1, \
    &ParkingOut::callback_upa_fr, this);

    sub_upa_bl_ = nh_.subscribe<sensor_msgs::Range>("upa_bl", 1, \
    &ParkingOut::callback_upa_bl, this);

    sub_upa_bcl_ = nh_.subscribe<sensor_msgs::Range>("upa_bcl", 1, \
    &ParkingOut::callback_upa_bcl, this);

    sub_upa_bcr_ = nh_.subscribe<sensor_msgs::Range>("upa_bcr", 1, \
    &ParkingOut::callback_upa_bcr, this);

    sub_upa_br_ = nh_.subscribe<sensor_msgs::Range>("upa_br", 1, \
    &ParkingOut::callback_upa_br, this);

    pub_move_ = nh_.advertise<std_msgs::Float32>("cmd_move", 1);

    pub_turn_ = nh_.advertise<std_msgs::Char>("cmd_turn", 1);
}

// DESTRUCTOR: called when this object is deleted to release memory 
ParkingOut::~ParkingOut(void)
{
    ROS_INFO("call destructor in parking_out");
}

// callbacks from custom callback queue
// callback of sub_car_speed_
void ParkingOut::callback_car_speed(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("call callback of car_speed: speed=%f", msg->data);
    msg_car_speed_.data = msg->data;

    time_end = ros::Time::now();

    // calculate the moved distance
    moved_distance += msg_car_speed_.data * (time_end - time_begin).toSec();

    time_begin = ros::Time::now();
}

// callback of sub_apa_lf_
void ParkingOut::callback_apa_lf(const sensor_msgs::Range::ConstPtr& msg)
{
    ROS_INFO("call callback of apa_lf: range=%f", msg->range);
    msg_apa_lf_.header.stamp = msg->header.stamp;
    msg_apa_lf_.header.frame_id = msg->header.frame_id;
    msg_apa_lf_.range = msg->range;
}

// callback of sub_apa_lb_
void ParkingOut::callback_apa_lb(const sensor_msgs::Range::ConstPtr& msg)
{
    ROS_INFO("call callback of apa_lb: range=%f", msg->range);
    msg_apa_lb_.header.stamp = msg->header.stamp;
    msg_apa_lb_.header.frame_id = msg->header.frame_id;
    msg_apa_lb_.range = msg->range;
}

// callback of sub_apa_rf_
void ParkingOut::callback_apa_rf(const sensor_msgs::Range::ConstPtr& msg)
{
    ROS_INFO("call callback of apa_rf: range=%f", msg->range);
    msg_apa_rf_.header.stamp = msg->header.stamp;
    msg_apa_rf_.header.frame_id = msg->header.frame_id;
    msg_apa_rf_.range = msg->range;
}

// callback of sub_apa_rb_
void ParkingOut::callback_apa_rb(const sensor_msgs::Range::ConstPtr& msg)
{
    ROS_INFO("call callback of apa_rb: range=%f", msg->range);
    msg_apa_rb_.header.stamp = msg->header.stamp;
    msg_apa_rb_.header.frame_id = msg->header.frame_id;
    msg_apa_rb_.range = msg->range;
}

// callback of sub_upa_fl_
void ParkingOut::callback_upa_fl(const sensor_msgs::Range::ConstPtr& msg)
{
    ROS_INFO("call callback of upa_fl: range=%f", msg->range);
    msg_upa_fl_.header.stamp = msg->header.stamp;
    msg_upa_fl_.header.frame_id = msg->header.frame_id;
    msg_upa_fl_.range = msg->range;
}

// callback of sub_upa_fcl_
void ParkingOut::callback_upa_fcl(const sensor_msgs::Range::ConstPtr& msg)
{
    ROS_INFO("call callback of upa_fcl: range=%f", msg->range);
    msg_upa_fcl_.header.stamp = msg->header.stamp;
    msg_upa_fcl_.header.frame_id = msg->header.frame_id;
    msg_upa_fcl_.range = msg->range;
}

// callback of sub_upa_fcr_
void ParkingOut::callback_upa_fcr(const sensor_msgs::Range::ConstPtr& msg)
{
    ROS_INFO("call callback of upa_fcr: range=%f", msg->range);
    msg_upa_fcr_.header.stamp = msg->header.stamp;
    msg_upa_fcr_.header.frame_id = msg->header.frame_id;
    msg_upa_fcr_.range = msg->range;
}

// callback of sub_upa_fr_
void ParkingOut::callback_upa_fr(const sensor_msgs::Range::ConstPtr& msg)
{
    ROS_INFO("call callback of upa_fr: range=%f", msg->range);
    msg_upa_fr_.header.stamp = msg->header.stamp;
    msg_upa_fr_.header.frame_id = msg->header.frame_id;
    msg_upa_fr_.range = msg->range;
}

// callback of sub_upa_bl_
void ParkingOut::callback_upa_bl(const sensor_msgs::Range::ConstPtr& msg)
{
    ROS_INFO("call callback of upa_bl: range=%f", msg->range);
    msg_upa_bl_.header.stamp = msg->header.stamp;
    msg_upa_bl_.header.frame_id = msg->header.frame_id;
    msg_upa_bl_.range = msg->range;
}

// callback of sub_upa_bcl_
void ParkingOut::callback_upa_bcl(const sensor_msgs::Range::ConstPtr& msg)
{
    ROS_INFO("call callback of upa_bcl: range=%f", msg->range);
    msg_upa_bcl_.header.stamp = msg->header.stamp;
    msg_upa_bcl_.header.frame_id = msg->header.frame_id;
    msg_upa_bcl_.range = msg->range;
}

// callback of sub_upa_bcr_
void ParkingOut::callback_upa_bcr(const sensor_msgs::Range::ConstPtr& msg)
{
    ROS_INFO("call callback of upa_bcr: range=%f", msg->range);
    msg_upa_bcr_.header.stamp = msg->header.stamp;
    msg_upa_bcr_.header.frame_id = msg->header.frame_id;
    msg_upa_bcr_.range = msg->range;
}

// callback of sub_upa_br_
void ParkingOut::callback_upa_br(const sensor_msgs::Range::ConstPtr& msg)
{
    ROS_INFO("call callback of upa_br: range=%f", msg->range);
    msg_upa_br_.header.stamp = msg->header.stamp;
    msg_upa_br_.header.frame_id = msg->header.frame_id;
    msg_upa_br_.range = msg->range;
}


// ************************************************************************
// function of getting out of perpendicular parking space on the left side
// ************************************************************************
void ParkingOut::parking_out_left_perpendicular()
{
    // turn straight
    msg_cmd_turn_.data = 'D';
    pub_turn_.publish(msg_cmd_turn_);
    // move forward with speed_parking_forward
    msg_cmd_move_.data = speed_parking_forward;
    pub_move_.publish(msg_cmd_move_);

    ros::Rate looprate(20);
    // move forward until car is out of parking space
    while (ros::ok())
    {
        // publish move and turn commands
        pub_move_.publish(msg_cmd_move_);
        pub_turn_.publish(msg_cmd_turn_);

        // a half of car is out of parking space
        if (moved_distance > distance_perpendicular_out)
        {
            // turn full left
            msg_cmd_turn_.data = 'L';
            pub_turn_.publish(msg_cmd_turn_);

            break;
        }

        looprate.sleep();
    }

    /***** PLAN A *****/
    // now parking out is finished, driver can get into car easily,
    // but car is not parallel to the street
    /*
    // stop
    msg_cmd_move_.data = 0;
    pub_move_.publish(msg_cmd_move_);

    parking_out_finished = true;
    */

    /***** PLAN B *****/
    // keep moving forward until car is complete out of parking space
    while (ros::ok())
    {
        // publish move and turn commands
        pub_move_.publish(msg_cmd_move_);
        pub_turn_.publish(msg_cmd_turn_);

        // right of car rear is out of parking space
        if (msg_apa_rb_.range > distance_search)
        {
            break;
        }

        looprate.sleep();
    }
    while (ros::ok())
    {
        // publish move and turn commands
        pub_move_.publish(msg_cmd_move_);
        pub_turn_.publish(msg_cmd_turn_);

        // car head is close to object (car) on the left side of street or 
        // car rear is complete out of parking space
        if (msg_apa_lf_.range < distance_search || \
        (msg_upa_bcl_.range > perpendicular_width && msg_upa_bcr_.range > perpendicular_width))
        {
            // stop
            msg_cmd_move_.data = 0;
            pub_move_.publish(msg_cmd_move_);
            // turn straight
            msg_cmd_turn_.data = 'D';
            pub_turn_.publish(msg_cmd_turn_);

            parking_out_finished = true;

            break;
        }
        else
        {
            // car is too close to parkwall (car) on the left side
            if (msg_apa_lb_.range < brake_distance_default)
            {
                // turn straight
                msg_cmd_turn_.data = 'D';
            }
            else
            {
                // keep turnning full left
                msg_cmd_turn_.data = 'L';
            }
        }

        looprate.sleep();
    }
}


// ************************************************************************
// function of getting out of perpendicular parking space on the right side
// ************************************************************************
void ParkingOut::parking_out_right_perpendicular()
{
    // turn straight
    msg_cmd_turn_.data = 'D';
    pub_turn_.publish(msg_cmd_turn_);
    // move forward with speed_parking_forward
    msg_cmd_move_.data = speed_parking_forward;
    pub_move_.publish(msg_cmd_move_);

    ros::Rate looprate(20);
    // move forward until a half of car is out of parking space
    while (ros::ok())
    {
        // publish move and turn commands
        pub_move_.publish(msg_cmd_move_);
        pub_turn_.publish(msg_cmd_turn_);

        // a half of car is out of parking space
        if (moved_distance > distance_perpendicular_out)
        {
            // turn full right
            msg_cmd_turn_.data = 'R';
            pub_turn_.publish(msg_cmd_turn_);

            break;
        }

        looprate.sleep();
    }

    /***** PLAN A *****/
    // now parking out is finished, driver can get into car easily,
    // but car is not parallel to the street
    /*
    // stop
    msg_cmd_move_.data = 0;
    pub_move_.publish(msg_cmd_move_);

    parking_out_finished = true;
    */

    /***** PLAN B *****/
    // keep moving forward until car is complete out of parking space
    while (ros::ok())
    {
        // publish move and turn commands
        pub_move_.publish(msg_cmd_move_);
        pub_turn_.publish(msg_cmd_turn_);

        // left of car rear is out of parking space
        if (msg_apa_lb_.range > distance_search)
        {
            break;
        }

        looprate.sleep();
    }
    while (ros::ok())
    {
        // publish move and turn commands
        pub_move_.publish(msg_cmd_move_);
        pub_turn_.publish(msg_cmd_turn_);

        // car head is close to object (car) on the right side of street or 
        // car rear is complete out of parking space
        if (msg_apa_rf_.range < distance_search || \
        (msg_upa_bcl_.range > perpendicular_width && msg_upa_bcr_.range > perpendicular_width))
        {
            // stop
            msg_cmd_move_.data = 0;
            pub_move_.publish(msg_cmd_move_);
            // turn straight
            msg_cmd_turn_.data = 'D';
            pub_turn_.publish(msg_cmd_turn_);

            parking_out_finished = true;

            break;
        }
        else
        {
            // car is too close to parkwall (car) on the right side
            if (msg_apa_rb_.range < brake_distance_default)
            {
                // turn straight
                msg_cmd_turn_.data = 'D';
            }
            else
            {
                // keep turnning full right
                msg_cmd_turn_.data = 'R';
            }
        }

        looprate.sleep();
    }
}


// ************************************************************************
// function of getting out of parallel parking space on the left side
// ************************************************************************
void ParkingOut::parking_out_left_parallel()
{
    // turn straight
    msg_cmd_turn_.data = 'D';
    pub_turn_.publish(msg_cmd_turn_);
    // move backward with speed_parking_backward
    msg_cmd_move_.data = speed_parking_backward;
    pub_move_.publish(msg_cmd_move_);

    ros::Rate looprate(20);
    // move until car is ready to turn right and move forward to get out of parking space
    while (ros::ok())
    {
        // publish move and turn commands
        pub_move_.publish(msg_cmd_move_);
        pub_turn_.publish(msg_cmd_turn_);

        // distance between car and parkwall (car) at front is larger than safe distance
        // or car rear is too close to parkwall (car) at back
        if (msg_upa_fcr_.range > distance_parallel_out || \
        (msg_upa_bl_.range < brake_distance_default || \
        msg_upa_bcl_.range < brake_distance_default || \
        msg_upa_bcr_.range < brake_distance_default || \
        msg_upa_br_.range < brake_distance_default))
        {
            // stop
            msg_cmd_move_.data = 0;
            pub_move_.publish(msg_cmd_move_);
            // turn full right
            msg_cmd_turn_.data = 'R';
            pub_turn_.publish(msg_cmd_turn_);
            // move forward
            msg_cmd_move_.data = speed_parking_forward;
            pub_move_.publish(msg_cmd_move_);

            break;
        }

        looprate.sleep();
    }

    // move forward until car head is out of parking space
    while (ros::ok())
    {
        // publish move and turn commands
        pub_move_.publish(msg_cmd_move_);
        pub_turn_.publish(msg_cmd_turn_);

        // car head is out of parking space
        if (msg_apa_lf_.range > distance_search && \
            msg_upa_fl_.range > distance_search)
        {
            // turn straight
            msg_cmd_turn_.data = 'D';
            pub_turn_.publish(msg_cmd_turn_);
            // move forward
            msg_cmd_move_.data = speed_parking_forward;
            pub_move_.publish(msg_cmd_move_);

            break;
        }
        else
        {
            // car head is too close to parkwall (car)
            if (msg_upa_fl_.range < brake_distance_default)
            {
                // stop
                msg_cmd_move_.data = 0;
                pub_move_.publish(msg_cmd_move_);
                // turn full left
                msg_cmd_turn_.data = 'L';
                pub_turn_.publish(msg_cmd_turn_);
                // move backward
                msg_cmd_move_.data = speed_parking_backward;
                pub_move_.publish(msg_cmd_move_);

                // car rear is too close to parkwall (car)
                if (msg_upa_bl_.range < brake_distance_default)
                {
                    // stop
                    msg_cmd_move_.data = 0;
                    pub_move_.publish(msg_cmd_move_);
                    // turn full right
                    msg_cmd_turn_.data = 'R';
                    pub_turn_.publish(msg_cmd_turn_);
                    // move forward
                    msg_cmd_move_.data = speed_parking_forward;
                    pub_move_.publish(msg_cmd_move_);
                }
            }
        }

        looprate.sleep();
    }

    // move forward until half of car rear is out of parking space
    while (ros::ok())
    {
        // publish move and turn commands
        pub_move_.publish(msg_cmd_move_);
        pub_turn_.publish(msg_cmd_turn_);

        // half of car rear is out of parking space
        if (msg_upa_bcl_.range > parallel_length/2)
        {
            // turn full left
            msg_cmd_turn_.data = 'L';
            pub_turn_.publish(msg_cmd_turn_);
            // move forward
            msg_cmd_move_.data = speed_parking_forward;
            pub_move_.publish(msg_cmd_move_);

            break;
        }

        looprate.sleep();
    }

    /***** PLAN A *****/
    // now parking out is finished, driver can get into car easily,
    // but car is not parallel to the street
    /*
    // stop
    msg_cmd_move_.data = 0;
    pub_move_.publish(msg_cmd_move_);

    parking_out_finished = true;
    */

    /***** PLAN B *****/
    // keep moving forward until car is complete out of parking space
    while (ros::ok())
    {
        // publish move and turn commands
        pub_move_.publish(msg_cmd_move_);
        pub_turn_.publish(msg_cmd_turn_);

        // car is complete out of parking space
        if (msg_upa_bl_.range > parallel_length)
        {
            // stop
            msg_cmd_move_.data = 0;
            pub_move_.publish(msg_cmd_move_);
            // turn straight
            msg_cmd_turn_.data = 'D';
            pub_turn_.publish(msg_cmd_turn_);

            parking_out_finished = true;

            break;
        }
        else
        {
            // car head is too close to object (car) at side of street
            if (msg_upa_fl_.range < parking_distance_min)
            {
                // turn straight
                msg_cmd_turn_.data = 'D';
            }
        }
    }

    looprate.sleep();
}



// ************************************************************************
// function of getting out of parallel parking space on the right side
// ************************************************************************
void ParkingOut::parking_out_right_parallel()
{
    // turn straight
    msg_cmd_turn_.data = 'D';
    pub_turn_.publish(msg_cmd_turn_);
    // move backward with speed_parking_backward
    msg_cmd_move_.data = speed_parking_backward;
    pub_move_.publish(msg_cmd_move_);

    ros::Rate looprate(20);
    // move until car is ready to turn right and move forward to get out of parking space
    while (ros::ok())
    {
        // publish move and turn commands
        pub_move_.publish(msg_cmd_move_);
        pub_turn_.publish(msg_cmd_turn_);

        // distance between car and parkwall (car) at front is larger than safe distance
        // or car rear is too close to parkwall (car) at back
        if (msg_upa_fcl_.range > distance_parallel_out || \
        (msg_upa_bl_.range < brake_distance_default || \
        msg_upa_bcl_.range < brake_distance_default || \
        msg_upa_bcr_.range < brake_distance_default || \
        msg_upa_br_.range < brake_distance_default))
        {
            // stop
            msg_cmd_move_.data = 0;
            pub_move_.publish(msg_cmd_move_);
            // turn full left
            msg_cmd_turn_.data = 'L';
            pub_turn_.publish(msg_cmd_turn_);
            // move forward
            msg_cmd_move_.data = speed_parking_forward;
            pub_move_.publish(msg_cmd_move_);

            break;
        }

        looprate.sleep();
    }

    // move forward until car head is out of parking space
    while (ros::ok())
    {
        // publish move and turn commands
        pub_move_.publish(msg_cmd_move_);
        pub_turn_.publish(msg_cmd_turn_);

        // car head is out of parking space
        if (msg_apa_rf_.range > distance_search && \
            msg_upa_fr_.range > distance_search)
        {
            // turn straight
            msg_cmd_turn_.data = 'D';
            pub_turn_.publish(msg_cmd_turn_);
            // move forward
            msg_cmd_move_.data = speed_parking_forward;
            pub_move_.publish(msg_cmd_move_);

            break;
        }
        else
        {
            // car head is too close to parkwall (car)
            if (msg_upa_fr_.range < brake_distance_default)
            {
                // stop
                msg_cmd_move_.data = 0;
                pub_move_.publish(msg_cmd_move_);
                // turn full right
                msg_cmd_turn_.data = 'R';
                pub_turn_.publish(msg_cmd_turn_);
                // move backward
                msg_cmd_move_.data = speed_parking_backward;
                pub_move_.publish(msg_cmd_move_);

                // car rear is too close to parkwall (car)
                if (msg_upa_br_.range < brake_distance_default)
                {
                    // stop
                    msg_cmd_move_.data = 0;
                    pub_move_.publish(msg_cmd_move_);
                    // turn full left
                    msg_cmd_turn_.data = 'L';
                    pub_turn_.publish(msg_cmd_turn_);
                    // move forward
                    msg_cmd_move_.data = speed_parking_forward;
                    pub_move_.publish(msg_cmd_move_);
                }
            }
        }

        looprate.sleep();
    }

    // move forward until half of car rear is out of parking space
    while (ros::ok())
    {
        // publish move and turn commands
        pub_move_.publish(msg_cmd_move_);
        pub_turn_.publish(msg_cmd_turn_);

        // half of car rear is out of parking space
        if (msg_upa_bcr_.range > parallel_length/2)
        {
            // turn full right
            msg_cmd_turn_.data = 'R';
            pub_turn_.publish(msg_cmd_turn_);
            // move forward
            msg_cmd_move_.data = speed_parking_forward;
            pub_move_.publish(msg_cmd_move_);

            break;
        }

        looprate.sleep();
    }

    /***** PLAN A *****/
    // now parking out is finished, driver can get into car easily,
    // but car is not parallel to the street
    /*
    // stop
    msg_cmd_move_.data = 0;
    pub_move_.publish(msg_cmd_move_);

    parking_out_finished = true;
    */

    /***** PLAN B *****/
    // keep moving forward until car is complete out of parking space
    while (ros::ok())
    {
        // publish move and turn commands
        pub_move_.publish(msg_cmd_move_);
        pub_turn_.publish(msg_cmd_turn_);

        // car is complete out of parking space
        if (msg_upa_br_.range > parallel_length)
        {
            // stop
            msg_cmd_move_.data = 0;
            pub_move_.publish(msg_cmd_move_);
            // turn straight
            msg_cmd_turn_.data = 'D';
            pub_turn_.publish(msg_cmd_turn_);

            parking_out_finished = true;

            break;
        }
        else
        {
            // car head is too close to object (car) at side of street
            if (msg_apa_rf_.range < parking_distance_min)
            {
                // turn straight
                msg_cmd_turn_.data = 'D';
            }
        }
    }

    looprate.sleep();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "parking_out");

    // create a node handle, use global callback queue
    ros::NodeHandle nh;
    //create a node handle for class, use custom callback queue
    ros::NodeHandle nh_c;
    // create custom callback queue
    ros::CallbackQueue callback_queue;
    // set custom callback queue
    nh_c.setCallbackQueue(&callback_queue);

    // create and initialize subscribers
    ros::Subscriber sub_parking_out_enable = nh.subscribe<std_msgs::Bool>("parking_out_enable", 1, \
    callback_parking_out_enable);

    // instantiating class object
    ParkingOut ParkingOut_obj(&nh_c);  // pass nh_c to class constructor

    // create AsyncSpinner, run it on all available cores to process custom callback queue
    sp_spinner.reset(new ros::AsyncSpinner(0, &callback_queue));

    // set loop rate
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ROS_INFO_STREAM_ONCE("node parking_out is running");
        if (parking_out_enable)
        {
            if (!trigger_spinner)
            {
                ROS_INFO("parking out enabled");

                time_begin = ros::Time::now();  // initialize time_begin

                // clear old callbacks in custom callback queue
                callback_queue.clear();
                // start spinners for custom callback queue
                sp_spinner->start();
                ROS_INFO("spinners start");

                trigger_spinner = true;
            }
            else
            {
                // check parking_space (saved in autoparking.h) and get out of the parking space
                switch (parking_space)
                {
                case SPACE_LEFT_PERPENDICULAR:
                    ROS_INFO("parking out of left perpendicular parking space");
                    ParkingOut_obj.parking_out_left_perpendicular();
                    break;

                case SPACE_LEFT_PARALLEL:
                    ROS_INFO("parking out of left parallel parking space");
                    ParkingOut_obj.parking_out_left_parallel();
                    break;

                case SPACE_RIGHT_PERPENDICULAR:
                    ROS_INFO("parking out of right perpendicular parking space");
                    ParkingOut_obj.parking_out_right_perpendicular();
                    break;

                case SPACE_RIGHT_PARALLEL:
                    ROS_INFO("parking out of right parallel parking space");
                    ParkingOut_obj.parking_out_right_parallel();
                    break;

                default:
                    ROS_INFO("parking space is unknown");
                    if (trigger_spinner)
                    {
                        // stop spinners for custom callback queue
                        sp_spinner->stop();
                        ROS_INFO("spinners stop");

                        // reset
                        parking_out_enable = false;
                        trigger_spinner = false;
                        parking_out_finished = false;
                        moved_distance = 0;
                    }
                }

                if (parking_out_finished)
                {
                    ROS_INFO("parking out finished");

                    // stop spinners for custom callback queue
                    sp_spinner->stop();
                    ROS_INFO("spinners stop");

                    // reset
                    parking_out_enable = false;
                    trigger_spinner = false;
                    parking_out_finished = false;
                    moved_distance = 0;
                }
            }
        }

        // process message in global callback queue
        ros::spinOnce();

        loop_rate.sleep();
    }

    // release AsyncSpinner object
    sp_spinner.reset();

    // wait for ROS threads to terminate
    ros::waitForShutdown();

    return 0;
}
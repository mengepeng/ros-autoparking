/******************************************************************
 * Filename: search_parking_space_lb.cpp
 * Version: v1.0
 * Author: Meng Peng
 * Date: 2020-04-28
 * Description: search parking space on the left side with apa_lb
 * 
 ******************************************************************/

#include "autopark/autoparking.h"
#include "autopark/search_parking_space_lb.h"

using namespace std;

static bool parking_enable = false;     // flag to enable parking
static bool trigger_check = false;      // flag to enable check function 
static bool trigger_spinner = false;    // flag to enable spinners

static float distance_min;              // minimum distance between car and object

boost::shared_ptr<ros::AsyncSpinner> sp_spinner;   // create a shared_ptr for AsyncSpinner object

// define struct of Times for stop situation during check parking space
struct Times
{
    ros::Time begin;
    ros::Time end;
    double duration;
    bool trigger;
} time_2, time_5;    // define two objects for case 2 and case 5


// callback from global callback queue
void callback_parking_enable(const std_msgs::Bool::ConstPtr& msg)
{
    ROS_INFO("call callback of parking_enable: %d", msg->data);
    parking_enable = msg->data;
}


// CONSTRUCTOR: called when this object is created to set up subscribers and publishers
SearchParkingSpaceLB::SearchParkingSpaceLB(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
    ROS_INFO("call constructor in search_parking_space_lb");

    sub_apa_lb_ = nh_.subscribe<sensor_msgs::Range>("apa_lb", 1, \
    &SearchParkingSpaceLB::callback_apa_lb, this);

    sub_car_speed_ = nh_.subscribe<std_msgs::Float32>("car_speed", 1, \
    &SearchParkingSpaceLB::callback_car_speed, this);

    pub_parking_space_ = nh_.advertise<std_msgs::Header>("parking_space_lb", 1);
}

// DESTRUCTOR: called when this object is deleted to release memory 
SearchParkingSpaceLB::~SearchParkingSpaceLB(void)
{
    ROS_INFO("call destructor in search_parking_space_lb");
}

// callbacks from custom callback queue
// callback of sub_car_speed_
void SearchParkingSpaceLB::callback_car_speed(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("call callback of car_speed: speed=%f", msg->data);
    msg_car_speed_.data = msg->data;

    if (vec_turnpoint_.size() == 1)
    {
        if (msg_car_speed_.data == 0 && !time_2.trigger)
        {
            time_2.begin = ros::Time::now();
            time_2.trigger = true;
        }
        if (msg_car_speed_.data != 0 && time_2.trigger)
        {
            time_2.end = ros::Time::now();
            time_2.trigger = false;

            time_2.duration += (time_2.end - time_2.begin).toSec();
        }
    }
    if (vec_turnpoint_.size() == 3)
    {
        if (msg_car_speed_.data == 0 && !time_5.trigger)
        {
            time_5.begin = ros::Time::now();
            time_5.trigger = true;
        }
        if (msg_car_speed_.data != 0 && time_5.trigger)
        {
            time_5.end = ros::Time::now();
            time_5.trigger = false;

            time_5.duration += (time_5.end - time_5.begin).toSec();
        }
    }
}

// callback of sub_apa_lb_
void SearchParkingSpaceLB::callback_apa_lb(const sensor_msgs::Range::ConstPtr& msg)
{
    ROS_INFO("call callback of apa_lb: range=%f", msg->range);
    msg_apa_lb_.header.stamp = msg->header.stamp;
    msg_apa_lb_.header.frame_id = msg->header.frame_id;
    msg_apa_lb_.range = msg->range;

    trigger_check = true;  // to start check function
}

// check function to find parking space
void SearchParkingSpaceLB::check_parking_space()
{
    ROS_INFO("call check function in search_parking_space_lb");
    if (que_apa_lb_.empty())
    {
        //detected object in the range of distance_search
        if (msg_apa_lb_.range < distance_search)
        {
            // add first valid range message to que_apa_lb_
            que_apa_lb_.push(msg_apa_lb_);
        }
    }
    else
    {
        // add new message to que_apa_lb_
        que_apa_lb_.push(msg_apa_lb_);

        // number of turn points in vec_turnpoint_
        static uint16_t size_vec_turnpoint = vec_turnpoint_.size();
        switch (size_vec_turnpoint)
        {
        case 0:     // no turn point in vec_turnpoint_
            // range increase: close to parking space (parking gap)
            if ((que_apa_lb_.back().range - que_apa_lb_.front().range) > range_diff)
            {
                // do nothing
            }
            // range decrease: away from parking space (parking gap)
            else if ((que_apa_lb_.front().range - que_apa_lb_.back().range) > range_diff)
            {
                vec_turnpoint_.push_back(que_apa_lb_.back());  // first turn point
                distance_min = que_apa_lb_.back().range;
            }
            // range is stable
            else
            {
                vec_turnpoint_.push_back(que_apa_lb_.front());   // first turn point
                distance_min = que_apa_lb_.front().range;
            }
            break;

        case 1:     // one turn point in vec_turnpoint_
            // range increase: close to parking space (parking gap)
            if ((que_apa_lb_.back().range - que_apa_lb_.front().range) > range_diff)
            {
                vec_turnpoint_.push_back(que_apa_lb_.front());    // second turn point
            }
            // range decrease: away from parking space (parking gap)
            else if ((que_apa_lb_.front().range - que_apa_lb_.back().range) > range_diff)
            {
                vec_turnpoint_.pop_back();      // delete the added first turn point
                vec_turnpoint_.push_back(que_apa_lb_.back());  // add the new first turn point
                distance_min = que_apa_lb_.back().range;
            }
            else
            {
                if (que_apa_lb_.back().range < distance_min)
                {
                    distance_min = que_apa_lb_.back().range;  // get new distance_min
                }
            }
            break;

        case 2:     // two turn points in vec_turnpoint_
            double obj_diff, obj_length;
            // time duration between the first and second turn point
            obj_diff = (vec_turnpoint_[1].header.stamp - vec_turnpoint_[0].header.stamp).toSec();
            // parallel length of object
            obj_length = msg_car_speed_.data * (obj_diff - time_2.duration);
            // reset time duration for stop
            time_2.duration = 0;
            
            if (obj_length < perpendicular_width && obj_length > car_width)
            {
                // perpendicular parking: 0 0 0 1  0 0 0 0
                setbit(msg_parking_space_.seq, 4);
                clrbit(msg_parking_space_.seq, 5);
            }
            else if (obj_length < parallel_width && obj_length > car_length)
            {
                // parallel parking: 0 0 1 0  0 0 0 0
                setbit(msg_parking_space_.seq, 5);
                clrbit(msg_parking_space_.seq, 4);
            }
            else
            {
                // do nothing
            }

            // range increase: close to parking space (parking gap)
            if ((que_apa_lb_.back().range - que_apa_lb_.front().range) > range_diff)
            {
                // do nothing
            }
            // range decrease: away from parking space (parking gap)
            else if ((que_apa_lb_.front().range - que_apa_lb_.back().range) > range_diff)
            {
                vec_turnpoint_.push_back(que_apa_lb_.front());    // third key point
                vec_turnpoint_.push_back(que_apa_lb_.front());    // fourth key point
            }
            // range is stable
            else
            {
                vec_turnpoint_.push_back(que_apa_lb_.front());   // third turn point
            }
            break;

        case 3:     // three turn points in vec_turnpoint_
            // range decrease: away from parking space (parking gap)
            if ((que_apa_lb_.front().range - que_apa_lb_.back().range) > range_diff)
            {
                vec_turnpoint_.push_back(que_apa_lb_.front());     // fourth turn point
            }
            /* the only way to find the fourth turn point */
            else
            {
                // do nothing
            }
            break;

        case 4:     // four turn points in vec_turnpoint_
            // range increase: close to parking space (parking gap)
            if ((que_apa_lb_.back().range - que_apa_lb_.front().range) > range_diff)
            {
                // do nothing
            }
            // range decrease: away from parking space (parking gap)
            else if ((que_apa_lb_.front().range - que_apa_lb_.back().range) > range_diff)
            {
                // do nothing
            }
            // range is stable
            else
            {
                vec_turnpoint_.push_back(que_apa_lb_.front());     // fifth turn point
            }
            /* the only way to find the fifth turn point */
            break;

        case 5:     // five turn points in vec_turnpoint_
            double space_diff, space_width, space_length;
            // time duration between the third and fourth turn point
            space_diff = (vec_turnpoint_[4].header.stamp - vec_turnpoint_[3].header.stamp).toSec();
            // width of space
            space_width = msg_car_speed_.data * (space_diff - time_5.duration);
            // reset time duration for stop
            time_5.duration = 0;

            // length of space
            space_length = min(vec_turnpoint_[3].range, vec_turnpoint_[4].range) - \
            min(distance_min, vec_turnpoint_[5].range);

            if ((space_width > perpendicular_width && space_length > perpendicular_length) || \
            (space_width > parallel_width && space_length > parallel_length))
            {
                // parking space on the left side
                setbit(msg_parking_space_.seq, 6);     // 0 1 x x  0 0 0 0
                // 0 1 0 1  0 0 0 0: perpendicular parking on the left side
                // 0 1 1 0  0 0 0 0: parallel parking space on the left side
                // else: not a valid parking space

                // in case of specific parking space with 3 walls
                // perpendicular parking space
                if (space_width < parallel_width)
                {
                    // perpendicular parking: 0 0 0 1  0 0 0 0
                    setbit(msg_parking_space_.seq, 4);
                    clrbit(msg_parking_space_.seq, 5);
                }
                // parallel parking space
                if (space_length < perpendicular_length)
                {
                    // parallel parking: 0 0 1 0  0 0 0 0
                    setbit(msg_parking_space_.seq, 5);
                    clrbit(msg_parking_space_.seq, 4);
                }

                // set time stamp
                msg_parking_space_.stamp = ros::Time::now();
                // publish parking space
                pub_parking_space_.publish(msg_parking_space_);
                // reset parking space state
                clrbit(msg_parking_space_.seq, 6);     // 0 0 x x  0 0 0 0
            }

            // delete fist message in que_apa_lb_
            que_apa_lb_.pop();
            // clear all messages in vec_turnpoint_
            vec_turnpoint_.clear();
            break;

        default:
            break;
        }

        if(!(que_apa_lb_.empty()))
            que_apa_lb_.pop();     // delete first message in to keep queue_size less than 2
    }

    trigger_check = false;   // to stop check function and wait for next trigger
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "search_parking_space_lb");

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

    // instantiating an object of class SearchParkingSpaceLB
    SearchParkingSpaceLB SearchParkingSpaceLB_lb(&nh_c);  // pass nh_c to class constructor

    // create AsyncSpinner, run it on all available cores to process custom callback queue
    sp_spinner.reset(new ros::AsyncSpinner(0, &callback_queue));

    // initialize Times: 
    time_2.duration = 0;
    time_2.trigger = false;

    time_5.duration = 0;
    time_5.trigger = false;

    // set loop rate very large or just do not use rate to ensure process as fast as possible
    ros::Rate loop_rate(1000);
    while (ros::ok())
    {
        if (parking_enable)
        {
            ROS_INFO_STREAM_ONCE("search parking space enabled");
            if (!trigger_spinner)
            {
                // clear old callbacks in custom callback queue
                callback_queue.clear();
                // start spinners for custom callback queue
                sp_spinner->start();
                ROS_INFO("Spinners enabled in search_parking_space_lb");

                trigger_spinner = true;
            }

            if (trigger_check)
            {
                // check new apa message to find parking space
                SearchParkingSpaceLB_lb.check_parking_space();
            }
        }
        else
        {
            ROS_INFO_STREAM_ONCE("search parking space disabled");
            if (trigger_spinner)
            {
                // stop spinners for custom callback queue
                sp_spinner->stop();
                ROS_INFO("Spinners disabled in search_parking_space_lb");

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
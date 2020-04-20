/******************************************************************
 * Filename: ultrasensors.cpp
 * Version: v1.0
 * Author: Meng Peng
 * Copyright(C): chuhang.tech
 * Date: 2020-04-15
 * Description: publish range messages of ultrasensors 
 * 
 ******************************************************************/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Range.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ultrasensors");
    ros::NodeHandle nh;

    ros::Publisher pub_apa_lf = nh.advertise<sensor_msgs::Range>("apa_lf", 1);
    ros::Publisher pub_apa_lb = nh.advertise<sensor_msgs::Range>("apa_lb", 1);
    ros::Publisher pub_apa_rf = nh.advertise<sensor_msgs::Range>("apa_rf", 1);
    ros::Publisher pub_apa_rb = nh.advertise<sensor_msgs::Range>("apa_rb", 1);

    ros::Publisher pub_upa_fl = nh.advertise<sensor_msgs::Range>("upa_fl", 1);
    ros::Publisher pub_upa_fcl = nh.advertise<sensor_msgs::Range>("upa_fcl", 1);
    ros::Publisher pub_upa_fcr = nh.advertise<sensor_msgs::Range>("upa_fcr", 1);
    ros::Publisher pub_upa_fr = nh.advertise<sensor_msgs::Range>("upa_fr", 1);

    ros::Publisher pub_upa_bl = nh.advertise<sensor_msgs::Range>("upa_bl", 1);
    ros::Publisher pub_upa_bcl = nh.advertise<sensor_msgs::Range>("upa_bcl", 1);
    ros::Publisher pub_upa_bcr = nh.advertise<sensor_msgs::Range>("upa_bcr", 1);
    ros::Publisher pub_upa_br = nh.advertise<sensor_msgs::Range>("upa_br", 1);

    // define range messages of ultrasensors
    sensor_msgs::Range range_apa_lf;
    sensor_msgs::Range range_apa_lb;
    sensor_msgs::Range range_apa_rf;
    sensor_msgs::Range range_apa_rb;

    sensor_msgs::Range range_upa_fl;
    sensor_msgs::Range range_upa_fcl;
    sensor_msgs::Range range_upa_fcr;
    sensor_msgs::Range range_upa_fr;

    sensor_msgs::Range range_upa_bl;
    sensor_msgs::Range range_upa_bcl;
    sensor_msgs::Range range_upa_bcr;
    sensor_msgs::Range range_upa_br;

    // set range values, it depends on the ultrasensors
    range_apa_lf.range = 1; // not real value, just an example
    range_apa_lb.range = 2;
    range_apa_rf.range = 3;
    range_apa_rb.range = 4;

    range_upa_fl.range = 5;
    range_upa_fcl.range = 6;
    range_upa_fcr.range = 7;
    range_upa_fr.range = 8;

    range_upa_bl.range = 9;
    range_upa_bcl.range = 10;
    range_upa_bcr.range = 11;
    range_upa_br.range = 12;

    // set loop iteration frequence
    ros::Rate loop_rate(50);
    while (nh.ok())
    {
        // output the published messages
        ROS_INFO("apa_lf: range = %f", range_apa_lf.range);
        ROS_INFO("apa_lb: range = %f", range_apa_lb.range);
        ROS_INFO("apa_rf: range = %f", range_apa_rf.range);
        ROS_INFO("apa_rb: range = %f", range_apa_rb.range);

        ROS_INFO("upa_fl: range = %f", range_upa_fl.range);
        ROS_INFO("upa_fcl: range = %f", range_upa_fcl.range);
        ROS_INFO("upa_fcr: range = %f", range_upa_fcr.range);
        ROS_INFO("upa_fr: range = %f", range_upa_fr.range);

        ROS_INFO("upa_bl: range = %f", range_upa_bl.range);
        ROS_INFO("upa_bcl: range = %f", range_upa_bcl.range);
        ROS_INFO("upa_bcr: range = %f", range_upa_bcr.range);
        ROS_INFO("upa_br: range = %f", range_upa_br.range);

        // publish the sensor range
        pub_apa_lf.publish(range_apa_lf);
        pub_apa_lb.publish(range_apa_lb);
        pub_apa_rf.publish(range_apa_rf);
        pub_apa_rb.publish(range_apa_rb);

        pub_upa_fl.publish(range_upa_fl);
        pub_upa_fcl.publish(range_upa_fcl);
        pub_upa_fcr.publish(range_upa_fcr);
        pub_upa_fr.publish(range_upa_fr);

        pub_upa_bl.publish(range_upa_bl);
        pub_upa_bcl.publish(range_upa_bcl);
        pub_upa_bcr.publish(range_upa_bcr);
        pub_upa_br.publish(range_upa_br);


        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
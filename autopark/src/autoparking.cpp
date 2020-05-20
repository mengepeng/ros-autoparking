/******************************************************************
 * Filename: autoparking.cpp
 * Version: v1.0
 * Author: Meng Peng
 * Date: 2020-04-28
 * Description: define global variables and funtions for this project
 * 
 ******************************************************************/

#include "autopark/autoparking.h"

const float range_diff = 0.2;                   // [m] range difference to distinguish turn point 
const float distance_search = 2;                // [m] maximum distance between car and parking space
const float parallel_width = 6;                 // [m] minimum width of parallel parking space
const float parallel_length = 2.5;              // [m] minimum length of parallel parking space
const float perpendicular_width = 2.5;          // [m] minimum width of perpendicular parking space
const float perpendicular_length = 5;           // [m] minimum length of perpendicular parking space
const float car_width = 1.8;                    // [m] minimum width of car
const float car_length = 4.8;                   // [m] minimum length of car
const float distance_apa = 4;                   // [m] distance between two apas at front and back of car
const float apa_width = 0.5;                    // [m] lateral range of apa

const float brake_distance_default = 0.3;       // [m] default brake distance
const float parking_distance = 0.4;             // [m] minimum distance between car and object when park in
const float move_distance_perpendicular = 1.5;  //[m] move distance before perpendicular parking in

const float speed_search_parking = 5;           // [m/s] car speed when search parking space
const float speed_parking_forward = 2;          // [m/s] car speed when move forward for parking
const float speed_parking_backward = -2;        // [m/s] car speed when move backward for parking

const float angle_straight = 0;                 // [°] angle for straight move
const float angle_left = 35;                    // [°] maximum angle for turn left
const float angle_right = -35;                  // [°] maximum angle for turn right
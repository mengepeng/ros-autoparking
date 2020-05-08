/******************************************************************
 * Filename: autoparking.h
 * Version: v1.0
 * Author: Meng Peng
 * Date: 2020-04-28
 * Description: declare global variables and funtions for this project
 * 
 ******************************************************************/

#ifndef AUTOPARKING_H_
#define AUTOPARKING_H_

#define setbit(x, y) x|=(1<<y)
#define clrbit(x, y) x&=~(1<<y)
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
#define SPACE_PARALLEL              0x34    // 0 0 1 0  0 0 1 0
#define SPACE_PERPENDICULAR         0x17    // 0 0 0 1  0 0 0 1
#define SPACE_LEFT_PARALLEL         0x96    // 0 1 1 0  0 0 0 0
#define SPACE_LEFT_PERPENDICULAR    0x80    // 0 1 0 1  0 0 0 0
#define SPACE_RIGHT_PARALLEL        0x06    // 0 0 0 0  0 1 1 0
#define SPACE_RIGHT_PERPENDICULAR   0x05    // 0 0 0 0  0 1 0 1


extern const float range_diff;              // range difference to distinguish turn point 
extern const float distance_search;         // maximum distance between car and parking space
extern const float parallel_width;          // minimum width of parallel parking space
extern const float parallel_length;         // minimum length of parallel parking space
extern const float perpendicular_width;     // minimum width of perpendicular parking space
extern const float perpendicular_length;    // minimum length of perpendicular parking space
extern const float car_width;               // minimum width of car
extern const float car_length;              // minimum length of car
extern const float distance_apa;            // distance between two apas at front and back
extern const float apa_width;               // lateral range of apa

#endif
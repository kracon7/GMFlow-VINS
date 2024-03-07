#pragma once
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>

extern int ROW;
extern int COL;
extern int FOCAL_LENGTH;
extern int NUM_OF_CAM;

extern std::string IMAGE_TOPIC_0;
extern std::string IMAGE_TOPIC_1;
extern std::string IMU_TOPIC;
extern std::string FISHEYE_MASK;
extern std::vector<std::string> CAM_INTRINSICS;
extern int MAX_CNT;
extern int MIN_DIST;
extern int WINDOW_SIZE;
extern int FREQ;
extern double F_THRESHOLD;
extern int SHOW_TRACK;
extern int SHOW_STEREO;
extern int EQUALIZE;
extern int FISHEYE;
extern int FLOW_BACK;
extern bool PUB_THIS_FRAME;

void readParameters(ros::NodeHandle &n);

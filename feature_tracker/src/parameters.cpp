#include "parameters.h"

std::string IMAGE_TOPIC_0;
std::string IMAGE_TOPIC_1;
std::string IMU_TOPIC;
std::vector<std::string> CAM_INTRINSICS;
std::string FISHEYE_MASK;
int MAX_CNT;
int MIN_DIST;
int WINDOW_SIZE;
int FREQ;
double F_THRESHOLD;
int SHOW_TRACK;
int SHOW_STEREO;
int EQUALIZE;
int ROW;
int COL;
int FOCAL_LENGTH;
int FISHEYE;
int FLOW_BACK;
bool PUB_THIS_FRAME;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readParameters(ros::NodeHandle &n)
{
    std::string config_file;
    config_file = readParam<std::string>(n, "config_file");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }
    std::string GVINS_FOLDER_PATH = readParam<std::string>(n, "gvins_folder");

    fsSettings["image_topic_0"] >> IMAGE_TOPIC_0;
    fsSettings["image_topic_1"] >> IMAGE_TOPIC_1;
    fsSettings["imu_topic"] >> IMU_TOPIC;
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    FREQ = fsSettings["freq"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    SHOW_STEREO = fsSettings["show_stereo"];
    EQUALIZE = fsSettings["equalize"];
    FISHEYE = fsSettings["fisheye"];
    if (FISHEYE == 1)
        FISHEYE_MASK = GVINS_FOLDER_PATH + "config/fisheye_mask.jpg";
    FLOW_BACK = fsSettings["flow_back"];

    // Load camera instrinsic parameters
    int pn = config_file.find_last_of('/');
    std::string configPath = config_file.substr(0, pn);
    std::string cam0Calib, cam1Calib;
    fsSettings["cam0_calib"] >> cam0Calib;
    fsSettings["cam1_calib"] >> cam1Calib;
    std::string cam0Path = configPath + "/" + cam0Calib;
    std::string cam1Path = configPath + "/" + cam1Calib;
    CAM_INTRINSICS.push_back(cam0Path);
    CAM_INTRINSICS.push_back(cam1Path);

    WINDOW_SIZE = 20;
    FOCAL_LENGTH = 460;
    PUB_THIS_FRAME = false;

    if (FREQ == 0)
        FREQ = 100;

    fsSettings.release();
}

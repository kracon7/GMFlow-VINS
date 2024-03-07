#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>

#include "feature_tracker.h"

#define SHOW_UNDISTORTION 0

vector<uchar> r_status;
vector<float> r_err;
queue<sensor_msgs::ImageConstPtr> img_buf;

ros::Publisher pub_img,pub_match;
ros::Publisher pub_restart;

FeatureTracker feature_tracker;
double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double last_image_time = 0;
bool init_pub = 0;


cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
    return img;
}


void img_callback
(
    const sensor_msgs::ImageConstPtr &img_0_msg, 
    const sensor_msgs::ImageConstPtr &img_1_msg
)
{
    TicToc t_r;
    if(first_image_flag)
    {
        first_image_flag = false;
        first_image_time = img_0_msg->header.stamp.toSec();
        last_image_time = img_0_msg->header.stamp.toSec();
        return;
    }
    // detect unstable camera stream
    if (img_0_msg->header.stamp.toSec() - last_image_time > 1.0 || img_0_msg->header.stamp.toSec() < last_image_time)
    {
        ROS_WARN("image discontinue! reset the feature tracker!");
        first_image_flag = true; 
        last_image_time = 0;
        pub_count = 1;
        std_msgs::Bool restart_flag;
        restart_flag.data = true;
        pub_restart.publish(restart_flag);
        return;
    }
    last_image_time = img_0_msg->header.stamp.toSec();
    // frequency control
    if (round(1.0 * pub_count / (img_0_msg->header.stamp.toSec() - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (img_0_msg->header.stamp.toSec() - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = img_0_msg->header.stamp.toSec();
            pub_count = 0;
        }
    }
    else
        PUB_THIS_FRAME = false;

    cv::Mat image0, image1;
    image0 = getImageFromMsg(img_0_msg);
    image1 = getImageFromMsg(img_1_msg);

    // ROS_INFO("** FTT ** ready to run feature tracker");

    double t = img_0_msg->header.stamp.toSec();
    gvins_msgs::StereoFeatureTrackPtr tracker_data = feature_tracker.trackImage(t, image0, image1);

    // ROS_INFO("** FTT ** finished running feature tracker");

   if (PUB_THIS_FRAME)
   {
        pub_count++;

        tracker_data->header = img_0_msg->header;
        tracker_data->track_0.header = img_0_msg->header;
        tracker_data->track_1.header = img_0_msg->header;
        tracker_data->header.frame_id = "world";

        // ROS_DEBUG("publish %f, at %f", feature_points->header.stamp.toSec(), ros::Time::now().toSec());
        // skip the first image; since no optical speed on frist image
        if (!init_pub)
        {
            init_pub = 1;
        }
        else
            pub_img.publish(tracker_data);

        if (SHOW_STEREO)
        {   
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_0_msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat temp_img, show_img;
            cv::vconcat(image0, image1, temp_img);
            cv::cvtColor(temp_img, show_img, cv::COLOR_GRAY2RGB);
            cv_ptr->image = show_img;

            for (size_t i = 0; i < feature_tracker.cur_pts.size(); i++)
            {
                double len = std::min(1.0, 1.0 * feature_tracker.track_cnt[i] / WINDOW_SIZE);
                cv::circle(show_img, feature_tracker.cur_pts[i], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
                //draw speed line
                /*
                Vector2d tmp_cur_un_pts (trackerData[i].cur_un_pts[j].x, trackerData[i].cur_un_pts[j].y);
                Vector2d tmp_pts_velocity (trackerData[i].pts_velocity[j].x, trackerData[i].pts_velocity[j].y);
                Vector3d tmp_prev_un_pts;
                tmp_prev_un_pts.head(2) = tmp_cur_un_pts - 0.10 * tmp_pts_velocity;
                tmp_prev_un_pts.z() = 1;
                Vector2d tmp_prev_uv;
                trackerData[i].m_camera->spaceToPlane(tmp_prev_un_pts, tmp_prev_uv);
                cv::line(tmp_img, trackerData[i].cur_pts[j], cv::Point2f(tmp_prev_uv.x(), tmp_prev_uv.y()), cv::Scalar(255 , 0, 0), 1 , 8, 0);
                */
                //char name[10];
                //sprintf(name, "%d", trackerData[i].ids[j]);
                //cv::putText(tmp_img, name, trackerData[i].cur_pts[j], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
            }
            
            map<int, cv::Point2f>::iterator mapIt;
            for (size_t i = 0; i < feature_tracker.cur_right_pts.size(); i++) 
            {
                double len = std::min(1.0, 1.0 * feature_tracker.track_cnt[i] / WINDOW_SIZE);
                cv::Point2f pt;
                pt.x = feature_tracker.cur_right_pts[i].x;
                pt.y = feature_tracker.cur_right_pts[i].y + feature_tracker.row;
                cv::circle(show_img, pt, 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);

                int id = feature_tracker.ids_right[i];
                mapIt = feature_tracker.prevLeftPtsMap.find(id);
                ROS_ASSERT(mapIt != feature_tracker.prevLeftPtsMap.end());
                if(mapIt != feature_tracker.prevLeftPtsMap.end())
                {
                    cv::line(show_img, mapIt->second, pt, cv::Scalar(0, 255, 0), 1);
                }
            }

            pub_match.publish(cv_ptr->toImageMsg());
        }
    }
    ROS_INFO("whole feature tracker processing costs: %f", t_r.toc());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature_tracker");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(n);

    cout << CAM_INTRINSICS[0] << endl << CAM_INTRINSICS[1] << endl;

    // Use the same parameters for camera intrinsics, for now...
    feature_tracker.readIntrinsicParameter(CAM_INTRINSICS);

    /*  Ignore fisheye camera for now
    if(FISHEYE)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            trackerData[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);
            if(!trackerData[i].fisheye_mask.data)
            {
                ROS_INFO("load mask fail");
                ROS_BREAK();
            }
            else
                ROS_INFO("load mask success");
        }
    }
    */

    message_filters::Subscriber<sensor_msgs::Image> image_0_sub(n, IMAGE_TOPIC_0, 1);
    message_filters::Subscriber<sensor_msgs::Image> image_1_sub(n, IMAGE_TOPIC_1, 1);
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(
            image_0_sub, image_1_sub, 10);
    sync.registerCallback(boost::bind(&img_callback, _1, _2));

    pub_img = n.advertise<gvins_msgs::StereoFeatureTrack>("feature", 1000);
    pub_match = n.advertise<sensor_msgs::Image>("feature_img",1000);
    pub_restart = n.advertise<std_msgs::Bool>("restart",1000);
    /*
    if (SHOW_TRACK)
        cv::namedWindow("vis", cv::WINDOW_NORMAL);
    */
    ros::spin();
    return 0;
}


// new points velocity is 0, pub or not?
// track cnt > 1 pub?
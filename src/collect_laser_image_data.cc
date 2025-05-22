/**
 * This program is used for collecting laser data and images.
 * Author: xinliangzhong@foxmail.com
 * Updated by: ChatGPT + Aksel April 2025
 */

#include <iostream>
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <std_msgs/String.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <tf/transform_listener.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <dynamic_reconfigure/server.h>
#include "../cfg/cpp/camera_laser_calibration/dynamic_rangeConfig.h"
#include "../include/camera_laser_calibration/config.h"

using namespace std;
using namespace cv;
using namespace Eigen;

// Global variables
cv::Mat K;
cv::Mat D;
string output_data_path;
ofstream outFile;
sensor_msgs::PointCloud cloud_out;
Mat frame;
cv::Mat org, img, tmp;
Point2f keypoint;

double range_min = 0.;
double range_max = 0.;

// Corner detection
void cornerDetect(const cv::Mat &img, const Point &left_up, const Point &right_down)
{
    Mat image = img.clone();
    cvtColor(image, image, COLOR_BGR2GRAY);
    Mat mask(image.rows, image.cols, CV_8U, Scalar(0));
    Mat detect_roi = image.rowRange(left_up.y, right_down.y).colRange(left_up.x, right_down.x);
    detect_roi.copyTo(mask.rowRange(left_up.y, right_down.y).colRange(left_up.x, right_down.x));

    vector<Point2f> corners;
    goodFeaturesToTrack(image, corners, 1, 0.01, 10, mask, 3, false, 0.04);
    cornerSubPix(image, corners, Size(3, 3), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 40, 0.001));

    cvtColor(image, image, COLOR_GRAY2BGR);
    circle(image, corners[0], 1, Scalar(0, 0, 255), -1, 8, 0);
    imshow("corner", image);
    cout << "Corners: " << corners[0].x << " " << corners[0].y << endl;
    keypoint = corners[0];
}

// Mouse callback
void on_mouse(int event, int x, int y, int flags, void *ustc)
{
    static Point pre_pt(-1, -1);
    static Point cur_pt(-1, -1);
    char temp[16];
    if (event == CV_EVENT_LBUTTONDOWN)
    {
        org.copyTo(img);
        sprintf(temp, "(%d,%d)", x, y);
        pre_pt = Point(x, y);
        putText(img, temp, pre_pt, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0, 255), 1, 8);
        circle(img, pre_pt, 2, Scalar(255, 0, 0), CV_FILLED, CV_AA, 0);
        imshow("img", img);
    }
    else if (event == CV_EVENT_MOUSEMOVE && !(flags & CV_EVENT_FLAG_LBUTTON))
    {
        img.copyTo(tmp);
        sprintf(temp, "(%d,%d)", x, y);
        cur_pt = Point(x, y);
        putText(tmp, temp, cur_pt, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
        imshow("img", tmp);
    }
    else if (event == CV_EVENT_MOUSEMOVE && (flags & CV_EVENT_FLAG_LBUTTON))
    {
        img.copyTo(tmp);
        sprintf(temp, "(%d,%d)", x, y);
        cur_pt = Point(x, y);
        putText(tmp, temp, cur_pt, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
        rectangle(tmp, pre_pt, cur_pt, Scalar(255, 0, 0), 1, 8, 0);
        imshow("img", tmp);
    }
    else if (event == CV_EVENT_LBUTTONUP)
    {
        org.copyTo(img);
        sprintf(temp, "(%d,%d)", x, y);
        cur_pt = Point(x, y);
        putText(img, temp, cur_pt, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
        circle(img, pre_pt, 2, Scalar(255, 0, 0), CV_FILLED, CV_AA, 0);
        rectangle(img, pre_pt, cur_pt, Scalar(255, 0, 0), 1, 8, 0);
        imshow("img", img);

        cornerDetect(org, pre_pt, cur_pt);
    }
}

// Dynamic reconfigure callback
void configCallback(camera_laser_calibration::dynamic_rangeConfig &config, uint32_t level)
{
    range_min = config.range_min * M_PI / 180.;
    range_max = config.range_max * M_PI / 180.;

    ROS_INFO_STREAM("range_min: " << range_min * 180. / M_PI << " degree");
    ROS_INFO_STREAM("range_max: " << range_max * 180. / M_PI << " degree");

    if (config.Save)
    {
        ROS_INFO_STREAM("Please Select a Rectangle in the image");

        string laser_s(config.laser_coor);
        stringstream ss(laser_s);
        double x, y;
        string s;
        ss >> s >> y;
        s.erase(s.end() - 1);
        stringstream ss_x(s);
        ss_x >> x;

        cout << "Points in laser: x = " << x << ", y = " << y << endl;

        org = frame.clone();
        cv::undistort(org, img, K, D);
        org = img.clone();
        org.copyTo(tmp);
        namedWindow("img");
        setMouseCallback("img", on_mouse, 0);
        imshow("img", img);
        cv::waitKey(0);
        destroyWindow("corner");
        destroyAllWindows();

        ROS_INFO_STREAM("Please modify the laser_coor according to RViz point selection.");
        outFile << x << " " << y << " " << keypoint.x << " " << keypoint.y << endl;
        ROS_INFO_STREAM("Save successfully.");

        config.Save = false;
    }
}

// Image topic callback
void callback(const cv_bridge::CvImage::ConstPtr &imgMsg)
{
    frame = imgMsg->image.clone();
    if (frame.channels() == 1)
        cvtColor(frame, frame, COLOR_GRAY2BGR);
}

// Main function
int main(int argc, char **argv)
{
    if (argc < 2)
    {
        cout << "Please Check the launch file" << endl;
    }

    string config_path(argv[1]);
    cout << "Config.path = " << config_path << endl;
    Config::setParameterFile(config_path);

    ros::init(argc, argv, "collect_laser_image_data"); // 🚨 FIXED NAME
    ros::NodeHandle nh;

    // Dynamic reconfigure server
    dynamic_reconfigure::Server<camera_laser_calibration::dynamic_rangeConfig> *server;
    server = new dynamic_reconfigure::Server<camera_laser_calibration::dynamic_rangeConfig>(nh);
    dynamic_reconfigure::Server<camera_laser_calibration::dynamic_rangeConfig>::CallbackType f;
    f = boost::bind(&configCallback, _1, _2);
    server->setCallback(f);

    string scan_topic, image_topic;
    nh.getParam("scan_topic", scan_topic);
    nh.getParam("image_topic", image_topic);
    nh.param<string>("output_data_path", output_data_path, "");

    outFile.open(output_data_path);

    // Load camera parameters
    double fx = Config::get<double>("fx");
    double fy = Config::get<double>("fy");
    double cx = Config::get<double>("cx");
    double cy = Config::get<double>("cy");
    double k1 = Config::get<double>("k1");
    double k2 = Config::get<double>("k2");
    double p1 = Config::get<double>("p1");
    double p2 = Config::get<double>("p2");

    K = (Mat_<double>(3, 3) << fx, 0., cx, 0., fy, cy, 0., 0., 1.);
    D = (Mat_<double>(5, 1) << k1, k2, p1, p2, 0.0);

    cout << "K:\n" << K << endl;
    cout << "D:\n" << D << endl;

    ROS_INFO_STREAM("image_topic: " << image_topic);

    ros::Subscriber imgSub = nh.subscribe(image_topic, 1, callback);

    ros::spin();
    return 0;
}


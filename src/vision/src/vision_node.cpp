
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include<tf/transform_broadcaster.h>
#include "geometry_msgs/PoseStamped.h"

/*
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag16h5.h"
#include "AprilTags/Tag25h7.h"
#include "AprilTags/Tag25h9.h"
#include "AprilTags/Tag36h9.h"
#include "AprilTags/Tag36h11.h"
*/
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <aruco/aruco.h>
#include <aruco/boarddetector.h>
#include <aruco/cvdrawingutils.h>

#include "circle_detector.h"

#define RED_CIRCLE 2
#define BLUE_CIRCLE 1

enum
{
    DETECTION_MODE_NONE,
    DETECTION_MODE_CIRCLE,
    DETECTION_MODE_APRILTAGS,
};

double image_width = 640;
double image_height = 480;
double circle_radius = 0.26;

double fx = 600;
double fy = 600;
double px = image_width / 2.0;
double py = image_height / 2.0;
double tag_size = 0.1785;

double camera_x_offset = 0.175;
double camera_y_offset = 0;
double camera_z_offset = 0;

std::string board_config_file_path;
aruco::BoardConfiguration board_config;
aruco::CameraParameters cam_param;
std::vector<int> marker_id_list;

void load_board_config(ros::NodeHandle& nh)
{
    nh.param<std::string>("board_config_file_path", board_config_file_path, "config/board.yaml");
    board_config.readFromFile(board_config_file_path.c_str());

    std::cout << "board_config_file_path: " << board_config_file_path << std::endl;
    std::cout << "board_config: number of markers: " << board_config.size() << std::endl;
    for (int i = 0 ; i < board_config.size(); i++)
    {
        std::cout << "board_config: marker[" << i << "].id: " << board_config[i].id << std::endl;
    }
}

void load_cam_param(ros::NodeHandle& nh)
{
    /*
    cv::Mat cameraMatrix(3, 3, CV_32FC1);
	cv::Mat distorsionCoeff(4, 1, CV_32FC1);
	cv::Size camSize(image_height, image_width);

    //cv::Mat CameraMatrix = cv::Mat::eye(3,3,CV_32FC1);
    //cv::Mat distorsionCoeff = cv::Mat::zeros(4,1,CV_32FC1);
    //cv::Size camSize(image_height, image_width);

    cameraMatrix.at<float>(0,0) = fx;
    cameraMatrix.at<float>(0,2) = px;
    cameraMatrix.at<float>(1,1) = fy;
    cameraMatrix.at<float>(1,2) = py;
    distorsionCoeff.at<float>(0,0) = 0.0;
    distorsionCoeff.at<float>(1,0) = 0.0;
    distorsionCoeff.at<float>(2,0) = 0.0;
    distorsionCoeff.at<float>(3,0) = 0.0;
    
    cam_param.setParams(cameraMatrix, distorsionCoeff, camSize);
    */

    
    float cam_mat[] = { 
        fx, 0.000000, px, 
        0.000000, fy, py, 
        0.000000, 0.000000, 1.000000 
    };
    
    float dis_mat[] = {0, 0, 0, 0, 0};
    // float dis_mat[] = {0.068055, -0.202252, -0.001100, 0.002548, 0.000000};
    
    cv::Mat cameraMatrix(3, 3, CV_32FC1, cam_mat);
    cv::Mat  distorsionCoeff(4, 1, CV_32FC1, dis_mat);
    cv::Size camSize(image_height, image_width);
    
    cam_param.setParams(cameraMatrix, distorsionCoeff, camSize);
}

void load_marker_id_list(ros::NodeHandle& nh)
{
    //nh.getParam("marker_id_list", marker_id_list);
    ros::param::get("/marker_id_list", marker_id_list);
    if (marker_id_list.size() < 1)
    {
        marker_id_list.push_back(100);
    }
    std::cout << "marker_id_list.size = " << marker_id_list.size() << std::endl;
    std::cout << "marker_id_list: [" << std::endl;
    for (int i = 0; i < marker_id_list.size() - 1; i++)
    {
        std::cout << marker_id_list[i] << ", ";
    }
    std::cout << marker_id_list[marker_id_list.size() - 1] << "]" << std::endl;
}

double get_projection_ratio(double circle_pixel_radius)
{
    double ratio = circle_radius / circle_pixel_radius;
    return ratio;
}

double get_relative_x(double circle_pixel_radius, double circle_pixel_center_x)
{
    double dpx = circle_pixel_center_x - px;
    double ratio = circle_radius / circle_pixel_radius;
    double rx = dpx * ratio;
    return rx;
}

double get_relative_y(double circle_pixel_radius, double circle_pixel_center_y)
{
    double dpy = circle_pixel_center_y - py;
    double ratio = circle_radius / circle_pixel_radius;
    double ry = dpy * ratio;
    return ry;
}

double get_relative_z(double circle_pixel_radius)
{
    double f = (fx + fy) / 2.0;
    double ratio = circle_radius / circle_pixel_radius;
    double rz = f * circle_radius / circle_pixel_radius;
    return rz;
}

uint8_t detection_mode = DETECTION_MODE_NONE;

bool detect_tag_only = false;
int jet_state_detect_apriltag = 6;
int jet_state_detect_circle = 11;
void jet_state_callback(const std_msgs::UInt8ConstPtr& jet_state_msg)
{
    if (detect_tag_only)
    {
        if (jet_state_msg->data == jet_state_detect_circle || jet_state_msg->data == jet_state_detect_apriltag)
        {
            detection_mode = DETECTION_MODE_APRILTAGS;
        }
        else
        {
            detection_mode = DETECTION_MODE_NONE;
        }
    }
    else
    {
        if (jet_state_msg->data == jet_state_detect_circle)
        {
            detection_mode = DETECTION_MODE_CIRCLE;
        }
        else if (jet_state_msg->data == jet_state_detect_apriltag)
        {
            detection_mode = DETECTION_MODE_APRILTAGS;
        }
        else
        {
            detection_mode = DETECTION_MODE_NONE;
        }
    }
    
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "vision");

    ros::NodeHandle np("~");

    int vid = 0;
    
    np.param<int>("vid", vid, 0);
    cv::VideoCapture cap(vid);

    if (!cap.isOpened())
    {
        std::cout << "ERROR: Can not open video device " << vid << std::endl;
        return -1;
    }

    bool show_image;
    np.param<bool>("show_image", show_image, true);
    np.param<bool>("detect_tag_only", detect_tag_only, true);
    np.param<int>("jet_state_detect_apriltag", jet_state_detect_apriltag, 6);
    np.param<int>("jet_state_detect_circle", jet_state_detect_circle, 11);

    std::string tag_code;
    np.param<std::string>("tag_code", tag_code, "16h5");
    np.param<double>("tag_size", tag_size, 0.1785);
    np.param<double>("image_width", image_width, 640);
    np.param<double>("image_height", image_height, 480);
    np.param<double>("circle_radius", circle_radius, 0.26);
    np.param<double>("camera_x_offset", camera_x_offset, 0.175);
    np.param<double>("camera_y_offset", camera_y_offset, 0);
    np.param<double>("camera_z_offset", camera_z_offset, 0);

    np.param<double>("fx", fx, 600);
    np.param<double>("fy", fy, 600);
    np.param<double>("px", px, image_width / 2);
    np.param<double>("py", py, image_height / 2);

    // marker_id_list
    load_marker_id_list(np);

    load_board_config(np);
    load_cam_param(np);

/*
    AprilTags::TagCodes m_tagCodes(AprilTags::tagCodes16h5);
    if (tag_code == "16h5") {
        m_tagCodes = AprilTags::tagCodes16h5;
    } else if (tag_code == "25h7") {
        m_tagCodes = AprilTags::tagCodes25h7;
    } else if (tag_code == "25h9") {
        m_tagCodes = AprilTags::tagCodes25h9;
    } else if (tag_code == "36h9") {
        m_tagCodes = AprilTags::tagCodes36h9;
    } else if (tag_code == "36h11") {
        m_tagCodes = AprilTags::tagCodes36h11;
    } else {
        m_tagCodes = AprilTags::tagCodes16h5;
      cout << "Invalid tag family specified, use default 16h5" << endl;
    }
*/
    CircleDetector circle_detector(BLUE_CIRCLE);  //1: blue, 2: red
    // AprilTags::TagDetector tag_detector(m_tagCodes);
    aruco::MarkerDetector marker_detector;
    aruco::BoardDetector board_detector;
    aruco::Board board_detected;
    std::vector<aruco::Marker> markers;

    ros::NodeHandle nh;
    ros::Subscriber jet_state_sub = nh.subscribe<std_msgs::UInt8>("/jet_state", 10, jet_state_callback);
    ros::Publisher target_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/vision/target_pos", 10);
    ros::Publisher detection_mode_pub = nh.advertise<std_msgs::UInt8>("/vision/detection_mode", 10);

    cv::Mat img, img_gray;

    bool detected = false;
    bool window_created = false;

    ros::Rate rate(30);

    while(ros::ok()) {
        ros::spinOnce();
        if (detection_mode != DETECTION_MODE_NONE)
        {
            if (!cap.isOpened())
            {
                std::cout << "opening camera" << std::endl;
                cap.open(vid);
            }
            cap >> img;
            if (detection_mode == DETECTION_MODE_CIRCLE)
            {
                detected = circle_detector.detect(img);
                
                if (detected)
                {
                    if (show_image)
                        circle_detector.draw(img);

                    double rx = get_relative_x(circle_detector.m_radius, circle_detector.m_center.x);
                    double ry = get_relative_y(circle_detector.m_radius, circle_detector.m_center.y);
                    double rz = get_relative_z(circle_detector.m_radius);

                    geometry_msgs::PoseStamped target_pos;

                    target_pos.header.stamp = ros::Time::now();
                    target_pos.header.frame_id = "circle";

                    target_pos.pose.position.x = ry + camera_x_offset;
                    target_pos.pose.position.y = -rx + camera_y_offset;
                    target_pos.pose.position.z = rz + camera_z_offset;

                    target_pos.pose.orientation = tf::createQuaternionMsgFromYaw(0);

                    target_pos_pub.publish(target_pos);
                }
            }
            else if (detection_mode == DETECTION_MODE_APRILTAGS)
            {
                //cv::cvtColor(img, img_gray, CV_BGR2GRAY);

                // std::cout << "before marker_detector.detect is called" << std::endl;

                // markers.clear();
                marker_detector.detect(img, markers, cam_param, 0.05, false);

                detected = false;
                
                aruco::Marker marker;
                for (int i = 0; i < markers.size() && (!detected); i++)
                {
                    for (int j = 0; i < marker_id_list.size() && (!detected); j++)
                    {
                        if (markers[i].id == marker_id_list[j])
                        {
                            marker = markers[i];
                            detected = true;
                            break;
                        }
                    }
                }

                // std::cout << "marker_detector.detect is called" << std::endl;

                //Detection of the board
                //float prob = board_detector.detect(markers, board_config, board_detected, cam_param, tag_size);
                //detected = prob > 0.0 && markers.size() > 0;
                //detected = markers.size() > 0;

                if (detected)
                {
                    
                    //aruco::Marker marker = markers[0];
                    if (show_image)
                    {
                        marker.draw(img, cv::Scalar(0,0,255), 2);
                        aruco::CvDrawingUtils::draw3dAxis(img, marker, cam_param);
                    }
                    

                    /*
                    if (show_image)
                    {
                        for (int i = 0; i < markers.size(); i++)
                        {
                            markers[i].draw(img, cv::Scalar(0,0,255), 2);
                            aruco::CvDrawingUtils::draw3dAxis(img, markers[i], cam_param);
                        }
                        
                    }
                    */
                    
                    /*
                    cv::Mat trans = board_detected.Tvec;
                    cv::Mat rot = board_detected.Rvec;

                    std::cout << "R:" << rot << std::endl;
                    std::cout << "T:" << trans << std::endl;

                    double tx = trans.at<float>(0, 0);
                    double ty = trans.at<float>(1, 0);
                    double tz = trans.at<float>(2, 0);

                    double rx = rot.at<float>(0, 0);
                    double ry = rot.at<float>(1, 0);
                    double rz = rot.at<float>(2, 0);
                    */

                    double t[3];
                    double q[4];

                    marker.OgreGetPoseParameters(t, q);

                    //geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(rx, ry, rz);

                    geometry_msgs::PoseStamped target_pos;

                    target_pos.header.stamp = ros::Time::now();
                    target_pos.header.frame_id = "marker";

                    target_pos.pose.position.x = t[1] + camera_x_offset; // coordinate transform
                    target_pos.pose.position.y = -t[0] + camera_y_offset;
                    target_pos.pose.position.z = t[2] + camera_z_offset;
                    //target_pos.pose.orientation = quat;
                    target_pos.pose.orientation.x = q[0];
                    target_pos.pose.orientation.y = q[1];
                    target_pos.pose.orientation.z = q[2];
                    target_pos.pose.orientation.w = q[3];

                    target_pos_pub.publish(target_pos);

                }
                /*
                std::vector<AprilTags::TagDetection> tag_detections = tag_detector.extractTags(img_gray);
                ROS_DEBUG("%d tag detected", (int)tag_detections.size());

                detected = tag_detections.size() > 0;

                if (detected)
                {
                    AprilTags::TagDetection detection = tag_detections[0];
                    if (show_image)
                        detection.draw(img);
                    Eigen::Matrix4d transform = detection.getRelativeTransform(tag_size, fx, fy, px, py);
                    Eigen::Matrix3d rot = transform.block(0, 0, 3, 3);
                    Eigen::Quaternion<double> rot_quaternion = Eigen::Quaternion<double>(rot);

                    //target_pos.pose.position.x = transform(0, 3);
                    //target_pos.pose.position.y = transform(1, 3);
                    //target_pos.pose.position.z = transform(2, 3);
                    double rx = transform(0, 3);
                    double ry = transform(1, 3);
                    double rz = transform(2, 3);

                    geometry_msgs::PoseStamped target_pos;

                    target_pos.header.stamp = ros::Time::now();
                    target_pos.header.frame_id = "apriltag";

                    target_pos.pose.position.x = ry + camera_x_offset; // coordinate transform
                    target_pos.pose.position.y = -rx + camera_y_offset;
                    target_pos.pose.position.z = rz + camera_z_offset;
                    target_pos.pose.orientation.x = rot_quaternion.x();
                    target_pos.pose.orientation.y = rot_quaternion.y();
                    target_pos.pose.orientation.z = rot_quaternion.z();
                    target_pos.pose.orientation.w = rot_quaternion.w();

                    target_pos_pub.publish(target_pos);
                }
                */
            }

            if (show_image)
            {
                cv::imshow("vision", img);
                window_created = true;
                cv::waitKey(1);
            }
        }
        else
        {
            if (window_created)
            {
                cv::destroyWindow("vision");
                window_created = false;
            }
            cap.release(); 
        }
        
        std_msgs::UInt8 detection_mode_msg;
        detection_mode_msg.data = detection_mode;
        detection_mode_pub.publish(detection_mode_msg);

        rate.sleep();
    }

    return 0;
}


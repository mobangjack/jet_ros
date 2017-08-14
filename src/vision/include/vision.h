/**
 * Copyright (c) 2017, Jack Mo (mobangjack@foxmail.com).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <std_srvs/Empty.h>

#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <aruco/aruco.h>
#include <aruco/boarddetector.h>
#include <aruco/cvdrawingutils.h>

#include <opencv2/opencv.hpp>
#include <ar_sys/utils.h>
#include "circle/circle_detector.h"

class Vision
{
public:
    Vision();

protected:
    // camera parameters
    cv::Mat camera_Rmat, camera_Tmat;
    aruco::CameraParameters cam_param;

    // marker configuration
    float marker_size;
    std::vector<int> marker_id_list;

    // circle configuration
    float circle_inner_radius;
    float circle_outer_radius;

    // marker detector
    aruco::MarkerDetector marker_detector;

    // circle detector
    CircleDetector circle_detector;

protected:
    // private node handle
    ros::NodeHandle nh;

    // subscribers
    image_transport::ImageTransport image_transport;
	image_transport::Subscriber image_sub;
    ros::Subscriber cam_info_sub;
    ros::Subscriber jet_state_sub;

    // publishers
    ros::Publisher target_pose_pub;
    ros::Publisher detection_mode_pub;
    image_transport::Publisher image_pub;

    // service servers
    ros::ServiceServer reload_camera_param_srv;
    ros::ServiceServer reload_detmod_param_srv;
    ros::ServiceServer reload_circle_param_srv;
    ros::ServiceServer reload_marker_param_srv;

protected:
    // subscriber callbacks
    void jet_state_callback(const std_msgs::UInt8ConstPtr& jet_state_msg);
    void image_callback(const sensor_msgs::ImageConstPtr& msg);
    void cam_info_callback(const sensor_msgs::CameraInfo &msg);

    // service callbacks
    bool reload_camera_param_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool reload_circle_param_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool reload_marker_param_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool reload_detmod_param_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

protected:
    std::vector<int> detection_mode_marker;
    std::vector<int> detection_mode_circle;
    int detection_mode;
    int circle_detection_color;
    int circle_detection_method;
    bool draw_result;
    bool draw_markers_cube;
    bool draw_markers_axis;
    bool cam_info_received;
    bool image_is_rectified;
    bool detect_markers_only;

    int spin_rate;

    cv::Mat in_image, result_image;
    cv::Mat target_Rvec, target_Rmat, target_Tmat;

    // target pose message to publish
    geometry_msgs::PoseStamped target_pose;

protected:
    bool is_marker_detection_mode(int detmod);
    bool is_circle_detection_mode(int detmod);
    bool need_capture(int detmod);
    bool detect_marker();
    bool detect_circle();
    void publish_detection_mode();
    void publish_target_pose();
    void publish_result_image();

protected:
    bool load_camera_param(ros::NodeHandle& nh);
    bool load_circle_param(ros::NodeHandle& nh);
    bool load_marker_param(ros::NodeHandle& nh);
    bool load_detmod_param(ros::NodeHandle& nh);

public:
    void spin();
};

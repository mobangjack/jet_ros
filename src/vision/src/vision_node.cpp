
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include<tf/transform_broadcaster.h>
#include "geometry_msgs/PoseStamped.h"

#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag16h5.h"
#include "AprilTags/Tag25h7.h"
#include "AprilTags/Tag25h9.h"
#include "AprilTags/Tag36h9.h"
#include "AprilTags/Tag36h11.h"

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
double tag_size = 0.163513;

double camera_x_offset = 0.175;
double camera_y_offset = 0;
double camera_z_offset = 0;

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
int jet_state_detect_circle = 10;
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
    
    np.param<double>("fx", fx, 600);
    np.param<double>("fy", fy, 600);
    np.param<double>("px", px, 320);
    np.param<double>("py", py, 240);
    np.param<std::string>("tag_code", tag_code, "16h5");
    np.param<double>("tag_size", tag_size, 0.163513);
    np.param<double>("image_width", image_width, 640);
    np.param<double>("image_height", image_height, 480);
    np.param<double>("circle_radius", circle_radius, 0.26);
    np.param<double>("camera_x_offset", camera_x_offset, 0.175);
    np.param<double>("camera_y_offset", camera_y_offset, 0);
    np.param<double>("camera_z_offset", camera_z_offset, 0);

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

    CircleDetector circle_detector(RED_CIRCLE);  //1: blue, 2: red=
    AprilTags::TagDetector tag_detector(m_tagCodes);

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
                cv::cvtColor(img, img_gray, CV_BGR2GRAY);
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


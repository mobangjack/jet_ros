
#include "vision.h"

Vision::Vision() : nh("~"), image_transport(nh), cam_info_received(false), detection_mode(0)
{
    ROS_INFO("Vision: loading parameters");
    // private parameters
    nh.param<int>("spin_rate", 30);
    nh.param<bool>("image_is_rectified", image_is_rectified, true); 
    nh.param<bool>("draw_result", draw_result, false);
    nh.param<bool>("draw_markers_cube", draw_markers_cube, false);
    nh.param<bool>("draw_markers_axis", draw_markers_axis, false);
    nh.param<bool>("detect_markers_only", detect_markers_only, false);

    // load parameters
    load_camera_param(nh);
    load_circle_param(nh);
    load_marker_param(nh);
    load_detmod_param(nh);

    ROS_INFO("Vision: initilaizing subscribers");
    // initialize subscribers
    image_sub = image_transport.subscribe("/image", 1, &Vision::image_callback, this);
    cam_info_sub = nh.subscribe("/camera_info", 1, &Vision::cam_info_callback, this);
    jet_state_sub = nh.subscribe("/jet/state", 10, &Vision::jet_state_callback, this);

    ROS_INFO("Vision: initilaizing publishers");
    // initialize publishers
    target_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/vision/target_pose", 10);
    detection_mode_pub = nh.advertise<std_msgs::UInt8>("/vision/detection_mode", 10);
    image_pub = image_transport.advertise("/vision/result", 1);

    ROS_INFO("Vision: initilaizing services");
    // initialize service servers
    reload_camera_param_srv = nh.advertiseService("/vision/reload_camera_param", &Vision::reload_camera_param_callback, this);
    reload_detmod_param_srv = nh.advertiseService("/vision/reload_detmod_param", &Vision::reload_detmod_param_callback, this);
    reload_circle_param_srv = nh.advertiseService("/vision/reload_circle_param", &Vision::reload_circle_param_callback, this);
    reload_marker_param_srv = nh.advertiseService("/vision/reload_marker_param", &Vision::reload_marker_param_callback, this);

    ROS_INFO("Vision: initilaizition done");
}

bool Vision::load_circle_param(ros::NodeHandle& nh)
{
    nh.param<float>("/vision/circle/inner_radius", circle_inner_radius, 0.45);
    nh.param<float>("/vision/circle/outer_radius", circle_outer_radius, 0.50);

    std::cout << "/vision/circle: {" << "inner_radius: " << circle_inner_radius
              << ", outer_radius: " << circle_outer_radius << "}" << std::endl;

    return true;
}

bool Vision::load_marker_param(ros::NodeHandle& nh)
{
    marker_id_list.clear();

    nh.param<float>("/vision/marker/size", marker_size, 0.15);
    ros::param::get("/vision/marker/id_list", marker_id_list);

    if (marker_id_list.size() < 1)
    {
        std::cout << "WARNING: param vision/marker/id_list is empty, a default id was added to it" << std::endl;
        marker_id_list.push_back(100);
    }

    std::cout << "/vision/marker: {" << "size: " << marker_size
              << ", id_list: [";
    for (int i = 0; i < marker_id_list.size() - 1; i++)
    {
        std::cout << marker_id_list[i] << ", ";
    }
    if (marker_id_list.size() > 0)
        std::cout << marker_id_list[marker_id_list.size() - 1];
    std::cout << "]" << std::endl;

    return true;
}

bool Vision::load_camera_param(ros::NodeHandle& nh)
{
    // camera extrinsic parameters
    std::string camera_info_url;
    ros::param::get("/usb_cam/camera_info_url", camera_info_url);
    std::cout << "Vision: camera_info_url: " << camera_info_url << std::endl;
    std::string camera_info_path = camera_info_url.substr(7, camera_info_url.length());
    std::cout << "Vision: camera_info_path: " << camera_info_path << std::endl;
    cv::FileStorage fs(camera_info_path, cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
        fs.release();
        return false;
    }
    fs["extrinsic_rotation"] >> camera_Rmat;
    fs["extrinsic_translation"] >> camera_Tmat;
    std::cout << "Extrinsic_R : " << std::endl << camera_Rmat << std::endl;
    std::cout << "Extrinsic_T : " << std::endl << camera_Tmat << std::endl;
    fs.release();
    return true;
}

bool Vision::load_detmod_param(ros::NodeHandle& nh)
{
    nh.param<int>("/vision/detection_mode/marker", detection_mode_marker, 6);
    nh.param<int>("/vision/detection_mode/circle", detection_mode_circle, 11);

    std::cout << "/vision/detection_mode: { marker: ";
    std::cout << detection_mode_marker;
    std::cout << ", circle: ";
    std::cout << detection_mode_circle;
    std::cout << "}" << std::endl;

    return true;
}

bool Vision::reload_camera_param_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    return load_camera_param(nh);
}

bool Vision::reload_circle_param_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    return load_circle_param(nh);
}

bool Vision::reload_marker_param_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    return load_marker_param(nh);
}

bool Vision::reload_detmod_param_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    return load_detmod_param(nh);
}

void Vision::publish_detection_mode()
{
    if(detection_mode_pub.getNumSubscribers() > 0)
    {
        std_msgs::UInt8 detection_mode_msg;
        detection_mode_msg.data = detection_mode;
        detection_mode_pub.publish(detection_mode_msg);
    }
}

void Vision::publish_target_pose()
{
    if(target_pose_pub.getNumSubscribers() > 0)
    {
        target_pose.pose.position.x = target_Tmat.ptr<float>(0)[0];
        target_pose.pose.position.y = target_Tmat.ptr<float>(0)[1];
        target_pose.pose.position.z = target_Tmat.ptr<float>(0)[2];

        Eigen::Matrix3d eigen_R;
        cv2eigen(target_Rmat, eigen_R);
        Eigen::Quaterniond q(eigen_R);
        q.normalize();

        target_pose.pose.orientation.x = q.x();
        target_pose.pose.orientation.y = q.y();
        target_pose.pose.orientation.z = q.z();
        target_pose.pose.orientation.w = q.w();

        target_pose.header.frame_id = "vision_target";
        target_pose.header.stamp = ros::Time::now();
        target_pose_pub.publish(target_pose);
    }
}

void Vision::publish_result_image()
{
    if(image_pub.getNumSubscribers() > 0)
    {
        //show input with augmented information
        cv_bridge::CvImage out_msg;
        out_msg.header.frame_id = "vision_result";
        out_msg.header.stamp = ros::Time::now();
        out_msg.encoding = sensor_msgs::image_encodings::RGB8;
        out_msg.image = result_image;
        image_pub.publish(out_msg.toImageMsg());
    }
}

bool Vision::process_marker()
{
    bool detected = false;
    // detection results will go into "markers"
    markers.clear();
    // ok, let's detect
    marker_detector.detect(in_image, markers, cam_param, marker_size, false);
    aruco::Marker marker;
    for (int i = 0; i < markers.size() && (!detected); i++)
    {
        /*
        if (draw_result)
        {
            markers[i].draw(result_image, cv::Scalar(0,0,255), 2);
            if (draw_markers_cube) aruco::CvDrawingUtils::draw3dCube(result_image, markers[i], cam_param);
            if (draw_markers_axis) aruco::CvDrawingUtils::draw3dAxis(result_image, markers[i], cam_param);
        }
        */
        for (int j = 0; j < marker_id_list.size() && (!detected); j++)
        {
            if (markers[i].id == marker_id_list[j])
            {
                marker = markers[i];
                detected = true;
                break;
            }
        }
    }

    if (detected)
    {
        if (draw_result)
        {
            marker.draw(result_image, cv::Scalar(0,0,255), 2);
            if (draw_markers_cube) aruco::CvDrawingUtils::draw3dCube(result_image, marker, cam_param);
            if (draw_markers_axis) aruco::CvDrawingUtils::draw3dAxis(result_image, marker, cam_param);
        }

        cv::Mat R33(3, 3, CV_32FC1);
        cv::Rodrigues(marker.Rvec, R33);
        
        // ROS port
        for (int i = 1; i < 3; i++)
        {
            R33.at<float>(i, i) = -R33.at<float>(i, i); // see @ ar_sys/src/utils.cpp
        }

        cv::Mat T31(3, 1, CV_32FC1);
        for (int i = 0; i < 3; i++)
        {
            T31.ptr(0)[i] = marker.Tvec.ptr(0)[i];
        }
        
        target_Rmat = camera_Rmat * R33;
        target_Tmat = camera_Rmat * T31 + camera_Tmat;
    }

    return detected;
}

bool Vision::process_circle()
{
    bool detected = circle_detector.detect(in_image);

    if (detected && draw_result)
    {
        circle_detector.draw(result_image);
    }

    if ((!detected) || (circle_detector.m_inner_score == 0))
    {
        detected = false;
        return false;
    }

    // 3x3 matrix (fx 0 cx, 0 fy cy, 0 0 1)
    // 4x1 matrix (k1,k2,p1,p2)
    float fx = cam_param.CameraMatrix.at<float>(0, 0);
    float fy = cam_param.CameraMatrix.at<float>(1, 1);
    float cx = cam_param.CameraMatrix.at<float>(0, 2);
    float cy = cam_param.CameraMatrix.at<float>(1, 2);

    float circle_radius = circle_detector.m_inner_score > 0 ? circle_inner_radius : circle_outer_radius;
    float circle_pixel_radius = circle_detector.m_radius;

    float ratio = circle_radius / circle_pixel_radius;
    
    float dpx = circle_detector.m_center.x - cx;
    float tx = dpx * ratio;

    float dpy = circle_detector.m_center.y - cy;
    float ty = dpy * ratio;

    float focal = (fx + fy) / 2.0;
    float tz = focal * ratio;

    cv::Mat R33 = cv::Mat::zeros(3, 3, CV_32FC1);
    for (int i = 0; i < 3; i++)
    {
        R33.at<float>(i, i) = 1; // No rotation for circle
    }

    cv::Mat T31 = cv::Mat::zeros(3, 1, CV_32FC1);
    T31.at<float>(0, 0) = tx;
    T31.at<float>(1, 0) = ty;
    T31.at<float>(2, 0) = tz;

    target_Rmat = camera_Rmat * R33;
    target_Tmat = camera_Rmat * T31 + camera_Tmat;

    return detected;
}

void Vision::image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    // std::cout << "vision::image_callback is called" << std::endl;
    if (!cam_info_received) {
        ROS_WARN("No camera info received, image callback will do nothing but return");
        return;
    }
    bool detected = false;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        in_image = cv_ptr->image;
        result_image = cv_ptr->image.clone();

        if (detection_mode == detection_mode_marker)
        {
            detected = process_marker();
        }
        else if (detection_mode == detection_mode_circle)
        {
            detected = process_circle();
        }
        if (detected)
        {
            publish_target_pose();
        }
        if (draw_result)
        {
            publish_result_image();
        }
        publish_detection_mode();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

// wait for one camerainfo, then shut down that subscriber
void Vision::cam_info_callback(const sensor_msgs::CameraInfo &msg)
{
    cam_param = ar_sys::getCamParams(msg, image_is_rectified);
    cam_info_received = true;
    cam_info_sub.shutdown();
    ROS_INFO("vision: camera parameters obtained, camera info subscriber was shut down");
}

void Vision::jet_state_callback(const std_msgs::UInt8ConstPtr& jet_state_msg)
{
    static uint8_t last_detection_mode = 0;
    static bool is_capturing = true;
    last_detection_mode = detection_mode;
    detection_mode = jet_state_msg->data;
    if (jet_state_msg->data == detection_mode_circle && detect_markers_only)
    {
        detection_mode = detection_mode_marker;
    }
    if (is_capturing)
    {
        std_srvs::Empty req;
        ros::service::call("/usb_cam/stop_capture", req);
        is_capturing = false;
    }
    if ((detection_mode == detection_mode_circle || detection_mode == detection_mode_marker) &&
        (last_detection_mode != detection_mode_circle && last_detection_mode != detection_mode_marker))
    {
        std_srvs::Empty req;
        ros::service::call("/usb_cam/start_capture", req);
    }
    if ((last_detection_mode == detection_mode_circle || last_detection_mode == detection_mode_marker) &&
        (detection_mode != detection_mode_circle && detection_mode != detection_mode_marker))
    {
        std_srvs::Empty req;
        ros::service::call("/usb_cam/stop_capture", req);
    }
}

void Vision::spin()
{
    ros::Rate rate(30);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "vision");

    Vision vision;

    vision.spin();
    
    return 0;
}


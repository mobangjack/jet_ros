
#include "vision_node.h"

VisionNode::VisionNode(ros::NodeHandle& nh) : image_transport(nh), cam_info_received(false)
{
    this->nh = nh;

    // private parameters
    nh.param<int>("spin_rate", 30);
    nh.param<bool>("image_is_rectified", image_is_rectified, true);
    nh.param<bool>("draw_markers", draw_markers, false);
    nh.param<bool>("draw_markers_cube", draw_markers_cube, false);
    nh.param<bool>("draw_markers_axis", draw_markers_axis, false);
    nh.param<bool>("detect_marker_only", detect_marker_only, false);
    nh.param<float>("camera_x_offset", camera_x_offset, 0.175);
    nh.param<float>("camera_y_offset", camera_y_offset, 0);
    nh.param<float>("camera_z_offset", camera_z_offset, 0);

    // load parameters
    load_camera_param(nh);
    load_circle_param(nh);
    load_marker_param(nh);
    load_detmod_param(nh);

    // initialize subscribers
    image_sub = image_transport.subscribe("image", 1, &VisionNode::image_callback, this);
	cam_info_sub = nh.subscribe("camera_info", 1, &VisionNode::cam_info_callback, this);
    jet_state_sub = nh.subscribe("jet_state", 10, &VisionNode::jet_state_callback, this);

    // initialize publishers
    target_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("target_pos", 10);
    detection_mode_pub = nh.advertise<std_msgs::UInt8>("detection_mode", 10);
    image_pub = image_transport.advertise("result", 1);

    // initialize service servers
    reload_camera_param_srv = nh.advertiseService("reload_camera_param", &VisionNode::reload_camera_param_callback, this);
    reload_circle_param_srv = nh.advertiseService("reload_circle_param", &VisionNode::reload_circle_param_callback, this);
    reload_marker_param_srv = nh.advertiseService("reload_marker_param", &VisionNode::reload_marker_param_callback, this);
}

bool VisionNode::load_camera_param(ros::NodeHandle& nh)
{
    nh.param<std::string>("cam_param_file_path", cam_param_file_path, "config/camera.yaml");
    std::cout << "cam_param_file_path: " << cam_param_file_path << std::endl;
    try
    {
        cam_param.readFromFile(cam_param_file_path.c_str());
        return true;
    }
    catch (cv::Exception& e)
    {
        ROS_ERROR("cv exception: %s", e.what());
        return false;
    }
}

bool VisionNode::load_circle_param(ros::NodeHandle& nh)
{
    nh.param<float>("circle/inner_radius", circle_inner_radius, 0.45);
    nh.param<float>("circle/outer_radius", circle_outer_radius, 0.50);

    std::cout << "circle: {" << "inner_radius: " << circle_inner_radius
              << ", outer_radius: " << circle_outer_radius << "}" << std::endl;
}

bool VisionNode::load_marker_param(ros::NodeHandle& nh)
{
    marker_id_list.clear();

    nh.param<float>("marker/size", marker_size, 0.15);
    nh.getParam("marker/id_list", marker_id_list);

    if (marker_id_list.size() < 1)
    {
        std::cout << "WARNING: param vision/marker/id_list is empty, a default id was added to it" << std::endl;
        marker_id_list.push_back(100);
    }

    std::cout << "marker: {" << "size: " << marker_size
              << ", id_list: [";
    for (int i = 0; i < marker_id_list.size() - 1; i++)
    {
        std::cout << marker_id_list[i] << ", ";
    }
    if (marker_id_list.size() > 0)
        std::cout << marker_id_list[marker_id_list.size() - 1];
    std::cout << "]" << std::endl;
}

bool VisionNode::load_detmod_param(ros::NodeHandle& nh)
{
    nh.param<int>("detection_mode/marker", detection_mode_marker, 6);
    nh.param<int>("detection_mode/circle", detection_mode_circle, 11);

    std::cout << "detection_mode: { marker: ";
    std::cout << detection_mode_marker;
    std::cout << ", circle: ";
    std::cout << detection_mode_circle;
    std::cout << "}" << std::endl;
}

bool VisionNode::reload_camera_param_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    return load_camera_param(nh);
}

bool VisionNode::reload_circle_param_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    return load_circle_param(nh);
}

bool VisionNode::reload_marker_param_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    return load_marker_param(nh);
}

bool VisionNode::reload_detmod_param_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    return load_detmod_param(nh);
}

void VisionNode::publish_detection_mode()
{
    if(detection_mode_pub.getNumSubscribers() > 0)
    {
        std_msgs::UInt8 detection_mode_msg;
        detection_mode_msg.data = detection_mode;
        detection_mode_pub.publish(detection_mode_msg);
    }
}

void VisionNode::publish_target_pos()
{
    if(target_pos_pub.getNumSubscribers() > 0)
    {
        target_pos.header.stamp = ros::Time::now();
        target_pos_pub.publish(target_pos);
    }
}

void VisionNode::publish_result_image()
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

bool VisionNode::process_marker()
{
    bool detected = false;
    // detection results will go into "markers"
    markers.clear();
    // ok, let's detect
    marker_detector.detect(in_image, markers, cam_param, marker_size, false);
    aruco::Marker marker;
    for (int i = 0; i < markers.size() && (!detected); i++)
    {
        if (draw_result)
        {
            markers[i].draw(result_image, cv::Scalar(0,0,255), 2);
            if (draw_markers_cube) aruco::CvDrawingUtils::draw3dCube(result_image, markers[i], cam_param);
            if (draw_markers_axis) aruco::CvDrawingUtils::draw3dAxis(result_image, markers[i], cam_param);
        }
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

    if (detected)
    {
        double t[3];
        double q[4];

        marker.OgreGetPoseParameters(t, q);

        target_pos.header.frame_id = "marker";

        target_pos.pose.position.x = t[1] + camera_x_offset;
        target_pos.pose.position.y = -t[0] + camera_y_offset;
        target_pos.pose.position.z = t[2] + camera_z_offset;
        target_pos.pose.orientation.x = q[0];
        target_pos.pose.orientation.y = q[1];
        target_pos.pose.orientation.z = q[2];
        target_pos.pose.orientation.w = q[3];
    }

    return detected;
}

bool VisionNode::process_circle()
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

    target_pos.header.frame_id = "circle";
    target_pos.pose.position.x = ty + camera_x_offset;
    target_pos.pose.position.y = -tx + camera_y_offset;
    target_pos.pose.position.z = tz + camera_z_offset;
    target_pos.pose.orientation = tf::createQuaternionMsgFromYaw(0);

    return detected;
}

void VisionNode::image_callback(const sensor_msgs::ImageConstPtr& msg)
{
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
            publish_target_pos();
            publish_detection_mode();
            if (draw_result) publish_result_image();
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

// wait for one camerainfo, then shut down that subscriber
void VisionNode::cam_info_callback(const sensor_msgs::CameraInfo &msg)
{
    cam_param = ar_sys::getCamParams(msg, image_is_rectified);
    cam_info_received = true;
    cam_info_sub.shutdown();
}

void VisionNode::jet_state_callback(const std_msgs::UInt8ConstPtr& jet_state_msg)
{
    detection_mode = jet_state_msg->data;
    if (jet_state_msg->data == detection_mode_circle && detect_marker_only)
    {
        detection_mode = detection_mode_marker;
    }
}

void VisionNode::spin()
{
    ros::Rate rate(spin_rate);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "vision_node");

    ros::NodeHandle np("~");

    VisionNode vision_node(np);
    vision_node.spin();
    
    return 0;
}


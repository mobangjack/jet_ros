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

#include "jet.h"

Jet::Jet() : nh("~"), uart_fd(-1), calied(false), use_guidance(false), 
freestyle(false), odom_callback_timer(60), vision_callback_timer(500)
{
    ROS_INFO("Jet: loading parameters");
    nh.param<int>("spin_rate", spin_rate, 50);

    nh.param<bool>("enable_serial", enable_serial, false); 
    nh.param<std::string>("serial_port", serial_port, "/dev/ttyTHS2"); 
    nh.param<int>("serial_baudrate", serial_baudrate, 115200);
    nh.param<bool>("use_guidance", use_guidance, false);

    // load parameters
    load_pid_param(nh);
    load_flight_param(nh);
    load_dropoint_param(nh);
    load_duration_param(nh);
    load_timeout_param(nh);

    if (enable_serial)
    {
        int ret = uart_open(&uart_fd, serial_port.c_str(), serial_baudrate, UART_OFLAG_WR);
        if (ret < 0) {
            fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
                    , serial_port.c_str());
        }
    }

    ROS_INFO("Jet: creating drone instance");
    // initialize dji drone
    ros::NodeHandle n;
    drone = new DJIDrone(n);

    ROS_INFO("Jet: initilaizing sevices");
    charge_srv = nh.advertiseService("/jet/charge", &Jet::charge_callback, this);
    cmd_grabber_srv = nh.advertiseService("/jet/grabber/cmd", &Jet::cmd_grabber_callback, this);
    stat_grabber_srv = nh.advertiseService("/jet/grabber/stat", &Jet::stat_grabber_callback, this);

    reload_pid_param_srv = nh.advertiseService("/jet/reload_pid_param", &Jet::reload_pid_param_callback, this);
    reload_flight_param_srv = nh.advertiseService("/jet/reload_flight_param", &Jet::reload_flight_param_callback, this);
    reload_dropoint_param_srv = nh.advertiseService("/jet/reload_dropoint_param", &Jet::reload_dropoint_param_callback, this);
    reload_duration_param_srv = nh.advertiseService("/jet/reload_duration_param", &Jet::reload_duration_param_callback, this);

    ROS_INFO("Jet: initilaizing subscribers");
    odometry_sub = nh.subscribe("/jet/odom_in", 10, &Jet::odometry_callback, this);
    vision_sub = nh.subscribe("/vision/target_pose", 10, &Jet::vision_callback, this);

    jet_state_pub = nh.advertise<std_msgs::UInt8>("/jet/state", 10);
    pose_calied_pub = nh.advertise<geometry_msgs::PoseStamped>("/jet/pose_calied", 10);

    ROS_INFO("Jet: initilaizing action servers");
    jet_nav_action_server = new JetNavActionServer(nh,
            "/jet/nav_action",
            boost::bind(&Jet::jet_nav_action_callback, this, _1), false);
    jet_nav_action_server->start();

    ROS_INFO("Jet: initilaizition done");

}

Jet::~Jet()
{
    delete drone;
    delete jet_nav_action_server;
    if (uart_fd != -1)
    {
        close(uart_fd);
        uart_fd = -1;
    }
}

void Jet::load_dropoint_param(ros::NodeHandle& nh)
{
    nh.param<double>("/jet/dropoint/x", dropoint.x, 0.0);
    nh.param<double>("/jet/dropoint/y", dropoint.y, 0.0);
    nh.param<double>("/jet/dropoint/z", dropoint.z, 0.0);

    std::cout << "/jet/dropoint: [" << dropoint.x << ", " 
              << dropoint.y << ", " << dropoint.z << "]" << std::endl;
}

void Jet::fill_pid_param(ros::NodeHandle& nh, int i, const char* axis)
{
    std::stringstream ss;
    ss << "/jet/pid/" << axis << "/";
    std::string root = ss.str();
    nh.param<float>(root + "kp", pid[i].kp, 0);
    nh.param<float>(root + "ki", pid[i].ki, 0);
    nh.param<float>(root + "kd", pid[i].kd, 0);
    nh.param<float>(root + "db", pid[i].db, 0);
    nh.param<float>(root + "it", pid[i].it, 0);
    nh.param<float>(root + "Emax", pid[i].Emax, 0);
    nh.param<float>(root + "Pmax", pid[i].Pmax, 0);
    nh.param<float>(root + "Imax", pid[i].Imax, 0);
    nh.param<float>(root + "Dmax", pid[i].Dmax, 0);
    nh.param<float>(root + "Omax", pid[i].Omax, 0);

    std::cout << "/jet/pid " << axis << " : { kp: " << pid[i].kp 
              << ", ki: " << pid[i].ki << ", kd: "<< pid[i].kd
              << ", db: " << pid[i].db << ", it: "<< pid[i].it
              << ", Emax: " << pid[i].Emax << ", Pmax: "<< pid[i].Pmax
              << ", Imax: " << pid[i].Imax << ", Dmax: "<< pid[i].Dmax
              << ", Omax: " << pid[i].Omax << "}" << std::endl;
}

void Jet::load_pid_param(ros::NodeHandle& nh)
{
    fill_pid_param(nh, 0, "xy");
    fill_pid_param(nh, 1, "xy");
    fill_pid_param(nh, 2, "z");
    fill_pid_param(nh, 3, "yaw");
}

void Jet::load_flight_param(ros::NodeHandle& nh)
{
    nh.param<float>("/jet/takeoff_height", takeoff_height, 1.2);
    nh.param<float>("/jet/landing_height", landing_height, 0.3);
    nh.param<float>("/jet/normal_altitude", normal_altitude, 1.2);
    std::cout << "/jet/takeoff_height: " << takeoff_height << std::endl;
    std::cout << "/jet/landing_height: " << landing_height << std::endl;
    std::cout << "/jet/normal_altitude: " << normal_altitude << std::endl;
}

void Jet::load_duration_param(ros::NodeHandle& nh)
{
    nh.getParam("duration", duration);
    std::cout << "durations: [" ;
    for (int i = 0; i < duration.size() - 1; i++)
    {
        std::cout << duration[i] << ", " ;
    }
    std::cout << duration[duration.size() - 1] << "]" << std::endl;
}

void Jet::load_timeout_param(ros::NodeHandle& nh)
{
    nh.param<int>("/jet/odom_callback_timeout", odom_callback_timeout, 60);
    nh.param<int>("/jet/vision_callback_timeout", vision_callback_timeout, 500);
    std::cout << "/jet/odom_callback_timeout: " << odom_callback_timeout << std::endl;
    std::cout << "/jet/vision_callback_timeout: " << vision_callback_timeout << std::endl;
}

void Jet::odometry_callback(const nav_msgs::OdometryConstPtr& odometry)
{
    jet_pos_raw[0] = odometry->pose.pose.position.x;
    jet_pos_raw[1] = odometry->pose.pose.position.y;
    jet_pos_raw[2] = odometry->pose.pose.position.z;
    jet_pos_raw[3] = tf::getYaw(odometry->pose.pose.orientation);

    if (use_guidance) // revert z if use guidance
    {
        jet_pos_raw[2] = -jet_pos_raw[2];
    }

    if (calied == false)
    {
        for (int i = 0; i < 3; i++)
        {
            jet_pos_bias[i] = jet_pos_raw[i];
        }
        jet_pos_bias[3] = 0; // yaw bias should be zero to infer the direction of jet
        calied = true;
    }

    calc_jet_pos_calied();
    pub_pose_calied();
    broadcast_tf_world2odom();
    broadcast_tf_odom2baselink();

    odom_callback_timer.reset(odom_callback_timeout);
}

void Jet::calc_jet_pos_calied()
{
    for (int i = 0; i < 4; i++)
    {
        jet_pos_calied[i] = jet_pos_raw[i] - jet_pos_bias[i];
    }
}

void Jet::pub_pose_calied()
{
    pose_calied.header.stamp = ros::Time::now();
    pose_calied.header.frame_id = "pose_calied";
    pose_calied.pose.position.x = jet_pos_calied[0];
    pose_calied.pose.position.y = jet_pos_calied[1];
    pose_calied.pose.position.z = jet_pos_calied[2];
    pose_calied.pose.orientation = tf::createQuaternionMsgFromYaw(jet_pos_calied[3]);
    pose_calied_pub.publish(pose_calied);
}

void Jet::pub_jet_state()
{
    std_msgs::UInt8 msg;
    msg.data = jet_state;
    jet_state_pub.publish(msg);
}

void Jet::broadcast_tf_world2odom()
{
    tf::Quaternion Rq = tf::createQuaternionFromYaw(jet_pos_bias[3]);
    tf::Vector3 Tv(jet_pos_bias[0], jet_pos_bias[1], jet_pos_bias[2]);
    tf::Transform transform(Rq, Tv);
    tf::StampedTransform stampedTransform(transform, ros::Time::now(), "world", "odom");
    world2odom_tb.sendTransform(stampedTransform);
}

void Jet::broadcast_tf_odom2baselink()
{
    tf::Quaternion Rq = tf::createQuaternionFromYaw(jet_pos_calied[3]);
    tf::Vector3 Tv(jet_pos_calied[0], jet_pos_calied[1], jet_pos_calied[2]);
    tf::Transform transform(Rq, Tv);
    tf::StampedTransform stampedTransform(transform, ros::Time::now(), "odom", "base_link");
    odom2baselink_tb.sendTransform(stampedTransform);
}

void Jet::tf(const float* local, float* global)
{
    double alpha = atan(local[0] / local[1]);
    double beta = alpha - jet_pos_calied[3];
    double r = sqrt(local[0] * local[0] + local[1] * local[1]);
    double x = r * sin(beta);
    double y = r * cos(beta);

    global[0] = x + jet_pos_calied[0];
    global[1] = y + jet_pos_calied[1];
    global[2] = jet_pos_calied[2] - local[2];
    global[3] = jet_pos_calied[3] + local[3];
    
    //for (int i = 0; i < 4; i++)
    //{
    //    std::cout << "Jet::tf global[" << i << "]: " << global[i] << std::endl;
    //}
}

void Jet::vision_callback(const geometry_msgs::PoseStamped& pose_stamped)
{
    vision_target_local_pos[0] = pose_stamped.pose.position.x;
    vision_target_local_pos[1] = pose_stamped.pose.position.y;
    vision_target_local_pos[2] = pose_stamped.pose.position.z;
    vision_target_local_pos[3] = tf::getYaw(pose_stamped.pose.orientation);

    // tf
    tf(vision_target_local_pos, vision_target_global_pos);

    vision_callback_timer.reset(vision_callback_timeout);
}

bool Jet::charge_callback(jet::Charge::Request& request, jet::Charge::Response& response)
{
    response.result = status();
    if (jet_state == STAND_BY) // Accept request ONLY when jet is standing by 
    {
        freestyle = true;
    }
    return true;
}

bool Jet::cmd_grabber_callback(jet::CmdGrabber::Request& request, jet::CmdGrabber::Response& response)
{
    response.result = cmd_grabber(request.cmd);
    return true;
}

bool Jet::stat_grabber_callback(jet::StatGrabber::Request& request, jet::StatGrabber::Response& response)
{
    response.result = stat_grabber();
    return true;
}

bool Jet::reload_pid_param_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    load_pid_param(nh);
    return true;
}

bool Jet::reload_dropoint_param_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    load_dropoint_param(nh);
    return true;
}

bool Jet::reload_duration_param_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    load_duration_param(nh);
    return true;
}

bool Jet::reload_flight_param_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    load_flight_param(nh);
    return true;
}

bool Jet::reload_timeout_param_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    load_timeout_param(nh);
    return true;
}

bool Jet::cmd_grabber(uint8_t c)
{
    if (!enable_serial)
    {
        return true;
    }
    if (uart_fd == -1)
    {
        return false;
    }
    uint8_t buf[4];
    buf[0] = 0xa5;
    buf[1] = c;
    buf[2] = 0xfe;
    CRC8Append(buf, 4, 0xff);
    write(uart_fd, buf, 4);
    return true;
}

uint8_t Jet::stat_grabber()
{
    if (!enable_serial)
    {
        return 0xff;
    }
    if (uart_fd == -1)
    {
        return 0xff;
    }
    uint8_t buf[4];
    memset(buf, 0, 4);
    read(uart_fd, buf, 4);
    if (buf[0] == 0xa5 && buf[2] == 0xfe && CRC8Check(buf, 4, 0xff))
    {
        return buf[1];
    }
    return 0xff;
}

bool Jet::control(uint8_t ground, float x, float y, float z, float yaw)
{
    uint8_t flag = Flight::HorizontalLogic::HORIZONTAL_VELOCITY |
                        Flight::VerticalLogic::VERTICAL_VELOCITY |
                        Flight::YawLogic::YAW_RATE |
                        Flight::SmoothMode::SMOOTH_ENABLE;
    if (ground)
    {
        // WORLD
        flag |= Flight::HorizontalCoordinate::HORIZONTAL_GROUND;
    } else {
        // BODY
        flag |= Flight::HorizontalCoordinate::HORIZONTAL_BODY;
    }

    return drone->attitude_control(flag, x, y, z, yaw);
}

bool Jet::goal_reached()
{
    for (int i = 0; i < 4; i++)
    {
        if (pid[i].out != 0)
            return false;
    }
    return true;
}

bool Jet::pid_control(uint8_t ground, float x, float y, float z, float yaw)
{
    if (ground && odom_callback_timer.timeout())
    {
        return false;
    }

    if ((!ground) && vision_callback_timer.timeout())
    {
        return false;
    }

    float ref[4];
    float fdb[4];
    float out[4];

    if (ground)
    {
        for (int i = 0; i < 4; i++)
        {
            fdb[i] = jet_pos_calied[i];
        }
    }
    else
    {
        for (int i = 0; i < 4; i++)
        {
            fdb[i] = 0;
        }
    }
    
    ref[0] = fdb[0] + x;
    ref[1] = fdb[1] + y;
    ref[2] = fdb[2] + z;
    ref[3] = fdb[3] + yaw;

    for (int i = 0 ; i < 4; i++)
    {
        out[i] = PID_Calc(&pid[i], ref[i], fdb[i]);
    }

    control(ground, out[0], out[1], out[2], out[3]);

    std::cout << "+------------------------- control loop -------------------------+" << std::endl;

    for (int i = 0; i < 4; i++)
    {
        //std::cout << std::fixed << std::setprecision(3) << ref[i] << "\t";
        printf("%.3f\t", ref[i]);
    }

    std::cout << std::endl;

    for (int i = 0; i < 4; i++)
    {
        //std::cout << std::fixed << std::setprecision(3) << fdb[i] << "\t";
        printf("%.3f\t", fdb[i]);
    }

    std::cout << std::endl;

    for (int i = 0; i < 4; i++)
    {
        //std::cout << std::fixed << std::setprecision(3) << out[i] << "\t";
        printf("%.3f\t", out[i]);
    }

    std::cout << std::endl;

    std::cout << "+----------------------------------------------------------------+" << std::endl;
    return goal_reached();
}

bool Jet::jet_nav_action_callback(const jet::JetNavGoalConstPtr& goal)
{
    float dst[4];
    float org[4];
    float dis[4];

    dst[0] = goal->x;
    dst[1] = goal->y;
    dst[2] = goal->z;
    dst[3] = goal->yaw;

    for (int i = 0; i < 4; i++)
    {
        org[i] = jet_pos_calied[i];
        dis[i] = dst[i] - org[i];
    }

    int progress[4] = { 0, 0, 0, 0 };

    float err[4];
    float det[4];

    while (!goal_reached()) {

        for (int i = 0; i < 4; i++)
        {
            err[i] = dst[i] - jet_pos_calied[i];
            det[i] = (100 * err[i]) / dis[i];
            progress[i] = 100 - (int)det[i];
        }

        pid_control(1, err[0], err[1], err[2], err[3]);

        //lazy evaluation
        for (int i = 0; i < 4; i++)
        {
            if (pid[i].out == 0) progress[i] = 100;
        }

        jet_nav_feedback.x_progress = progress[0];
        jet_nav_feedback.y_progress = progress[1];
        jet_nav_feedback.z_progress = progress[2];
        jet_nav_feedback.yaw_progress = progress[3];

        jet_nav_action_server->publishFeedback(jet_nav_feedback);

        usleep(20000); // 50hz
    }

    jet_nav_result.result = true;
    jet_nav_action_server->setSucceeded(jet_nav_result);

    return true;
}

uint8_t Jet::status()
{
    return drone->flight_status;
}

bool Jet::doStandby()
{
    return true;
}

bool Jet::doGrabBullets()
{
    return cmd_grabber(1);
}

bool Jet::doRequestControl()
{
    return drone->request_sdk_permission_control();
}

bool Jet::doTakeoff()
{
    return drone->takeoff();
}

bool Jet::doToNormalAltitude()
{
    float ex = 0;
    float ey = 0;
    float ez = normal_altitude - jet_pos_calied[2]; // adjust altitude
    float eyaw = 0;

    return pid_control(1, ex, ey, ez, eyaw);
}

bool Jet::doFlyToCar()
{
    float ex = dropoint.x - jet_pos_calied[0];
    float ey = dropoint.y - jet_pos_calied[1];
    float ez = 0;
    float eyaw = 0;

    return pid_control(1, ex, ey, ez, eyaw);
}

bool Jet::doFindCar()
{
    //return (!vision_callback_timer.timeout());
    return true;
}

bool Jet::doServeCar()
{
    if (vision_callback_timer.timeout())
    {
        std::cout << "WARNING@Jet::doServeCar(): vision callback timeout, control is stopped" << std::endl;
        return false;
    }

    float ex = vision_target_local_pos[0];
    float ey = vision_target_local_pos[1];
    float ez = dropoint.z - vision_target_local_pos[2];
    float eyaw = 0;

    return pid_control(0, ex, ey, ez, eyaw); // body
}

bool Jet::doDropBullets()
{
    return cmd_grabber(0);
}

bool Jet::doBackToNormalAltitude()
{
    float ex = 0;
    float ey = 0;
    float ez = normal_altitude - jet_pos_calied[2]; // normal altitude
    float eyaw = 0;

    return pid_control(1, ex, ey, ez, eyaw);
}

bool Jet::doFlyBack()
{
    float ex = 0 - jet_pos_calied[0];
    float ey = 0 - jet_pos_calied[1];
    float ez = 0;
    float eyaw = 0;

    return pid_control(1, ex, ey, ez, eyaw);
}

bool Jet::doFindPark()
{
    //return (!vision_callback_timer.timeout());
    return true;
}

bool Jet::doVisualServoLanding()
{
    if (vision_callback_timer.timeout())
    {
        std::cout << "WARNING@Jet::doVisualServoLanding(): vision callback timeout, control is stopped" << std::endl;
        return false;
    }

    float ex = vision_target_local_pos[0];
    float ey = vision_target_local_pos[1];
    float ez = landing_height - vision_target_local_pos[2];
    float eyaw = 0;

    return pid_control(0, ex, ey, ez, eyaw); // body frame
}

bool Jet::doLanding()
{
    return drone->landing();
}

bool Jet::doReleaseControl()
{
    return drone->release_sdk_permission_control();
}

bool Jet::cmd_jet_state(uint8_t cmd)
{
    if (cmd <= RELEASE_CONTROL)
    {
        jet_state = cmd;
    }
}

bool Jet::action(uint8_t cmd)
{
    cmd_jet_state(cmd);
    switch (cmd)
    {
        case STAND_BY:
        std::cout << "\nAction: " << "Standby" << std::endl;
        return doStandby();

        case GRAB_BULLETS:
        std::cout << "\nAction: " << "Grab Bullets" << std::endl;
        return doGrabBullets();

        case REQUEST_CONTROL:
        std::cout << "\nAction: " << "Request Control" << std::endl;
        return doRequestControl();

        case TAKE_OFF:
        std::cout << "\nAction: " << "Takeoff" << std::endl;
        return doTakeoff();

        case TO_NORMAL_ALTITUDE:
        std::cout << "\nAction: " << "To Normal Altitude" << std::endl;
        return doToNormalAltitude();

        case FLY_TO_CAR:
        std::cout << "\nAction: " << "Fly to Car" << std::endl;
        return doFlyToCar();

        case FIND_CAR:
        std::cout << "\nAction: " << "Find Car" << std::endl;
        return doFindCar();

        case SERVE_CAR:
        std::cout << "\nAction: " << "Serve Car" << std::endl;
        return doServeCar();

        case DROP_BULLETS:
        std::cout << "\nAction: " << "Drop Bullets" << std::endl;
        return doDropBullets();

        case BACK_TO_NORMAL_ALTITUDE:
        std::cout << "\nAction: " << "Back to Normal Altitude" << std::endl;
        return doBackToNormalAltitude();

        case FLY_BACK:
        std::cout << "\nAction: " << "Fly Back" << std::endl;
        return doFlyBack();

        case FIND_PARK:
        std::cout << "\nAction: " << "Find Park" << std::endl;
        return doFindPark();

        case VISUAL_SERVO_LANDING:
        std::cout << "\nAction: " << "Visual Servo Landing" << std::endl;
        return doVisualServoLanding();

        case LANDING:
        std::cout << "\nAction: " << "Landing" << std::endl;
        return doLanding();

        case RELEASE_CONTROL:
        std::cout << "\nAction: " << "Release Control" << std::endl;
        return doReleaseControl();

        default:
        return false;
    }
}

void Jet::stateMachine()
{
    static bool success = false;
    static uint32_t tick = 0;
    switch (jet_state)
    {
        case STAND_BY:
        if (!success)
        {
            success = doStandby();
            std::cout << "stateMachine: " << "Standby" << std::endl;
        }
        else if (tick < duration[STAND_BY])
        {
            tick++;
            std::cout << "stateMachine: " << "Standby@Tick: " << tick << std::endl;
        }
        else
        {
            tick = 0;
            success = false;
            jet_state = GRAB_BULLETS;
            std::cout << "stateMachine: " << "Standby->Grab Bullets" << tick << std::endl;
        }
        break;

        case GRAB_BULLETS:
        if (!success)
        {
            success = doGrabBullets();
            std::cout << "stateMachine: " << "Grab Bullets" << std::endl;
        }
        else if (tick < duration[GRAB_BULLETS])
        {
            tick++;
            std::cout << "stateMachine: " << "Grab Bullets@Tick: " << tick << std::endl;
        }
        else
        {
            tick = 0;
            success = false;
            jet_state = REQUEST_CONTROL;
            std::cout << "stateMachine: " << "Grab Bullets->Request Control" << tick << std::endl;
        }
        break;

        case REQUEST_CONTROL:
        if (!success)
        {
            success = doRequestControl();
            std::cout << "stateMachine: " << "Request Control" << std::endl;
        }
        else if (tick < duration[REQUEST_CONTROL])
        {
            tick++;
            std::cout << "stateMachine: " << "Request Control@Tick: " << tick << std::endl;
        }
        else
        {
            tick = 0;
            success = false;
            jet_state = TAKE_OFF;
            std::cout << "stateMachine: " << "Request Control->Takeoff" << tick << std::endl;
        }
        break;

        case TAKE_OFF:
        if (!success)
        {
            success = doTakeoff();
            std::cout << "stateMachine: " << "Takeoff" << std::endl;
        }
        else if (tick < duration[TAKE_OFF])
        {
            tick++;
            std::cout << "stateMachine: " << "Takeoff@Tick: " << tick << std::endl;
        }
        else
        {
            tick = 0;
            success = false;
            jet_state = TO_NORMAL_ALTITUDE;
            std::cout << "stateMachine: " << "Takeoff->To Normal Altitude" << tick << std::endl;
        }
        break;

        case TO_NORMAL_ALTITUDE:
        if (!success)
        {
            success = doToNormalAltitude();
            std::cout << "stateMachine: " << "To Normal Altitude" << std::endl;
        }
        else if (tick < duration[TO_NORMAL_ALTITUDE])
        {
            tick++;
            std::cout << "stateMachine: " << "To Normal Altitude@Tick: " << tick << std::endl;
        }
        else
        {
            tick = 0;
            success = false;
            jet_state = FLY_TO_CAR;
            std::cout << "stateMachine: " << "To Normal Altitude->Fly to Car" << tick << std::endl;
        }
        break;

        case FLY_TO_CAR:
        if (!success)
        {
            success = doFlyToCar();
            std::cout << "stateMachine: " << "Fly to Car" << std::endl;
        }
        else if (tick < duration[FLY_TO_CAR])
        {
            tick++;
            std::cout << "stateMachine: " << "Fly to Car@Tick: " << tick << std::endl;
        }
        else
        {
            tick = 0;
            success = false;
            jet_state = FIND_CAR;
            std::cout << "stateMachine: " << "Fly to Car->Find Car" << tick << std::endl;
        }
        break;

        case FIND_CAR:
        if (!success)
        {
            success = doFindCar();
            std::cout << "stateMachine: " << "Find Car" << std::endl;
        }
        else if (tick < duration[FIND_CAR])
        {
            tick++;
            std::cout << "stateMachine: " << "Find Car@Tick: " << tick << std::endl;
        }
        else
        {
            tick = 0;
            success = false;
            jet_state = SERVE_CAR;
            std::cout << "stateMachine: " << "Find Car->Serve Car" << tick << std::endl;
        }
        break;

        case SERVE_CAR:
        if (!success)
        {
            success = doServeCar();
            std::cout << "stateMachine: " << "Serve Car" << std::endl;
        }
        else if (tick < duration[SERVE_CAR])
        {
            tick++;
            std::cout << "stateMachine: " << "Serve Car@Tick: " << tick << std::endl;
        }
        else
        {
            tick = 0;
            success = false;
            jet_state = DROP_BULLETS;
            std::cout << "stateMachine: " << "Serve Car->Drop Bullets" << tick << std::endl;
        }
        break;

        case DROP_BULLETS:
        if (!success)
        {
            success = doDropBullets();
            std::cout << "stateMachine: " << "Drop Bullets" << std::endl;
        }
        else if (tick < duration[DROP_BULLETS])
        {
            tick++;
            std::cout << "stateMachine: " << "Drop Bullets@Tick: " << tick << std::endl;
        }
        else
        {
            tick = 0;
            success = false;
            jet_state = BACK_TO_NORMAL_ALTITUDE;
            std::cout << "stateMachine: " << "Drop Bullets->Back to Normal Altitude" << tick << std::endl;
        }
        break;

        case BACK_TO_NORMAL_ALTITUDE:
        if (!success)
        {
            success = doBackToNormalAltitude();
            std::cout << "stateMachine: " << "Back to Normal Altitude" << std::endl;
        }
        else if (tick < duration[BACK_TO_NORMAL_ALTITUDE])
        {
            tick++;
            std::cout << "stateMachine: " << "Back to Normal Altitude@Tick: " << tick << std::endl;
        }
        else
        {
            tick = 0;
            success = false;
            jet_state = FLY_BACK;
            std::cout << "stateMachine: " << "Back to Normal Altitude->Fly Back" << tick << std::endl;
        }
        break;

        case FLY_BACK:
        if (!success)
        {
            success = doFlyBack();
            std::cout << "stateMachine: " << "Fly Back" << std::endl;
        }
        else if (tick < duration[FLY_BACK])
        {
            tick++;
            std::cout << "stateMachine: " << "Fly Back@Tick: " << tick << std::endl;
        }
        else
        {
            tick = 0;
            success = false;
            jet_state = FIND_PARK;
            std::cout << "stateMachine: " << "Fly Back->Find Park" << tick << std::endl;
        }
        break;

        case FIND_PARK:
        if (!success)
        {
            success = doFindPark();
            std::cout << "stateMachine: " << "Find Park" << std::endl;
        }
        else if (tick < duration[FIND_PARK])
        {
            tick++;
            std::cout << "stateMachine: " << "Find Park@Tick: " << tick << std::endl;
        }
        else
        {
            tick = 0;
            success = false;
            jet_state = VISUAL_SERVO_LANDING;
            std::cout << "stateMachine: " << "Find Park->Visual Servo Landing" << tick << std::endl;
        }
        break;

        case VISUAL_SERVO_LANDING:
        if (!success)
        {
            success = doVisualServoLanding();
            std::cout << "stateMachine: " << "Visual Servo Landing" << std::endl;
        }
        else if (tick < duration[VISUAL_SERVO_LANDING])
        {
            tick++;
            std::cout << "stateMachine: " << "Visual Servo Landing@Tick: " << tick << std::endl;
        }
        else
        {
            tick = 0;
            success = false;
            jet_state = LANDING;
            std::cout << "stateMachine: " << "Visual Servo Landing->Landing" << tick << std::endl;
        }
        break;

        case LANDING:
        if (!success)
        {
            success = doLanding();
            std::cout << "stateMachine: " << "Landing" << std::endl;
        }
        else if (tick < duration[LANDING])
        {
            tick++;
            std::cout << "stateMachine: " << "Landing@Tick: " << tick << std::endl;
        }
        else
        {
            tick = 0;
            success = false;
            jet_state = RELEASE_CONTROL;
            std::cout << "stateMachine: " << "Landing->Standby" << tick << std::endl;

            calied = false; // re-calibrate odom
        }
        break;

        case RELEASE_CONTROL:
        if (!success)
        {
            success = doReleaseControl();
            std::cout << "stateMachine: " << "Release Control" << std::endl;
        }
        else if (tick < duration[RELEASE_CONTROL])
        {
            tick++;
            std::cout << "stateMachine: " << "Release Control@Tick: " << tick << std::endl;
        }
        else
        {
            tick = 0;
            success = false;
            jet_state = STAND_BY;
            std::cout << "stateMachine: " << "Release Control->Standby" << tick << std::endl;
            if (freestyle)
            {
                freestyle = false; // clear freestyle flag
                std::cout << "+---------------------Jetbang Free Style Done----------------------+" << std::endl;
                help();
            }
        }
        break;

        default:
        jet_state = STAND_BY;
        break;
    }
}

void Jet::help()
{
    printf("\r\n");
    printf("+-------------------------- < Main menu > -------------------------+\n");
    printf("| [0]  Stand-by                 | [1]  Grab Bullets                |\n");
	printf("| [2]  Request Control          | [3]  Takeoff                     |\n");
    printf("| [4]  To Normal Altitude       | [5]  Fly to Car                  |\n");
    printf("| [6]  Find Car                 | [7]  Serve Car                   |\n");
	printf("| [8]  Drop bullets             | [9]  Back to Normal Altitude     |\n");	
	printf("| [a]  Fly Back                 | [b]  Find Park                   |\n");	
    printf("| [c]  Visual Servo Landing     | [d]  Landing                     |\n");	
    printf("| [e]  Release Control          | [f]  Jetbang Free Style          |\n");	
    printf("| [g]  Pause Free Style         | [h]  Resume Free Style           |\n");
    printf("| [i]  Cutoff Free Style        | [j]  Help                        |\n");	
    printf("+------------------------------------------------------------------+\n");
}

void Jet::spin()
{
    help();

    ros::Rate rate(spin_rate);

    while (ros::ok())
    {
        ros::spinOnce();

        char c = kbhit();

        if (c == 'f')
        {
            std::cout << "\nAction: " << "Jetbang Free Style" << std::endl;
            freestyle = true;
            jet_state = STAND_BY;
        }
        if (c == 'g')
        {
            std::cout << "\nAction: " << "Pause Free Style" << std::endl;
            freestyle = false;
        }
        if (c == 'h')
        {
            std::cout << "\nAction: " << "Resume Free Style" << std::endl;
            freestyle = true;
        }
        if (c == 'i')
        {
            std::cout << "\nAction: " << "Cutoff Free Style" << std::endl;
            freestyle = false;
            jet_state = STAND_BY;
        }

        if (freestyle)
        {
            std::cout << "Jetbang Free Styling" << std::endl;
            stateMachine();
        }

        if (c >= '0' && c <= '9')
        {
            action(c - '0');
        }
        if (c >= 'a' && c <= 'e')
        {
            action(c - 'a' + 10);
        }
        if (c == 'j')
        {
            help();
        }

        pub_jet_state();

        rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "jet");

    Jet jet;

    jet.spin();

    return 0;

}

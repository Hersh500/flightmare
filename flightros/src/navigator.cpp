/**
 * @file navigator.cpp
 * @brief Accepts navigation commands and publishes commands to UAV
 */

#include <flightros/navigator.hpp>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <algorithm>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
#include <vector>
#include <cmath>
#include <ctgmath>
#include <random>
#include <iostream>

/* Resets for Forest */
/*
#define RESET_POS_X 62.0
#define RESET_POS_Y 90.0
#define RESET_POS_Z 33.0
*/

/* Resets for Warehouse box  */
#define RESET_POS_X -9.0
#define RESET_POS_Y 15.0
#define RESET_POS_Z 2.0

#define RESET_REQ_DURATION 2.0
#define POS_STD 1.0

#define TIME_HORIZON 0.5
#define GOAL_POSITION_Y 20.0
#define X_THRESHOLD 20.0
#define Z_THRESHOLD 1.5
#define PI 3.14159
#define MAX_HEADING PI/4.0 // +/- PI/4 = +/- 45 deg
#define MAX_VELOCITY 3.0
#define RUNNING_Z_PGAIN 0.5
#define YAW_PGAIN 0.1
#define YAW_PGAIN_RESET 1.0

Navigator::Navigator(ros::NodeHandle* nh, double freq) : _nh(*nh), 
                                                         _rate(freq){

    initializePublishers();
    initializeSubscribers();
    initializeServices();

    // command variables
    _cmd_heading = 0.0;
    _cmd_velocity = 0.0;

    _last_cmd = ros::Time::now();
    _not_reset = ros::Time::now();

    _goal_pos.x = RESET_POS_X;
    _goal_pos.y = RESET_POS_Y + GOAL_POSITION_Y;
    _goal_pos.z = RESET_POS_Z;

    _in_collision.data = false;
}

/* ---------------------------------- */
/*      INITIALIZATION FUNCTIONS      */
/* ---------------------------------- */

void Navigator::initializeSubscribers(){
    _odom_sub = _nh.subscribe
        ("hummingbird/ground_truth/odometry", 10, &Navigator::_odom_cb, this);
    _camera_sub = _nh.subscribe
        ("hummingbird/camera/image", 10, &Navigator::_camera_cb, this);
    _depth_sub = _nh.subscribe
        ("hummingbird/camera/noisy_depth", 10, &Navigator::_depth_cb, this);
    _uncertainty_sub = _nh.subscribe
        ("hummingbird/camera/uncertainty", 10, &Navigator::_uncertainty_cb, this);
    _collision_sub = _nh.subscribe
        ("hummingbird/collision", 10, &Navigator::_collision_cb, this);
}

void Navigator::initializePublishers(){
    _cmd_pub = _nh.advertise<geometry_msgs::TwistStamped>
        ("hummingbird/autopilot/velocity_command", 10);
}

void Navigator::initializeServices(){
    _quadstate_server = _nh.advertiseService("navigator/get_state", &Navigator::_quadstate_cb, this);
    // _client = _nh.serviceClient<flightros::QuadState>("learner/get_state");
}

/* ---------------------------------- */
/*         CALLBACK FUNCTIONS         */
/* ---------------------------------- */

bool Navigator::_quadstate_cb(flightros::QuadState::Request &req,
                   flightros::QuadState::Response &res){



    std::string::size_type sz;
    _cmd_heading = std::stof(req.in.substr(2),&sz) * MAX_HEADING;
    _cmd_velocity = (std::stof(req.in.substr(2).substr(sz)) + 1.)/2. * MAX_VELOCITY;
    _last_cmd = ros::Time::now();

    // ROS_INFO_STREAM(_cmd_heading);
    // ROS_INFO_STREAM(_cmd_velocity);
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);
    std::normal_distribution<float> distribution(0.0, POS_STD);
    x_mod = distribution(generator);
    // y_mod = distribution(generator);
    y_mod = 0.0;

    if (req.in[0] == '0'){
        ROS_WARN("QuadState service: Resetting simulation");

        x_mod = distribution(generator);
        y_mod = distribution(generator);
        // blocking operation: move drone to reset position
        while ( true ){

            // check is ROS is shutting down since this callback is blocking
            if (ros::isShuttingDown()){
                ros::shutdown();
            }

            geometry_msgs::TwistStamped vel_msg;
            vel_msg.header.stamp = ros::Time::now();
            // Reset to a random position
            
            vel_msg.twist.linear.x = std::clamp( 1.0 * (RESET_POS_X - _curr_pos.x - x_mod), -10.0, 10.0 );
            vel_msg.twist.linear.y = std::clamp( 1.0 * (RESET_POS_Y - _curr_pos.y - y_mod), -10.0, 10.0 );
            vel_msg.twist.linear.z = std::clamp( 1.0 * (RESET_POS_Z - _curr_pos.z), -10.0, 10.0 );
            vel_msg.twist.angular.x = 0;
            vel_msg.twist.angular.y = 0;
            vel_msg.twist.angular.z = YAW_PGAIN_RESET * (0 - _yaw);
            _cmd_pub.publish(vel_msg);
            // ROS_INFO_STREAM("Resetting drone position");

            if ((std::pow(_curr_pos.x + x_mod -RESET_POS_X, 2) + std::pow(_curr_pos.y + y_mod -RESET_POS_Y, 2) + std::pow(_curr_pos.z-RESET_POS_Z, 2)) > 1.0){
                _not_reset = ros::Time::now();
            }
            else if (ros::Time::now() - _not_reset > ros::Duration(RESET_REQ_DURATION)){
                ROS_INFO_STREAM("Drone is reset");                
                break;
            }

            // spin
            ros::spinOnce();
            _rate.sleep();

        }

        // fill relevant fields
        res.header = _rgb.header;

        std_msgs::Bool done;
        done.data = false;
        res.done = done;
        std_msgs::Bool crash_msg;
        crash_msg.data = false;
        res.crash = crash_msg;

        cv_bridge::CvImagePtr cv_ptr_rgb, cv_ptr_depth;
        try{
            cv_ptr_rgb = cv_bridge::toCvCopy(_rgb, sensor_msgs::image_encodings::BGR8);
            cv_ptr_depth = cv_bridge::toCvCopy(_depth, sensor_msgs::image_encodings::MONO8);

            // split & merge attempt
            cv::Mat rgb_channels[3];
            cv::split(cv_ptr_rgb->image, rgb_channels);
            cv::Mat rgbd_channels[4] = {rgb_channels[0],
                                        rgb_channels[1],
                                        rgb_channels[2],
                                        cv_ptr_depth->image};
            std::vector<int> sizes{cv_ptr_rgb->image.rows, cv_ptr_rgb->image.cols};
            cv::Mat outMat(sizes, CV_32FC4);
            cv::merge(rgbd_channels, 4, outMat);
            // outMat = outMat * (1.0f/255.0f);
            sensor_msgs::ImagePtr rgbd_msg = cv_bridge::CvImage(_rgb.header, "8UC4", outMat).toImageMsg();
            res.image = *rgbd_msg;

            // Also set the uncertainty here
            res.prob_image = _uncertainty;

            // mixChannels attempt
            // std::vector<int> sizes{cv_ptr_rgb->image.rows, cv_ptr_rgb->image.cols};
            // cv::Mat outMat(sizes, CV_32FC4);
            // const cv::Mat inMat[] = {cv_ptr_rgb->image, cv_ptr_depth->image};
            // const int from_to[] = {0,0, 1,1, 2,2, 3,3};
            // cv::mixChannels(inMat, 2, &outMat, 1, from_to, 4);
            // sensor_msgs::ImagePtr rgbd_msg = cv_bridge::CvImage(_rgb.header, "32FC4", outMat).toImageMsg();
            // res.image = *rgbd_msg;
        }
        catch (cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            // return;
        }

        res.current_position = _curr_pos;
        res.goal_position = _goal_pos;

        _in_collision.data = false;

    }

    else if (req.in[0] == '1'){
        res.header = _rgb.header;
        std_msgs::Bool done;
        done.data = (_curr_pos.y >= _goal_pos.y);
        res.done = done;
        // collision || backwards || x too much || z too much
        std_msgs::Bool crash_msg;
        crash_msg.data = ( _in_collision.data ||
                           _curr_pos.y < (RESET_POS_Y + y_mod - 5.) ||
                           ( _curr_pos.x > (RESET_POS_X + x_mod + X_THRESHOLD) || _curr_pos.x < (RESET_POS_X + x_mod - X_THRESHOLD)) ||
                           ( _curr_pos.z < RESET_POS_Z - Z_THRESHOLD || _curr_pos.z > RESET_POS_Z + Z_THRESHOLD) );
        if (_in_collision.data) {
            ROS_INFO("Collision with environment!");
        } else if (crash_msg.data) {
            ROS_INFO("out of bounds!");
        }
        res.crash = crash_msg;
        
        cv_bridge::CvImagePtr cv_ptr_rgb, cv_ptr_depth;
        try{
            cv_ptr_rgb = cv_bridge::toCvCopy(_rgb, sensor_msgs::image_encodings::BGR8);
            cv_ptr_depth = cv_bridge::toCvCopy(_depth, sensor_msgs::image_encodings::MONO8);

            // split & merge attempt
            cv::Mat rgb_channels[3];
            cv::split(cv_ptr_rgb->image, rgb_channels);
            cv::Mat rgbd_channels[4] = {rgb_channels[0],
                                        rgb_channels[1],
                                        rgb_channels[2],
                                        cv_ptr_depth->image};

            std::vector<int> sizes{cv_ptr_rgb->image.rows, cv_ptr_rgb->image.cols};
            cv::Mat outMat(sizes, CV_32FC4);
            cv::merge(rgbd_channels, 4, outMat);
            sensor_msgs::ImagePtr rgbd_msg = cv_bridge::CvImage(_rgb.header, "8UC4", outMat).toImageMsg();
            res.image = *rgbd_msg;
            res.prob_image = _uncertainty;

            // mixChannels attempt
            // std::vector<int> sizes{cv_ptr_rgb->image.rows, cv_ptr_rgb->image.cols};
            // cv::Mat outMat(sizes, CV_32FC4);
            // const cv::Mat inMat[] = {cv_ptr_rgb->image, cv_ptr_depth->image};
            // const int from_to[] = {0,0, 1,1, 2,2, 3,3};
            // cv::mixChannels(inMat, 2, &outMat, 1, from_to, 4);
            // sensor_msgs::ImagePtr rgbd_msg = cv_bridge::CvImage(_rgb.header, "32FC4", outMat).toImageMsg();
            // res.image = *rgbd_msg;
        }
        catch (cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            // return;
        }

        res.current_position = _curr_pos;
        res.goal_position = _goal_pos;

    }

    else {
        ROS_ERROR("Get State service request value not recognized (0 or 1 supported)!");
    }

    return true;
}

void Navigator::_odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
    _odom = *msg;
    _curr_pos = _odom.pose.pose.position;
    _curr_orient = _odom.pose.pose.orientation;
    // ROS_INFO("curr_orient.wxyz = (%f, %f, %f, %f)", _curr_orient.w, _curr_orient.x, _curr_orient.y, _curr_orient.z);
    _yaw = asin(2*_curr_orient.x*_curr_orient.y + 2*_curr_orient.z*_curr_orient.w);
    /*
    if (_curr_orient.w < 0) {
        Eigen::Quaterniond q(-_curr_orient.w, -_curr_orient.x, -_curr_orient.y, -_curr_orient.z);
        auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
        _yaw = euler[2]; // map frame yaw angle
    } else {
        Eigen::Quaterniond q(_curr_orient.w, _curr_orient.x, _curr_orient.y, _curr_orient.z);
        auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
        _yaw = euler[2]; // map frame yaw angle
    }
    */
//    ROS_INFO("Yaw est is %f\n", _yaw);
}

void Navigator::_uncertainty_cb(const sensor_msgs::Image::ConstPtr& msg){
    _uncertainty = *msg;
    _uncertainty.header.stamp = ros::Time::now();
}

void Navigator::_camera_cb(const sensor_msgs::Image::ConstPtr& msg){
    _rgb = *msg;
    _rgb.header.stamp = ros::Time::now();
}

void Navigator::_depth_cb(const sensor_msgs::Image::ConstPtr& msg){
    _depth = *msg;
    _depth.header.stamp = ros::Time::now();
}

void Navigator::_collision_cb(const std_msgs::Bool::ConstPtr& msg){
    if (!(_in_collision.data))
        _in_collision = *msg;
}

/* ---------------------------------- */
/*            RUN FUNCTION            */
/* ---------------------------------- */

void Navigator::run(){

    while (ros::ok()){

        // if last message was recent enough
        if (ros::Time::now() - _last_cmd < ros::Duration(TIME_HORIZON)){
            geometry_msgs::TwistStamped vel_msg;
            vel_msg.header.stamp = ros::Time::now();
            
            // ROS_INFO("cmd_velocity is %f\n", _cmd_velocity);
            // ROS_INFO("cur_heading is %f\n", _yaw);
            // ROS_INFO("-------------------");
            vel_msg.twist.linear.x = _cmd_velocity * (-sin(_yaw));
            vel_msg.twist.linear.y = _cmd_velocity * cos(_yaw);
            vel_msg.twist.linear.z = RUNNING_Z_PGAIN * (RESET_POS_Z - _curr_pos.z);

            // assume incoming heading double is from [-PI/4, PI/4]
            double yaw_change = std::clamp(_cmd_heading, -MAX_HEADING, MAX_HEADING);
            vel_msg.twist.angular.x = 0;
            vel_msg.twist.angular.y = 0;
            if ((_yaw < -MAX_HEADING && yaw_change < 0) || (_yaw > MAX_HEADING && yaw_change > 0))
                vel_msg.twist.angular.z = 0;
            else
                vel_msg.twist.angular.z = YAW_PGAIN * yaw_change;
            _cmd_pub.publish(vel_msg);
        }

        // if not in reset mode and don't have a recent heading/vel command
        else {
            // command zero velocity
            geometry_msgs::TwistStamped vel_msg;
            vel_msg.header.stamp = ros::Time::now();
            vel_msg.twist.linear.x = 0;
            vel_msg.twist.linear.y = 0;
            vel_msg.twist.linear.z = RUNNING_Z_PGAIN * (RESET_POS_Z - _curr_pos.z);
            vel_msg.twist.angular.x = 0;
            vel_msg.twist.angular.y = 0;
            vel_msg.twist.angular.z = 0;
            _cmd_pub.publish(vel_msg);
        }

        // spin
        ros::spinOnce();
        _rate.sleep();
    }

}

/* ---------------------------------- */
/*          INSTANTIATE NODE          */
/* ---------------------------------- */

int main(int argc, char** argv){

    ros::init(argc, argv, "navigator");
    ros::NodeHandle nh;
    Navigator Navigator_Node(&nh, 30.0);
    ROS_INFO("Navigator: running");
    Navigator_Node.run();

    return 0;
}

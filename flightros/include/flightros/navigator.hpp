#ifndef NAVIGATOR_H_
#define NAVIGATOR_H_

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <flightros/QuadState.h>
#include <sensor_msgs/Image.h>

class Navigator{

    private:

        // ROS stuff
        ros::NodeHandle _nh;
        ros::Rate _rate;

        // subscribers
        ros::Subscriber _cmd_heading_sub;
        ros::Subscriber _cmd_velocity_sub;
        ros::Subscriber _odom_sub;
        ros::Subscriber _camera_sub;
        ros::Subscriber _depth_sub;
        ros::Subscriber _collision_sub;

        // publishers
        ros::Publisher _cmd_pub;

        // services
        ros::ServiceServer _quadstate_server;
        // ros::ServiceClient _client;
        // flightros::QuadState _quadstate_srv;

        // service callback functions
        bool _quadstate_cb(flightros::QuadState::Request  &req,
                           flightros::QuadState::Response &res);

        // initialization functions
        void initializeSubscribers();
        void initializePublishers();
        void initializeServices();

        // subscriber callback functions
        void _cmd_heading_cb(const std_msgs::Float32::ConstPtr& msg);
        void _cmd_velocity_cb(const std_msgs::Float32::ConstPtr& msg);
        double _cmd_heading, _cmd_velocity;
        void _odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
        nav_msgs::Odometry _odom;
        void _camera_cb(const sensor_msgs::Image::ConstPtr& msg);
        void _depth_cb(const sensor_msgs::Image::ConstPtr& msg);
        sensor_msgs::Image _rgb, _depth;
        void _collision_cb(const std_msgs::Bool::ConstPtr& msg);
        std_msgs::Bool _in_collision;

        // timing variables
        ros::Time _last_cmd;
        ros::Time _not_reset;

        // other variables
        geometry_msgs::Point _curr_pos, _goal_pos;
        geometry_msgs::Quaternion _curr_orient;

    public:

        Navigator(ros::NodeHandle* nh, double freq);

        void run();

};

#endif
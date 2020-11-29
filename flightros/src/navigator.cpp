/**
 * @file navigator.cpp
 * @brief Accepts navigation commands and publishes commands to UAV
 */

#include <flightros/navigator.hpp>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <algorithm>
#include <string>

#define TIME_HORIZON 1.0
#define GOAL_POSITION_Y 20.0

Navigator::Navigator(ros::NodeHandle* nh, double freq) : _nh(*nh), 
                                                         _rate(freq){

    initializePublishers();
    initializeSubscribers();
    initializeServices();

    // command variables
    _cmd_heading = 0.0;
    _cmd_velocity = 0.0;

    _last_cmd = ros::Time::now();

    _goal_pos.x = 0.;
    _goal_pos.x = GOAL_POSITION_Y;
    _goal_pos.x = 0.;

}

/* ---------------------------------- */
/*      INITIALIZATION FUNCTIONS      */
/* ---------------------------------- */

void Navigator::initializeSubscribers(){
    _cmd_heading_sub = _nh.subscribe
        ("learner/cmd_heading", 10, &Navigator::_cmd_heading_cb, this);
    _cmd_velocity_sub = _nh.subscribe
        ("learner/cmd_velocity", 10, &Navigator::_cmd_velocity_cb, this);
    _odom_sub = _nh.subscribe
        ("hummingbird/ground_truth/odometry", 10, &Navigator::_odom_cb, this);
    _camera_sub = _nh.subscribe
        ("hummingbird/camera/image", 10, &Navigator::_camera_cb, this);
    _depth_sub = _nh.subscribe
        ("hummingbird/camera/depth", 10, &Navigator::_depth_cb, this);
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

bool Navigator::_quadstate_cb(flightros::QuadState::Request  &req,
                   flightros::QuadState::Response &res){
    std::string str0("0");
    if (str0.compare(req.in) == 0){
        // reset simulation
        // TODO
        ROS_WARN("QuadState service: Resetting simulation");
    }
    else {
        res.header = _rgb.header;
        std_msgs::Bool done;
        done.data = (_curr_pos.y >= _goal_pos.y);
        res.done = done;
        res.crash; // TODO
        res.image = _rgb; // TODO make this CV_32FC4 encoding with depth info
        res.current_position = _curr_pos;
        res.goal_position = _goal_pos;
    }
}

void Navigator::_cmd_heading_cb(const std_msgs::Float32::ConstPtr& msg){
    if (_cmd_heading > 3.14/4. || _cmd_heading < -3.14/4.)
        ROS_WARN("Heading command not within [-PI/4, PI/4]");
    _cmd_heading = msg->data;
}

void Navigator::_cmd_velocity_cb(const std_msgs::Float32::ConstPtr& msg){
    _cmd_velocity = msg->data;
}

void Navigator::_odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
    _odom = *msg;
    _curr_pos = _odom.pose.pose.position;
    _curr_orient = _odom.pose.pose.orientation;
}

void Navigator::_camera_cb(const sensor_msgs::Image::ConstPtr& msg){
    _rgb = *msg;
    _rgb.header.stamp = ros::Time::now();
}

void Navigator::_depth_cb(const sensor_msgs::Image::ConstPtr& msg){
    _depth = *msg;
    _depth.header.stamp = ros::Time::now();
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
            
            vel_msg.twist.linear.x = 0;
            vel_msg.twist.linear.y = _cmd_velocity;
            vel_msg.twist.linear.z = 0;

            Eigen::Quaterniond q(_curr_orient.w, _curr_orient.x, _curr_orient.y, _curr_orient.z);
            auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
            double yaw = euler[2]; // map frame yaw angle
            // assume incoming heading double is from [-PI/4, PI/4]
            double yaw_change = std::clamp(_cmd_heading, -3.1415927/4., 3.1415927/4.);

            vel_msg.twist.angular.x = 0;
            vel_msg.twist.angular.y = 0;
            vel_msg.twist.angular.z = 1.0 * yaw_change;
            _cmd_pub.publish(vel_msg);
        }
        else {
            // command zero velocity
            geometry_msgs::TwistStamped vel_msg;
            vel_msg.header.stamp = ros::Time::now();
            vel_msg.twist.linear.x = 0;
            vel_msg.twist.linear.y = 0;
            vel_msg.twist.linear.z = 0;
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

int main(int argc, char** argv){

    ros::init(argc, argv, "navigator");
    ros::NodeHandle nh;
    Navigator Navigator_Node(&nh, 30.0);
    ROS_INFO("Navigator: running");
    Navigator_Node.run();

    return 0;
}
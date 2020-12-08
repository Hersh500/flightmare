#pragma once

#include <memory>

// ros
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/sensors/rgb_camera.hpp"

// Noisy params 1 - medium lookahead
/*
#define NOISE_STD_ 10.0
#define MAX_DEPTH_IN_PCL_ 7.5
*/
// Noisy params 2 - short lookahead
#define NOISE_STD_ 10.0
#define MAX_DEPTH_IN_PCL_ 6.9

// Noise-free params
/*
#define NOISE_STD_ 0.0
#define MAX_DEPTH_IN_PCL_ 20
*/


using namespace flightlib;

namespace flightros {

class DepthConverter {
 public:
  DepthConverter(const ros::NodeHandle& nh,
                 const ros::NodeHandle& pnh);
  ~DepthConverter(); 
  void depthCallback(const sensor_msgs::ImageConstPtr& depth_img);
  Eigen::Matrix3f K;
  Eigen::Matrix3f invK;
  

 private:
  ros::NodeHandle nh_; 
  ros::NodeHandle pnh_; 

  ros::Subscriber sub_depth;
  ros::Publisher cloud_pub;
  image_transport::Publisher noisy_pub;
};
}

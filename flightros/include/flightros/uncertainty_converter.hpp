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
#include <tf/transform_listener.h>

// Octomap
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

// Flightmare
#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/sensors/rgb_camera.hpp"
#include "flightros/flight_pilot.hpp"

using namespace flightlib;

namespace flightros {

class UncertaintyConverter {
 public:
   UncertaintyConverter(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
   ~UncertaintyConverter(); 
   // Need to deserialize this message
   // see: https://github.com/OctoMap/octomap_msgs/blob/melodic-devel/include/octomap_msgs/conversions.h
   void mapCallback(const octomap_msgs::Octomap& msg);
   // Use K matrix to convert from point cloud to pixel coords.
   void pcCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
   Eigen::Matrix3f K;
 private:
  int first = 0;
  bool got_pcl = false;
  ros::NodeHandle nh_; 
  ros::NodeHandle pnh_; 
  tf::TransformListener listener;

  sensor_msgs::PointCloud2 pcl_msg;
  ros::Subscriber sub_map;
  ros::Subscriber sub_pcl;
  image_transport::Publisher uncertainty_pub;
};
}

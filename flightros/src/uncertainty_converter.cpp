// Flightmare stuff
#include "flightros/uncertainty_converter.hpp"
#include "flightros/flight_pilot.hpp"
#include "flightros/depth_converter.hpp"

// Ros stuff
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// general steps:
// 1. subscribe to octomap
// 2. subscribe to robot pose
// 3. from current robot pose, calculate the octree nodes in the camera FOV 
// 3.5: calculate the min and the max for the bounding box leaf iterator.
// 4. Iterate through the octree nodes, and get the probabilities stored in each one
// 4.5: see bounding box leaf iterator: https://octomap.github.io/octomap/doc/classleaf__bbx__iterator.html#aabbd209b13fa272284890103e6292d00
// 5. Put probabilities in a mono8 image, and publish that.

namespace flightros {
UncertaintyConverter::UncertaintyConverter(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : nh_(nh),
    pnh_(pnh) {
    sub_map = nh_.subscribe("/octomap_full", 1, &UncertaintyConverter::mapCallback, this);
    image_transport::ImageTransport it(nh_);
    uncertainty_pub = it.advertise("camera/uncertainty", 1);
}
    
UncertaintyConverter::~UncertaintyConverter() {}

void UncertaintyConverter::mapCallback(const octomap_msgs::Octomap& msg) {
    tf::StampedTransform cur_transform;
    // see: https://github.com/OctoMap/octomap_msgs/blob/melodic-devel/include/octomap_msgs/conversions.h
    octomap::AbstractOcTree* tree = octomap_msgs::fullMsgToMap(msg);

    // TODO: calculate the correct bounding box given the current robot coords.
    // min_bbox, max_bbox = ...;

    /*
    for(OcTreeTYPE::leaf_bbx_iterator it = tree->begin_leafs_bbx(min,max),
       end=tree->end_leafs_bbx(); it!= end; ++it) {
        //manipulate node, e.g.:
        std::cout << "Node center: " << it.getCoordinate() << std::endl;
        std::cout << "Node size: " << it.getSize() << std::endl;
        std::cout << "Node value: " << it->getValue() << std::endl;
    }
    */

}

}

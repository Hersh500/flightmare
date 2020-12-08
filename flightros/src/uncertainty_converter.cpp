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
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_transport/image_transport.h>

#include <math.h>

// general steps:
// 1. subscribe to octomap
// 3. from current point cloud, calculate the octree nodes in the camera FOV 
// 4. Iterate through the octree nodes, and get the probabilities stored in each one
// 5. Put entropies in a mono8 image, and publish that.

namespace flightros {

UncertaintyConverter::UncertaintyConverter(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : nh_(nh),
    pnh_(pnh) {
    sub_map = nh_.subscribe("/octomap_full", 1, &UncertaintyConverter::mapCallback2, this);
    sub_pcl = nh_.subscribe("point_cloud", 1, &UncertaintyConverter::pcCallback, this);
    sub_depth = nh_.subscribe("camera/noisy_depth", 1, &UncertaintyConverter::depthCallback, this);

    image_transport::ImageTransport it(nh_);
    uncertainty_pub = it.advertise("camera/uncertainty", 1);

    float fl = (FlightPilot::im_height/2.0)/tan((M_PI * FlightPilot::fov/180.0)/2.0);
    K(0, 0) = fl; 
    K(0, 1) = 0; 
    K(0, 2) = FlightPilot::im_width/2;
    K(1, 0) = 0;
    K(1, 1) = FlightPilot::im_height/2;
    K(2, 0) = 0;
    K(2, 1) = 0;
    K(2, 2) = 1;
    invK = K.inverse();
}
    
UncertaintyConverter::~UncertaintyConverter() {}

void UncertaintyConverter::pcCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    got_pcl = true;
    pcl_msg = *msg;
    pcl_msg.header.stamp = ros::Time::now();
}

void UncertaintyConverter::depthCallback(const sensor_msgs::ImageConstPtr &msg) {
    got_depth = true;
    _depth_img = *msg;
    // _depth_img.header.stamp = ros::Time::now();
}

void UncertaintyConverter::mapCallback(const octomap_msgs::Octomap& msg) {
    tf::StampedTransform cur_transform;
    octomap::OcTree* tree = (octomap::OcTree*) octomap_msgs::fullMsgToMap(msg);

    if (!got_pcl) {
        ROS_ERROR("Cannot output uncertainty, don't have point cloud.");
        return;
    }
    // transform stored point cloud into world frame
    sensor_msgs::PointCloud2 tf_pcl;
    bool output = pcl_ros::transformPointCloud("world", pcl_msg, tf_pcl, listener);
    if (!output) {
        ROS_ERROR("Unable to transform point cloud in uncertainty converter!");
        return;
    }

    // get OcTree nodes for each point in point cloud
    sensor_msgs::PointCloud2Iterator<float> iter_x(tf_pcl, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(tf_pcl, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(tf_pcl, "z");

    sensor_msgs::PointCloud2Iterator<float> iter_x_og(pcl_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y_og(pcl_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z_og(pcl_msg, "z");

    cv::Mat u_image = cv::Mat::ones(FlightPilot::im_height, FlightPilot::im_width, CV_8UC1) * 255;

    float d = FlightPilot::depth_scale/100.0f;

    uint8_t num = 0;
    
    // instead of iterating through the point cloud, iterate through the depth image to get uncertainties
    
    // get uncertainties, add to u_image.
    for (; iter_x_og != iter_x_og.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_x_og, ++iter_y_og, ++iter_z_og) {
        Eigen::Vector3f world_pt(*iter_x_og, *iter_z_og, *iter_y_og);
        // TODO: constrain depth here if necessary
        octomap::OcTreeNode *node;
        try {
            node = tree->search(*iter_x, *iter_y, *iter_z);
        } catch (std::exception &e) {
            ROS_ERROR("Error getting points from point cloud!");
            return;
        }

        // float occ_entropy = 0.5f;
        uint8_t occ_entropy = 255;
        if (node) {
            float logodds = (float) node->getLogOdds();
            float occ_prob = exp(logodds)/(1 + exp(logodds));
            occ_prob = -occ_prob * log2(occ_prob) - (1 - occ_prob) * log2(1 - occ_prob);
            occ_entropy = (uint8_t) (255.0f * occ_prob);
        }
        Eigen::Vector3f cam_pt = K * world_pt;
        
        // Now apply inverse calculation to fill U image
        int x = (int) cam_pt.x()/(cam_pt.z());
        int y = (int)(-(cam_pt.y()/(cam_pt.z()) - (FlightPilot::im_height/2)));
        /*
        if (first < 2 && (world_pt.x() != 0 || world_pt.y() != 0 || world_pt.z() != 0)) {
            ROS_INFO("World pt = (%f, %f, %f)", *iter_x_og, *iter_z_og, *iter_y_og);
            ROS_INFO("cam pt = (%d, %d)", y, x);
            ROS_INFO("value: %d", num);
            first++;
        }
        */
        if (x >= 0 && x < FlightPilot::im_width && y >= 0 && y < FlightPilot::im_height) {
            u_image.at<uint8_t>(y, x) = occ_entropy;
        } else {
            // ROS_INFO("Out of bounds point! (%d, %d)", y, x);
        }
    }

    delete tree;
    
    std_msgs::Header header;
    header.frame_id = "camera_pose";
    header.stamp = ros::Time::now();
    try {
        sensor_msgs::ImageConstPtr img_msg = cv_bridge::CvImage(header, "mono8", u_image).toImageMsg();
        uncertainty_pub.publish(img_msg);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("error in uncertainty converter: %s", e.what()); 
    }
}

void UncertaintyConverter::mapCallback2(const octomap_msgs::Octomap& msg) {
    if (!got_depth) {
        ROS_ERROR("Cannot output uncertainty, don't have depth image.");
        return;
    }
    cv_bridge::CvImagePtr depth_img = cv_bridge::toCvCopy(_depth_img, "mono8");

    octomap::OcTree* tree = (octomap::OcTree*) octomap_msgs::fullMsgToMap(msg);


    cv::Mat u_image = cv::Mat::ones(FlightPilot::im_height, FlightPilot::im_width, CV_8UC1) * 255;
    
    float width_factor = 0; // -img->image.cols/2;
    float height_factor = depth_img->image.rows/2;
    for (int i = 0; i < depth_img->image.rows; i=i + 1) {
        for (int j = 0; j < depth_img->image.cols; j=j + 1) {
            uint8_t id = depth_img->image.at<uint8_t>(i, j);
            if (id != 0) {
                float d = FlightPilot::depth_scale/100.0 * id;

                // If point is too far away, select some free space point 
                if (d > std::min((double)8, (double)MAX_DEPTH_IN_PCL_)) {
                    d = 5;
                }

                Eigen::Vector3f image_point((j + width_factor) * d, (-i + height_factor) * d, d);
                Eigen::Vector3f camera_point = invK * image_point;
                
                // Transform this point into the world frame
                geometry_msgs::Point pt;
                pt.x = camera_point.x();
                pt.y = camera_point.z();
                pt.z = camera_point.y();
                
                geometry_msgs::PointStamped pt_stamped;
                geometry_msgs::PointStamped pt_stamped_out;
                pt_stamped.header = std_msgs::Header();
                pt_stamped.header.stamp = ros::Time::now();
                pt_stamped.header.frame_id = "camera_pose";
                pt_stamped.point = pt;
                
                octomap::OcTreeNode *node;
                try {
                    listener.transformPoint("world", msg.header.stamp, pt_stamped, "camera_pose", pt_stamped_out);
                    node = tree->search(pt_stamped_out.point.x,
                                        pt_stamped_out.point.y,
                                        pt_stamped_out.point.z);
                } catch (std::exception &e) {
                    ROS_ERROR("Error transforming points or accessing node! %s", e.what());
                    return;
                }

                uint8_t occ_entropy = 255;
                if (node) {
                    float logodds = (float) node->getLogOdds();
                    float occ_prob = exp(logodds)/(1 + exp(logodds));
                    occ_prob = -occ_prob * log2(occ_prob) - (1 - occ_prob) * log2(1 - occ_prob);
                    occ_entropy = (uint8_t) (255.0f * occ_prob);
                }
                u_image.at<uint8_t>(i, j) = occ_entropy;
            }
        }
    }

    delete tree;
    std_msgs::Header header;
    header.frame_id = "camera_pose";
    header.stamp = ros::Time::now();
    try {
        sensor_msgs::ImageConstPtr img_msg = cv_bridge::CvImage(header, "mono8", u_image).toImageMsg();
        uncertainty_pub.publish(img_msg);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("error in uncertainty converter: %s", e.what()); 
    }
}

}

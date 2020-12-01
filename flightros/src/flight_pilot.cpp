#include "flightros/flight_pilot.hpp"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Bool.h>

namespace flightros {

FlightPilot::FlightPilot(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
  : nh_(nh),
    pnh_(pnh),
    scene_id_(UnityScene::WAREHOUSE),
    unity_ready_(false),
    unity_render_(false),
    receive_id_(0),
    main_loop_freq_(50.0) {
  // load parameters
  if (!loadParams()) {
    ROS_WARN("[%s] Could not load all parameters.",
             pnh_.getNamespace().c_str());
  } else {
    ROS_INFO("[%s] Loaded all parameters.", pnh_.getNamespace().c_str());
  }

  // quad initialization
  quad_ptr_ = std::make_shared<Quadrotor>();

  // add mono camera
  rgb_camera_ = std::make_shared<RGBCamera>();
  Vector<3> B_r_BC(x_t, y_t, z_t);
  Matrix<3, 3> R_BC = R_BC_quat.toRotationMatrix();
  std::cout << R_BC << std::endl;
  rgb_camera_->setFOV(fov);
  rgb_camera_->setWidth(im_width);
  rgb_camera_->setHeight(im_height);
  rgb_camera_->setRelPose(B_r_BC, R_BC);
  rgb_camera_->enableDepth(true);
  rgb_camera_->setDepthScale(depth_scale/100.0);
  quad_ptr_->addRGBCamera(rgb_camera_);

  // initialization
  quad_state_.setZero();
  quad_ptr_->reset(quad_state_);


  // initialize subscriber call backs
  sub_state_est_ = nh_.subscribe("flight_pilot/state_estimate", 1,
                                 &FlightPilot::poseCallback, this);

  timer_main_loop_ = nh_.createTimer(ros::Rate(main_loop_freq_),
                                     &FlightPilot::mainLoopCallback, this);


  // wait until the gazebo and unity are loaded
  ros::Duration(5.0).sleep();

  // connect unity
  setUnity(unity_render_);
  connectUnity();

  // add a rgb camera image publisher
  image_transport::ImageTransport it(nh_);
  image_pub = it.advertise("camera/image", 1);
  depth_image_pub = it.advertise("camera/depth", 1);
  collision_pub = nh_.advertise<std_msgs::Bool>("collision", 1);


  // initialize the reset service
  // reset_service = nh_.advertiseService("reset_quad_state", &FlightPilot::resetState);
}

FlightPilot::~FlightPilot() {}

void FlightPilot::poseCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    quad_state_.x[QS::POSX] = (Scalar)msg->pose.pose.position.x;
    quad_state_.x[QS::POSY] = (Scalar)msg->pose.pose.position.y;
    quad_state_.x[QS::POSZ] = (Scalar)msg->pose.pose.position.z;
    quad_state_.x[QS::ATTW] = (Scalar)msg->pose.pose.orientation.w;
    quad_state_.x[QS::ATTX] = (Scalar)msg->pose.pose.orientation.x;
    quad_state_.x[QS::ATTY] = (Scalar)msg->pose.pose.orientation.y;
    quad_state_.x[QS::ATTZ] = (Scalar)msg->pose.pose.orientation.z;

    quad_ptr_->setState(quad_state_);

    if (unity_render_ && unity_ready_) {
        unity_bridge_ptr_->getRender(0);
        unity_bridge_ptr_->handleOutput();
    }
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0.0, 0.1, 0.6));
    tf::Quaternion q;
    Eigen::Quaternion eigen_q = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    tf::quaternionEigenToTF(eigen_q, q);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "hummingbird/base_link", "camera_pose"));
}

void FlightPilot::mainLoopCallback(const ros::TimerEvent &event) {
  if (unity_render_ && unity_ready_) {
    message_num = message_num + 1;
    unity_bridge_ptr_->getRender(message_num);
    unity_bridge_ptr_->handleOutput();
  }
    cv::Mat image;
    cv::Mat depth_image;
    cv_bridge::CvImagePtr cv_ptr;

    std::vector<std::shared_ptr<RGBCamera>> rgb_cameras = quad_ptr_->getCameras();
    auto cam = rgb_cameras[0];
    bool success = cam->getRGBImage(image);
    if (!success) {
        ROS_INFO("unable to publish image because no rgb image");
        return;
    }
    sensor_msgs::ImageConstPtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    image_pub.publish(msg);

    success = cam->getDepthMap(depth_image);
    if (!success) {
        ROS_INFO("unable to publish depth image!");
        return;
    }
    cv::Mat depth_conv;
    cv::cvtColor(depth_image, depth_conv, CV_BGR2GRAY);

    double min_depth, max_depth;
    cv::minMaxLoc(depth_conv, &min_depth, &max_depth);

    sensor_msgs::ImageConstPtr depth_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", depth_conv).toImageMsg();
    cv_bridge::CvImageConstPtr depth_img_msg = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::MONO8);
    depth_image_pub.publish(depth_msg);
    
    // Publish the current collision state
    // Seems like the collision flag is broken. 
    // Instead can use a circular buffer on the minimum depth value in the image for like 5 consecutive images.

    if (min_depth < 30) {
        num_min_depths_past_threshold++;
    } else {
        num_min_depths_past_threshold = 0;
    }

    std_msgs::Bool collision_msg;
    if (num_min_depths_past_threshold > 3) {
        collision_msg.data = true;
    } else {
        collision_msg.data = false;
    }

    collision_pub.publish(collision_msg);
}

bool FlightPilot::setUnity(const bool render) {
  unity_render_ = render;
  if (unity_render_ && unity_bridge_ptr_ == nullptr) {
    // create unity bridge
    unity_bridge_ptr_ = UnityBridge::getInstance();
    unity_bridge_ptr_->addQuadrotor(quad_ptr_);
    ROS_INFO("[%s] Unity Bridge is created.", pnh_.getNamespace().c_str());
  }
  return true;
}

bool FlightPilot::connectUnity() {
  if (!unity_render_ || unity_bridge_ptr_ == nullptr) return false;
  unity_ready_ = unity_bridge_ptr_->connectUnity(scene_id_);
  return unity_ready_;
}

bool FlightPilot::loadParams(void) {
  // load parameters
  quadrotor_common::getParam("main_loop_freq", main_loop_freq_, pnh_);
  quadrotor_common::getParam("unity_render", unity_render_, pnh_);

  return true;
}

/*
void FlightPilot::publishPointCloud(cv_bridge::CvImageConstPtr depth_img) {

    std::vector<float> points;
    //hardcoded right now, need to change this later
    float depth_scale = 0.2;
    float FOV = 90.0;
    float fl = (240/2.0)/tan((M_PI * FOV/180.0)/2.0);
    Eigen::Matrix3f K;
    // the y of the image frame is not aligned with the y of the world axis...
    // in the image frame, y is up, while in the world frame, z is up...
    // currently have just switched the axes here, but there should be a consistent way to keep track, maybe through tf?

    K(0, 0) = fl;
    K(0, 1) = 0;
    K(0, 2) = 360.0/2;
    K(1, 0) = 0;
    K(1, 1) = fl;
    K(1, 2) = 240.0/2;
    K(2, 0) = 0;
    K(2, 1) = 0;
    K(2, 2) = 1;

    Eigen::Matrix3f invK = K.inverse();

    for (int i = 0; i < depth_img->image.rows; i++) {
        const uint8_t *row_ptr = depth_img->image.ptr<uint8_t>(i);
        for (int j = 0; j < depth_img->image.cols; j++) {
            uint8_t id = row_ptr[j];
            if (id != 0) {
                float d = depth_scale * id;
                Eigen::Vector3f image_point(j * d, i * d, d);
                Eigen::Vector3f camera_point = invK * image_point;
                points.push_back(camera_point.x());
                // switching between z and y here.
                points.push_back(-camera_point.z());  // need to add a negative here?
                points.push_back(camera_point.y());
            }
        }
    }
    
    int n_points = points.size();
    sensor_msgs::PointCloud2 cloud_msg;
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);

    modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::PointField::FLOAT32,
                                  "y", 1, sensor_msgs::PointField::FLOAT32,
                                  "z", 1, sensor_msgs::PointField::FLOAT32);

    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(n_points);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

    cloud_msg.height = 1;
    cloud_msg.width = n_points;
    cloud_msg.header = std_msgs::Header();
    cloud_msg.header.frame_id = "camera_pose";

    for(size_t i=0; i<(unsigned long)n_points/3; ++i, ++iter_x, ++iter_y, ++iter_z){
        *iter_x = points[3*i+0];
        *iter_y = points[3*i+1];
        *iter_z = points[3*i+2];

        // std::cerr << *iter_x << " " << *iter_y << " " << *iter_z << std::endl;
    }
    cloud_pub.publish(cloud_msg);
}
*/

}  // namespace flightros

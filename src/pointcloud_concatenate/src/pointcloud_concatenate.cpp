#include "pointcloud_concatenate/pointcloud_concatenate.hpp"

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <thread>

// Constructor
PointcloudConcatenate::PointcloudConcatenate()
: Node("pointcloud_concatenate")
{
  // Initialise variables / parameters to class variables
  handleParams();

  // Initialization tf2 listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialise publishers and subscribers
  // Use SensorDataQoS (best_effort) to be compatible with ros_gz_bridge
  auto sensor_qos = rclcpp::SensorDataQoS();
  sub_cloud_in1_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "cloud_in1", sensor_qos,
    std::bind(&PointcloudConcatenate::subCallbackCloudIn1, this, std::placeholders::_1));
  sub_cloud_in2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "cloud_in2", sensor_qos,
    std::bind(&PointcloudConcatenate::subCallbackCloudIn2, this, std::placeholders::_1));
  sub_cloud_in3_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "cloud_in3", sensor_qos,
    std::bind(&PointcloudConcatenate::subCallbackCloudIn3, this, std::placeholders::_1));
  sub_cloud_in4_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "cloud_in4", sensor_qos,
    std::bind(&PointcloudConcatenate::subCallbackCloudIn4, this, std::placeholders::_1));
  pub_cloud_out_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_out", 1);

  // Create timer for periodic update
  auto period = std::chrono::duration<double>(1.0 / param_hz_);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&PointcloudConcatenate::update, this));
}

// Destructor
PointcloudConcatenate::~PointcloudConcatenate() {
  RCLCPP_INFO(this->get_logger(), "Destructing PointcloudConcatenate...");
}

void PointcloudConcatenate::subCallbackCloudIn1(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  cloud_in1_ = *msg;
  cloud_in1_received_ = true;
  cloud_in1_received_recent_ = true;
}

void PointcloudConcatenate::subCallbackCloudIn2(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  cloud_in2_ = *msg;
  cloud_in2_received_ = true;
  cloud_in2_received_recent_ = true;
}

void PointcloudConcatenate::subCallbackCloudIn3(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  cloud_in3_ = *msg;
  cloud_in3_received_ = true;
  cloud_in3_received_recent_ = true;
}

void PointcloudConcatenate::subCallbackCloudIn4(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  cloud_in4_ = *msg;
  cloud_in4_received_ = true;
  cloud_in4_received_recent_ = true;
}

void PointcloudConcatenate::handleParams() {
  RCLCPP_INFO(this->get_logger(), "Loading parameters...");

  // Target frame
  this->declare_parameter<std::string>("target_frame", "base_link");
  this->get_parameter("target_frame", param_frame_target_);
  if (param_frame_target_.empty()) {
    param_frame_target_ = "base_link";
    RCLCPP_WARN(this->get_logger(), "Param 'target_frame' is not set. Setting to default value: %s",
                param_frame_target_.c_str());
  }

  // Number of pointclouds
  this->declare_parameter<int>("clouds", 2);
  this->get_parameter("clouds", param_clouds_);

  // Frequency to update/publish
  this->declare_parameter<double>("hz", 10.0);
  this->get_parameter("hz", param_hz_);

  RCLCPP_INFO(this->get_logger(), "Parameters loaded.");
}

void PointcloudConcatenate::update() {
  // Is run periodically and handles calling the different methods

  if (pub_cloud_out_->get_subscription_count() > 0 && param_clouds_ >= 1) {
    // Initialise pointclouds
    sensor_msgs::msg::PointCloud2 cloud_to_concat;
    cloud_out_ = cloud_to_concat; // Clear the output pointcloud

    // Track success of transforms
    bool success = true;

    // Sleep if no pointclouds have been received yet
    if ((!cloud_in1_received_) && (!cloud_in2_received_) && (!cloud_in3_received_) && (!cloud_in4_received_)) {
      RCLCPP_WARN(this->get_logger(), "No pointclouds received yet. Waiting 1 second...");
      std::this_thread::sleep_for(std::chrono::seconds(1));
      return;
    }

    // Concatenate the first pointcloud
    if (param_clouds_ >= 1 && success && cloud_in1_received_) {
      if (!cloud_in1_received_recent_) {
        RCLCPP_WARN(this->get_logger(), "Cloud 1 was not received since last update, reusing last received message...");
      }
      cloud_in1_received_recent_ = false;

      // Transform pointcloud to the target frame
      success = pcl_ros::transformPointCloud(param_frame_target_, cloud_in1_, cloud_out_, *tf_buffer_);
      if (!success) {
        RCLCPP_WARN(this->get_logger(), "Transforming cloud 1 from %s to %s failed!",
                    cloud_in1_.header.frame_id.c_str(), param_frame_target_.c_str());
      }
    }

    // Concatenate the second pointcloud
    if (param_clouds_ >= 2 && success && cloud_in2_received_) {
      if (!cloud_in2_received_recent_) {
        RCLCPP_WARN(this->get_logger(), "Cloud 2 was not received since last update, reusing last received message...");
      }
      cloud_in2_received_recent_ = false;

      success = pcl_ros::transformPointCloud(param_frame_target_, cloud_in2_, cloud_to_concat, *tf_buffer_);
      if (!success) {
        RCLCPP_WARN(this->get_logger(), "Transforming cloud 2 from %s to %s failed!",
                    cloud_in2_.header.frame_id.c_str(), param_frame_target_.c_str());
      }

      if (success) {
        pcl::concatenatePointCloud(cloud_out_, cloud_to_concat, cloud_out_);
      }
    }

    // Concatenate the third pointcloud
    if (param_clouds_ >= 3 && success && cloud_in3_received_) {
      if (!cloud_in3_received_recent_) {
        RCLCPP_WARN(this->get_logger(), "Cloud 3 was not received since last update, reusing last received message...");
      }
      cloud_in3_received_recent_ = false;

      success = pcl_ros::transformPointCloud(param_frame_target_, cloud_in3_, cloud_to_concat, *tf_buffer_);
      if (!success) {
        RCLCPP_WARN(this->get_logger(), "Transforming cloud 3 from %s to %s failed!",
                    cloud_in3_.header.frame_id.c_str(), param_frame_target_.c_str());
      }

      if (success) {
        pcl::concatenatePointCloud(cloud_out_, cloud_to_concat, cloud_out_);
      }
    }

    // Concatenate the fourth pointcloud
    if (param_clouds_ >= 4 && success && cloud_in4_received_) {
      if (!cloud_in4_received_recent_) {
        RCLCPP_WARN(this->get_logger(), "Cloud 4 was not received since last update, reusing last received message...");
      }
      cloud_in4_received_recent_ = false;

      success = pcl_ros::transformPointCloud(param_frame_target_, cloud_in4_, cloud_to_concat, *tf_buffer_);
      if (!success) {
        RCLCPP_WARN(this->get_logger(), "Transforming cloud 4 from %s to %s failed!",
                    cloud_in4_.header.frame_id.c_str(), param_frame_target_.c_str());
      }

      if (success) {
        pcl::concatenatePointCloud(cloud_out_, cloud_to_concat, cloud_out_);
      }
    }

    // Publish the concatenated pointcloud
    if (success) {
      publishPointcloud(cloud_out_);
    }
  }
}

void PointcloudConcatenate::publishPointcloud(sensor_msgs::msg::PointCloud2 cloud) {
  // Update the timestamp
  cloud.header.stamp = this->now();
  // Publish
  pub_cloud_out_->publish(cloud);
}

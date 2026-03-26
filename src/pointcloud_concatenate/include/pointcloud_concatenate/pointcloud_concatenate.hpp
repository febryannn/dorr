#pragma once
#ifndef POINTCLOUD_CONCATENATE
#define POINTCLOUD_CONCATENATE

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <pcl_ros/transforms.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

class PointcloudConcatenate : public rclcpp::Node {
public:
  PointcloudConcatenate();
  ~PointcloudConcatenate();

  void handleParams();
  void update();

private:
  void subCallbackCloudIn1(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void subCallbackCloudIn2(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void subCallbackCloudIn3(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void subCallbackCloudIn4(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void publishPointcloud(sensor_msgs::msg::PointCloud2 cloud);

  // Parameters
  std::string param_frame_target_;
  int param_clouds_;
  double param_hz_;

  // Publisher and subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_in1_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_in2_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_in3_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_in4_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_out_;

  // Timer for periodic update
  rclcpp::TimerBase::SharedPtr timer_;

  // Pointcloud storage
  sensor_msgs::msg::PointCloud2 cloud_in1_;
  sensor_msgs::msg::PointCloud2 cloud_in2_;
  sensor_msgs::msg::PointCloud2 cloud_in3_;
  sensor_msgs::msg::PointCloud2 cloud_in4_;
  sensor_msgs::msg::PointCloud2 cloud_out_;
  bool cloud_in1_received_ = false;
  bool cloud_in2_received_ = false;
  bool cloud_in3_received_ = false;
  bool cloud_in4_received_ = false;
  bool cloud_in1_received_recent_ = false;
  bool cloud_in2_received_recent_ = false;
  bool cloud_in3_received_recent_ = false;
  bool cloud_in4_received_recent_ = false;

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

#endif

#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "udp_bot/msg/terima_udp.hpp"
#include "udp_bot/msg/kirim_offset_udp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/crop_box.h>

#define TO_DEG 57.295779513
#define TO_RAD 0.01745329252

using namespace std::chrono_literals;

class RgbdCamLocNode : public rclcpp::Node
{
public:
  RgbdCamLocNode() : Node("rgbd_cam_loc")
  {
    // Declare parameters
    this->declare_parameter("min_cam_d", 0.6);
    this->declare_parameter("max_cam_d", 8.0);
    this->declare_parameter("init_robotx", 0.0);
    this->declare_parameter("init_roboty", 0.0);
    this->declare_parameter("init_robotw", 0.0);
    this->declare_parameter("maxf_points_out", 1.0);
    this->declare_parameter("map_pcd_path", "/home/agv1/alfian/old_agv_ws/map_p206_final.pcd");

    min_cam_d_ = this->get_parameter("min_cam_d").as_double();
    max_cam_d_ = this->get_parameter("max_cam_d").as_double();
    robotx_ = this->get_parameter("init_robotx").as_double();
    roboty_ = this->get_parameter("init_roboty").as_double();
    robotw_ = this->get_parameter("init_robotw").as_double();
    freq_points_ = this->get_parameter("maxf_points_out").as_double();
    std::string map_pcd_path = this->get_parameter("map_pcd_path").as_string();

    status_pub_points_ = 0;
    status_init_odom_ = 0;
    roll_ = pitch_ = yaw_ = 0.0;
    vx_lokal_ = vy_lokal_ = vw_lokal_ = 0.0f;
    vx_global_ = vy_global_ = vw_global_ = 0.0f;
    odox_buf_ = odoy_buf_ = odow_buf_ = 0.0f;
    odox_offset_ = odoy_offset_ = odow_offset_ = 0.0f;
    odom_data_received_ = false;

    cov_pos_.fill(0.0);
    cov_vel_.fill(0.0);

    // Subscribers
    sub_cam_raw_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "full_pointcloud", 10,
      std::bind(&RgbdCamLocNode::cam_filter_callback, this, std::placeholders::_1));

    sub_robot_odom_ = this->create_subscription<udp_bot::msg::TerimaUdp>(
      "data_terima_udp", 10,
      std::bind(&RgbdCamLocNode::robot_odom_callback, this, std::placeholders::_1));

    sub_robot_offset_ = this->create_subscription<udp_bot::msg::KirimOffsetUdp>(
      "offset_kirim_udp", 10,
      std::bind(&RgbdCamLocNode::robot_offset_callback, this, std::placeholders::_1));

    // Publishers
    pub_cam_filter_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 10);
    pub_robot_offset_ = this->create_publisher<udp_bot::msg::KirimOffsetUdp>("offset_kirim_udp", 10);
    pub_robot_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    pub_robot_pos_ = this->create_publisher<geometry_msgs::msg::Pose>("robot_pos", 10);
    pub_init_pos_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);
    pub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_cloud", 10);

    // TF
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Load map PCD
    map_pcd_ = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(map_pcd_path, *map_pcd_) == -1)
    {
      RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", map_pcd_path.c_str());
    }
    else
    {
      pcl::toROSMsg(*map_pcd_, output_map_);
      output_map_.header.stamp = this->now();
      output_map_.header.frame_id = "map";
    }

    RCLCPP_INFO(this->get_logger(), "WAIT DATA FROM ROBOT POS & RGBD CAMERA");

    // Wait for odom data before starting
    wait_timer_ = this->create_wall_timer(100ms, std::bind(&RgbdCamLocNode::wait_for_data, this));
  }

private:
  void wait_for_data()
  {
    if (odom_data_received_) {
      wait_timer_->cancel();
      RCLCPP_INFO(this->get_logger(), "Pos and Pointcloud RECEIVE!!");

      std::this_thread::sleep_for(2s);
      RCLCPP_INFO(this->get_logger(), "[RGBD CAM LOCALIZATION PROCESS BEGIN]");

      // Publish initial pose and map
      publish_init_pose_and_map();

      // Start timers
      robot_odom_timer_ = this->create_wall_timer(
        50ms, std::bind(&RgbdCamLocNode::robot_odom_pub_callback, this));

      localization_process_timer_ = this->create_wall_timer(
        100ms, std::bind(&RgbdCamLocNode::localization_process_callback, this));

      int freq_ms = static_cast<int>(1000.0 / freq_points_);
      pub_cam_filter_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(freq_ms),
        std::bind(&RgbdCamLocNode::pub_cam_filter_callback, this));
    }
  }

  void publish_init_pose_and_map()
  {
    geometry_msgs::msg::PoseWithCovarianceStamped init_pos;
    init_pos.header.stamp = this->now();
    init_pos.header.frame_id = "map";
    init_pos.pose.pose.position.x = robotx_;
    init_pos.pose.pose.position.y = roboty_;
    tf2::Quaternion init_pos_q;
    init_pos_q.setRPY(0, 0, robotw_ * TO_RAD);
    init_pos.pose.pose.orientation = tf2::toMsg(init_pos_q);

    std::array<double, 36> cov_init_pos;
    cov_init_pos.fill(0.0);
    cov_init_pos[0] = 0.05;   // xx
    cov_init_pos[7] = 0.05;   // yy
    cov_init_pos[35] = 0.1;   // yaw-yaw
    std::copy(cov_init_pos.begin(), cov_init_pos.end(), init_pos.pose.covariance.begin());

    for (int i = 0; i <= 2; i++)
    {
      pub_map_->publish(output_map_);
      pub_init_pos_->publish(init_pos);
      std::this_thread::sleep_for(1s);
    }
  }

  void cam_filter_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (pub_cam_filter_->get_subscription_count() <= 0)
    {
      return;
    }

    input_points2_ = *msg;
    RCLCPP_DEBUG(this->get_logger(), "[point_cloud_converter] Got a PointCloud2 with %d points",
                 input_points2_.width * input_points2_.height);
    status_pub_points_ = 1;
  }

  void pub_cam_filter_callback()
  {
    if (status_pub_points_ == 1)
    {
      pcl::PCLPointCloud2::Ptr cam_cloud(new pcl::PCLPointCloud2());
      pcl::PCLPointCloud2::Ptr cam_cloud_filtered(new pcl::PCLPointCloud2());
      pcl::PointCloud<pcl::PointXYZ>::Ptr cam_cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());

      // Convert and filter
      pcl_conversions::toPCL(input_points2_, *cam_cloud);

      // Crop filter
      pcl::PointXYZ minPt, maxPt;
      pcl::fromPCLPointCloud2(*cam_cloud, *cam_cloud_xyz);
      pcl::getMinMax3D(*cam_cloud_xyz, minPt, maxPt);
      pcl::CropBox<pcl::PCLPointCloud2> cb;
      cb.setInputCloud(cam_cloud);
      cb.setMin(Eigen::Vector4f(min_cam_d_, minPt.y, minPt.z, 0.));
      cb.setMax(Eigen::Vector4f(max_cam_d_, maxPt.y, maxPt.z, 0.));
      cb.filter(*cam_cloud);

      // Voxel grid filter
      pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
      vg.setInputCloud(cam_cloud);
      vg.setLeafSize(0.06f, 0.06f, 0.06f);
      vg.filter(*cam_cloud);

      // Statistical Outlier Removal Filter
      pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
      sor.setInputCloud(cam_cloud);
      sor.setMeanK(30);
      sor.setStddevMulThresh(2.0);
      sor.filter(*cam_cloud_filtered);

      pcl_conversions::fromPCL(*cam_cloud_filtered, output_points2_);
      pub_cam_filter_->publish(output_points2_);
      status_pub_points_ = 0;
    }
  }

  void robot_odom_callback(const udp_bot::msg::TerimaUdp::SharedPtr msg)
  {
    odox_buf_ = msg->posisi_x_buffer;
    odoy_buf_ = msg->posisi_y_buffer;
    odow_buf_ = msg->sudut_w_buffer;

    vx_lokal_ = msg->kecepatan_robotx;
    vy_lokal_ = msg->kecepatan_roboty;
    vw_lokal_ = msg->kecepatan_robotw;

    vx_global_ = msg->vx_global;
    vy_global_ = msg->vy_global;
    vw_global_ = msg->vw_global;

    odom_data_received_ = true;
  }

  void robot_offset_callback(const udp_bot::msg::KirimOffsetUdp::SharedPtr msg)
  {
    odox_offset_ = msg->posisi_x_offset;
    odoy_offset_ = msg->posisi_y_offset;
    odow_offset_ = msg->sudut_w_offset;
  }

  void robot_odom_pub_callback()
  {
    if (status_init_odom_ == 0)
    {
      odox_offset_ = odox_buf_ - robotx_;
      odoy_offset_ = odoy_buf_ - roboty_;
      odow_offset_ = odow_buf_ - robotw_;

      auto offset_msg = udp_bot::msg::KirimOffsetUdp();
      offset_msg.posisi_x_offset = odox_offset_;
      offset_msg.posisi_y_offset = odoy_offset_;
      offset_msg.sudut_w_offset = odow_offset_;
      pub_robot_offset_->publish(offset_msg);

      RCLCPP_INFO(this->get_logger(), "[ODOMETRY INITIAL LOCATION DONE]");
      status_init_odom_ = 1;
    }
    else
    {
      robotx_ = odox_buf_ - odox_offset_;
      roboty_ = odoy_buf_ - odoy_offset_;
      robotw_ = odow_buf_ - odow_offset_;
      while (robotw_ > 180) robotw_ -= 360;
      while (robotw_ < -180) robotw_ += 360;

      geometry_msgs::msg::Pose pos_msg;
      pos_msg.position.x = robotx_;
      pos_msg.position.y = roboty_;
      tf2::Quaternion q;
      q.setRPY(0, 0, robotw_ * TO_RAD);
      pos_msg.orientation = tf2::toMsg(q);

      nav_msgs::msg::Odometry robot_odom;
      robot_odom.header.stamp = this->now();
      robot_odom.header.frame_id = "map";
      robot_odom.pose.pose.position = pos_msg.position;
      robot_odom.pose.pose.orientation = pos_msg.orientation;
      std::copy(cov_pos_.begin(), cov_pos_.end(), robot_odom.pose.covariance.begin());
      robot_odom.child_frame_id = "odom";
      robot_odom.twist.twist.linear.x = vx_lokal_;
      robot_odom.twist.twist.linear.y = vy_lokal_;
      robot_odom.twist.twist.angular.z = vw_lokal_;
      std::copy(cov_vel_.begin(), cov_vel_.end(), robot_odom.twist.covariance.begin());
      pub_robot_odom_->publish(robot_odom);

      // Broadcast tf robot odom
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped.header.stamp = this->now();
      transform_stamped.header.frame_id = "odom";
      transform_stamped.child_frame_id = "base_link";
      transform_stamped.transform.translation.x = robotx_;
      transform_stamped.transform.translation.y = roboty_;
      transform_stamped.transform.translation.z = 0.0;
      transform_stamped.transform.rotation = pos_msg.orientation;

      tf_broadcaster_->sendTransform(transform_stamped);
    }
  }

  void localization_process_callback()
  {
    RCLCPP_INFO(this->get_logger(), "[LOCALIZATION PROCESS LOOP]");
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_->lookupTransform("map", "base_link", rclcpp::Time(0));

      geometry_msgs::msg::Pose robot_position;
      robot_position.position.x = transform_stamped.transform.translation.x;
      robot_position.position.y = transform_stamped.transform.translation.y;
      robot_position.position.z = transform_stamped.transform.translation.z;
      robot_position.orientation.x = transform_stamped.transform.rotation.x;
      robot_position.orientation.y = transform_stamped.transform.rotation.y;
      robot_position.orientation.z = transform_stamped.transform.rotation.z;
      robot_position.orientation.w = transform_stamped.transform.rotation.w;

      pub_robot_pos_->publish(robot_position);
    }
    catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      return;
    }
  }

  // Member variables
  double min_cam_d_, max_cam_d_, freq_points_;
  double robotx_, roboty_, robotw_;
  int status_pub_points_;
  int status_init_odom_;
  double roll_, pitch_, yaw_;
  float vx_lokal_, vy_lokal_, vw_lokal_;
  float vx_global_, vy_global_, vw_global_;
  float odox_buf_, odoy_buf_, odow_buf_;
  float odox_offset_, odoy_offset_, odow_offset_;
  bool odom_data_received_;

  std::array<double, 36> cov_pos_;
  std::array<double, 36> cov_vel_;

  sensor_msgs::msg::PointCloud2 input_points2_;
  sensor_msgs::msg::PointCloud2 output_points2_;
  sensor_msgs::msg::PointCloud2 output_map_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_pcd_;

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cam_raw_;
  rclcpp::Subscription<udp_bot::msg::TerimaUdp>::SharedPtr sub_robot_odom_;
  rclcpp::Subscription<udp_bot::msg::KirimOffsetUdp>::SharedPtr sub_robot_offset_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cam_filter_;
  rclcpp::Publisher<udp_bot::msg::KirimOffsetUdp>::SharedPtr pub_robot_offset_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_robot_odom_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_robot_pos_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_init_pos_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;

  // TF
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Timers
  rclcpp::TimerBase::SharedPtr wait_timer_;
  rclcpp::TimerBase::SharedPtr robot_odom_timer_;
  rclcpp::TimerBase::SharedPtr localization_process_timer_;
  rclcpp::TimerBase::SharedPtr pub_cam_filter_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RgbdCamLocNode>());
  rclcpp::shutdown();
  return 0;
}

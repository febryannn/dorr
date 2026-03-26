#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "udp_bot/msg/terima_udp.hpp"
#include "udp_bot/msg/kirim_offset_udp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

#define TO_DEG 57.295779513
#define TO_RAD 0.01745329252

using namespace std::chrono_literals;

class OdomNode : public rclcpp::Node
{
public:
  OdomNode() : Node("odom")
  {
    odox_buf_ = odoy_buf_ = odow_buf_ = 0.0f;
    odox_offset_ = odoy_offset_ = odow_offset_ = 0.0f;
    status_init_pos_ = 0;
    robotx_ = roboty_ = robotw_ = 0.0f;
    vx_lokal_ = vy_lokal_ = vw_lokal_ = 0.0f;
    vx_global_ = vy_global_ = vw_global_ = 0.0f;
    odom_data_received_ = false;

    sub_robot_odom_ = this->create_subscription<udp_bot::msg::TerimaUdp>(
      "data_terima_udp", 10,
      std::bind(&OdomNode::robot_odom_callback, this, std::placeholders::_1));

    sub_robot_offset_ = this->create_subscription<udp_bot::msg::KirimOffsetUdp>(
      "offset_kirim_udp", 10,
      std::bind(&OdomNode::robot_offset_callback, this, std::placeholders::_1));

    pub_robot_offset_ = this->create_publisher<udp_bot::msg::KirimOffsetUdp>("offset_kirim_udp", 10);
    pub_robot_pos_ = this->create_publisher<geometry_msgs::msg::Pose>("robot_pos", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(this->get_logger(), "WAIT DATA FROM ODOMETRY");

    // In ROS2, waitForMessage is not available. Use a wait timer instead.
    wait_timer_ = this->create_wall_timer(100ms, std::bind(&OdomNode::wait_for_data, this));
  }

private:
  void wait_for_data()
  {
    if (odom_data_received_) {
      wait_timer_->cancel();
      RCLCPP_INFO(this->get_logger(), "ODOMETRY RECEIVE!!");

      // Sleep for 2 seconds then start processing
      std::this_thread::sleep_for(2s);
      RCLCPP_INFO(this->get_logger(), "[ODOMETRY LOCATION BEGIN]");

      fusion_process_timer_ = this->create_wall_timer(
        10ms, std::bind(&OdomNode::fusion_process_callback, this));
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

  void fusion_process_callback()
  {
    if (status_init_pos_ == 0)
    {
      robotx_ = roboty_ = robotw_ = 0;

      odox_offset_ = odox_buf_ - robotx_;
      odoy_offset_ = odoy_buf_ - roboty_;
      odow_offset_ = odow_buf_ - robotw_;

      auto offset_msg = udp_bot::msg::KirimOffsetUdp();
      offset_msg.posisi_x_offset = odox_offset_;
      offset_msg.posisi_y_offset = odoy_offset_;
      offset_msg.sudut_w_offset  = odow_offset_;
      pub_robot_offset_->publish(offset_msg);

      RCLCPP_INFO(this->get_logger(), "[ODOMETRY INITIAL LOCATION DONE]");
      status_init_pos_ = 1;
    }
    else
    {
      robotx_ = odox_buf_ - odox_offset_;
      roboty_ = odoy_buf_ - odoy_offset_;
      robotw_ = odow_buf_ - odow_offset_;
      while (robotw_ > 180) robotw_ -= 360;
      while (robotw_ < -180) robotw_ += 360;

      auto pos_msg = geometry_msgs::msg::Pose();
      pos_msg.position.x = robotx_;
      pos_msg.position.y = roboty_;
      tf2::Quaternion q;
      q.setRPY(0, 0, robotw_ * TO_RAD);
      pos_msg.orientation = tf2::toMsg(q);
      pub_robot_pos_->publish(pos_msg);

      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped.header.stamp = this->now();
      transform_stamped.header.frame_id = "map";
      transform_stamped.child_frame_id = "odom";

      transform_stamped.transform.translation.x = robotx_;
      transform_stamped.transform.translation.y = roboty_;
      transform_stamped.transform.translation.z = 0.0;

      transform_stamped.transform.rotation = pos_msg.orientation;

      tf_broadcaster_->sendTransform(transform_stamped);
    }
    RCLCPP_INFO(this->get_logger(), "X: %f Y: %f W: %f", robotx_, roboty_, robotw_);
  }

  // Member variables
  float odox_buf_, odoy_buf_, odow_buf_;
  float odox_offset_, odoy_offset_, odow_offset_;
  int status_init_pos_;
  float robotx_, roboty_, robotw_;
  float vx_lokal_, vy_lokal_, vw_lokal_;
  float vx_global_, vy_global_, vw_global_;
  bool odom_data_received_;

  rclcpp::Subscription<udp_bot::msg::TerimaUdp>::SharedPtr sub_robot_odom_;
  rclcpp::Subscription<udp_bot::msg::KirimOffsetUdp>::SharedPtr sub_robot_offset_;
  rclcpp::Publisher<udp_bot::msg::KirimOffsetUdp>::SharedPtr pub_robot_offset_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_robot_pos_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::TimerBase::SharedPtr wait_timer_;
  rclcpp::TimerBase::SharedPtr fusion_process_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomNode>());
  rclcpp::shutdown();
  return 0;
}

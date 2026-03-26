#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>

#include <eigen3/Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "udp_bot/msg/terima_udp.hpp"
#include "udp_bot/msg/kirim_offset_udp.hpp"
// TODO: marvelmind_nav is not available in ROS2 yet.
// Uncomment the following includes when a ROS2 port becomes available.
// #include "marvelmind_nav/msg/hedge_pos_ang.hpp"
// #include "marvelmind_nav/msg/hedge_quality.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "lokalisasi/fusion_gps_odom.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace Eigen;
using namespace std::chrono_literals;

#define TO_DEG 57.295779513
#define TO_RAD 0.01745329252

#define r_robot_gps 0.0    ////jarak beacon ke pusat robot 0.0m
#define offset_gpsx 0.0    //4m
#define offset_gpsy 0.0    //5m
#define gps_f_sample 0.0

class FusionGpsOdomNode : public rclcpp::Node
{
public:
  FusionGpsOdomNode() : Node("fusion_gps_odom")
  {
    odox_buf_ = odoy_buf_ = odow_buf_ = 0.0f;
    odox_offset_ = odoy_offset_ = odow_offset_ = 0.0f;
    gpsx_buf_ = gpsy_buf_ = gpsw_buf_ = 0.0f;
    gpsx_ = gpsy_ = gpsw_ = 0.0f;
    gpsx_prev_ = gpsy_prev_ = gpsw_prev_ = 0.0f;
    gps_quality_ = gps_q_ = 0.0f;
    status_init_pos_ = 0;
    robotx_ = roboty_ = robotw_ = 0.0f;
    vx_lokal_ = vy_lokal_ = vw_lokal_ = 0.0f;
    vx_global_ = vy_global_ = vw_global_ = 0.0f;
    vx_gps_ = vy_gps_ = vw_gps_ = 0.0f;
    w_mean_ = w_mean_sin_ = w_mean_cos_ = 0.0f;
    odom_data_received_ = false;
    // TODO: gps_data_received_ and quality_data_received_ for marvelmind
    // gps_data_received_ = false;
    // quality_data_received_ = false;

    sub_robot_odom_ = this->create_subscription<udp_bot::msg::TerimaUdp>(
      "data_terima_udp", 10,
      std::bind(&FusionGpsOdomNode::robot_odom_callback, this, std::placeholders::_1));

    // TODO: marvelmind_nav subscriptions - uncomment when available in ROS2
    // sub_robot_hedge_ = this->create_subscription<marvelmind_nav::msg::HedgePosAng>(
    //   "hedge_pos_ang", 10,
    //   std::bind(&FusionGpsOdomNode::robot_hedge_callback, this, std::placeholders::_1));
    //
    // sub_hedge_quality_ = this->create_subscription<marvelmind_nav::msg::HedgeQuality>(
    //   "hedge_quality", 10,
    //   std::bind(&FusionGpsOdomNode::hedge_quality_callback, this, std::placeholders::_1));

    pub_robot_offset_ = this->create_publisher<udp_bot::msg::KirimOffsetUdp>("offset_kirim_udp", 10);
    pub_robot_pos_ = this->create_publisher<geometry_msgs::msg::Pose>("robot_pos", 10);
    pub_robot_gps_ = this->create_publisher<geometry_msgs::msg::Pose>("robot_gps", 10);
    pub_robot_odo_ = this->create_publisher<geometry_msgs::msg::Pose>("robot_odo", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(this->get_logger(), "WAIT DATA FROM INDOOR GPS AND ODOMETRY");

    // In ROS2, waitForMessage is not available. Use a wait timer instead.
    wait_timer_ = this->create_wall_timer(100ms, std::bind(&FusionGpsOdomNode::wait_for_data, this));
  }

private:
  void wait_for_data()
  {
    // TODO: Also check gps_data_received_ and quality_data_received_ when marvelmind is available
    if (odom_data_received_) {
      wait_timer_->cancel();
      RCLCPP_INFO(this->get_logger(), "INDOOR GPS AND ODOMETRY RECEIVE!!");

      // Sleep for 4 seconds then start processing
      std::this_thread::sleep_for(4s);
      RCLCPP_INFO(this->get_logger(), "[SENSOR FUSION BEGIN]");

      fusion_process_timer_ = this->create_wall_timer(
        10ms, std::bind(&FusionGpsOdomNode::fusion_process_callback, this));
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

  // TODO: marvelmind_nav callbacks - uncomment when available in ROS2
  // void robot_hedge_callback(const marvelmind_nav::msg::HedgePosAng::SharedPtr msg)
  // {
  //   gpsw_buf_ = (float)msg->angle;
  //   gpsx_buf_ = (float)msg->x_m - (cosf(gpsw_buf_ * TO_RAD) * r_robot_gps) - offset_gpsx;
  //   gpsy_buf_ = (float)msg->y_m - (sinf(gpsw_buf_ * TO_RAD) * r_robot_gps) - offset_gpsy;
  //
  //   vx_gps_ = (gpsx_buf_ - gpsx_prev_) * gps_f_sample;
  //   vy_gps_ = (gpsy_buf_ - gpsy_prev_) * gps_f_sample;
  //   float vw_temp;
  //   vw_temp = gpsw_buf_ - gpsw_prev_;
  //   while (vw_temp > 180) vw_temp -= 360;
  //   while (vw_temp < -180) vw_temp += 360;
  //   vw_gps_ = vw_temp * TO_RAD * gps_f_sample;
  //
  //   gpsx_prev_ = gpsx_buf_;
  //   gpsy_prev_ = gpsy_buf_;
  //   gpsw_prev_ = gpsw_buf_;
  //
  //   gps_data_received_ = true;
  // }
  //
  // void hedge_quality_callback(const marvelmind_nav::msg::HedgeQuality::SharedPtr msg)
  // {
  //   gps_quality_ = (float)(msg->quality_percents) / 100;
  //   quality_data_received_ = true;
  // }

  void fusion_process_callback()
  {
    gpsx_ = gpsx_buf_;
    gpsy_ = gpsy_buf_;
    gpsw_ = gpsw_buf_;
    gps_q_ = gps_quality_;

    if (status_init_pos_ == 0)  ////untuk inisialisasi posisi dan orientasi secara global
    {
      RCLCPP_INFO(this->get_logger(), "WAIT HIGH QUALITY INDOOR GPS DATA......");
      if (gps_q_ > 0.95)
      {
        ///unutk offset inisialisasi (-x, -y, -w)
        odox_offset_ = odox_buf_ - gpsx_;
        odoy_offset_ = odoy_buf_ - gpsy_;
        odow_offset_ = odow_buf_ - gpsw_;

        auto offset_msg = udp_bot::msg::KirimOffsetUdp();
        offset_msg.posisi_x_offset = odox_offset_;
        offset_msg.posisi_y_offset = odoy_offset_;
        offset_msg.sudut_w_offset  = odow_offset_;
        pub_robot_offset_->publish(offset_msg);

        robotx_ = odox_buf_ - odox_offset_;
        roboty_ = odoy_buf_ - odoy_offset_;
        robotw_ = odow_buf_ - odow_offset_;
        while (robotw_ > 180) robotw_ -= 360;
        while (robotw_ < -180) robotw_ += 360;

        RCLCPP_INFO(this->get_logger(), "ROBOT POS INIT DONE!!!!!");
        status_init_pos_ = 1;

        Xk_prev(0) = robotx_;
        Xk_prev(1) = roboty_;
        Xk_prev(2) = robotw_;
      }
    }
    else  // untuk continuous localization
    {
      Uk(0) = vx_global_; Uk(1) = vy_global_; Uk(2) = vw_global_ * TO_DEG;
      Xkp = A * Xk_prev + B * Uk + Wk;
      while (Xkp(2) > 180) Xkp(2) -= 360;
      while (Xkp(2) < -180) Xkp(2) += 360;

      Pkp = A * Pk_prev * A.transpose() + Qk;
      Pkp = Pkp.array() * I.array();  // just take diagonal variance

      Gps_Q(0, 0) = exp(-(pow((gpsx_ - Xkp(0)) / 0.02, 2)));
      Gps_Q(1, 1) = exp(-(pow((gpsy_ - Xkp(1)) / 0.02, 2)));
      Gps_Q(2, 2) = exp(-(pow((gpsw_ - Xkp(2)) / 10.0, 2)));
      R = (I - Gps_Q * H) * R_init + Gps_Q * (H * Pkp * H.transpose());

      Kg = H * Pkp * H.transpose() + R;
      Kg = Pkp * H.transpose() * Kg.inverse();

      yk(0) = gpsx_; yk(1) = gpsy_; yk(2) = gpsw_;
      Yk = C * yk + Zk;

      while (Yk(2) - Xkp(2) > 180) { Yk(2) -= 360; }
      while (Yk(2) - Xkp(2) < -180) { Yk(2) += 360; }
      Xk = Xkp + Kg * (Yk - H * Xkp);
      while (Xk(2) > 180) Xk(2) -= 360;
      while (Xk(2) < -180) Xk(2) += 360;

      Pk = (I - Kg * H) * Pkp;

      Xk_prev = Xk;
      Pk_prev = Pk;

      robotx_ = Xk(0);
      roboty_ = Xk(1);
      robotw_ = Xk(2);
      RCLCPP_INFO(this->get_logger(), "X: %f Y: %f W: %f", robotx_, roboty_, robotw_);

      odox_offset_ = odox_buf_ - robotx_;
      odoy_offset_ = odoy_buf_ - roboty_;
      odow_offset_ = odow_buf_ - robotw_;

      auto offset_msg = udp_bot::msg::KirimOffsetUdp();
      offset_msg.posisi_x_offset = odox_offset_;
      offset_msg.posisi_y_offset = odoy_offset_;
      offset_msg.sudut_w_offset  = odow_offset_;
      pub_robot_offset_->publish(offset_msg);
    }

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

    auto gps_msg = geometry_msgs::msg::Pose();
    gps_msg.position.x = gpsx_;
    gps_msg.position.y = gpsy_;
    tf2::Quaternion q_gps;
    q_gps.setRPY(0, 0, gpsw_ * TO_RAD);
    gps_msg.orientation = tf2::toMsg(q_gps);
    pub_robot_gps_->publish(gps_msg);

    auto odo_msg = geometry_msgs::msg::Pose();
    odo_msg.position.x = Xkp(0);
    odo_msg.position.y = Xkp(1);
    tf2::Quaternion q_odo;
    q_odo.setRPY(0, 0, Xkp(2) * TO_RAD);
    odo_msg.orientation = tf2::toMsg(q_odo);
    pub_robot_odo_->publish(odo_msg);
  }

  // Member variables
  float odox_buf_, odoy_buf_, odow_buf_;
  float odox_offset_, odoy_offset_, odow_offset_;
  float gpsx_buf_, gpsy_buf_, gpsw_buf_;
  float gpsx_, gpsy_, gpsw_;
  float gpsx_prev_, gpsy_prev_, gpsw_prev_;
  float gps_quality_, gps_q_;
  int status_init_pos_;
  float robotx_, roboty_, robotw_;
  float vx_lokal_, vy_lokal_, vw_lokal_;
  float vx_global_, vy_global_, vw_global_;
  float vx_gps_, vy_gps_, vw_gps_;
  float w_mean_, w_mean_sin_, w_mean_cos_;
  bool odom_data_received_;
  // bool gps_data_received_;
  // bool quality_data_received_;

  rclcpp::Subscription<udp_bot::msg::TerimaUdp>::SharedPtr sub_robot_odom_;
  // TODO: marvelmind_nav subscription types - uncomment when available
  // rclcpp::Subscription<marvelmind_nav::msg::HedgePosAng>::SharedPtr sub_robot_hedge_;
  // rclcpp::Subscription<marvelmind_nav::msg::HedgeQuality>::SharedPtr sub_hedge_quality_;

  rclcpp::Publisher<udp_bot::msg::KirimOffsetUdp>::SharedPtr pub_robot_offset_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_robot_pos_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_robot_gps_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_robot_odo_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::TimerBase::SharedPtr wait_timer_;
  rclcpp::TimerBase::SharedPtr fusion_process_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FusionGpsOdomNode>());
  rclcpp::shutdown();
  return 0;
}

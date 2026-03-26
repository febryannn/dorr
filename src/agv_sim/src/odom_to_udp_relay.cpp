#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "udp_bot_msgs/msg/terima_udp.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class OdomToUdpRelay : public rclcpp::Node {
public:
    OdomToUdpRelay() : Node("odom_to_udp_relay") {
        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&OdomToUdpRelay::odom_cb, this, std::placeholders::_1));
        sub_vel_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&OdomToUdpRelay::vel_cb, this, std::placeholders::_1));
        pub_ = create_publisher<udp_bot_msgs::msg::TerimaUdp>("data_terima_udp", 10);
        RCLCPP_INFO(get_logger(), "[RELAY] /odom + /cmd_vel -> data_terima_udp");
    }

private:
    void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
        out_.posisi_x_buffer = (float)msg->pose.pose.position.x;
        out_.posisi_y_buffer = (float)msg->pose.pose.position.y;
        tf2::Quaternion q(
            msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        double r, p, y;
        tf2::Matrix3x3(q).getRPY(r, p, y);
        out_.sudut_w_buffer = (float)(y * 180.0 / M_PI);
        double ovx = msg->twist.twist.linear.x;
        double ovw = msg->twist.twist.angular.z;
        if (std::abs(ovx) > 1e-3 || std::abs(ovw) > 1e-3) {
            out_.kecepatan_robotx = (float)ovx;
            out_.kecepatan_roboty = 0.0f;
            out_.kecepatan_robotw = (float)ovw;
            out_.vx_global = (float)ovx;
            out_.vy_global = (float)msg->twist.twist.linear.y;
            out_.vw_global = (float)ovw;
        }
        pub_->publish(out_);
    }

    void vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg) {
        out_.kecepatan_robotx = (float)msg->linear.x;
        out_.kecepatan_roboty = (float)msg->linear.y;
        out_.kecepatan_robotw = (float)msg->angular.z;
        out_.vx_global = (float)msg->linear.x;
        out_.vy_global = (float)msg->linear.y;
        out_.vw_global = (float)msg->angular.z;
        pub_->publish(out_);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_vel_;
    rclcpp::Publisher<udp_bot_msgs::msg::TerimaUdp>::SharedPtr pub_;
    udp_bot_msgs::msg::TerimaUdp out_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomToUdpRelay>());
    rclcpp::shutdown();
    return 0;
}

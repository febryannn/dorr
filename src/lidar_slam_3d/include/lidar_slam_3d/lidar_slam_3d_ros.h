#ifndef LIDAR_SLAM_3D_ROS_H
#define LIDAR_SLAM_3D_ROS_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <thread>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_srvs/srv/empty.hpp>
#include "map_builder.h"
#include "floor_filter.h"

typedef Eigen::Matrix<float, 6, 1> Vector6f;

class LidarSlam3dRos : public rclcpp::Node
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LidarSlam3dRos();
    ~LidarSlam3dRos() {}

private:
    struct dual_pose {
        Vector6f offset_pose, robot_pose;
    };
    dual_pose getPose(const Eigen::Matrix4f& T);

    void publishLoop();
    void publishMap();
    void publishConstraintList();
    void publishPose(const Vector6f& pose, const rclcpp::Time& t);
    void publishPath(const Vector6f& pose, const rclcpp::Time& t);
    void publishTf(const Vector6f& pose, const rclcpp::Time& t);
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
    void laserCallback1(const sensor_msgs::msg::LaserScan::SharedPtr laser_msg);
    void laserCallback2(const sensor_msgs::msg::LaserScan::SharedPtr laser_msg);
    void laserCallback3(const sensor_msgs::msg::LaserScan::SharedPtr laser_msg);
    void laserCallback4(const sensor_msgs::msg::LaserScan::SharedPtr laser_msg);

private:
    std::shared_ptr<std::thread> publish_thread_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_point_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr floor_points_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr constraint_list_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_1, laser_sub_2, laser_sub_3, laser_sub_4;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    nav_msgs::msg::Path path_msg_;

    laser_geometry::LaserProjection projector_laser;
    pcl::PointCloud<pcl::PointXYZ> laser_xyz1, laser_xyz2, laser_xyz3, laser_xyz4;

    Eigen::Matrix4f odom_pose_, odom_offset_;
    double odom_roll, odom_pitch, odom_yaw;
    Eigen::Matrix4f sensor_pose_;
    double sensor_roll, sensor_pitch, sensor_yaw;

    std::string base_frame_;
    std::string map_frame_;
    std::string odom_frame_;
    std::string sensor_frame_;
    double publish_freq_;
    double min_scan_distance_;
    bool enable_floor_filter_;

    lidar_slam_3d::FloorFilter floor_filter_;
    lidar_slam_3d::MapBuilder map_builder_;
};

#endif // LIDAR_SLAM_3D_ROS_H

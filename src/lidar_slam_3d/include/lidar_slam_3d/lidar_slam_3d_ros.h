#ifndef LIDAR_SLAM_3D_ROS_H
#define LIDAR_SLAM_3D_ROS_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <thread>
#include <tf/tf.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_srvs/Empty.h>
#include "map_builder.h"
#include "floor_filter.h"

typedef Eigen::Matrix<float, 6, 1> Vector6f;

class LidarSlam3dRos
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
    void publishPose(const Vector6f& pose, const ros::Time& t);
    void publishPath(const Vector6f& pose, const ros::Time& t);
    void publishTf(const Vector6f& pose, const ros::Time& t);
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
    void laserCallback1(const sensor_msgs::LaserScan::ConstPtr& laser_msg);
    void laserCallback2(const sensor_msgs::LaserScan::ConstPtr& laser_msg);
    void laserCallback3(const sensor_msgs::LaserScan::ConstPtr& laser_msg);

private:
    std::shared_ptr<std::thread> publish_thread_;
    ros::Publisher map_pub_;
    ros::Publisher path_pub_;
    ros::Publisher pose_pub_;
    ros::Publisher filtered_point_cloud_pub_;
    ros::Publisher floor_points_pub_;
    ros::Publisher constraint_list_pub_;
    ros::Subscriber point_cloud_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber laser_sub_1, laser_sub_2, laser_sub_3;
    tf::TransformBroadcaster tf_broadcaster_;
    tf2_ros::TransformListener* tf_listener;
    tf2_ros::Buffer tf_listener_buffer;
    nav_msgs::Path path_msg_;

    tf::TransformListener listener_laser1, listener_laser2, listener_laser3;
    laser_geometry::LaserProjection projector_laser;
    pcl::PointCloud<pcl::PointXYZ> laser_xyz1, laser_xyz2, laser_xyz3;

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

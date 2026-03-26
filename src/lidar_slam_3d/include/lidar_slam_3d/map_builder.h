#ifndef MAP_BUILDER_H
#define MAP_BUILDER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/crop_box.h>
#include <g2o/core/sparse_optimizer.h>
#include <chrono>
#include <mutex>
#include "math_func.h"
#include "key_frame.h"
#include <cmath>

namespace lidar_slam_3d
{

class MapBuilder
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MapBuilder();
    ~MapBuilder() {}

    void addPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& point_cloud, Eigen::Matrix4f odom_pose, 
                       Eigen::Matrix4f odom_offset, pcl::PointCloud<pcl::PointXYZ> laser_cloud1, pcl::PointCloud<pcl::PointXYZ> laser_cloud2, pcl::PointCloud<pcl::PointXYZ> laser_cloud3);

    Eigen::Matrix4f getTransformation() { return pose_; }
    void getMap(sensor_msgs::PointCloud2& map_msg)
    {
        std::unique_lock<std::mutex> locker(map_mutex_);
        pcl::toROSMsg(map_, map_msg);
    }
    void getPoseGraph(std::vector<Eigen::Vector3d>& nodes,
                      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& edges);

private:
    void downSample(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr& sampled_cloud);

private:
    pcl::PointCloud<pcl::PointXYZI> map_;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> submap_;

    pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt_;
    g2o::SparseOptimizer optimizer_;

    Eigen::Matrix4f pose_;
    Eigen::Matrix4f last_update_pose_;
    float voxel_grid_leaf_size_;
    float map_update_distance_;
    int submap_size_;
    int sequence_num_;
    bool first_point_cloud_;
    std::mutex map_mutex_;
};

} // namespace lidar_slam_3d

#endif // MAP_BUILDER_H

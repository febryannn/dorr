#include <iostream>
#include <cmath>
#include <mutex>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/crop_box.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_hull.h>

class SimpleMappingNode : public rclcpp::Node
{
public:
  SimpleMappingNode()
  : Node("simple_mapping"),
    status_pub_points_(0),
    status_map_time_(0),
    status_update_map_(0),
    status_map_build_(0),
    map_cloud_(new pcl::PCLPointCloud2()),
    map_cloud_temp_(new pcl::PCLPointCloud2()),
    map_hull_pub_(new pcl::PCLPointCloud2()),
    accumulated_cloud_(new pcl::PCLPointCloud2())
  {
    // Declare parameters
    this->declare_parameter("maxf_points_out", 1.0);
    this->declare_parameter("maxf_map_out", 1.0);
    this->declare_parameter("min_cam_d", 0.6);
    this->declare_parameter("max_cam_d", 8.0);

    freq_points_ = this->get_parameter("maxf_points_out").as_double();
    freq_map_ = this->get_parameter("maxf_map_out").as_double();
    min_cam_d_ = this->get_parameter("min_cam_d").as_double();
    max_cam_d_ = this->get_parameter("max_cam_d").as_double();

    RCLCPP_INFO(this->get_logger(), "WAIT DATA FROM ROBOT POS & RGBD CAMERA");

    // Subscribers and Publishers
    sub_cam_raw_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "camera/depth_registered/points", 10,
      std::bind(&SimpleMappingNode::cam_filter_callback, this, std::placeholders::_1));

    pub_cam_filter_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cam_pcd", 10);

    pub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_pcd", 10);

    RCLCPP_INFO(this->get_logger(), "[MAPPING PROCESS BEGIN]");

    // Timers
    auto map_period = std::chrono::duration<double>(1.0 / freq_map_);
    mapping_process_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(map_period),
      std::bind(&SimpleMappingNode::mapping_process_callback, this));

    auto filter_period = std::chrono::duration<double>(1.0 / freq_points_);
    pub_cam_filter_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(filter_period),
      std::bind(&SimpleMappingNode::pub_cam_filter_callback, this));

    save_map_timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&SimpleMappingNode::save_map_callback, this));
  }

private:
  void cam_filter_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    input_points2_ = *msg;
    status_pub_points_ = 1;
  }

  void pub_cam_filter_callback()
  {
    if (status_pub_points_ == 1) {
      pcl::PCLPointCloud2::Ptr cam_cloud(new pcl::PCLPointCloud2());
      pcl::PCLPointCloud2::Ptr cam_cloud_filtered(new pcl::PCLPointCloud2());
      pcl::PointCloud<pcl::PointXYZ>::Ptr cam_cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());

      ///////// Convert and filtering object///////////////
      pcl_conversions::toPCL(input_points2_, *cam_cloud);
      ///////////crop filter
      pcl::PointXYZ minPt, maxPt;
      pcl::fromPCLPointCloud2(*cam_cloud, *cam_cloud_xyz);
      pcl::getMinMax3D(*cam_cloud_xyz, minPt, maxPt);
      pcl::CropBox<pcl::PCLPointCloud2> cb;
      cb.setInputCloud(cam_cloud);
      cb.setMin(Eigen::Vector4f(minPt.x, minPt.y, min_cam_d_, 0.));
      cb.setMax(Eigen::Vector4f(maxPt.x, maxPt.y, max_cam_d_, 0.));
      cb.filter(*cam_cloud);
      /////voxel grid filter
      pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
      vg.setInputCloud(cam_cloud);
      vg.setLeafSize(0.02f, 0.02f, 0.02f);
      vg.filter(*cam_cloud_filtered);

      sensor_msgs::msg::PointCloud2 filtered_msg;
      pcl_conversions::fromPCL(*cam_cloud_filtered, filtered_msg);
      pub_cam_filter_->publish(filtered_msg);

      // Accumulate filtered point cloud internally (replaces laser_assembler)
      {
        std::lock_guard<std::mutex> lock(accumulation_mutex_);
        if (accumulated_cloud_->width == 0) {
          *accumulated_cloud_ = *cam_cloud_filtered;
        } else {
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr acc_xyz(new pcl::PointCloud<pcl::PointXYZRGB>());
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_xyz(new pcl::PointCloud<pcl::PointXYZRGB>());
          pcl::fromPCLPointCloud2(*accumulated_cloud_, *acc_xyz);
          pcl::fromPCLPointCloud2(*cam_cloud_filtered, *new_xyz);
          *acc_xyz += *new_xyz;
          pcl::toPCLPointCloud2(*acc_xyz, *accumulated_cloud_);
        }
      }

      status_pub_points_ = 0;
    }
  }

  void mapping_process_callback()
  {
    if (status_map_time_ == 0) {
      RCLCPP_INFO(this->get_logger(), "[MAPPING PROCESS LOOP]");
      status_map_time_ = 1;
    } else if (status_map_time_ == 1) {
      // Get accumulated cloud (replaces laser_assembler service call)
      pcl::PCLPointCloud2::Ptr assembled_cloud(new pcl::PCLPointCloud2());
      {
        std::lock_guard<std::mutex> lock(accumulation_mutex_);
        if (accumulated_cloud_->width == 0) {
          RCLCPP_INFO(this->get_logger(), "No accumulated points yet");
          return;
        }
        *assembled_cloud = *accumulated_cloud_;
        // Reset accumulation
        accumulated_cloud_.reset(new pcl::PCLPointCloud2());
      }

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr assembled_xyz(new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl::fromPCLPointCloud2(*assembled_cloud, *assembled_xyz);
      RCLCPP_INFO(this->get_logger(), "Got cloud with %zu points", assembled_xyz->size());

      if (assembled_xyz->size() != 0) {
        *map_cloud_temp_ = *assembled_cloud;

        ////////filter voxel grid/////////
        pcl::VoxelGrid<pcl::PCLPointCloud2> vg1;
        vg1.setInputCloud(map_cloud_temp_);
        vg1.setLeafSize(0.02f, 0.02f, 0.02f);
        vg1.filter(*map_cloud_temp_);
        //////////////Statistical Outlier Removal Filter//////////////////////////
        pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
        sor.setInputCloud(map_cloud_temp_);
        sor.setMeanK(30);
        sor.setStddevMulThresh(2.0);
        sor.filter(*map_cloud_temp_);

        ////////////////////////////////////////////////////////
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_hull(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_temp_hull(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        pcl::fromPCLPointCloud2(*map_cloud_, *map_hull);
        pcl::fromPCLPointCloud2(*map_cloud_temp_, *map_temp_hull);
        ///////////add 2 point near camera for concave hull//////
        pcl::PointXYZRGB pt;
        pt.x = 0.8; pt.y = 0.38; pt.z = 0.0;
        pt.r = pt.g = pt.b = 0;
        map_temp_hull->push_back(pt);
        pt.x = 0.8; pt.y = -0.38; pt.z = 0.0;
        pt.r = pt.g = pt.b = 0;
        map_temp_hull->push_back(pt);
        //////////////Create a set of planar coefficients with X=Y=0,Z=1
        //// with equation: ax + by + cz + d = 0
        coefficients->values.resize(4);
        coefficients->values[0] = coefficients->values[1] = 0;
        coefficients->values[2] = 1.0;
        coefficients->values[3] = 0.0;
        // Project the model coefficients
        pcl::ProjectInliers<pcl::PointXYZRGB> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(map_temp_hull);
        proj.setModelCoefficients(coefficients);
        proj.filter(*map_temp_hull);
        // Calculate Hull
        pcl::ConcaveHull<pcl::PointXYZRGB> hull_calculator;
        std::vector<pcl::Vertices> polygons;
        hull_calculator.setInputCloud(map_temp_hull);
        hull_calculator.setAlpha(10.0);
        hull_calculator.reconstruct(*map_temp_hull, polygons);
        int dim = hull_calculator.getDimension();
        // Crop Hull
        pcl::CropHull<pcl::PointXYZRGB> crop_filter;
        crop_filter.setInputCloud(map_hull);
        crop_filter.setHullCloud(map_temp_hull);
        crop_filter.setHullIndices(polygons);
        crop_filter.setDim(dim);
        crop_filter.setCropOutside(false);
        crop_filter.filter(*map_hull);

        pcl::fromPCLPointCloud2(*map_cloud_temp_, *map_temp_hull);
        *map_hull += *map_temp_hull;
        pcl::toPCLPointCloud2(*map_hull, *map_cloud_);
        map_cloud_->header.frame_id = "map";
        ////////filter voxel grid/////////
        pcl::VoxelGrid<pcl::PCLPointCloud2> vg2;
        vg2.setInputCloud(map_cloud_);
        vg2.setLeafSize(0.02f, 0.02f, 0.02f);
        vg2.filter(*map_cloud_);
        ///for convert and publish map in sensor_msgs::msg::PointCloud2
        pcl_conversions::fromPCL(*map_cloud_, map_pub_);
        RCLCPP_INFO(this->get_logger(), "Now MAP have %d points",
                     map_pub_.height * map_pub_.width);
        status_map_build_ = 1;
        pub_map_->publish(map_pub_);
      }
    }
  }

  void save_map_callback()
  {
    if (status_map_build_ == 1) {
      /////////for save map in pcd file
      pcl::PointCloud<pcl::PointXYZRGB> map_save;
      pcl::fromPCLPointCloud2(*map_cloud_, map_save);
      pcl::io::savePCDFileASCII("/home/agv1/agv_ws/map_p206.pcd", map_save);
    }
  }

  // Subscribers / Publishers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cam_raw_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cam_filter_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;

  // Timers
  rclcpp::TimerBase::SharedPtr mapping_process_timer_;
  rclcpp::TimerBase::SharedPtr pub_cam_filter_timer_;
  rclcpp::TimerBase::SharedPtr save_map_timer_;

  // Data
  sensor_msgs::msg::PointCloud2 input_points2_;
  sensor_msgs::msg::PointCloud2 map_pub_;
  int status_pub_points_;
  int status_map_time_;
  int status_update_map_;
  int status_map_build_;
  double freq_points_, freq_map_, min_cam_d_, max_cam_d_;

  pcl::PCLPointCloud2::Ptr map_cloud_;
  pcl::PCLPointCloud2::Ptr map_cloud_temp_;
  pcl::PCLPointCloud2::Ptr map_hull_pub_;

  // Accumulated point cloud (replaces laser_assembler)
  pcl::PCLPointCloud2::Ptr accumulated_cloud_;
  std::mutex accumulation_mutex_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleMappingNode>());
  rclcpp::shutdown();
  return 0;
}

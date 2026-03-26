#include <iostream>
#include <cmath>
#include <mutex>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "mapping/srv/input_pos.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"

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

#define TO_DEG 57.295779513
#define TO_RAD 0.01745329252

class ManualMappingNode : public rclcpp::Node
{
public:
  ManualMappingNode()
  : Node("manual_mapping"),
    status_pub_points_(0),
    status_map_time_(0),
    status_update_map_(0),
    status_map_build_(0),
    robot_posx_(0.0f),
    robot_posy_(0.0f),
    robot_posw_(0.0f),
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
      "full_pointcloud", 10,
      std::bind(&ManualMappingNode::cam_filter_callback, this, std::placeholders::_1));

    pub_cam_filter_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cam_pcd", 10);
    pub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_pcd", 10);

    // Transform broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    RCLCPP_INFO(this->get_logger(), "[MAPPING PROCESS BEGIN]");

    // Service server
    update_pos_srv_ = this->create_service<mapping::srv::InputPos>(
      "update_pos",
      std::bind(&ManualMappingNode::update_pos_callback, this,
                std::placeholders::_1, std::placeholders::_2));

    // Timers
    auto filter_period = std::chrono::duration<double>(1.0 / freq_points_);
    pub_cam_filter_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(filter_period),
      std::bind(&ManualMappingNode::pub_cam_filter_callback, this));

    robot_pos_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&ManualMappingNode::robot_pos_callback, this));

    auto map_period = std::chrono::duration<double>(1.0 / freq_map_);
    mapping_process_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(map_period),
      std::bind(&ManualMappingNode::mapping_process_callback, this));

    save_map_timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&ManualMappingNode::save_map_callback, this));
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
      cb.setMin(Eigen::Vector4f(min_cam_d_, minPt.y, minPt.z, 0.));
      cb.setMax(Eigen::Vector4f(max_cam_d_, maxPt.y, maxPt.z, 0.));
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
    if (status_map_time_ == 1) {
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

        pcl::fromPCLPointCloud2(*map_cloud_, *map_hull);
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
        status_map_time_ = 0;
      }
    }
  }

  void save_map_callback()
  {
    if (status_map_build_ == 1) {
      /////////for save map in pcd file
      pcl::PointCloud<pcl::PointXYZRGB> map_save;
      pcl::fromPCLPointCloud2(*map_cloud_, map_save);
      pcl::io::savePCDFileASCII("/home/agv1/alfian/agv_ws/map_p206_final_test.pcd", map_save);
    }
  }

  void update_pos_callback(
    const std::shared_ptr<mapping::srv::InputPos::Request> request,
    std::shared_ptr<mapping::srv::InputPos::Response> response)
  {
    robot_posx_ = request->posx;
    robot_posy_ = request->posy;
    robot_posw_ = request->posw;

    response->status_pos = true;

    // Sleep for 2 seconds then start mapping
    rclcpp::sleep_for(std::chrono::seconds(2));
    status_map_time_ = 1;
    RCLCPP_INFO(this->get_logger(), "X: %f Y: %f W: %f",
                 robot_posx_, robot_posy_, robot_posw_);
  }

  void robot_pos_callback()
  {
    geometry_msgs::msg::Pose pos_msg;
    pos_msg.position.x = robot_posx_;
    pos_msg.position.y = robot_posy_;
    tf2::Quaternion q;
    q.setRPY(0, 0, robot_posw_ * TO_RAD);
    pos_msg.orientation = tf2::toMsg(q);

    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = this->now();
    transform_stamped.header.frame_id = "map";
    transform_stamped.child_frame_id = "odom";
    transform_stamped.transform.translation.x = robot_posx_;
    transform_stamped.transform.translation.y = robot_posy_;
    transform_stamped.transform.translation.z = 0.0;
    transform_stamped.transform.rotation = pos_msg.orientation;
    // Broadcast the transform
    tf_broadcaster_->sendTransform(transform_stamped);
  }

  // Subscribers / Publishers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cam_raw_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cam_filter_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;

  // Service
  rclcpp::Service<mapping::srv::InputPos>::SharedPtr update_pos_srv_;

  // Transform broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Timers
  rclcpp::TimerBase::SharedPtr mapping_process_timer_;
  rclcpp::TimerBase::SharedPtr pub_cam_filter_timer_;
  rclcpp::TimerBase::SharedPtr robot_pos_timer_;
  rclcpp::TimerBase::SharedPtr save_map_timer_;

  // Data
  sensor_msgs::msg::PointCloud2 input_points2_;
  sensor_msgs::msg::PointCloud2 map_pub_;
  int status_pub_points_;
  int status_map_time_;
  int status_update_map_;
  int status_map_build_;
  float robot_posx_, robot_posy_, robot_posw_;
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
  rclcpp::spin(std::make_shared<ManualMappingNode>());
  rclcpp::shutdown();
  return 0;
}

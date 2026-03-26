#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/common/common.h>
#include <pcl_ros/point_cloud.h>
#include <pointcloud_to_grid_core.hpp>

nav_msgs::OccupancyGridPtr intensity_grid(new nav_msgs::OccupancyGrid);
nav_msgs::OccupancyGridPtr height_grid(new nav_msgs::OccupancyGrid);
GridMap grid_map;
ros::Publisher pub_igrid, pub_hgrid;
ros::Subscriber sub_pc2;

double cell_size_;
double position_x_, position_y_;
double length_x_, length_y_;
std::string cloud_in_topic_;
double intensity_factor_, height_factor_;
// std::string frame_out_;
std::string mapi_topic_name_, maph_topic_name_;

pcl::PointCloud<pcl::PointXYZ>::Ptr cam_cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointXYZ minPt, maxPt;

PointXY getIndex(double x, double y){
  PointXY ret;
  ret.x = int(fabs(x - grid_map.topleft_x) / grid_map.cell_size);
  ret.y = int(fabs(y - grid_map.topleft_y) / grid_map.cell_size);
  return ret;
}

void pointcloudCallback(const pcl::PCLPointCloud2 &msg)
{
  pcl::fromPCLPointCloud2(msg, *cam_cloud_xyz);
  if(cam_cloud_xyz->size() < 10){
    ROS_INFO_STREAM("pcd xyz points < 10 Points");
    return;
  }
  pcl::getMinMax3D (*cam_cloud_xyz, minPt, maxPt);
  grid_map.position_x = minPt.x + (maxPt.x - minPt.x)/2;
  grid_map.position_y = minPt.y + (maxPt.y - minPt.y)/2;
  grid_map.length_x = maxPt.x - minPt.x + 2.0;
  grid_map.length_y = maxPt.y - minPt.y + 2.0;
  grid_map.paramRefresh();

  pcl::PointCloud<pcl::PointXYZI> out_cloud;
  pcl::fromPCLPointCloud2(msg, out_cloud);
  // Initialize grid
  grid_map.initGrid(intensity_grid);
  grid_map.initGrid(height_grid);
  // width*height/cell_size^2 eg width = 20m, height = 30m, cell_size data size = 6000
  // or cell_num_x * cell_num_ys
  // -128 127 int8[] data
  std::vector<signed char> hpoints(grid_map.cell_num_x * grid_map.cell_num_y);
  std::vector<signed char> ipoints(grid_map.cell_num_x * grid_map.cell_num_y);
  // initialize grid vectors: -128
  for (auto& p : hpoints){p = 0;}
  for (auto& p : ipoints){p = 0;}
  //for (int i = 0; i < out_cloud.points.size(); ++i) // out_cloud.points[i].x instead of out_point.x
  for (auto out_point : out_cloud)
  {
    if (out_point.x > 0.01 || out_point.x < -0.01){
      if (out_point.x > grid_map.bottomright_x && out_point.x < grid_map.topleft_x)
      {
        if (out_point.y > grid_map.bottomright_y && out_point.y < grid_map.topleft_y)
        {
          PointXY cell = getIndex(out_point.x, out_point.y);
          if (cell.x < grid_map.cell_num_x && cell.y < grid_map.cell_num_y){
            ipoints[(grid_map.cell_num_x * grid_map.cell_num_y) - (cell.y * grid_map.cell_num_x + cell.x)] = out_point.intensity * grid_map.intensity_factor;
            hpoints[(grid_map.cell_num_x * grid_map.cell_num_y) - (cell.y * grid_map.cell_num_x + cell.x)] = out_point.z * grid_map.height_factor;
          }
          else{
            ROS_WARN_STREAM("Cell out of range: " << cell.x << " - " << grid_map.cell_num_x << " ||| " << cell.y << " - " << grid_map.cell_num_y);
          }
        }
      }
    }
  }
  intensity_grid->header.stamp = ros::Time::now();
  intensity_grid->header.frame_id = msg.header.frame_id; // TODO
  intensity_grid->info.map_load_time = ros::Time::now();
  intensity_grid->data = ipoints;
  height_grid->header.stamp = ros::Time::now();
  height_grid->header.frame_id = msg.header.frame_id; // TODO
  height_grid->info.map_load_time = ros::Time::now();  
  height_grid->data = hpoints;
  pub_igrid.publish(intensity_grid);
  pub_hgrid.publish(height_grid);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "slam3d_to_grid_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_("~");

  nh_.param("cell_size", cell_size_, 0.02);
  nh_.param("position_x", position_x_, 0.5);
  nh_.param("position_y", position_y_, 3.0);
  nh_.param("length_x", length_x_, 13.0);
  nh_.param("length_y", length_y_, 9.0);
  nh_.param("cloud_in_topic", cloud_in_topic_, std::string("map_cloud"));
  nh_.param("intensity_factor", intensity_factor_, 1.0);
  nh_.param("height_factor", height_factor_, 1.0);
  // nh_.param("frame_out", frame_out_, "map");
  nh_.param("mapi_topic_name", mapi_topic_name_, std::string("mapgrid_i"));
  nh_.param("maph_topic_name", maph_topic_name_, std::string("mapgrid_h"));
  

  grid_map.cell_size = cell_size_;
  grid_map.position_x = position_x_;
  grid_map.position_y = position_y_;
  grid_map.length_x = length_x_;
  grid_map.length_y = length_y_;
  grid_map.cloud_in_topic = cloud_in_topic_;
  grid_map.intensity_factor = intensity_factor_;
  grid_map.height_factor = height_factor_;
  //grid_map.frame_out = frame_out_;
  grid_map.mapi_topic_name = mapi_topic_name_;
  grid_map.maph_topic_name = maph_topic_name_;
  grid_map.initGrid(intensity_grid);
  grid_map.initGrid(height_grid);
  grid_map.paramRefresh();

  pub_igrid = nh.advertise<nav_msgs::OccupancyGrid>(grid_map.mapi_topic_name, 1);
  pub_hgrid = nh.advertise<nav_msgs::OccupancyGrid>(grid_map.maph_topic_name, 1);
  sub_pc2 = nh.subscribe(grid_map.cloud_in_topic, 1, pointcloudCallback);
  ros::spin();
  return 0;
}
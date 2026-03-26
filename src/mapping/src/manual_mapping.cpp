#include "iostream"
#include "math.h"
#include "ros/ros.h"
#include "mapping/input_pos.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "laser_assembler/AssembleScans.h"
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

tf2_ros::TransformBroadcaster* tf_broadcaster;
// tf2_ros::TransformListener* tf_listener;
// tf2_ros::Buffer tf_listener_buffer;

ros::Subscriber sub_cam_raw;
ros::Publisher pub_cam_filter;
sensor_msgs::PointCloud2 input_points2;
sensor_msgs::PointCloud output_points;
int status_pub_points;

ros::ServiceClient client;
laser_assembler::AssembleScans srv;
ros::Publisher pub_map;
sensor_msgs::PointCloud2 map_temp, map_pub;
ros::Timer mapping_process_timer;
int status_map_time, status_update_map, status_map_build;
double freq_map;
pcl::PCLPointCloud2::Ptr map_cloud (new pcl::PCLPointCloud2());
pcl::PCLPointCloud2::Ptr map_cloud_temp (new pcl::PCLPointCloud2());
pcl::PCLPointCloud2::Ptr map_hull_pub (new pcl::PCLPointCloud2());

float robot_posx, robot_posy, robot_posw;
ros::ServiceServer update_pos_srv;
ros::Timer robot_pos_timer;
ros::Time map_begin;

ros::Timer pub_cam_filter_timer;
double freq_points, min_cam_d, max_cam_d;

ros::Timer save_map_timer;

void cam_filter_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    // ROS_INFO_STREAM("[FILTER CALLBACK]");
    if (pub_cam_filter.getNumSubscribers () <= 0)
    {
      //ROS_DEBUG ("[point_cloud_converter] Got a PointCloud2 with %d points.", msg->width * msg->height);
      return;
    }
    
    // Convert to the new PointCloud format
    if (!sensor_msgs::convertPointCloud2ToPointCloud (*msg, output_points))
    {
      ROS_ERROR ("[point_cloud_converter] Conversion from sensor_msgs::PointCloud2 to sensor_msgs::PointCloud failed!");
      return;
    }
    input_points2 = *msg;
    ROS_DEBUG ("[point_cloud_converter] Publishing a PointCloud with %d points", (int)output_points.points.size ());
    status_pub_points = 1;
}

void pub_cam_filter_callback(const ros::TimerEvent& event)
{
  if(status_pub_points == 1){
    pcl::PCLPointCloud2::Ptr cam_cloud (new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cam_cloud_filtered (new pcl::PCLPointCloud2());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cam_cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>());

    ///////// Convert and  filtering object///////////////
    pcl_conversions::toPCL(input_points2,*cam_cloud);
    ///////////crop filter
    pcl::PointXYZ minPt, maxPt;
    pcl::fromPCLPointCloud2(*cam_cloud, *cam_cloud_xyz);
    pcl::getMinMax3D (*cam_cloud_xyz, minPt, maxPt);
    pcl::CropBox<pcl::PCLPointCloud2> cb;
    cb.setInputCloud (cam_cloud);
    cb.setMin(Eigen::Vector4f(min_cam_d, minPt.y, minPt.z, 0.)); /// cb.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
    cb.setMax(Eigen::Vector4f(max_cam_d, maxPt.y, maxPt.z, 0.)); /// cb.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    cb.filter (*cam_cloud);
    /////voxel grid filter
    pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
    vg.setInputCloud (cam_cloud);
    vg.setLeafSize (0.02f, 0.02f, 0.02f);
    vg.filter (*cam_cloud_filtered);
    pcl_conversions::fromPCL(*cam_cloud_filtered,input_points2);
    ////////////////////////////////////////////////////////////////

    sensor_msgs::convertPointCloud2ToPointCloud (input_points2, output_points);
    pub_cam_filter.publish (output_points);
    status_pub_points = 0;
  }
}

void mapping_process_callback(const ros::TimerEvent& event)
{
    // ROS_INFO_STREAM("[MAPPING PROCESS LOOP]");
    if(status_map_time == 1)
    {
        srv.request.begin = map_begin;
        srv.request.end = ros::Time::now();
        if (client.call(srv))
        {
            ROS_INFO_STREAM("Got cloud with " << srv.response.cloud.points.size() << " points");
            if(srv.response.cloud.points.size() != 0)
            {
                sensor_msgs::convertPointCloudToPointCloud2(srv.response.cloud, map_temp);
                
                pcl_conversions::toPCL(map_temp,*map_cloud_temp);
                ////////filter voxel grid/////////
                pcl::VoxelGrid<pcl::PCLPointCloud2> vg1;
                vg1.setInputCloud (map_cloud_temp);
                vg1.setLeafSize (0.02f, 0.02f, 0.02f);
                vg1.filter (*map_cloud_temp);
                //////////////Statistical Outlier Removal Filter//////////////////////////
                pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
                sor.setInputCloud (map_cloud_temp);
                sor.setMeanK (30);
                sor.setStddevMulThresh (2.0);
                sor.filter (*map_cloud_temp);

                ////////////////////////////////////////////////////////
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_hull (new pcl::PointCloud<pcl::PointXYZRGB>());
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_temp_hull (new pcl::PointCloud<pcl::PointXYZRGB>());
                pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
                
                pcl::fromPCLPointCloud2(*map_cloud, *map_hull);
                pcl::fromPCLPointCloud2(*map_cloud_temp, *map_temp_hull);
                *map_hull += *map_temp_hull;
                pcl::toPCLPointCloud2(*map_hull, *map_cloud);
                map_cloud->header.frame_id = "map";
                ////////filter voxel grid/////////
                pcl::VoxelGrid<pcl::PCLPointCloud2> vg2;
                vg2.setInputCloud (map_cloud);
                vg2.setLeafSize (0.02f, 0.02f, 0.02f);
                vg2.filter (*map_cloud);
                ///for convert and publish map in sensor_msgs::PointCloud2
                pcl_conversions::fromPCL(*map_cloud,map_pub);
                // pcl_conversions::fromPCL(*map_hull_pub,map_pub);
                ROS_INFO_STREAM("Now MAP have " << map_pub.height*map_pub.width << " points");
                status_map_build = 1;
                pub_map.publish(map_pub);
                status_map_time = 0;
            }   
        }
        else
        {
            ROS_INFO_STREAM("Service call failed");
        }   
    }   
    // ROS_INFO_STREAM("X: " << robotx << " Y: " << roboty << " W: " << robotw);
}

void save_map_callback(const ros::TimerEvent& event)
{
    if(status_map_build == 1)
    {
        /////////for save map in pcd file
        pcl::PointCloud<pcl::PointXYZRGB> map_save;
        pcl::fromPCLPointCloud2(*map_cloud, map_save);
        pcl::io::savePCDFileASCII ("/home/agv1/alfian/agv_ws/map_p206_final_test.pcd", map_save);
    }
}

bool update_pos_callback(mapping::input_pos::Request  &req, mapping::input_pos::Response &res)
{
    robot_posx = req.posx;
    robot_posy = req.posy;
    robot_posw = req.posw;

    res.status_pos = true;

    ros::Duration(2.0).sleep(); // sleep for 2 second
    map_begin = ros::Time::now();
    status_map_time = 1;
    ROS_INFO_STREAM("X: " << robot_posx << " Y: " << robot_posy << " W: " << robot_posw);
    return true;
}

void robot_pos_callback(const ros::TimerEvent& event)
{
    geometry_msgs::Pose pos_msg;
    pos_msg.position.x = robot_posx;
    pos_msg.position.y = robot_posy;
    tf2::Quaternion q;
    q.setRPY(0,0,robot_posw*TO_RAD);
    pos_msg.orientation = tf2::toMsg(q);

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";  // Parent frame (e.g., odom)
    transformStamped.child_frame_id = "odom";  // Child frame (robot's base frame)
    transformStamped.transform.translation.x = robot_posx;
    transformStamped.transform.translation.y = robot_posy;
    transformStamped.transform.translation.z = 0.0;  // Assuming 2D robot
    transformStamped.transform.rotation = pos_msg.orientation;  // Use the same orientation as Pose
    // Broadcast the transform
    tf_broadcaster->sendTransform(transformStamped);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "manual_mapping");
  ros::NodeHandle nh;
  ros::NodeHandle nh_("~");

  nh_.param("maxf_points_out", freq_points, 1.0);
  nh_.param("maxf_map_out", freq_map, 1.0);
  nh_.param("min_cam_d", min_cam_d, 0.6);
  nh_.param("max_cam_d", max_cam_d, 8.0);

  ROS_INFO_STREAM("WAIT DATA FROM ROBOT POS & RGBD CAMERA");
  ros::topic::waitForMessage<sensor_msgs::PointCloud2>("full_pointcloud");
  ros::service::waitForService("assemble_scans");
  ROS_INFO_STREAM("Pos and Pointcloud RECEIVE!!");

  sub_cam_raw = nh.subscribe("full_pointcloud", 10, cam_filter_callback);
  pub_cam_filter = nh.advertise<sensor_msgs::PointCloud>("cam_pcd", 10);

  client = nh.serviceClient<laser_assembler::AssembleScans>("assemble_scans");
  pub_map = nh.advertise<sensor_msgs::PointCloud2>("map_pcd", 10);

  tf_broadcaster = new tf2_ros::TransformBroadcaster();
  // tf_listener = new tf2_ros::TransformListener(tf_listener_buffer);

  ros::Duration(2.0).sleep(); // sleep for 2 second
  ROS_INFO_STREAM("[MAPPING PROCESS BEGIN]");

  ros::ServiceServer update_pos_srv = nh.advertiseService("update_pos", update_pos_callback);

  pub_cam_filter_timer = nh.createTimer(ros::Duration(1.0/freq_points), pub_cam_filter_callback);
  robot_pos_timer = nh.createTimer(ros::Duration(0.01), robot_pos_callback);
  mapping_process_timer = nh.createTimer(ros::Duration(1.0/freq_map), mapping_process_callback);
  save_map_timer = nh.createTimer(ros::Duration(2.0), save_map_callback);
  
  ros::spin();

  return 0;
}
#include "iostream"
#include "math.h"
#include "ros/ros.h"
#include <udp_bot/terima_udp.h>
#include <udp_bot/kirim_offset_udp.h>
#include "sensor_msgs/point_cloud_conversion.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
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

#define TO_DEG 57.295779513
#define TO_RAD 0.01745329252

// float robotx, roboty, robotw;
tf2_ros::TransformBroadcaster* tf_broadcaster;
tf2_ros::TransformBroadcaster* tf_broadcaster_odom;
tf2_ros::TransformListener* tf_listener;
tf2_ros::Buffer tf_listener_buffer;

double cov_pos[36] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

double cov_vel[36] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

geometry_msgs::Pose robot_position;
geometry_msgs::Quaternion robot_orientation;
nav_msgs::Odometry robot_odom;

ros::Subscriber sub_cam_raw, sub_robot_odom;
ros::Publisher pub_cam_filter, pub_robot_odom, pub_robot_pos;
sensor_msgs::PointCloud2 input_points2, output_points2;
int status_pub_points;
ros::Timer pub_cam_filter_timer;
double freq_points, min_cam_d, max_cam_d;

double robotx, roboty, robotw;
int status_init_odom;
double roll, pitch, yaw;
float vx_lokal, vy_lokal, vw_lokal;
float vx_global, vy_global, vw_global;
float odox_buf, odoy_buf, odow_buf;
float odox_offset, odoy_offset, odow_offset;
ros::Publisher pub_robot_offset;
udp_bot::kirim_offset_udp offset_msg;
ros::Subscriber sub_robot_offset;
geometry_msgs::Pose pos_msg;

ros::Timer localization_process_timer, robot_odom_timer;

ros::Publisher pub_map, pub_init_pos;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_pcd (new pcl::PointCloud<pcl::PointXYZRGB>);
sensor_msgs::PointCloud2 output_map;
geometry_msgs::PoseWithCovarianceStamped init_pos;


void cam_filter_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    // ROS_INFO_STREAM("[FILTER CALLBACK]");
    if (pub_cam_filter.getNumSubscribers () <= 0)
    {
      //ROS_DEBUG ("[point_cloud_converter] Got a PointCloud2 with %d points.", msg->width * msg->height);
      return;
    }

    input_points2 = *msg;
    ROS_DEBUG ("[point_cloud_converter] Got a PointCloud2 with %d points", input_points2.width * input_points2.height);
    status_pub_points = 1;
}

void pub_cam_filter_callback(const ros::TimerEvent& event)
{
    if(status_pub_points == 1)
    {
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
        vg.setLeafSize (0.06f, 0.06f, 0.06f);
        vg.filter (*cam_cloud);
         //////////////Statistical Outlier Removal Filter//////////////////////////
        pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
        sor.setInputCloud (cam_cloud);
        sor.setMeanK (30);
        sor.setStddevMulThresh (2.0);
        sor.filter (*cam_cloud_filtered);
        
        pcl_conversions::fromPCL(*cam_cloud_filtered,output_points2);
        pub_cam_filter.publish (output_points2);
        status_pub_points = 0;
    }
}

void robot_odom_callback(const udp_bot::terima_udp::ConstPtr& msg)
{
    // ROS_INFO_STREAM("ROBOT POS ODOM RCV");
    odox_buf = msg->posisi_x_buffer;
    odoy_buf = msg->posisi_y_buffer;
    odow_buf = msg->sudut_w_buffer;

    vx_lokal = msg->kecepatan_robotx;
    vy_lokal = msg->kecepatan_roboty;
    vw_lokal = msg->kecepatan_robotw;

    vx_global = msg->vx_global;
    vy_global = msg->vy_global;
    vw_global = msg->vw_global;
    // ROS_INFO_STREAM("VX: " << vx_lokal << " VY: " << vy_lokal << " VW: " << vw_lokal);

}

void robot_offset_callback(const udp_bot::kirim_offset_udp::ConstPtr& msg)
{
    odox_offset = msg->posisi_x_offset;
    odoy_offset = msg->posisi_y_offset;
    odow_offset = msg->sudut_w_offset;
}

void robot_odom_pub_callback(const ros::TimerEvent& event)
{
    if(status_init_odom == 0)
    {
        // robotx = roboty = robotw = 0;
        odox_offset = odox_buf - robotx;
        odoy_offset = odoy_buf - roboty;
        odow_offset = odow_buf - robotw;
        
        offset_msg.posisi_x_offset = odox_offset; ///odox_buf untuk reset ke 0 bila ada data prev
        offset_msg.posisi_y_offset = odoy_offset; ///odoy_buf untuk reset ke 0 bila ada data prev
        offset_msg.sudut_w_offset  = odow_offset; ///odow_buf untuk reset ke 0 bila ada data prev
        pub_robot_offset.publish(offset_msg);
        
        ROS_INFO_STREAM("[ODOMETRY INITIAL LOCATION DONE]");
        status_init_odom = 1;
    }
    else
    {
        robotx = odox_buf - odox_offset;
        roboty = odoy_buf - odoy_offset;
        robotw = odow_buf - odow_offset;
        while (robotw > 180) robotw -= 360;
        while (robotw< -180) robotw += 360;

        pos_msg.position.x = robotx;
        pos_msg.position.y = roboty;
        tf2::Quaternion q;
        q.setRPY(0,0,robotw*TO_RAD);
        pos_msg.orientation = tf2::toMsg(q);

        // ROS_INFO_STREAM("[ROBOT ODOM LOOP]");
        robot_odom.header.stamp = ros::Time::now();
        robot_odom.header.frame_id = "map";
        robot_odom.pose.pose.position = pos_msg.position;
        robot_odom.pose.pose.orientation = pos_msg.orientation;
        std::copy(std::begin(cov_pos), std::end(cov_pos), std::begin(robot_odom.pose.covariance));
        robot_odom.child_frame_id = "odom";
        robot_odom.twist.twist.linear.x = vx_lokal;
        robot_odom.twist.twist.linear.y = vy_lokal;
        robot_odom.twist.twist.angular.z = vw_lokal;
        std::copy(std::begin(cov_vel), std::end(cov_vel), std::begin(robot_odom.twist.covariance));
        //publish the message
        pub_robot_odom.publish(robot_odom);
        
        /////broadcast tf robot odom////////////////
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "odom";  // Parent frame (e.g., odom)
        transformStamped.child_frame_id = "base_link";  // Child frame (robot's base frame)
        transformStamped.transform.translation.x = robotx;
        transformStamped.transform.translation.y = roboty;
        transformStamped.transform.translation.z = 0.0;  // Assuming 2D robot
        transformStamped.transform.rotation = pos_msg.orientation;  // Use the same orientation as Pose

        // Broadcast the transform
        tf_broadcaster->sendTransform(transformStamped);
    } 
}

void localization_process_callback(const ros::TimerEvent& event)
{
    ROS_INFO_STREAM("[LOCALIZATION PROCESS LOOP]");
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tf_listener_buffer.lookupTransform("map", "base_link", ros::Time(0)); 

      robot_position.position.x = transformStamped.transform.translation.x;
      robot_position.position.y = transformStamped.transform.translation.y;
      robot_position.position.z = transformStamped.transform.translation.z;
      robot_position.orientation.x = transformStamped.transform.rotation.x;
      robot_position.orientation.y = transformStamped.transform.rotation.y;
      robot_position.orientation.z = transformStamped.transform.rotation.z;
      robot_position.orientation.w = transformStamped.transform.rotation.w;

      pub_robot_pos.publish(robot_position);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());

      return;
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rgbd_cam_loc");
  ros::NodeHandle nh;
  ros::NodeHandle nh_("~");

  nh_.param("min_cam_d", min_cam_d, 0.6);
  nh_.param("max_cam_d", max_cam_d, 8.0);
  nh_.param("init_robotx", robotx, 0.0);
  nh_.param("init_roboty", roboty, 0.0);
  nh_.param("init_robotw", robotw, 0.0);
  nh_.param("maxf_points_out", freq_points, 1.0);

  ROS_INFO_STREAM("WAIT DATA FROM ROBOT POS & RGBD CAMERA");
  // ros::topic::waitForMessage<sensor_msgs::PointCloud2>("full_pointcloud");
  ros::topic::waitForMessage<udp_bot::terima_udp>("data_terima_udp");
  ROS_INFO_STREAM("Pos and Pointcloud RECEIVE!!");

  sub_cam_raw = nh.subscribe("full_pointcloud", 10, cam_filter_callback);
  sub_robot_odom = nh.subscribe("data_terima_udp", 10, robot_odom_callback);
  sub_robot_offset = nh.subscribe("offset_kirim_udp", 10, robot_offset_callback);
  
  pub_cam_filter = nh.advertise<sensor_msgs::PointCloud2>("cloud", 10);
  pub_robot_offset = nh.advertise<udp_bot::kirim_offset_udp>("offset_kirim_udp", 10);
  pub_robot_odom = nh.advertise<nav_msgs::Odometry>("odom", 10);
  pub_robot_pos = nh.advertise<geometry_msgs::Pose>("robot_pos", 10);

  tf_broadcaster = new tf2_ros::TransformBroadcaster();
  tf_broadcaster_odom = new tf2_ros::TransformBroadcaster();
  tf_listener = new tf2_ros::TransformListener(tf_listener_buffer);

  ros::Duration(2.0).sleep(); // sleep for 2 second
  ROS_INFO_STREAM("[RGBD CAM LOCALIZATION PROCESS BEGIN]");

  /////////pub init pos & map////////////////////////////
  pub_init_pos = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 10);
  init_pos.header.stamp = ros::Time::now();
  init_pos.header.frame_id = "map";
  init_pos.pose.pose.position.x = robotx;
  init_pos.pose.pose.position.y = roboty;
  tf2::Quaternion init_pos_q;
  init_pos_q.setRPY(0,0,robotw*TO_RAD);
  init_pos.pose.pose.orientation = tf2::toMsg(init_pos_q);
  double cov_init_pos[36] = { 0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.1};
  std::copy(std::begin(cov_init_pos), std::end(cov_init_pos), std::begin(init_pos.pose.covariance));

  pub_map = nh.advertise<sensor_msgs::PointCloud2>("map_cloud", 10);
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/agv1/alfian/old_agv_ws/map_p206_final.pcd", *map_pcd) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  // /////voxel grid filter
  // pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  // vg.setInputCloud (map_pcd);
  // vg.setLeafSize (0.05f, 0.05f, 0.05f);
  // vg.filter (*map_pcd);
  pcl::toROSMsg(*map_pcd, output_map);
  output_map.header.stamp = ros::Time::now();
  output_map.header.frame_id = "map"; 
  output_map.header.seq = 1; 

  for(int i=0; i<=2; i++)
  {
    pub_map.publish (output_map);
    pub_init_pos.publish (init_pos);
    ros::Duration(1.0).sleep();
  }

  robot_odom_timer = nh.createTimer(ros::Duration(0.05), robot_odom_pub_callback);
  localization_process_timer = nh.createTimer(ros::Duration(0.1), localization_process_callback);
  pub_cam_filter_timer = nh.createTimer(ros::Duration(1.0/freq_points), pub_cam_filter_callback);
  
  ros::spin();

  return 0;
}
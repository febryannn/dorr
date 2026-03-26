#include "lidar_slam_3d_ros.h"
#include "geo_transform.h"
#include <pcl_conversions/pcl_conversions.h>

LidarSlam3dRos::LidarSlam3dRos()
{
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string point_cloud_topic, odom_topic, laser_topic1, laser_topic2, laser_topic3;
    Eigen::Translation3f offset_translation(0.0, 0.0, 0.0);
    Eigen::AngleAxisf offset_rotation (0.0, Eigen::Vector3f::UnitZ ());
    odom_offset_ = (offset_translation * offset_rotation).matrix ();

    private_nh.param("base_frame", base_frame_, std::string("base_link"));
    private_nh.param("map_frame", map_frame_, std::string("map"));
    private_nh.param("odom_frame", odom_frame_, std::string("odom"));
    private_nh.param("sensor_frame", sensor_frame_, std::string("camera_link"));
    private_nh.param("publish_freq", publish_freq_, 0.2);
    private_nh.param("point_cloud_topic", point_cloud_topic, std::string("velodyne_points"));
    private_nh.param("odom_topic", odom_topic, std::string("odom"));
    private_nh.param("laser_topic1", laser_topic1, std::string("scan1"));
    private_nh.param("laser_topic2", laser_topic2, std::string("scan2"));
    private_nh.param("laser_topic3", laser_topic3, std::string("scan3"));
    private_nh.param("min_scan_distance", min_scan_distance_, 2.0);
    private_nh.param("enable_floor_filter", enable_floor_filter_, true);

    map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("map_cloud", 1, true);
    path_pub_ = nh.advertise<nav_msgs::Path>("path", 1, true);
    pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("pose", 1, true);
    filtered_point_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("filtered_point_cloud", 1, true);
    floor_points_pub_ = nh.advertise<sensor_msgs::PointCloud2>("floor_points", 1, true);
    constraint_list_pub_ = nh.advertise<visualization_msgs::MarkerArray>("constraint_list", 1, true);

    point_cloud_sub_ = nh.subscribe(point_cloud_topic, 10000, &LidarSlam3dRos::pointCloudCallback, this);
    odom_sub_ = nh.subscribe(odom_topic, 10000, &LidarSlam3dRos::odomCallback, this);
    laser_sub_1 = nh.subscribe(laser_topic1, 10000, &LidarSlam3dRos::laserCallback1, this);
    laser_sub_2 = nh.subscribe(laser_topic2, 10000, &LidarSlam3dRos::laserCallback2, this);
    laser_sub_3 = nh.subscribe(laser_topic3, 10000, &LidarSlam3dRos::laserCallback3, this);

    tf_listener = new tf2_ros::TransformListener(tf_listener_buffer);

    publish_thread_.reset(new std::thread(std::bind(&LidarSlam3dRos::publishLoop, this)));
}

LidarSlam3dRos::dual_pose LidarSlam3dRos::getPose(const Eigen::Matrix4f& T)
{
    dual_pose dual_pose_;
    Eigen::Matrix4f robot_pose_;

    robot_pose_ = T*odom_pose_.inverse();

    Eigen::Matrix4f pose_temp;
    pose_temp = robot_pose_;
    tf::Matrix3x3 data_R;
    double data_roll, data_pitch, data_yaw;
    data_R.setValue(pose_temp(0, 0), pose_temp(0, 1), pose_temp(0, 2),
               pose_temp(1, 0), pose_temp(1, 1), pose_temp(1, 2),
            pose_temp(2, 0), pose_temp(2, 1), pose_temp(2, 2));
    data_R.getRPY(data_roll, data_pitch, data_yaw);
    Eigen::AngleAxisf data_rotation (data_yaw, Eigen::Vector3f::UnitZ ());
    Eigen::Translation3f data_translation (pose_temp(0,3), pose_temp(1,3), 0);
    robot_pose_ = (data_translation * data_rotation).matrix ();

    odom_offset_ = robot_pose_;

    Vector6f pose;
    tf::Matrix3x3 R;
    double roll, pitch, yaw;

    pose(0) = robot_pose_(0, 3);
    pose(1) = robot_pose_(1, 3);
    pose(2) = robot_pose_(2, 3);
    R.setValue(robot_pose_(0, 0), robot_pose_(0, 1), robot_pose_(0, 2),
               robot_pose_(1, 0), robot_pose_(1, 1), robot_pose_(1, 2),
               robot_pose_(2, 0), robot_pose_(2, 1), robot_pose_(2, 2));
    R.getRPY(roll, pitch, yaw);
    pose(3) = roll;
    pose(4) = pitch;
    pose(5) = yaw;

    dual_pose_.offset_pose = pose;

    pose(0) = T(0, 3);
    pose(1) = T(1, 3);
    pose(2) = T(2, 3);
    R.setValue(T(0, 0), T(0, 1), T(0, 2),
               T(1, 0), T(1, 1), T(1, 2),
               T(2, 0), T(2, 1), T(2, 2));
    R.getRPY(roll, pitch, yaw);
    pose(3) = roll;
    pose(4) = pitch;
    pose(5) = yaw;

    dual_pose_.robot_pose = pose;

    return dual_pose_;
}

void LidarSlam3dRos::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg)
{
    geometry_msgs::TransformStamped transformStamped;
    try{
        transformStamped = tf_listener_buffer.lookupTransform(base_frame_, sensor_frame_, ros::Time(0)); 
        
        Eigen::Translation3f sensor_translation(transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
        tf2::Quaternion q_sensor;
        tf2::fromMsg(transformStamped.transform.rotation, q_sensor);
        tf2::Matrix3x3 m_sensor(q_sensor);
        m_sensor.getRPY(sensor_roll,sensor_pitch,sensor_yaw);
        Eigen::AngleAxisf sensor_rotation (sensor_yaw, Eigen::Vector3f::UnitZ ());
        sensor_pose_ = (sensor_translation * sensor_rotation).matrix ();
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*point_cloud_msg, *point_cloud);

    pcl::PointCloud<pcl::PointXYZI>::Ptr clipped_point_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    for(const pcl::PointXYZI& point: point_cloud->points) {
        double r = sqrt(square(point.x) + square(point.y));
        if (r > min_scan_distance_) {
            clipped_point_cloud->push_back(point);
        }
    }

    pcl::transformPointCloud(*clipped_point_cloud, *clipped_point_cloud, sensor_pose_);

    if(enable_floor_filter_) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_point_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr floor_point_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        floor_filter_.filter(clipped_point_cloud, filtered_point_cloud, floor_point_cloud);
        map_builder_.addPointCloud(filtered_point_cloud, odom_pose_, odom_offset_, laser_xyz1, laser_xyz2, laser_xyz3);

        sensor_msgs::PointCloud2 floor_cloud_msg;
        pcl::toROSMsg(*floor_point_cloud, floor_cloud_msg);
        floor_cloud_msg.header.stamp = ros::Time::now();
        floor_cloud_msg.header.frame_id = point_cloud_msg->header.frame_id;
        floor_points_pub_.publish(floor_cloud_msg);

        sensor_msgs::PointCloud2 filtered_cloud_msg;
        pcl::toROSMsg(*filtered_point_cloud, filtered_cloud_msg);
        filtered_cloud_msg.header.stamp = ros::Time::now();
        filtered_cloud_msg.header.frame_id = point_cloud_msg->header.frame_id;
        filtered_point_cloud_pub_.publish(filtered_cloud_msg);
    }
    else {
        map_builder_.addPointCloud(clipped_point_cloud, odom_pose_, odom_offset_, laser_xyz1, laser_xyz2, laser_xyz3);
    }

    dual_pose pose = getPose(map_builder_.getTransformation());
    publishPose(pose.robot_pose, point_cloud_msg->header.stamp);
    publishTf(pose.offset_pose, point_cloud_msg->header.stamp);
    publishPath(pose.robot_pose, point_cloud_msg->header.stamp);
}

void LidarSlam3dRos::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    Eigen::Translation3f odom_translation(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z);
    tf2::Quaternion q;
    tf2::fromMsg(odom_msg->pose.pose.orientation, q);
    tf2::Matrix3x3 m(q);
    m.getRPY(odom_roll,odom_pitch,odom_yaw);
    Eigen::AngleAxisf odom_rotation (odom_yaw, Eigen::Vector3f::UnitZ ());
    odom_pose_ = (odom_translation * odom_rotation).matrix ();
}

void LidarSlam3dRos::laserCallback1(const sensor_msgs::LaserScan::ConstPtr& laser_msg)
{   

    if(!listener_laser1.waitForTransform(
            laser_msg->header.frame_id,
            map_frame_,
            laser_msg->header.stamp + ros::Duration().fromSec(laser_msg->ranges.size()*laser_msg->time_increment),
            ros::Duration(1.0))){
        return;
    }

    sensor_msgs::PointCloud2 laser_cloud;
    projector_laser.transformLaserScanToPointCloud(map_frame_, *laser_msg, laser_cloud, listener_laser2);

    pcl::PCLPointCloud2 laser_pc2;
    pcl_conversions::toPCL(laser_cloud, laser_pc2);
    pcl::fromPCLPointCloud2(laser_pc2, laser_xyz1);
}

void LidarSlam3dRos::laserCallback2(const sensor_msgs::LaserScan::ConstPtr& laser_msg)
{   

    if(!listener_laser2.waitForTransform(
            laser_msg->header.frame_id,
            map_frame_,
            laser_msg->header.stamp + ros::Duration().fromSec(laser_msg->ranges.size()*laser_msg->time_increment),
            ros::Duration(1.0))){
        return;
    }

    sensor_msgs::PointCloud2 laser_cloud;
    projector_laser.transformLaserScanToPointCloud(map_frame_, *laser_msg, laser_cloud, listener_laser2);

    pcl::PCLPointCloud2 laser_pc2;
    pcl_conversions::toPCL(laser_cloud, laser_pc2);
    pcl::fromPCLPointCloud2(laser_pc2, laser_xyz2);
}

void LidarSlam3dRos::laserCallback3(const sensor_msgs::LaserScan::ConstPtr& laser_msg)
{   

    if(!listener_laser3.waitForTransform(
            laser_msg->header.frame_id,
            map_frame_,
            laser_msg->header.stamp + ros::Duration().fromSec(laser_msg->ranges.size()*laser_msg->time_increment),
            ros::Duration(1.0))){
        return;
    }

    sensor_msgs::PointCloud2 laser_cloud;
    projector_laser.transformLaserScanToPointCloud(map_frame_, *laser_msg, laser_cloud, listener_laser3);

    pcl::PCLPointCloud2 laser_pc2;
    pcl_conversions::toPCL(laser_cloud, laser_pc2);
    pcl::fromPCLPointCloud2(laser_pc2, laser_xyz3);
}


void LidarSlam3dRos::publishPose(const Vector6f& pose, const ros::Time& t)
{
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = t;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose.position.x = pose(0);
    pose_msg.pose.position.y = pose(1);
    pose_msg.pose.position.z = pose(2);
    tf::Quaternion q;
    q.setRPY(pose(3), pose(4), pose(5));
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    pose_pub_.publish(pose_msg);
}

void LidarSlam3dRos::publishPath(const Vector6f& pose, const ros::Time& t)
{
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = t;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose.position.x = pose(0);
    pose_msg.pose.position.y = pose(1);
    pose_msg.pose.position.z = pose(2);

    tf::Quaternion q;
    q.setRPY(pose(3), pose(4), pose(5));
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    path_msg_.poses.push_back(pose_msg);

    path_msg_.header.stamp = t;
    path_msg_.header.frame_id = map_frame_;
    path_pub_.publish(path_msg_);
}

void LidarSlam3dRos::publishTf(const Vector6f& pose, const ros::Time& t)
{
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pose(0), pose(1), pose(2)));
    tf::Quaternion q;
    q.setRPY(pose(3), pose(4), pose(5));
    transform.setRotation(q);
    tf_broadcaster_.sendTransform(tf::StampedTransform(transform, t, map_frame_, odom_frame_));
}

void LidarSlam3dRos::publishMap()
{
    sensor_msgs::PointCloud2 map_msg;

    map_builder_.getMap(map_msg);
    map_msg.header.stamp = ros::Time::now();
    map_msg.header.frame_id = map_frame_;
    map_pub_.publish(map_msg);
}

void LidarSlam3dRos::publishConstraintList()
{
    std::vector<Eigen::Vector3d> graph_nodes;
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> graph_edges;

    map_builder_.getPoseGraph(graph_nodes, graph_edges);

    visualization_msgs::MarkerArray marker_array;

    visualization_msgs::Marker marker;
    marker.header.frame_id = map_frame_;
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.scale.x = 0.15;
    marker.scale.y = 0.15;
    marker.scale.z = 0.15;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(0);

    visualization_msgs::Marker edge;
    edge.header.frame_id = map_frame_;
    edge.header.stamp = ros::Time::now();
    edge.action = visualization_msgs::Marker::ADD;
    edge.id = 0;
    edge.type = visualization_msgs::Marker::LINE_STRIP;
    edge.scale.x = 0.1;
    edge.scale.y = 0.1;
    edge.scale.z = 0.1;
    edge.color.r = 0.0;
    edge.color.g = 1.0;
    edge.color.b = 0.0;
    edge.color.a = 1.0;

    int id = 0;
    for (int i = 0; i < graph_nodes.size(); ++i) {
        marker.id = id;
        marker.pose.position.x = graph_nodes[i](0);
        marker.pose.position.y = graph_nodes[i](1);
        marker.pose.position.z = graph_nodes[i](2);
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker_array.markers.push_back(visualization_msgs::Marker(marker));
        id++;
    }

    for (int i = 0; i < graph_edges.size(); ++i) {
        edge.points.clear();
        geometry_msgs::Point p;
        p.x = graph_edges[i].first(0);
        p.y = graph_edges[i].first(1);
        p.z = graph_edges[i].first(2);
        edge.points.push_back(p);
        p.x = graph_edges[i].second(0);
        p.y = graph_edges[i].second(1);
        p.z = graph_edges[i].second(2);
        edge.points.push_back(p);
        edge.id = id;
        edge.pose.orientation.x = 0.0;
        edge.pose.orientation.y = 0.0;
        edge.pose.orientation.z = 0.0;
        edge.pose.orientation.w = 1.0;
        marker_array.markers.push_back(visualization_msgs::Marker(edge));
        id++;
    }

    constraint_list_pub_.publish(marker_array);
}

void LidarSlam3dRos::publishLoop()
{
    ros::Rate rate(publish_freq_);

    while (ros::ok()) {
        publishMap();
        publishConstraintList();
        rate.sleep();
    }
}

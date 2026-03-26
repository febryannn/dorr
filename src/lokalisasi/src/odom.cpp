#include "iostream"
#include "math.h"
#include "ros/ros.h"
#include <udp_bot/terima_udp.h>
#include <udp_bot/kirim_offset_udp.h>
#include "geometry_msgs/Pose.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"

#define TO_DEG 57.295779513
#define TO_RAD 0.01745329252

float odox_buf, odoy_buf, odow_buf;
float odox_offset, odoy_offset, odow_offset;

int status_init_pos;
float robotx, roboty, robotw;

float vx_lokal, vy_lokal, vw_lokal;
float vx_global, vy_global, vw_global;

ros::Subscriber sub_robot_odom;
ros::Subscriber sub_robot_offset;

ros::Publisher pub_robot_offset;
udp_bot::kirim_offset_udp offset_msg;

ros::Publisher pub_robot_pos;
geometry_msgs::Pose pos_msg;

tf2_ros::TransformBroadcaster* tf_broadcaster;

ros::Timer fusion_process_timer;


void robot_odom_callback(const udp_bot::terima_udp::ConstPtr& msg)
{
    // ROS_INFO_STREAM("ROBOT POS ODOM RCV");
    odox_buf = msg->posisi_x_buffer;
    odoy_buf = msg->posisi_y_buffer;
    odow_buf = msg->sudut_w_buffer;

    vx_lokal = msg->kecepatan_robotx;
    vy_lokal = msg->kecepatan_roboty;
    vw_lokal = msg->kecepatan_robotw;

    // //// vx, vy (m/s) dan vw (rad/s) 
    // vx_global = vx_lokal*sinf((odow_buf-odow_offset)*TO_RAD) + vy_lokal*cosf((odow_buf-odow_offset)*TO_RAD);
    // vy_global = vx_lokal*-cosf((odow_buf-odow_offset)*TO_RAD) + vy_lokal*sinf((odow_buf-odow_offset)*TO_RAD);
    // vw_global = vw_lokal;
    vx_global = msg->vx_global;
    vy_global = msg->vy_global;
    vw_global = msg->vw_global;
}

void robot_offset_callback(const udp_bot::kirim_offset_udp::ConstPtr& msg)
{
    odox_offset = msg->posisi_x_offset;
    odoy_offset = msg->posisi_y_offset;
    odow_offset = msg->sudut_w_offset;
}

void fusion_process_callback(const ros::TimerEvent& event)
{
    if(status_init_pos == 0)
    {
        robotx = roboty = robotw = 0;

        odox_offset = odox_buf - robotx;
        odoy_offset = odoy_buf - roboty;
        odow_offset = odow_buf - robotw;
        
        offset_msg.posisi_x_offset = odox_offset; ///odox_buf untuk reset ke 0 bila ada data prev
        offset_msg.posisi_y_offset = odoy_offset; ///odoy_buf untuk reset ke 0 bila ada data prev
        offset_msg.sudut_w_offset  = odow_offset; ///odow_buf untuk reset ke 0 bila ada data prev
        pub_robot_offset.publish(offset_msg);
        
        ROS_INFO_STREAM("[ODOMETRY INITIAL LOCATION DONE]");
        status_init_pos = 1;
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
        pub_robot_pos.publish(pos_msg);

        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "map";  // Parent frame (e.g., odom)
        transformStamped.child_frame_id = "odom";  // Child frame (robot's base frame)

        transformStamped.transform.translation.x = robotx;
        transformStamped.transform.translation.y = roboty;
        transformStamped.transform.translation.z = 0.0;  // Assuming 2D robot

        transformStamped.transform.rotation = pos_msg.orientation;  // Use the same orientation as Pose

        // Broadcast the transform
        tf_broadcaster->sendTransform(transformStamped);
    }  
    ROS_INFO_STREAM("X: " << robotx << " Y: " << roboty << " W: " << robotw);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom");
  ros::NodeHandle nh;

  sub_robot_odom = nh.subscribe("data_terima_udp", 10, robot_odom_callback);
  sub_robot_offset = nh.subscribe("offset_kirim_udp", 10, robot_offset_callback);

  pub_robot_offset = nh.advertise<udp_bot::kirim_offset_udp>("offset_kirim_udp", 10);
  pub_robot_pos = nh.advertise<geometry_msgs::Pose>("robot_pos", 10);

  tf_broadcaster = new tf2_ros::TransformBroadcaster();

  ROS_INFO_STREAM("WAIT DATA FROM ODOMETRY");
  ros::topic::waitForMessage<udp_bot::terima_udp>("data_terima_udp");
  ROS_INFO_STREAM("ODOMETRY RECEIVE!!");

  ros::Duration(2.0).sleep(); // sleep for 2 second
  ROS_INFO_STREAM("[ODOMETRY LOCATION BEGIN]");

  fusion_process_timer = nh.createTimer(ros::Duration(0.01), fusion_process_callback);

  ros::spin();

  return 0;
}
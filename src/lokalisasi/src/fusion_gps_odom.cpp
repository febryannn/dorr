#include "iostream"
#include "math.h"
#include <eigen3/Eigen/Dense>
#include "ros/ros.h"
#include <udp_bot/terima_udp.h>
#include <udp_bot/kirim_offset_udp.h>
#include "marvelmind_nav/hedge_pos_ang.h"
#include "marvelmind_nav/hedge_quality.h"
#include "geometry_msgs/Pose.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "lokalisasi/fusion_gps_odom.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"

using namespace Eigen;

#define TO_DEG 57.295779513
#define TO_RAD 0.01745329252

#define r_robot_gps 0.0 ////jarak beacon ke pusat robot 0.0m
#define offset_gpsx 0.0 //4m
#define offset_gpsy 0.0  //5m
#define gps_f_sample 0.0

// #define r_robot_gps 0.23 ////jarak beacon ke pusat robot 0.23m
// #define offset_gpsx 0.77 //4m
// #define offset_gpsy -0.98  //5m
// #define gps_f_sample 8.0

float odox_buf, odoy_buf, odow_buf;
float odox_offset, odoy_offset, odow_offset;
float gpsx_buf, gpsy_buf, gpsw_buf;
float gpsx, gpsy, gpsw;
float gpsx_prev, gpsy_prev, gpsw_prev;
float gps_quality, gps_q;

int status_init_pos;
float robotx, roboty, robotw;

float vx_lokal, vy_lokal, vw_lokal;
float vx_global, vy_global, vw_global;
float vx_gps, vy_gps, vw_gps;

float w_mean, w_mean_sin, w_mean_cos;

ros::Subscriber sub_robot_odom;
ros::Subscriber sub_robot_hedge;
ros::Subscriber sub_hedge_quality;

ros::Publisher pub_robot_offset;
udp_bot::kirim_offset_udp offset_msg;

ros::Publisher pub_robot_pos;
geometry_msgs::Pose pos_msg;

ros::Publisher pub_robot_gps;
geometry_msgs::Pose gps_msg;

ros::Publisher pub_robot_odo;
geometry_msgs::Pose odo_msg;

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

void robot_hedge_callback(const marvelmind_nav::hedge_pos_ang::ConstPtr& msg)
{
    // ROS_INFO_STREAM("x: " << msg->x_m << "y: " << msg->y_m << "w: " << msg->angle);
    gpsw_buf = (float)msg->angle;
    gpsx_buf = (float)msg->x_m - (cosf(gpsw_buf*TO_RAD)*r_robot_gps) - offset_gpsx; ////jarak beacon ke pusat robot 0.23m
    gpsy_buf = (float)msg->y_m - (sinf(gpsw_buf*TO_RAD)*r_robot_gps) - offset_gpsy; ////jarak beacon ke pusat robot 0.23m

    vx_gps = (gpsx_buf - gpsx_prev)*gps_f_sample;
    vy_gps = (gpsy_buf - gpsy_prev)*gps_f_sample;
    float vw_temp;
    vw_temp = gpsw_buf - gpsw_prev;
    while (vw_temp > 180) vw_temp -= 360;
    while (vw_temp < -180) vw_temp += 360;
    vw_gps = vw_temp*TO_RAD*gps_f_sample;

    gpsx_prev = gpsx_buf;
    gpsy_prev = gpsy_buf;
    gpsw_prev = gpsw_buf;    

}

void hedge_quality_callback(const marvelmind_nav::hedge_quality::ConstPtr& msg)
{
    gps_quality = (float)(msg->quality_percents)/100;
    // ROS_INFO_STREAM("GPS QUALITY: " << gps_quality);
}

void fusion_process_callback(const ros::TimerEvent& event)
{
  gpsx = gpsx_buf;
  gpsy = gpsy_buf;
  gpsw = gpsw_buf;
  gps_q = gps_quality;

  if(status_init_pos == 0) ////untuk inisialisasi posisi dan orientasi secara global
  {
    ROS_INFO_STREAM("WAIT HIGH QUALITY INDOOR GPS DATA......");
    if(gps_q > 0.95)
    {
      ///unutk offset inisialisasi (-x, -y, -w)
      odox_offset = odox_buf - gpsx; 
      odoy_offset = odoy_buf - gpsy;
      odow_offset = odow_buf - gpsw;

      offset_msg.posisi_x_offset = odox_offset; ///odox_buf untuk reset ke 0 bila ada data prev
      offset_msg.posisi_y_offset = odoy_offset; ///odoy_buf untuk reset ke 0 bila ada data prev
      offset_msg.sudut_w_offset  = odow_offset; ///odow_buf untuk reset ke 0 bila ada data prev
      pub_robot_offset.publish(offset_msg);
      
      robotx = odox_buf - odox_offset;
      roboty = odoy_buf - odoy_offset;
      robotw = odow_buf - odow_offset;
      while (robotw > 180) robotw -= 360;
      while (robotw< -180) robotw += 360;

      ROS_INFO_STREAM("ROBOT POS INIT DONE!!!!!");
      status_init_pos = 1;

      Xk_prev(0) = robotx;
      Xk_prev(1) = roboty;
      Xk_prev(2) = robotw;
    }
  }
  else //untuk continuous loacalization
  {
    // if(gps_q > 0.95) //mode full gps
    // {
    //   odox_offset = odox_buf - gpsx;
    //   odoy_offset = odoy_buf - gpsy;
    //   odow_offset = odow_buf - gpsw;
      
    //   msg.posisi_x_offset = odox_offset; ///odox_buf untuk reset ke 0 bila ada data prev
    //   msg.posisi_y_offset = odoy_offset; ///odoy_buf untuk reset ke 0 bila ada data prev
    //   msg.sudut_w_offset  = odow_offset; ///odow_buf untuk reset ke 0 bila ada data prev
    //   pub_robot_offset.publish(msg);
      
    //   robotx = odox_buf - odox_offset;
    //   roboty = odoy_buf - odoy_offset;
    //   robotw = odow_buf - odow_offset;
    //   while (robotw > 180) robotw -= 360;
    //   while (robotw< -180) robotw += 360;
    // }
    // else if(gps_q >= 0.70 && gps_q <= 0.95) //mode combine gps dan odom
    // {
    //   ///calculate angle first////////
    //   robotw = odow_buf - odow_offset;
    //   while (robotw > 180) robotw -= 360;
    //   while (robotw< -180) robotw += 360;
    //   w_mean_sin = sinf(robotw*TO_RAD)*0.9 + sinf(gpsw*TO_RAD)*0.1;
    //   w_mean_cos = cosf(robotw*TO_RAD)*0.9 + cosf(gpsw*TO_RAD)*0.1;
    //   w_mean = (float)atan2(w_mean_sin,w_mean_cos);
    //   robotw = w_mean * TO_DEG;
    //   ///then calculate x and y//////////
    //   robotx = 0.9*(odox_buf-odox_offset) + 0.1*(gpsx);
    //   roboty = 0.9*(odoy_buf-odoy_offset) + 0.1*(gpsy);

    //   odox_offset = odox_buf - robotx;
    //   odoy_offset = odoy_buf - roboty;
    //   odow_offset = odow_buf - robotw;
      
    //   msg.posisi_x_offset = odox_offset; ///odox_buf untuk reset ke 0 bila ada data prev
    //   msg.posisi_y_offset = odoy_offset; ///odoy_buf untuk reset ke 0 bila ada data prev
    //   msg.sudut_w_offset  = odow_offset; ///odow_buf untuk reset ke 0 bila ada data prev
    //   pub_robot_offset.publish(msg);
    // }
    // else //mode odom
    // {
    //   robotx = odox_buf - odox_offset;
    //   roboty = odoy_buf - odoy_offset;
    //   robotw = odow_buf - odow_offset;
    //   while (robotw > 180) robotw -= 360;
    //   while (robotw < -180) robotw += 360;
    // }

    // ROS_INFO_STREAM("X: " << robotx << " Y: " << roboty << " W: " << robotw);
    // ROS_INFO_STREAM("Vx: " << vx_global << " Vy: " << vy_global << " Vw: " << vw_global);
    // ROS_INFO_STREAM("Gx: " << vx_gps << " Gy: " << vy_gps << " Gw: " << vw_gps);
    // if(!(vx_global == 0.0 && vy_global == 0.0))
    // {
      Uk(0) = vx_global; Uk(1) = vy_global; Uk(2) = vw_global*TO_DEG;
      Xkp = A*Xk_prev + B*Uk + Wk;
      // Xkp(2) = odow_buf - odow_offset;
      while (Xkp(2) > 180) Xkp(2) -= 360;
      while (Xkp(2) < -180) Xkp(2) += 360;

      // if(gps_q > 0.90)
      // {
      //   Pk_prev = (Matrix <float, 3, 3>() << 
      //              pow(0.05,2), 0.0,        0.0,
      //              0.0,        pow(0.05,2), 0.0,
      //              0.0,        0.0,        pow(5.0,2)).finished();
      // }
      Pkp = A*Pk_prev*A.transpose() + Qk;
      Pkp = Pkp.array() * I.array(); //just take diagonal variance

      // if(gps_q > 0.001) gps_q = 0.001;
      Gps_Q(0,0) = exp(-(pow((gpsx-Xkp(0))/0.02,2)));
      Gps_Q(1,1) = exp(-(pow((gpsy-Xkp(1))/0.02,2)));
      Gps_Q(2,2) = exp(-(pow((gpsw-Xkp(2))/10.0,2)));
      // ROS_INFO_STREAM("X: " << Gps_Q(0,0) << " Y: " << Gps_Q(1,1) << " W: " << Gps_Q(2,2));
      R = (I-Gps_Q*H)*R_init + Gps_Q*(H*Pkp*H.transpose());
      // R = R_init;

      Kg = H*Pkp*H.transpose() + R;
      Kg = Pkp*H.transpose() * Kg.inverse();

      yk(0) = gpsx; yk(1) = gpsy; yk(2) = gpsw;
      Yk = C*yk + Zk;

      while(Yk(2) - Xkp(2) > 180){Yk(2) -= 360;}
      while(Yk(2) - Xkp(2) < -180){Yk(2) += 360;}
      Xk = Xkp + Kg*(Yk-H*Xkp);
      while (Xk(2) > 180) Xk(2) -= 360;
      while (Xk(2) < -180) Xk(2) += 360;

      Pk = (I - Kg*H) * Pkp;
      // Pk = (I - Kg*H)*Pkp + Kg*(H*Pk_init*H.transpose());

      Xk_prev = Xk;
      Pk_prev = Pk;

      robotx = Xk(0);
      roboty = Xk(1);
      robotw = Xk(2);
      ROS_INFO_STREAM("X: " << robotx << " Y: " << roboty << " W: " << robotw);

      odox_offset = odox_buf - robotx;
      odoy_offset = odoy_buf - roboty;
      odow_offset = odow_buf - robotw;
      
      offset_msg.posisi_x_offset = odox_offset; ///odox_buf untuk reset ke 0 bila ada data prev
      offset_msg.posisi_y_offset = odoy_offset; ///odoy_buf untuk reset ke 0 bila ada data prev
      offset_msg.sudut_w_offset  = odow_offset; ///odow_buf untuk reset ke 0 bila ada data prev
      pub_robot_offset.publish(offset_msg);
    }
    // ROS_INFO_STREAM("X: " << Kg(0,0) << " Y: " << Kg(1,1) << " W: " << Kg(2,2));

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

    gps_msg.position.x = gpsx;
    gps_msg.position.y = gpsy;
    tf2::Quaternion q_gps;
    q_gps.setRPY(0,0,gpsw*TO_RAD);
    gps_msg.orientation = tf2::toMsg(q_gps);
    pub_robot_gps.publish(gps_msg);

    odo_msg.position.x = Xkp(0);
    odo_msg.position.y = Xkp(1);
    tf2::Quaternion q_odo;
    q_odo.setRPY(0,0,Xkp(2)*TO_RAD);
    odo_msg.orientation = tf2::toMsg(q_odo);
    pub_robot_odo.publish(odo_msg);
    
  // }
  

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fusion_gps_odom");
  ros::NodeHandle nh;

  sub_robot_odom = nh.subscribe("data_terima_udp", 10, robot_odom_callback);
  sub_robot_hedge = nh.subscribe("hedge_pos_ang", 10, robot_hedge_callback);
  sub_hedge_quality = nh.subscribe("hedge_quality", 10, hedge_quality_callback);

  pub_robot_offset = nh.advertise<udp_bot::kirim_offset_udp>("offset_kirim_udp", 10);
  pub_robot_pos = nh.advertise<geometry_msgs::Pose>("robot_pos", 10);
  pub_robot_gps = nh.advertise<geometry_msgs::Pose>("robot_gps", 10);
  pub_robot_odo = nh.advertise<geometry_msgs::Pose>("robot_odo", 10);

  tf_broadcaster = new tf2_ros::TransformBroadcaster();

  ROS_INFO_STREAM("WAIT DATA FROM INDOOR GPS AND ODOMETRY");
  ros::topic::waitForMessage<marvelmind_nav::hedge_pos_ang>("hedge_pos_ang");
  ros::topic::waitForMessage<marvelmind_nav::hedge_quality>("hedge_quality");
  ros::topic::waitForMessage<udp_bot::terima_udp>("data_terima_udp");
  ROS_INFO_STREAM("INDOOR GPS AND ODOMETRY RECEIVE!!");

  ros::Duration(4.0).sleep(); // sleep for 4 second
  ROS_INFO_STREAM("[SENSOR FUSION BEGIN]");

  fusion_process_timer = nh.createTimer(ros::Duration(0.01), fusion_process_callback);

  ros::spin();

  return 0;
}
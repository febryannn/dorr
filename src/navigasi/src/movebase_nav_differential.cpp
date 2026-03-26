#include "iostream"
#include "math.h"
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <udp_bot/terima_udp.h>
#include <udp_bot/kirim_kecepatan_udp.h>
#include <udp_bot/kirim_offset_udp.h>

#define TO_DEG 57.295779513
#define TO_RAD 0.01745329252

#define kp_trans 1.0
#define ki_trans 0.0
#define kd_trans 0.0

#define kp_rot 0.048
#define ki_rot 0.0
#define kd_rot 0.05

#define D 0.14 // titik pusat agv ke pusat motor

float VT = 0.2;
float VR = 0.5;

float robotx, roboty, robotw;
double roll, pitch, yaw;

float vx_out, vy_out, vw_out;
uint8_t tombol_temp;
int tombol0, tombol1;

int status_mulai;
int status_routine;

float odox_buf, odoy_buf, odow_buf;
float odox_offset, odoy_offset, odow_offset;
float robot_initx, robot_inity, robot_initw;

ros::Subscriber sub_robot_pos;
ros::Subscriber sub_tombol;
ros::Subscriber sub_cmd_vel;
ros::Subscriber sub_goalPose_to_moveBase;

ros::Publisher pub_robot_vel;
udp_bot::kirim_kecepatan_udp vel_msg;

ros::Publisher pub_robot_offset;
udp_bot::kirim_offset_udp offset_msg;

ros::Publisher pub_goalPose_to_moveBase;

ros::Timer simple_nav_timer;

int jalan_posisi_sudut(float targetx, float targety, float targetw, float v_max, float w_max,
                        float posx, float posy, float posw, float tolerance_x, float tolerance_y, float tolerance_w)
{
    static float error_pos[3];
    static float previous_error_pos[3];
    static float p_pos[3], i_pos[3], d_pos[3];
    static float v_out[3];

    posx = posx - D*cosf(posw*TO_RAD);
    posy = posy - D*sinf(posw*TO_RAD);

    targetx = targetx - D*cosf(targetw*TO_RAD);
    targety = targety - D*sinf(targetw*TO_RAD);
    if (fabs(targetx - posx) >= fabs(tolerance_x) || fabs(targety - posy) >= fabs(tolerance_y))
	{
		targetw = atan2f(targety - posy, targetx - posx) * TO_DEG;
        if(fabs(targetw - posw) > 90.0) 
        {
            if(targetw - posw > 180){targetw -= 360;}
            else if(targetw - posw < -180){targetw += 360;}

            if(fabs(targetw - posw) > 90.0) targetw += 180;
        }
	}

    error_pos[0] = targetx - posx;
    error_pos[1] = targety - posy;

    error_pos[2] = targetw - posw;
    if(error_pos[2] > 180){targetw -= 360;}
    else if(error_pos[2] < -180){targetw += 360;}
    error_pos[2] = targetw - posw;

    for(int i = 0; i < 3; i++)
	{
        if(i != 2){
            p_pos[i] = kp_trans * error_pos[i];
            i_pos[i] += ki_trans * error_pos[i];
            d_pos[i] = kd_trans * (error_pos[i] - previous_error_pos[i]);
        }
        else{
            p_pos[i] = kp_rot * error_pos[i];
            i_pos[i] += ki_rot * error_pos[i];
            d_pos[i] = kd_rot * (error_pos[i] - previous_error_pos[i]);
        }
		previous_error_pos[i] = error_pos[i];

        if(i != 2){
            if(i_pos[i] > v_max) i_pos[i] = v_max;
		    else if(i_pos[i] < -v_max) i_pos[i] = -v_max;
        }
        else{
            if(i_pos[i] > w_max) i_pos[i] = w_max;
		    else if(i_pos[i] < -w_max) i_pos[i] = -w_max;
        }

		v_out[i] = p_pos[i] + i_pos[i] + d_pos[i];

        if(i != 2){
            if(v_out[i] > v_max) v_out[i] = v_max;
		    else if(v_out[i] < -v_max) v_out[i] = -v_max;
        }
        else{
            if(v_out[i] > w_max) v_out[i] = w_max;
		    else if(v_out[i] < -w_max) v_out[i] = -w_max;
        }
	}

    ROS_INFO_STREAM("Vx_global: " << v_out[0] << " Vy_global: " << v_out[1] << " Vw_global: " << v_out[2]);
    if( fabs(error_pos[0]) <= fabs(tolerance_x) && fabs(error_pos[1]) <= fabs(tolerance_y) && fabs(error_pos[2]) <= fabs(tolerance_w) )
    {
        vx_out = 0;
        vy_out = 0;
        vw_out = 0;

        ROS_INFO_STREAM("Vx_local: " << vx_out << " Vy_local: " << vy_out << " Vw_local: " << vw_out);
        return 1;
    }
    else
    {
        // v_kanan = (v_out[0]/ R) * cosf(posw * conv_radian) + (v_out[1]/ R) * sinf(posw * conv_radian) + (L/2*R) * v_out[2];
	    // v_kiri = (v_out[0]/ R) * cosf(posw * conv_radian) + (v_out[1]/ R) * sinf(posw * conv_radian) - (L/2*R) * v_out[2];

        vx_out = 0;
        vy_out = v_out[0]*cosf(posw*TO_RAD) + v_out[1]*sinf(posw*TO_RAD);
        vw_out = v_out[2]; 

        ROS_INFO_STREAM("Vx_local: " << vx_out << " Vy_local: " << vy_out << " Vw_local: " << vw_out << " POSW: " << posw);
        return 0;
    }
    

}

void robot_pos_callback(const geometry_msgs::Pose::ConstPtr& msg)
{
    robotx = (float)msg->position.x;
    roboty = (float)msg->position.y;
    tf2::Quaternion q;
    tf2::fromMsg(msg->orientation, q);
    tf2::Matrix3x3 m(q);
    m.getRPY(roll,pitch,yaw);
    robotw = (float)yaw*TO_DEG;

    // ROS_INFO_STREAM("X: " << robotx << " Y: " << roboty << " W: " << robotw);
}

void tombol_odom_callback(const udp_bot::terima_udp::ConstPtr& msg)
{
    odox_buf = msg->posisi_x_buffer;
    odoy_buf = msg->posisi_y_buffer;
    odow_buf = msg->sudut_w_buffer;

    tombol_temp = msg->tombol;
    tombol0 = (int)( (tombol_temp & 0x01) >> 0);
    tombol1 = (int)( (tombol_temp & 0x02) >> 1);
    // ROS_INFO_STREAM("Tombol0: " << tombol0 << " Tombol1: " <<tombol1);
}

void nav_process_callback(const ros::TimerEvent& event)
{
    // if(tombol0 == 0) status_mulai = 1;
    if(status_mulai == 0)
    {
        // robot_initx = robot_inity = robot_initw = 0;

        // odox_offset = odox_buf - robot_initx;
        // odoy_offset = odoy_buf - robot_inity;
        // odow_offset = odow_buf - robot_initw;
        
        // offset_msg.posisi_x_offset = odox_offset; ///odox_buf untuk reset ke 0 bila ada data prev
        // offset_msg.posisi_y_offset = odoy_offset; ///odoy_buf untuk reset ke 0 bila ada data prev
        // offset_msg.sudut_w_offset  = odow_offset; ///odow_buf untuk reset ke 0 bila ada data prev
        // pub_robot_offset.publish(offset_msg);
        status_mulai = 1;
    }

    if(status_mulai == 1)
    {
        // // ROS_INFO_STREAM("START JALAN POSISI SUDUT!!!!!");
        // ROS_INFO_STREAM("Status _routine: " << status_routine);
        // switch(status_routine)
        // {
        //     // case 0:
        //     //     ROS_INFO_STREAM("CASE 0");
        //     //     if(jalan_posisi_sudut(1.0, 0.5, 180.0, VT, VR, 
        //     //        robotx, roboty, robotw, 0.05, 0.05, 3))
        //     //     {
        //     //         status_routine++;
        //     //     }
        //     // break;

        //     // case 1:
        //     //     ROS_INFO_STREAM("CASE 1");
        //     //     if(jalan_posisi_sudut(0.5, 1.5, 135.0, VT, VR, 
        //     //        robotx, roboty, robotw, 0.05, 0.05, 3))
        //     //     {
        //     //         status_routine++;
        //     //     }
        //     // break;

        //     // case 2:
        //     //     ROS_INFO_STREAM("CASE 2");
        //     //     if(jalan_posisi_sudut(0.05, 2.0, -45.0, VT, VR, 
        //     //        robotx, roboty, robotw, 0.05, 0.05, 3))
        //     //     {
        //     //         status_routine++;
        //     //     }
        //     // break;

        //     default:
        //         ROS_INFO_STREAM("CASE DIAM");
        //         vx_out = 0;
        //         vy_out = 0;
        //         vw_out = 0;
        //     break;
        // }

        // vel_msg.kecepatan_x = vx_out; 
        // vel_msg.kecepatan_y = vy_out; 
        // vel_msg.kecepatan_sudut = vw_out; 
        // pub_robot_vel.publish(vel_msg);

    }
}

void sub_cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    vx_out = msg->linear.x; 
    vy_out = msg->linear.y; 
    vw_out = msg->angular.z; 
    vel_msg.kecepatan_x = vy_out; 
    vel_msg.kecepatan_y = vx_out; 
    vel_msg.kecepatan_sudut = vw_out;  
    pub_robot_vel.publish(vel_msg);

    // ROS_INFO_STREAM("VX: " << vx_out << " VY: " << vy_out << " VW: " << vw_out);
}

void goalPose_to_moveBase_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    pub_goalPose_to_moveBase.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "movebase_nav_differential");
  ros::NodeHandle nh;

  sub_robot_pos = nh.subscribe("robot_pos", 10, robot_pos_callback);
  sub_tombol = nh.subscribe("data_terima_udp", 10, tombol_odom_callback);
  sub_goalPose_to_moveBase = nh.subscribe("goal_pose", 10, goalPose_to_moveBase_callback);
  sub_cmd_vel = nh.subscribe("cmd_vel", 10, sub_cmd_vel_callback);
  pub_robot_vel = nh.advertise<udp_bot::kirim_kecepatan_udp>("kecepatan_kirim_udp", 10);
  pub_robot_offset = nh.advertise<udp_bot::kirim_offset_udp>("offset_kirim_udp", 10);
  pub_goalPose_to_moveBase = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);

  ROS_INFO_STREAM("WAIT ROBOT POS DATA");
  ros::topic::waitForMessage<udp_bot::terima_udp>("data_terima_udp");

  ros::Duration(2.0).sleep(); // sleep for 4 second
  ROS_INFO_STREAM("[SIMPLE NAVIGATION DIFFERENTIAL BEGIN]");

  simple_nav_timer = nh.createTimer(ros::Duration(0.01), nav_process_callback);

  ros::spin();

  return 0;
}
#include "iostream"
#include "math.h"
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <udp_bot/terima_udp.h>
#include <udp_bot/kirim_kecepatan_udp.h>
#include <udp_bot/kirim_offset_udp.h>

#define TO_DEG 57.295779513
#define TO_RAD 0.01745329252

#define kp_trans 1.0
#define ki_trans 0.0
#define kd_trans 0.0

#define kp_rot 0.085
#define ki_rot 0.0
#define kd_rot 0.1

float VT = 0.3;
float VR = 0.4;

float robotx, roboty, robotw;
double roll, pitch, yaw;

float vx_out, vy_out, vw_out;
uint8_t tombol_temp;
int tombol0, tombol1;

int status_mulai;
int status_routine;

ros::Subscriber sub_robot_pos;
ros::Subscriber sub_tombol;

ros::Publisher pub_robot_vel;
udp_bot::kirim_kecepatan_udp vel_msg;

ros::Publisher pub_robot_offset;
udp_bot::kirim_offset_udp offset_msg;

ros::Timer simple_nav_timer;

int jalan_posisi_sudut(float targetx, float targety, float targetw, float v_max, float w_max,
                        float posx, float posy, float posw, float tolerance_x, float tolerance_y, float tolerance_w)
{
    static float error_pos[3];
    static float previous_error_pos[3];
    static float p_pos[3], i_pos[3], d_pos[3];
    static float v_out[3];

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
    if( abs(error_pos[0]) <= abs(tolerance_x) && abs(error_pos[1]) <= abs(tolerance_y) && abs(error_pos[2]) <= abs(tolerance_w) )
    {
        vx_out = 0;
        vy_out = 0;
        vw_out = 0;

        ROS_INFO_STREAM("Vx_local: " << vx_out << " Vy_local: " << vy_out << " Vw_local: " << vw_out);
        return 1;
    }
    else
    {
        vx_out = v_out[0]*sinf(posw*TO_RAD) + v_out[1]*-cosf(posw*TO_RAD);
        vy_out = v_out[0]*cosf(posw*TO_RAD) + v_out[1]*sinf(posw*TO_RAD);
        vw_out = v_out[2];
        ROS_INFO_STREAM("Vx_local: " << vx_out << " Vy_local: " << vy_out << " Vw_local: " << vw_out);
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

void tombol_callback(const udp_bot::terima_udp::ConstPtr& msg)
{
    tombol_temp = msg->tombol;
    tombol0 = (int)( (tombol_temp & 0x01) >> 0);
    tombol1 = (int)( (tombol_temp & 0x02) >> 1);
    // ROS_INFO_STREAM("Tombol0: " << tombol0 << " Tombol1: " <<tombol1);
}

void nav_process_callback(const ros::TimerEvent& event)
{
    // if(tombol0 == 0) status_mulai = 1;
    status_mulai = 1;

    if(status_mulai == 1)
    {
        // ROS_INFO_STREAM("START JALAN POSISI SUDUT!!!!!");
        ROS_INFO_STREAM("Status _routine: " << status_routine);
        switch(status_routine)
        {
            case 0:
                ROS_INFO_STREAM("CASE 0");
                if(jalan_posisi_sudut(0.4, 4.3, 0.0, VT, VR, 
                   robotx, roboty, robotw, 0.05, 0.05, 3))
                {
                    status_routine++;
                }
            break;

            case 1:
                ROS_INFO_STREAM("CASE 1");
                if(jalan_posisi_sudut(5.3, 4.3, 0.0, VT, VR, 
                   robotx, roboty, robotw, 0.05, 0.05, 3))
                {
                    status_routine++;
                }
            break;

            case 2:
                ROS_INFO_STREAM("CASE 2");
                if(jalan_posisi_sudut(5.3, 4.3, -90.0, VT, VR, 
                   robotx, roboty, robotw, 0.05, 0.05, 3))
                {
                    status_routine++;
                }
            break;

            case 3:
                ROS_INFO_STREAM("CASE 3");
                if(jalan_posisi_sudut(5.3, 1.0, -90.0, VT, VR, 
                   robotx, roboty, robotw, 0.05, 0.05, 3))
                {
                    status_routine++;
                }
            break;

            case 4:
                ROS_INFO_STREAM("CASE 4");
                if(jalan_posisi_sudut(4.4, 0.06, -90.0, VT, VR,
                   robotx, roboty, robotw, 0.05, 0.05, 3))
                {
                    status_routine++;
                }
            break;

            case 5:
                ROS_INFO_STREAM("CASE 5");
                if(jalan_posisi_sudut(4.4, 0.06, -90.0, VT, VR,
                   robotx, roboty, robotw, 0.05, 0.05, 3))
                {
                    status_routine++;
                }
            break;

            case 6:
                ROS_INFO_STREAM("CASE 7");
                if(jalan_posisi_sudut(1.0, 0.06, -90.0, VT, VR,
                   robotx, roboty, robotw, 0.05, 0.05, 3))
                {
                    status_routine++;
                }
            break;

            case 7:
                ROS_INFO_STREAM("CASE 7");
                if(jalan_posisi_sudut(0.4, 0.6, -90.0, VT, VR,
                   robotx, roboty, robotw, 0.05, 0.05, 3))
                {
                    status_routine++;
                }
            break;

            case 8:
                ROS_INFO_STREAM("CASE 8");
                if(jalan_posisi_sudut(0.4, 0.6, 180.0, VT, VR,
                   robotx, roboty, robotw, 0.05, 0.05, 3))
                {
                    status_routine++;
                }
            break;

            case 9:
                ROS_INFO_STREAM("CASE 9");
                if(jalan_posisi_sudut(0.4, 2.0, 180.0, VT, VR,
                   robotx, roboty, robotw, 0.05, 0.05, 3))
                {
                    status_routine++;
                }
            break;

            case 10:
                ROS_INFO_STREAM("CASE 10");
                if(jalan_posisi_sudut(0.4, 2.0, -90.0, VT, VR,
                   robotx, roboty, robotw, 0.05, 0.05, 3))
                {
                    status_routine++;
                }
            break;

            case 11:
                ROS_INFO_STREAM("CASE 11");
                if(jalan_posisi_sudut(3.4, 2.0, -90.0, VT, VR,
                   robotx, roboty, robotw, 0.05, 0.05, 3))
                {
                    status_routine++;
                }
            break;

            // case 4:
            //     ROS_INFO_STREAM("CASE 4");
            //     if(jalan_posisi_sudut(0.9, 0.11, 180.0, 0.3, 0.45, 
            //        robotx, roboty, robotw, 0.05, 0.05, 3))
            //     {
            //         status_routine++;
            //     }
            // break;

            // case 5:
            //     ROS_INFO_STREAM("CASE 5");
            //     if(jalan_posisi_sudut(0.4, 0.8, -180.0, 0.3, 0.45, 
            //        robotx, roboty, robotw, 0.05, 0.05, 3))
            //     {
            //         status_routine = 0;
            //     }
            // break;

            default:
                ROS_INFO_STREAM("CASE DIAM");
                vx_out = 0;
                vy_out = 0;
                vw_out = 0;
            break;
        }
        vel_msg.kecepatan_x = vx_out; 
        vel_msg.kecepatan_y = vy_out; 
        vel_msg.kecepatan_sudut = vw_out; 
        pub_robot_vel.publish(vel_msg);

    }



}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_nav_omni");
  ros::NodeHandle nh;

  ROS_INFO_STREAM("WAIT ROBOT POS DATA");
  ros::topic::waitForMessage<geometry_msgs::Pose>("robot_pos");
  ros::topic::waitForMessage<udp_bot::terima_udp>("data_terima_udp");

  sub_robot_pos = nh.subscribe("robot_pos", 10, robot_pos_callback);
  sub_tombol = nh.subscribe("data_terima_udp", 10, tombol_callback);
  pub_robot_vel = nh.advertise<udp_bot::kirim_kecepatan_udp>("kecepatan_kirim_udp", 10);
  pub_robot_offset = nh.advertise<udp_bot::kirim_offset_udp>("offset_kirim_udp", 10);

  ros::Duration(2.0).sleep(); // sleep for 4 second
  ROS_INFO_STREAM("[SIMPLE NAVIGATION BEGIN]");

  simple_nav_timer = nh.createTimer(ros::Duration(0.01), nav_process_callback);

  ros::spin();

  return 0;
}
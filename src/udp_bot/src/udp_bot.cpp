#include "ros/ros.h"
#include "std_msgs/String.h"
#include <udp_bot/kirim_kecepatan_udp.h>
#include <udp_bot/kirim_offset_udp.h>
#include <udp_bot/terima_udp.h>
#include "geometry_msgs/Pose.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "iostream"
#include <cstring> 

#include "udp_bot/UdpSocket.h"
#ifdef _WIN32
    #include "getopt.h"
#else
    #include <unistd.h>
#endif

#define TO_DEG 57.295779513
#define TO_RAD 0.01745329252

////untuk ip tujuan kirim data dan portnya ETH
const char *addr = "169.254.1.15";
uint16_t port = 4444;
////untuk bind dan local port ETH
const char *listenAddr = "169.254.1.16"; 
uint16_t localPort = 8888;

////untuk ip tujuan kirim data dan portnya Wifi
const char *addr_lan = "192.168.0.162";
uint16_t port_lan = 8787;
////untuk bind dan local port Wifi
const char *listenAddr_lan = "192.168.0.161";
uint16_t localPort_lan = 9999;



class UnicastApp {
public:
    ros::Publisher terima_udp_pub;
    ros::Subscriber kirim_kecepatan_udp_sub;
    ros::Subscriber kirim_offset_udp_sub;
    udp_bot::terima_udp msg;

    ros::Timer udp_send_timer;
    
    char stm_kirim[64] = {'t','e','l'};
    char stm_terima[70];

    //////terima udp///
    float posisi_x_buffer, posisi_y_buffer, sudut_w_buffer;
    float kecepatan_robotx, kecepatan_roboty, kecepatan_robotw;
    int cnt_send_kecepatan;
    float sudut_servo_act;
    uint8_t tombol;
    float vx_global, vy_global, vw_global;
    //////////////////////
    /////kirim udp/////
    float kecepatan_x, kecepatan_y, kecepatan_sudut;
    float posisi_x_offset, posisi_y_offset, sudut_w_offset;
    float sudut_servo;
    //////////////////////////////////

    // UDP Multicast
    UnicastApp(ros::NodeHandle *nh, const char *remoteAddr, const char *listenAddr, uint16_t localPort, uint16_t port);

    virtual ~UnicastApp() = default;

    void onReceiveData(const char *data, size_t size) ;

    void sendMsg(const char *data, size_t len);

    void udp_kecepatan_send_callback(const udp_bot::kirim_kecepatan_udp::ConstPtr& msg);

    void udp_offset_send_callback(const udp_bot::kirim_offset_udp::ConstPtr& msg);

    void send_udp_data_callback(const ros::TimerEvent& event);

private:
    sockets::SocketOpt m_socketOpts;
    sockets::UdpSocket<UnicastApp> m_unicast;
};

UnicastApp::UnicastApp(ros::NodeHandle *nh, const char *remoteAddr, const char *listenAddr, uint16_t localPort, uint16_t port) : m_socketOpts({ sockets::TX_BUFFER_SIZE, sockets::RX_BUFFER_SIZE, listenAddr}), m_unicast(*this, &m_socketOpts) {
    
    terima_udp_pub = nh->advertise<udp_bot::terima_udp>("data_terima_udp", 10);
    kirim_kecepatan_udp_sub = nh->subscribe("kecepatan_kirim_udp", 10, &UnicastApp::udp_kecepatan_send_callback, this);
    kirim_offset_udp_sub = nh->subscribe("offset_kirim_udp", 10, &UnicastApp::udp_offset_send_callback, this);
    
    sockets::SocketRet ret = m_unicast.startUnicast(remoteAddr, localPort, port);
    if (ret.m_success) {
        std::cout << "Listening on UDP " << listenAddr << ":" << localPort << " sending to " << remoteAddr << ":" << port << "\n";
    } else {
        std::cout << "Error: " << ret.m_msg << "\n";
        exit(1); // NOLINT
    }

    udp_send_timer = nh->createTimer(ros::Duration(0.01), &UnicastApp::send_udp_data_callback, this);
    
}

void UnicastApp::sendMsg(const char *data, size_t len) {
    auto ret = m_unicast.sendMsg(data, len);
    if (!ret.m_success) {
        std::cout << "Send Error: " << ret.m_msg << "\n";
    }
}

void usage() {
    std::cout << "UnicastApp -a <remoteAddr> -l <localPort> -p <port> -L <listenAddr>\n";
}

void UnicastApp::onReceiveData(const char *data, size_t size) {
    for(int i=0; i<size; i++){
        stm_terima[i] = data[i];
    }

    memcpy(&posisi_x_buffer, stm_terima + 3, 4);
    memcpy(&posisi_y_buffer, stm_terima + 7, 4);
    memcpy(&sudut_w_buffer, stm_terima + 11, 4);
    memcpy(&kecepatan_robotx, stm_terima + 15, 4);
    memcpy(&kecepatan_roboty, stm_terima + 19, 4);
    memcpy(&kecepatan_robotw, stm_terima + 23, 4);
    memcpy(&sudut_servo_act, stm_terima + 27, 4);
    memcpy(&tombol, stm_terima + 31, 1);
    memcpy(&vx_global, stm_terima + 32, 4);
    memcpy(&vy_global, stm_terima + 36, 4);
    memcpy(&vw_global, stm_terima + 40, 4);

    msg.posisi_x_buffer = posisi_x_buffer;
    msg.posisi_y_buffer = posisi_y_buffer;
    msg.sudut_w_buffer = sudut_w_buffer;
    msg.kecepatan_robotx = kecepatan_robotx;
    msg.kecepatan_roboty = kecepatan_roboty;
    msg.kecepatan_robotw = kecepatan_robotw;
    msg.sudut_servo_act = sudut_servo_act;
    msg.tombol = tombol;
    msg.vx_global = vx_global;
    msg.vy_global = vy_global;
    msg.vw_global = vw_global;
    terima_udp_pub.publish(msg);

    // std::string str(reinterpret_cast<const char *>(data), size);
    // std::cout << "Received: " << str << "\n";
    // ROS_INFO_STREAM("DATA:" << sudut_w_buffer << "\n");
}


void UnicastApp::udp_kecepatan_send_callback(const udp_bot::kirim_kecepatan_udp::ConstPtr& msg)
{
    // ROS_INFO_STREAM("vx: " << msg->kecepatan_x << ", vy: " << msg->kecepatan_y << ", vw: " << msg->kecepatan_sudut);
    kecepatan_x     = msg->kecepatan_x;
    kecepatan_y     = msg->kecepatan_y;
    kecepatan_sudut = msg->kecepatan_sudut;
    sudut_servo     = msg->sudut_servo;
    
    /////for safety
    cnt_send_kecepatan = 0;
}

void UnicastApp::udp_offset_send_callback(const udp_bot::kirim_offset_udp::ConstPtr& msg)
{
    // ROS_INFO_STREAM("ofs_x: " << msg->posisi_x_offset << ", ofs_y: " << msg->posisi_y_offset << ", ofs_w: " << msg->sudut_w_offset);
    posisi_x_offset = msg->posisi_x_offset; 
    posisi_y_offset = msg->posisi_y_offset;
    sudut_w_offset  = msg->sudut_w_offset;
}

void UnicastApp::send_udp_data_callback(const ros::TimerEvent& event)
{   
    if(cnt_send_kecepatan >= 100) //jika lebih dari 1s tidak terima kecepatan
    {
        kecepatan_x     = 0;
        kecepatan_y     = 0;
        kecepatan_sudut = 0;
    }
    if(cnt_send_kecepatan >= 100) cnt_send_kecepatan = 100;
	else cnt_send_kecepatan++;
    // ROS_INFO_STREAM(cnt_send_kecepatan);

    memcpy(stm_kirim + 3, &kecepatan_x, 4);
    memcpy(stm_kirim + 7, &kecepatan_y, 4);
    memcpy(stm_kirim + 11, &kecepatan_sudut, 4);
    memcpy(stm_kirim + 15, &posisi_x_offset, 4);
    memcpy(stm_kirim + 19, &posisi_y_offset, 4);
    memcpy(stm_kirim + 23, &sudut_w_offset, 4);
    memcpy(stm_kirim + 27, &sudut_servo, 4);

    UnicastApp::sendMsg(stm_kirim, sizeof(stm_kirim));
    // ROS_INFO_STREAM("vx: " << kecepatan_x << ", vy: " << kecepatan_y << ", vw: " << kecepatan_sudut);
}


class UnicastLan {
public:
    ros::Subscriber sub_robot_pos;
    ros::Subscriber sub_robot_gps;
    ros::Subscriber sub_robot_odo;

    ros::Timer pos_send_timer;

    char lan_kirim[40] = {'t','e','u'};
    float robot_posx, robot_posy, robot_posw;
    double roll, pitch, yaw;

    float robot_gpsx, robot_gpsy, robot_gpsw;
    double roll_gps, pitch_gps, yaw_gps;

    float robot_odox, robot_odoy, robot_odow;
    double roll_odo, pitch_odo, yaw_odo;

    // UDP Multicast
    UnicastLan(ros::NodeHandle *nh, const char *remoteAddr, const char *listenAddr, uint16_t localPort, uint16_t port);

    virtual ~UnicastLan() = default;

    void onReceiveData(const char *data, size_t size) ;

    void sendMsg(const char *data, size_t len);

    void robot_pos_callback(const geometry_msgs::Pose::ConstPtr& msg);

    void robot_gps_callback(const geometry_msgs::Pose::ConstPtr& msg);

    void robot_odo_callback(const geometry_msgs::Pose::ConstPtr& msg);

    void send_pos_data_callback(const ros::TimerEvent& event);

private:
    sockets::SocketOpt m_socketOpts;
    sockets::UdpSocket<UnicastLan> m_unicast;
};

UnicastLan::UnicastLan(ros::NodeHandle *nh, const char *remoteAddr, const char *listenAddr, uint16_t localPort, uint16_t port) : m_socketOpts({ sockets::TX_BUFFER_SIZE, sockets::RX_BUFFER_SIZE, listenAddr}), m_unicast(*this, &m_socketOpts) {

    sub_robot_pos = nh->subscribe("robot_pos", 10, &UnicastLan::robot_pos_callback, this);
    sub_robot_gps = nh->subscribe("robot_gps", 10, &UnicastLan::robot_gps_callback, this);
    sub_robot_odo = nh->subscribe("robot_odo", 10, &UnicastLan::robot_odo_callback, this);
    
    sockets::SocketRet ret = m_unicast.startUnicast(remoteAddr, localPort, port);
    if (ret.m_success) {
        std::cout << "Listening on UDP " << listenAddr << ":" << localPort << " sending to " << remoteAddr << ":" << port << "\n";
    } else {
        std::cout << "Error: " << ret.m_msg << "\n";
        exit(1); // NOLINT
    }

    pos_send_timer = nh->createTimer(ros::Duration(0.01), &UnicastLan::send_pos_data_callback, this);

}

void UnicastLan::sendMsg(const char *data, size_t len) {
    auto ret = m_unicast.sendMsg(data, len);
    if (!ret.m_success) {
        std::cout << "Send Error: " << ret.m_msg << "\n";
    }
}

void UnicastLan::onReceiveData(const char *data, size_t size) {
    // std::string str(reinterpret_cast<const char *>(data), size);
    // std::cout << "Received: " << str << "\n";
}

void UnicastLan::robot_pos_callback(const geometry_msgs::Pose::ConstPtr& msg)
{
    robot_posx = (float)msg->position.x;
    robot_posy = (float)msg->position.y;
    tf2::Quaternion q;
    tf2::fromMsg(msg->orientation, q);
    tf2::Matrix3x3 m(q);
    m.getRPY(roll,pitch,yaw);
    robot_posw = (float)yaw*TO_DEG;

    // ROS_INFO_STREAM("X: " << robot_posx << " Y: " << robot_posy << " W: " << robot_posw);
}

void UnicastLan::robot_gps_callback(const geometry_msgs::Pose::ConstPtr& msg)
{
    robot_gpsx = (float)msg->position.x;
    robot_gpsy = (float)msg->position.y;
    tf2::Quaternion q_gps;
    tf2::fromMsg(msg->orientation, q_gps);
    tf2::Matrix3x3 m_gps(q_gps);
    m_gps.getRPY(roll_gps,pitch_gps,yaw_gps);
    robot_gpsw = (float)yaw_gps*TO_DEG;

    // ROS_INFO_STREAM("X: " << robot_posx << " Y: " << robot_posy << " W: " << robot_posw);
}

void UnicastLan::robot_odo_callback(const geometry_msgs::Pose::ConstPtr& msg)
{
    robot_odox = (float)msg->position.x;
    robot_odoy = (float)msg->position.y;
    tf2::Quaternion q_odo;
    tf2::fromMsg(msg->orientation, q_odo);
    tf2::Matrix3x3 m_odo(q_odo);
    m_odo.getRPY(roll_odo,pitch_odo,yaw_odo);
    robot_odow = (float)yaw_odo*TO_DEG;

    // ROS_INFO_STREAM("X: " << robot_posx << " Y: " << robot_posy << " W: " << robot_posw);
}

void UnicastLan::send_pos_data_callback(const ros::TimerEvent& event)
{   
    memcpy(lan_kirim + 3, &robot_posx, 4);
    memcpy(lan_kirim + 7, &robot_posy, 4);
    memcpy(lan_kirim + 11, &robot_posw, 4);
    memcpy(lan_kirim + 15, &robot_gpsx, 4);
    memcpy(lan_kirim + 19, &robot_gpsy, 4);
    memcpy(lan_kirim + 23, &robot_gpsw, 4);
    memcpy(lan_kirim + 27, &robot_odox, 4);
    memcpy(lan_kirim + 31, &robot_odoy, 4);
    memcpy(lan_kirim + 35, &robot_odow, 4);
    UnicastLan::sendMsg(lan_kirim, sizeof(lan_kirim));
    
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "udp_bot");
  ros::NodeHandle nh;

  auto *app = new UnicastApp(&nh, addr, listenAddr, localPort, port);
  
//   auto *lan = new UnicastLan(&nh, addr_lan, listenAddr_lan, localPort_lan, port_lan);

  ros::spin();

  return 0;
}
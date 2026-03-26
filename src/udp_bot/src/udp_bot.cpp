#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "udp_bot/msg/kirim_kecepatan_udp.hpp"
#include "udp_bot/msg/kirim_offset_udp.hpp"
#include "udp_bot/msg/terima_udp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
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
    rclcpp::Publisher<udp_bot::msg::TerimaUdp>::SharedPtr terima_udp_pub;
    rclcpp::Subscription<udp_bot::msg::KirimKecepatanUdp>::SharedPtr kirim_kecepatan_udp_sub;
    rclcpp::Subscription<udp_bot::msg::KirimOffsetUdp>::SharedPtr kirim_offset_udp_sub;
    udp_bot::msg::TerimaUdp msg;

    rclcpp::TimerBase::SharedPtr udp_send_timer;

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
    UnicastApp(rclcpp::Node::SharedPtr node, const char *remoteAddr, const char *listenAddr, uint16_t localPort, uint16_t port);

    virtual ~UnicastApp() = default;

    void onReceiveData(const char *data, size_t size);

    void sendMsg(const char *data, size_t len);

    void udp_kecepatan_send_callback(const udp_bot::msg::KirimKecepatanUdp::SharedPtr msg);

    void udp_offset_send_callback(const udp_bot::msg::KirimOffsetUdp::SharedPtr msg);

    void send_udp_data_callback();

private:
    sockets::SocketOpt m_socketOpts;
    sockets::UdpSocket<UnicastApp> m_unicast;
};

UnicastApp::UnicastApp(rclcpp::Node::SharedPtr node, const char *remoteAddr, const char *listenAddr, uint16_t localPort, uint16_t port) : m_socketOpts({ sockets::TX_BUFFER_SIZE, sockets::RX_BUFFER_SIZE, listenAddr}), m_unicast(*this, &m_socketOpts) {

    terima_udp_pub = node->create_publisher<udp_bot::msg::TerimaUdp>("data_terima_udp", 10);
    kirim_kecepatan_udp_sub = node->create_subscription<udp_bot::msg::KirimKecepatanUdp>(
        "kecepatan_kirim_udp", 10, std::bind(&UnicastApp::udp_kecepatan_send_callback, this, std::placeholders::_1));
    kirim_offset_udp_sub = node->create_subscription<udp_bot::msg::KirimOffsetUdp>(
        "offset_kirim_udp", 10, std::bind(&UnicastApp::udp_offset_send_callback, this, std::placeholders::_1));

    sockets::SocketRet ret = m_unicast.startUnicast(remoteAddr, localPort, port);
    if (ret.m_success) {
        std::cout << "Listening on UDP " << listenAddr << ":" << localPort << " sending to " << remoteAddr << ":" << port << "\n";
    } else {
        std::cout << "Error: " << ret.m_msg << "\n";
        exit(1); // NOLINT
    }

    udp_send_timer = node->create_wall_timer(
        std::chrono::milliseconds(10), std::bind(&UnicastApp::send_udp_data_callback, this));
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
    for(int i=0; i<(int)size; i++){
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
    terima_udp_pub->publish(msg);
}


void UnicastApp::udp_kecepatan_send_callback(const udp_bot::msg::KirimKecepatanUdp::SharedPtr msg)
{
    kecepatan_x     = msg->kecepatan_x;
    kecepatan_y     = msg->kecepatan_y;
    kecepatan_sudut = msg->kecepatan_sudut;
    sudut_servo     = msg->sudut_servo;

    /////for safety
    cnt_send_kecepatan = 0;
}

void UnicastApp::udp_offset_send_callback(const udp_bot::msg::KirimOffsetUdp::SharedPtr msg)
{
    posisi_x_offset = msg->posisi_x_offset;
    posisi_y_offset = msg->posisi_y_offset;
    sudut_w_offset  = msg->sudut_w_offset;
}

void UnicastApp::send_udp_data_callback()
{
    if(cnt_send_kecepatan >= 100) //jika lebih dari 1s tidak terima kecepatan
    {
        kecepatan_x     = 0;
        kecepatan_y     = 0;
        kecepatan_sudut = 0;
    }
    if(cnt_send_kecepatan >= 100) cnt_send_kecepatan = 100;
	else cnt_send_kecepatan++;

    memcpy(stm_kirim + 3, &kecepatan_x, 4);
    memcpy(stm_kirim + 7, &kecepatan_y, 4);
    memcpy(stm_kirim + 11, &kecepatan_sudut, 4);
    memcpy(stm_kirim + 15, &posisi_x_offset, 4);
    memcpy(stm_kirim + 19, &posisi_y_offset, 4);
    memcpy(stm_kirim + 23, &sudut_w_offset, 4);
    memcpy(stm_kirim + 27, &sudut_servo, 4);

    UnicastApp::sendMsg(stm_kirim, sizeof(stm_kirim));
}


class UnicastLan {
public:
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_robot_pos;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_robot_gps;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_robot_odo;

    rclcpp::TimerBase::SharedPtr pos_send_timer;

    char lan_kirim[40] = {'t','e','u'};
    float robot_posx, robot_posy, robot_posw;
    double roll, pitch, yaw;

    float robot_gpsx, robot_gpsy, robot_gpsw;
    double roll_gps, pitch_gps, yaw_gps;

    float robot_odox, robot_odoy, robot_odow;
    double roll_odo, pitch_odo, yaw_odo;

    // UDP Multicast
    UnicastLan(rclcpp::Node::SharedPtr node, const char *remoteAddr, const char *listenAddr, uint16_t localPort, uint16_t port);

    virtual ~UnicastLan() = default;

    void onReceiveData(const char *data, size_t size);

    void sendMsg(const char *data, size_t len);

    void robot_pos_callback(const geometry_msgs::msg::Pose::SharedPtr msg);

    void robot_gps_callback(const geometry_msgs::msg::Pose::SharedPtr msg);

    void robot_odo_callback(const geometry_msgs::msg::Pose::SharedPtr msg);

    void send_pos_data_callback();

private:
    sockets::SocketOpt m_socketOpts;
    sockets::UdpSocket<UnicastLan> m_unicast;
};

UnicastLan::UnicastLan(rclcpp::Node::SharedPtr node, const char *remoteAddr, const char *listenAddr, uint16_t localPort, uint16_t port) : m_socketOpts({ sockets::TX_BUFFER_SIZE, sockets::RX_BUFFER_SIZE, listenAddr}), m_unicast(*this, &m_socketOpts) {

    sub_robot_pos = node->create_subscription<geometry_msgs::msg::Pose>(
        "robot_pos", 10, std::bind(&UnicastLan::robot_pos_callback, this, std::placeholders::_1));
    sub_robot_gps = node->create_subscription<geometry_msgs::msg::Pose>(
        "robot_gps", 10, std::bind(&UnicastLan::robot_gps_callback, this, std::placeholders::_1));
    sub_robot_odo = node->create_subscription<geometry_msgs::msg::Pose>(
        "robot_odo", 10, std::bind(&UnicastLan::robot_odo_callback, this, std::placeholders::_1));

    sockets::SocketRet ret = m_unicast.startUnicast(remoteAddr, localPort, port);
    if (ret.m_success) {
        std::cout << "Listening on UDP " << listenAddr << ":" << localPort << " sending to " << remoteAddr << ":" << port << "\n";
    } else {
        std::cout << "Error: " << ret.m_msg << "\n";
        exit(1); // NOLINT
    }

    pos_send_timer = node->create_wall_timer(
        std::chrono::milliseconds(10), std::bind(&UnicastLan::send_pos_data_callback, this));
}

void UnicastLan::sendMsg(const char *data, size_t len) {
    auto ret = m_unicast.sendMsg(data, len);
    if (!ret.m_success) {
        std::cout << "Send Error: " << ret.m_msg << "\n";
    }
}

void UnicastLan::onReceiveData(const char *data, size_t size) {
    (void)data;
    (void)size;
}

void UnicastLan::robot_pos_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    robot_posx = (float)msg->position.x;
    robot_posy = (float)msg->position.y;
    tf2::Quaternion q;
    tf2::fromMsg(msg->orientation, q);
    tf2::Matrix3x3 m(q);
    m.getRPY(roll,pitch,yaw);
    robot_posw = (float)yaw*TO_DEG;
}

void UnicastLan::robot_gps_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    robot_gpsx = (float)msg->position.x;
    robot_gpsy = (float)msg->position.y;
    tf2::Quaternion q_gps;
    tf2::fromMsg(msg->orientation, q_gps);
    tf2::Matrix3x3 m_gps(q_gps);
    m_gps.getRPY(roll_gps,pitch_gps,yaw_gps);
    robot_gpsw = (float)yaw_gps*TO_DEG;
}

void UnicastLan::robot_odo_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    robot_odox = (float)msg->position.x;
    robot_odoy = (float)msg->position.y;
    tf2::Quaternion q_odo;
    tf2::fromMsg(msg->orientation, q_odo);
    tf2::Matrix3x3 m_odo(q_odo);
    m_odo.getRPY(roll_odo,pitch_odo,yaw_odo);
    robot_odow = (float)yaw_odo*TO_DEG;
}

void UnicastLan::send_pos_data_callback()
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
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("udp_bot");

  auto *app = new UnicastApp(node, addr, listenAddr, localPort, port);
  (void)app;

//   auto *lan = new UnicastLan(node, addr_lan, listenAddr_lan, localPort_lan, port_lan);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}

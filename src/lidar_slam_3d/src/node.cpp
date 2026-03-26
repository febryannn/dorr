#include "lidar_slam_3d_ros.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarSlam3dRos>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

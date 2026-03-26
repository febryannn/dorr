#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pointcloud_to_grid_core.hpp>

class Slam3dToGridNode : public rclcpp::Node
{
public:
    Slam3dToGridNode()
    : Node("slam3d_to_grid_node")
    {
        this->declare_parameter("cell_size", 0.02);
        this->declare_parameter("position_x", 0.5);
        this->declare_parameter("position_y", 3.0);
        this->declare_parameter("length_x", 13.0);
        this->declare_parameter("length_y", 9.0);
        this->declare_parameter("cloud_in_topic", std::string("map_cloud"));
        this->declare_parameter("intensity_factor", 1.0);
        this->declare_parameter("height_factor", 1.0);
        this->declare_parameter("mapi_topic_name", std::string("mapgrid_i"));
        this->declare_parameter("maph_topic_name", std::string("mapgrid_h"));

        double cell_size, position_x, position_y, length_x, length_y;
        double intensity_factor, height_factor;
        std::string cloud_in_topic, mapi_topic_name, maph_topic_name;

        this->get_parameter("cell_size", cell_size);
        this->get_parameter("position_x", position_x);
        this->get_parameter("position_y", position_y);
        this->get_parameter("length_x", length_x);
        this->get_parameter("length_y", length_y);
        this->get_parameter("cloud_in_topic", cloud_in_topic);
        this->get_parameter("intensity_factor", intensity_factor);
        this->get_parameter("height_factor", height_factor);
        this->get_parameter("mapi_topic_name", mapi_topic_name);
        this->get_parameter("maph_topic_name", maph_topic_name);

        grid_map_.cell_size = cell_size;
        grid_map_.position_x = position_x;
        grid_map_.position_y = position_y;
        grid_map_.length_x = length_x;
        grid_map_.length_y = length_y;
        grid_map_.cloud_in_topic = cloud_in_topic;
        grid_map_.intensity_factor = intensity_factor;
        grid_map_.height_factor = height_factor;
        grid_map_.mapi_topic_name = mapi_topic_name;
        grid_map_.maph_topic_name = maph_topic_name;

        intensity_grid_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
        height_grid_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();

        grid_map_.initGrid(intensity_grid_, this->get_logger());
        grid_map_.initGrid(height_grid_, this->get_logger());
        grid_map_.paramRefresh(this->get_logger());

        pub_igrid_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            grid_map_.mapi_topic_name, 1);
        pub_hgrid_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            grid_map_.maph_topic_name, 1);
        sub_pc2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            grid_map_.cloud_in_topic, 1,
            std::bind(&Slam3dToGridNode::pointcloudCallback, this, std::placeholders::_1));
    }

private:
    PointXY getIndex(double x, double y)
    {
        PointXY ret;
        ret.x = int(fabs(x - grid_map_.topleft_x) / grid_map_.cell_size);
        ret.y = int(fabs(y - grid_map_.topleft_y) / grid_map_.cell_size);
        return ret;
    }

    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cam_cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cam_cloud_xyz);

        if(cam_cloud_xyz->size() < 10){
            RCLCPP_INFO_STREAM(this->get_logger(), "pcd xyz points < 10 Points");
            return;
        }

        pcl::PointXYZ minPt, maxPt;
        pcl::getMinMax3D(*cam_cloud_xyz, minPt, maxPt);
        grid_map_.position_x = minPt.x + (maxPt.x - minPt.x) / 2;
        grid_map_.position_y = minPt.y + (maxPt.y - minPt.y) / 2;
        grid_map_.length_x = maxPt.x - minPt.x + 2.0;
        grid_map_.length_y = maxPt.y - minPt.y + 2.0;
        grid_map_.paramRefresh(this->get_logger());

        pcl::PointCloud<pcl::PointXYZI> out_cloud;
        pcl::fromROSMsg(*msg, out_cloud);

        // Initialize grid
        grid_map_.initGrid(intensity_grid_, this->get_logger());
        grid_map_.initGrid(height_grid_, this->get_logger());

        std::vector<signed char> hpoints(grid_map_.cell_num_x * grid_map_.cell_num_y);
        std::vector<signed char> ipoints(grid_map_.cell_num_x * grid_map_.cell_num_y);
        // initialize grid vectors
        for (auto& p : hpoints){p = 0;}
        for (auto& p : ipoints){p = 0;}

        for (auto out_point : out_cloud)
        {
            if (out_point.x > 0.01 || out_point.x < -0.01){
                if (out_point.x > grid_map_.bottomright_x && out_point.x < grid_map_.topleft_x)
                {
                    if (out_point.y > grid_map_.bottomright_y && out_point.y < grid_map_.topleft_y)
                    {
                        PointXY cell = getIndex(out_point.x, out_point.y);
                        if (cell.x < grid_map_.cell_num_x && cell.y < grid_map_.cell_num_y){
                            ipoints[(grid_map_.cell_num_x * grid_map_.cell_num_y) - (cell.y * grid_map_.cell_num_x + cell.x)] = out_point.intensity * grid_map_.intensity_factor;
                            hpoints[(grid_map_.cell_num_x * grid_map_.cell_num_y) - (cell.y * grid_map_.cell_num_x + cell.x)] = out_point.z * grid_map_.height_factor;
                        }
                        else{
                            RCLCPP_WARN_STREAM(this->get_logger(), "Cell out of range: " << cell.x << " - " << grid_map_.cell_num_x << " ||| " << cell.y << " - " << grid_map_.cell_num_y);
                        }
                    }
                }
            }
        }

        auto now = this->now();
        intensity_grid_->header.stamp = now;
        intensity_grid_->header.frame_id = msg->header.frame_id;
        intensity_grid_->info.map_load_time = now;
        intensity_grid_->data = ipoints;
        height_grid_->header.stamp = now;
        height_grid_->header.frame_id = msg->header.frame_id;
        height_grid_->info.map_load_time = now;
        height_grid_->data = hpoints;
        pub_igrid_->publish(*intensity_grid_);
        pub_hgrid_->publish(*height_grid_);
    }

    GridMap grid_map_;
    nav_msgs::msg::OccupancyGrid::SharedPtr intensity_grid_;
    nav_msgs::msg::OccupancyGrid::SharedPtr height_grid_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_igrid_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_hgrid_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc2_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Slam3dToGridNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

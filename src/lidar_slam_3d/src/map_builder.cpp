#include "map_builder.h"
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace lidar_slam_3d
{

typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>> SlamBlockSolver;
typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

MapBuilder::MapBuilder() :
    first_point_cloud_(true), sequence_num_(0),
    pose_(Eigen::Matrix4f::Identity()), last_update_pose_(Eigen::Matrix4f::Identity()),
    // submap_size_(10), voxel_grid_leaf_size_(0.12), map_update_distance_(0.01)
    submap_size_(20), voxel_grid_leaf_size_(0.12), map_update_distance_(0.01)
{
    ndt_.setTransformationEpsilon(0.0001);
    ndt_.setStepSize(0.01);
    ndt_.setResolution(0.1);
    ndt_.setMaximumIterations(50);

    ////// ini 3 baris error
    // SlamBlockSolver::LinearSolverType* linear_solver = new SlamLinearSolver;
    // SlamBlockSolver* solver_ptr = new SlamBlockSolver(linear_solver);
    // g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr); // L-M
    /////referensi dari github cara solver error 3 baris diatas
    //g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);  // line 356
    //g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> (linearSolver));
    //g2o::OptimizationAlgorithmLevenberg * solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr); // line 357
    //g2o::OptimizationAlgorithmLevenberg * solver = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<g2o::BlockSolver_6_3> (solver_ptr));
    ///// ini hasil perbaikan 3 baris error diatas
    SlamBlockSolver::LinearSolverType* linear_solver = new SlamLinearSolver;
    SlamBlockSolver* solver_ptr = new SlamBlockSolver(std::unique_ptr<SlamBlockSolver::LinearSolverType> (linear_solver));
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<SlamBlockSolver> (solver_ptr)); // L-M

    optimizer_.setAlgorithm(solver);
    optimizer_.setVerbose(false);
}

void MapBuilder::downSample(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr& sampled_cloud)
{
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(voxel_grid_leaf_size_, voxel_grid_leaf_size_, voxel_grid_leaf_size_);
    voxel_grid_filter.setInputCloud(input_cloud);
    voxel_grid_filter.filter(*sampled_cloud);
}

void MapBuilder::addPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& point_cloud, Eigen::Matrix4f odom_pose,
                               Eigen::Matrix4f odom_offset, pcl::PointCloud<pcl::PointXYZ> laser_cloud1, pcl::PointCloud<pcl::PointXYZ> laser_cloud2, pcl::PointCloud<pcl::PointXYZ> laser_cloud3)
{
    auto t1 = std::chrono::steady_clock::now();
    sequence_num_++;

    pose_ = odom_offset*odom_pose;
    if(first_point_cloud_) {
        first_point_cloud_ = false;
        pcl::transformPointCloud(*point_cloud, *point_cloud, pose_);
        map_ += *point_cloud;
        submap_.push_back(point_cloud);
        ndt_.setInputTarget(point_cloud);
        return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr sampled_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    downSample(point_cloud, sampled_cloud);

    pcl::PointCloud<pcl::PointXYZI> output_cloud;
    ndt_.setInputSource(sampled_cloud);
    ndt_.align(output_cloud, pose_);

    pose_ = ndt_.getFinalTransformation();

    Eigen::Matrix4f pose_temp;
    pose_temp = pose_;
    tf2::Matrix3x3 data_R;
    double data_roll, data_pitch, data_yaw;
    data_R.setValue(pose_temp(0, 0), pose_temp(0, 1), pose_temp(0, 2),
               pose_temp(1, 0), pose_temp(1, 1), pose_temp(1, 2),
            pose_temp(2, 0), pose_temp(2, 1), pose_temp(2, 2));
    data_R.getRPY(data_roll, data_pitch, data_yaw);
    Eigen::AngleAxisf data_rotation (data_yaw, Eigen::Vector3f::UnitZ ());
    Eigen::Translation3f data_translation (pose_temp(0,3), pose_temp(1,3), 0);
    pose_ = (data_translation * data_rotation).matrix ();

    bool converged = ndt_.hasConverged();
    double fitness_score = ndt_.getFitnessScore();
    int final_num_iteration = ndt_.getFinalNumIteration();
    (void)final_num_iteration;

    if(!converged) {
        RCLCPP_WARN(rclcpp::get_logger("map_builder"), "NDT does not converge!!!");
    }

    float delta = sqrt(square(pose_(0, 3) - last_update_pose_(0, 3)) +
                       square(pose_(1, 3) - last_update_pose_(1, 3)));
    (void)delta;

    // if(delta > map_update_distance_)
    if (true)
    {
        last_update_pose_ = pose_;

        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::transformPointCloud(*point_cloud, *transformed_cloud, pose_);
        submap_.push_back(transformed_cloud);
        while(submap_.size() > static_cast<size_t>(submap_size_)) {
            submap_.erase(submap_.begin());
        }

        std::unique_lock<std::mutex> locker(map_mutex_);
        map_ += *transformed_cloud;

        pcl::PointCloud<pcl::PointXYZI>::Ptr map_temp (new pcl::PointCloud<pcl::PointXYZI>());
        *map_temp = map_;
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setLeafSize(voxel_grid_leaf_size_, voxel_grid_leaf_size_, voxel_grid_leaf_size_);
        vg.setInputCloud(map_temp);
        vg.filter(*map_temp);
        map_ = *map_temp;

        for (size_t i = 0; i < laser_cloud1.points.size(); ++i) {
            float r, theta, midx, midy;
            r = sqrt( pow(laser_cloud1.points[i].x-pose_(0,3),2) +
                           pow(laser_cloud1.points[i].y-pose_(1,3),2) );
            theta = atan2(laser_cloud1.points[i].y - pose_(1,3), laser_cloud1.points[i].x - pose_(0,3));

            r -= 0.1;
            midx = pose_(0,3) + 0.5*r * cos(theta);
            midy = pose_(1,3) + 0.5*r * sin(theta);

            pcl::PCLPointCloud2::Ptr laser_temp1 (new pcl::PCLPointCloud2());
            pcl::toPCLPointCloud2(map_, *laser_temp1);
            pcl::CropBox<pcl::PCLPointCloud2> cb1;
            cb1.setInputCloud (laser_temp1);
            cb1.setMin(Eigen::Vector4f(-0.5*r, -0.02, 0.0, 0.0));
            cb1.setMax(Eigen::Vector4f(0.5*r, 0.02, 2.0, 0.0));
            cb1.setTranslation(Eigen::Vector3f(midx, midy, 0.0));
            cb1.setRotation(Eigen::Vector3f (0.0f,0.0f, theta));
            cb1.setNegative (true);
            cb1.filter (*laser_temp1);
            pcl::fromPCLPointCloud2(*laser_temp1, map_);
        }
        for (size_t i = 0; i < laser_cloud2.points.size(); ++i) {
            float r, theta, midx, midy;
            r = sqrt( pow(laser_cloud2.points[i].x-pose_(0,3),2) +
                           pow(laser_cloud2.points[i].y-pose_(1,3),2) );
            theta = atan2(laser_cloud2.points[i].y - pose_(1,3), laser_cloud2.points[i].x - pose_(0,3));

            r -= 0.1;
            midx = pose_(0,3) + 0.5*r * cos(theta);
            midy = pose_(1,3) + 0.5*r * sin(theta);

            pcl::PCLPointCloud2::Ptr laser_temp2 (new pcl::PCLPointCloud2());
            pcl::toPCLPointCloud2(map_, *laser_temp2);
            pcl::CropBox<pcl::PCLPointCloud2> cb2;
            cb2.setInputCloud (laser_temp2);
            cb2.setMin(Eigen::Vector4f(-0.5*r, -0.02, 0.0, 0.0));
            cb2.setMax(Eigen::Vector4f(0.5*r, 0.02, 2.0, 0.0));
            cb2.setTranslation(Eigen::Vector3f(midx, midy, 0.0));
            cb2.setRotation(Eigen::Vector3f (0.0f,0.0f, theta));
            cb2.setNegative (true);
            cb2.filter (*laser_temp2);
            pcl::fromPCLPointCloud2(*laser_temp2, map_);
        }
        for (size_t i = 0; i < laser_cloud3.points.size(); ++i) {
            float r, theta, midx, midy;
            r = sqrt( pow(laser_cloud3.points[i].x-pose_(0,3),2) +
                           pow(laser_cloud3.points[i].y-pose_(1,3),2) );
            theta = atan2(laser_cloud3.points[i].y - pose_(1,3), laser_cloud3.points[i].x - pose_(0,3));

            r -= 0.1;
            midx = pose_(0,3) + 0.5*r * cos(theta);
            midy = pose_(1,3) + 0.5*r * sin(theta);

            pcl::PCLPointCloud2::Ptr laser_temp3 (new pcl::PCLPointCloud2());
            pcl::toPCLPointCloud2(map_, *laser_temp3);
            pcl::CropBox<pcl::PCLPointCloud2> cb3;
            cb3.setInputCloud (laser_temp3);
            cb3.setMin(Eigen::Vector4f(-0.5*r, -0.02, 0.0, 0.0));
            cb3.setMax(Eigen::Vector4f(0.5*r, 0.02, 2.0, 0.0));
            cb3.setTranslation(Eigen::Vector3f(midx, midy, 0.0));
            cb3.setRotation(Eigen::Vector3f (0.0f,0.0f, theta));
            cb3.setNegative (true);
            cb3.filter (*laser_temp3);
            pcl::fromPCLPointCloud2(*laser_temp3, map_);
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        for(size_t i = 0; i < submap_.size(); ++i) {
            *target_cloud += *submap_[i];
        }

        ndt_.setInputTarget(target_cloud);
    }

    auto t2 = std::chrono::steady_clock::now();
    auto delta_t = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

    std::cout << "-------------------------------------" << std::endl;
    std::cout << "Sequence number: " << sequence_num_ << std::endl;
    std::cout << "Map size: " << map_.size() << " points." << std::endl;
    std::cout << "Fitness score: " << fitness_score << std::endl;
    std::cout << "Cost time: " << delta_t.count() * 1000.0 << "ms." << std::endl;
    std::cout << "-------------------------------------" << std::endl;
}

void MapBuilder::getPoseGraph(std::vector<Eigen::Vector3d>& nodes,
                              std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& edges)
{
    for(g2o::SparseOptimizer::VertexIDMap::iterator it = optimizer_.vertices().begin(); it != optimizer_.vertices().end(); ++it) {
        g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(it->second);
        Eigen::Vector3d pt = v->estimate().translation();
        nodes.push_back(pt);
    }

   for(g2o::SparseOptimizer::EdgeSet::iterator it = optimizer_.edges().begin(); it != optimizer_.edges().end(); ++it) {
       g2o::EdgeSE3* e = dynamic_cast<g2o::EdgeSE3*>(*it);
       g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(e->vertices()[0]);
       g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(e->vertices()[1]);
       Eigen::Vector3d pt1 = v1->estimate().translation();
       Eigen::Vector3d pt2 = v2->estimate().translation();
       edges.push_back(std::make_pair(pt1, pt2));
   }
}

} // namespace lidar_slam_3d

#ifndef GLOBAL_PLANNING_HPP
#define GLOBAL_PLANNING_HPP

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <random>

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include "jps_basis/data_utils.h"
#include "jps_planner/jps_planner.h"
#include "visual/visualizer.hpp"
#include "VoxelGraph/eigen_conversions.h"
#include "VoxelGraph/VoxelGraph.hpp"

using namespace JPS;


class GlobalPlanner {
    private:
        ros::NodeHandle nh;
        ros::Subscriber mapSub;
        ros::Subscriber startGoalSub;

        Visualizer visualizer;
        std::vector<Eigen::Vector3d> startGoal;
        
        /****** Create JPS Map ******/
        std::shared_ptr<VoxelMapUtil> map_util = std::make_shared<VoxelMapUtil>();

        /****** Declare a planner pointer ******/
        std::unique_ptr<JPSPlanner3D> planner_ptr = std::make_unique<JPSPlanner3D>(true);

        // Map dimensions
        std::vector<signed char> grid_;
        Eigen::Vector3i grid_size_;
        const Vec3f origin_ = Vec3f(-25.0f, -25.0f, 0.0f);
        const Vec3f max_bounds_ = Vec3f(25.0f, 25.0f, 5.0f);
        double resolution_ = 0.5;
        std::vector<Eigen::Vector3d> PointCloud_;
        vec_Vecf<3> path_;



    public:
        GlobalPlanner(ros::NodeHandle &nh_) : nh(nh_), visualizer(nh) {
            mapSub = nh.subscribe("/voxel_map", 1, &GlobalPlanner::mapCallBack, this, ros::TransportHints().tcpNoDelay());
            startGoalSub = nh.subscribe("/move_base_simple/goal", 1, &GlobalPlanner::startGoalCallback, this, ros::TransportHints().tcpNoDelay());
        }

        inline void mapCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg) {

            if (startGoal.size() == 2) {
                if(PointCloud_.empty()) {
                    Eigen::Matrix<float, 3, -1> cloud1;
                    EigenPointCloudConversions::PointCloud2ToEigen<float>(*msg, cloud1);
                    
                    const float* msg_float_ptr = reinterpret_cast<const float*>(msg->data.data());
                    auto cloud2 = Eigen::Map<const Eigen::Matrix<float, 4, -1>>(msg_float_ptr, 4, msg->width * msg->height);
                    
                    EigenPointCloudConversions::PointCloud2ToVector<double>(*msg, PointCloud_);
                    

                    grid_size_ = VoxelGraph::initializeGrid<double>(origin_, max_bounds_, resolution_, grid_);

                    VoxelGraph::setOccupancy<double>(PointCloud_, origin_, resolution_, grid_, grid_size_);
                    map_util->setMap(origin_, grid_size_, grid_, resolution_);

                    planner_ptr->setMapUtil(map_util);
                    planner_ptr->updateMap();
                    planner_ptr->plan(startGoal[0], startGoal[1], 0.5, true); // Plan a path

                    path_ = planner_ptr->getRawPath();

                }

                std::vector<Eigen::Vector3d> route;
                for(auto it : path_) {
                    route.push_back(it.cast<double>());
                }
                visualizer.visualize_path(route);

                // Plan the path and safe corridors
                visualizer.visualizeStartGoal(startGoal[0], 0.5, startGoal.size());
                visualizer.visualizeSphere(startGoal[1], 0.5);
            }
        }

        inline void startGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {            
            if (startGoal.size() >= 2) {
                startGoal.clear();
            }
            startGoal.push_back(Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, 0.5));
        }        

};

#endif
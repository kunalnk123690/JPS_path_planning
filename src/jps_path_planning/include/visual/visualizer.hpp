#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include <iostream>
#include <memory>
#include <chrono>
#include <cmath>
#include <vector>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Visualizer for the planner
class Visualizer {
    private:
        ros::NodeHandle nh;

        // These are publishers for path, waypoints on the trajectory,
        // the entire trajectory, the mesh of free-space polytopes,
        // the edge of free-space polytopes, and spheres for safety radius
        ros::Publisher routePub;
        ros::Publisher wayPointsPub;
        ros::Publisher spherePub;


    public:
        Visualizer(ros::NodeHandle &nh_) : nh(nh_) {
            routePub = nh.advertise<visualization_msgs::Marker>("/visualizer/route", 10);
            wayPointsPub = nh.advertise<visualization_msgs::Marker>("/visualizer/waypoints", 10);        
            spherePub = nh.advertise<visualization_msgs::Marker>("/visualizer/spheres", 1000);
        }


        // Visualize the front-end path
        inline void visualize_path(const std::vector<Eigen::Vector3d> &route) {
            visualization_msgs::Marker routeMarker, wayPointsMarker;

            routeMarker.id = 0;
            routeMarker.type = visualization_msgs::Marker::LINE_LIST;
            routeMarker.header.stamp = ros::Time::now();
            routeMarker.header.frame_id = "odom";
            routeMarker.pose.orientation.w = 1.00;
            routeMarker.action = visualization_msgs::Marker::ADD;
            routeMarker.ns = "route";
            routeMarker.color.r = 1.00;
            routeMarker.color.g = 0.00;
            routeMarker.color.b = 0.00;
            routeMarker.color.a = 1.00;
            routeMarker.scale.x = 0.2;

            wayPointsMarker = routeMarker;
            wayPointsMarker.id = -wayPointsMarker.id - 1;
            wayPointsMarker.type = visualization_msgs::Marker::SPHERE_LIST;
            wayPointsMarker.ns = "waypoints";
            wayPointsMarker.color.r = 0.00;
            wayPointsMarker.color.g = 0.00;
            wayPointsMarker.color.b = 1.00;
            wayPointsMarker.scale.x = 0.35;
            wayPointsMarker.scale.y = 0.35;
            wayPointsMarker.scale.z = 0.35;


            if (route.size() > 0) {
                bool first = true;
                Eigen::Vector3d last;
                for (auto it : route) {
                    if (first) {
                        first = false;
                        last = it;
                        continue;
                    }
                    geometry_msgs::Point point;

                    point.x = last(0);
                    point.y = last(1);
                    point.z = last(2);
                    routeMarker.points.push_back(point);
                    point.x = it(0);
                    point.y = it(1);
                    point.z = it(2);
                    routeMarker.points.push_back(point);
                    last = it;
                }

                routePub.publish(routeMarker);
            }

            for (int i = 1; i < route.size()-1; i++) {
                geometry_msgs::Point point;
                point.x = route[i](0);
                point.y = route[i](1);
                point.z = route[i](2);
                wayPointsMarker.points.push_back(point);
            }
            wayPointsPub.publish(wayPointsMarker);        

        }



        // Visualize all spheres with centers sphs and the same radius
        inline void visualizeSphere(const Eigen::Vector3d &center, const double &radius) {
            visualization_msgs::Marker sphereMarkers, sphereDeleter;

            sphereMarkers.id = 0;
            sphereMarkers.type = visualization_msgs::Marker::SPHERE_LIST;
            sphereMarkers.header.stamp = ros::Time::now();
            sphereMarkers.header.frame_id = "odom";
            sphereMarkers.pose.orientation.w = 1.00;
            sphereMarkers.action = visualization_msgs::Marker::ADD;
            sphereMarkers.ns = "spheres";
            sphereMarkers.color.r = 0.00;
            sphereMarkers.color.g = 1.00;
            sphereMarkers.color.b = 0.00;
            sphereMarkers.color.a = 1.00;
            sphereMarkers.scale.x = radius * 2.0;
            sphereMarkers.scale.y = radius * 2.0;
            sphereMarkers.scale.z = radius * 2.0;

            sphereDeleter = sphereMarkers;
            sphereDeleter.action = visualization_msgs::Marker::DELETE;

            geometry_msgs::Point point;
            point.x = center(0);
            point.y = center(1);
            point.z = center(2);
            sphereMarkers.points.push_back(point);

            spherePub.publish(sphereDeleter);
            spherePub.publish(sphereMarkers);
        }

        inline void visualizeStartGoal(const Eigen::Vector3d &center, const double &radius, const int sg) {
            visualization_msgs::Marker sphereMarkers, sphereDeleter;

            sphereMarkers.id = sg;
            sphereMarkers.type = visualization_msgs::Marker::SPHERE_LIST;
            sphereMarkers.header.stamp = ros::Time::now();
            sphereMarkers.header.frame_id = "odom";
            sphereMarkers.pose.orientation.w = 1.00;
            sphereMarkers.action = visualization_msgs::Marker::ADD;
            sphereMarkers.ns = "StartGoal";
            sphereMarkers.color.r = 0.00;
            sphereMarkers.color.g = 0.00;
            sphereMarkers.color.b = 1.00;
            sphereMarkers.color.a = 1.00;
            sphereMarkers.scale.x = radius * 2.0;
            sphereMarkers.scale.y = radius * 2.0;
            sphereMarkers.scale.z = radius * 2.0;

            sphereDeleter = sphereMarkers;
            sphereDeleter.action = visualization_msgs::Marker::DELETEALL;

            geometry_msgs::Point point;
            point.x = center(0);
            point.y = center(1);
            point.z = center(2);
            sphereMarkers.points.push_back(point);

            if (sg == 0) {
                spherePub.publish(sphereDeleter);
                ros::Duration(1.0e-9).sleep();
                sphereMarkers.header.stamp = ros::Time::now();
            }
            spherePub.publish(sphereMarkers);
        }
};

#endif
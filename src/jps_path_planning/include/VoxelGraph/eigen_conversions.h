#ifndef EIGEN_CONVERSIONS_H
#define EIGEN_CONVERSIONS_H

#include <iostream>
#include <Eigen/Dense>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

namespace EigenPointCloudConversions {

template <typename T>
inline void EigenToPointCloud(const Eigen::Matrix<T, 3, -1>& inputCloud, sensor_msgs::PointCloud& cloud) {
    // Resize msg.points to match the number of columns in the Eigen matrix
    cloud.points.resize(inputCloud.cols());

    // Create a map to write directly into msg.points, converting Scalar to float
    Eigen::Map<Eigen::Matrix<float, 3, -1>> map(reinterpret_cast<float*>(cloud.points.data()), 3, inputCloud.cols());
    
    // Assign the Eigen matrix to mapped memory, casting to float if necessary
    map = inputCloud.template cast<float>();
}    


template <typename T>
inline void PointCloudToEigen(const sensor_msgs::PointCloud& cloud, Eigen::Matrix<T, 3, -1>& outputCloud) {
    
    // Create a map from the cloud's data
    const float* pointsPtr = reinterpret_cast<const float*>(cloud.points.data());
    Eigen::Map<const Eigen::Matrix<float, 3, -1>> mat(pointsPtr, 3, cloud.points.size());

    // Cast the data to the desired Scalar type if necessary
    if constexpr (std::is_same_v<T, float>) {
        outputCloud = mat;
    }
    else {
        outputCloud = mat.template cast<T>();
    }
}



template <typename T>
inline void PointCloudToVector(const sensor_msgs::PointCloud& cloud, std::vector<Eigen::Matrix<T, 3, 1>>& outputCloud) {

    // Map the raw data into an Eigen::Matrix (sensor_msgs::PointCloud uses float internally)
    Eigen::Map<const Eigen::Matrix<float, 3, -1>> points_map(reinterpret_cast<const float*>(cloud.points.data()), 3, cloud.points.size());

    // Resize the output vector to hold all points
    outputCloud.resize(points_map.cols());

    if constexpr (std::is_same_v<T, float>) {
        Eigen::Map<Eigen::Matrix<float, 3, -1>> mapped_result(outputCloud[0].data(), 3, points_map.cols());
        mapped_result = points_map;
    }
    else {
        Eigen::Map<Eigen::Matrix<T, 3, -1>> mapped_result(outputCloud[0].data(), 3, points_map.cols());
        mapped_result = points_map.template cast<T>();
    }
}


template <typename T>
inline void VectorToPointCloud(const std::vector<Eigen::Matrix<T, 3, 1>>& inputCloud, sensor_msgs::PointCloud& cloud) {

    // Resize the sensor_msgs::PointCloud to match the input vector
    cloud.points.resize(inputCloud.size());

    // Map the vector memory into an Eigen::Matrix
    Eigen::Map<const Eigen::Matrix<T, 3, -1>> points_map(reinterpret_cast<const T*>(inputCloud.data()), 3, inputCloud.size());

    // Map the sensor_msgs::PointCloud memory
    Eigen::Map<Eigen::Matrix<float, 3, -1>> cloud_map(reinterpret_cast<float*>(cloud.points.data()), 3, inputCloud.size());

    // Copy data from the input vector to the sensor_msgs::PointCloud with type casting
    cloud_map = points_map.template cast<float>();
}


template <typename T>
inline void PointCloud2ToEigen(const sensor_msgs::PointCloud2& cloud_msg, Eigen::Matrix<T, 3, -1>& result) {

    // Interpret the PointCloud2 data as an Eigen 4xN matrix of floats
    const float* pointsPtr = reinterpret_cast<const float*>(cloud_msg.data.data());
    auto cloud2 = Eigen::Map<const Eigen::Matrix<float, 4, -1>>(pointsPtr, 4, cloud_msg.width * cloud_msg.height);

    // Extract the first 3 rows (X, Y, Z) and cast them to type T
    if constexpr (std::is_same_v<T, float>) {
        result = Eigen::Map<const Eigen::Matrix<float, 3, -1>>(cloud2.data(), 3, cloud2.cols());
    }
    else {
        result = cloud2.topRows(3).template cast<T>();
    }
}



template <typename T>
inline void PointCloud2ToVector(const sensor_msgs::PointCloud2& cloud_msg, std::vector<Eigen::Matrix<T, 3, 1>>& result) {

    // Interpret the PointCloud2 data as an Eigen 4xN matrix of floats
    const float* pointsPtr = reinterpret_cast<const float*>(cloud_msg.data.data());
    auto cloud2 = Eigen::Map<const Eigen::Matrix<float, 4, -1>>(pointsPtr, 4, cloud_msg.width * cloud_msg.height);

    // Resize the output vector to hold all points
    result.resize(cloud2.cols());

    if constexpr (std::is_same_v<T, float>) {
        Eigen::Map<Eigen::Matrix<float, 3, -1>> mapped_result(result[0].data(), 3, cloud2.cols());
        mapped_result = cloud2.topRows(3);
    }
    else {
        Eigen::Map<Eigen::Matrix<T, 3, -1>> mapped_result(result[0].data(), 3, cloud2.cols());
        mapped_result = cloud2.topRows(3).template cast<T>();
    }
}


} // namespace EigenCloudConversions

#endif // EIGEN_CONVERSIONS_H
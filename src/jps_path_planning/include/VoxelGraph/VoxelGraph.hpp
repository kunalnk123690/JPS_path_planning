#ifndef VOXELMAP_HPP
#define VOXELMAP_HPP

#include <iostream>
#include <Eigen/Dense>
#include <vector>


namespace VoxelGraph {

    inline int getLinearIndex(const Eigen::Matrix<int, 3, 1>& index, 
                                 const Eigen::Matrix<int, 3, 1>& grid_size) {
        return index(0) + index(1) * grid_size(0) + index(2) * grid_size(0) * grid_size(1);
    }


    template<typename T>
    inline Eigen::Matrix<int, 3, 1> initializeGrid(const Eigen::Matrix<T, 3, 1>& min_bounds, 
                                                      const Eigen::Matrix<T, 3, 1>& max_bounds,
                                                      T resolution, std::vector<signed char>& grid) {
        // Ensure valid bounds
        if ((max_bounds.array() <= min_bounds.array()).any()) {
            throw std::invalid_argument("max_bounds must be greater than min_bounds in all dimensions.");
        }

        // Calculate grid size (Use ceil to ensure full coverage)
        Eigen::Matrix<int, 3, 1> grid_size = ((max_bounds - min_bounds) / resolution).array().ceil().template cast<int>();

        // Prevent zero-sized grid
        if ((grid_size.array() == 0).any()) {
            throw std::invalid_argument("Grid size must be non-zero in all dimensions.");
        }

        // Initialize the 1D grid vector
        int total_size = grid_size(0) * grid_size(1) * grid_size(2);
        grid.assign(total_size, 0); // Default value set to 0

        return grid_size;
    }


    template<typename T>
    inline Eigen::Matrix<int, 3, 1> computePointIndex(const Eigen::Matrix<T, 3, 1>& point, 
                                                         const Eigen::Matrix<T, 3, 1>& min_bounds, 
                                                         const T resolution) {
        // Compute the 3D grid index
        return ((point - min_bounds) / resolution).array().floor().template cast<int>();
    }



    template<typename T>
    inline void setOccupancy(const std::vector<Eigen::Matrix<T, 3, 1>>& pointCloud, 
                             const Eigen::Matrix<T, 3, 1>& min_bounds, 
                             const float& resolution, 
                             std::vector<signed char>& grid, 
                             const Eigen::Matrix<int, 3, 1>& grid_size) {
        
        Eigen::Matrix<int, 3, 1> idx;
        
        for (const auto& point : pointCloud) {
            // Compute the 3D grid index
            idx = computePointIndex<T>(point, min_bounds, resolution);

            // Check bounds
            if (idx(0) < grid_size(0) && idx(1) < grid_size(1) && idx(2) < grid_size(2)) {
                // Compute the linear index
                int linear_idx = getLinearIndex(idx, grid_size);

                // Set a high cost for obstacles (e.g., 100)
                grid[linear_idx] = 100;
            }
        }
    }


    template<typename T>
    inline bool checkOccupancy(const Eigen::Matrix<T, 3, 1>& point,
                               const std::vector<signed char>& grid, 
                               const Eigen::Matrix<T, 3, 1>& min_bounds, 
                               const T resolution, 
                               const Eigen::Matrix<int, 3, 1>& grid_size) {
        // Compute the 3D grid index
        Eigen::Matrix<int, 3, 1> idx = computePointIndex<T>(point, min_bounds, resolution);

        // Check bounds
        if (idx(0) < grid_size(0) && idx(1) < grid_size(1) && idx(2) < grid_size(2)) {
            // Compute the linear index
            int linear_idx = getLinearIndex(idx, grid_size);

            // Return true if the voxel is occupied
            return grid[linear_idx] > 0;
        }

        // Out of bounds implies not occupied
        return false;
    }

}


#endif
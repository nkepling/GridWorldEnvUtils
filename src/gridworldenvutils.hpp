#pragma once

#include <vector>
#include <utility> // For std::pair
#include <Eigen/Dense>
#include <random> 
#include <optional> // Required for std::optional
#include <map>      // Fixed: Added for std::map
#include <vector>   // Fixed: Added for std::vector
#include <omp.h>    // Fixed: Added for OpenMP
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>        // For Eigen types
#include <pybind11/stl.h>          // For STL containers like std::vector
#include <pybind11/numpy.h>        // For NumPy arrays
#include "gridworldenvutils.hpp"

namespace py = pybind11;

namespace gridworld {

    bool checkFOV(const Eigen::Vector2i& agent_pos, const Eigen::Vector2i& target_pos, const Eigen::Vector2i& agent_dir, float fov_angle, float fov_distance, const std::vector<std::vector<bool>>& obstacles);

    std::vector<Eigen::Vector2i> getVisibleCells(const Eigen::Vector2i& agent_pos, const Eigen::Vector2i& agent_dir, float fov_angle, float fov_distance, const std::vector<std::vector<bool>>& obstacles);

    bool checkLineOfSight(const Eigen::Vector2i& agent_pos, const Eigen::Vector2i& target_pos, const std::vector<std::vector<bool>>& obstacles);

    bool is_in_bounds(const int& x_coord, const int& y_coord, const std::vector<std::vector<bool>>& obstacles);

    std::vector<Eigen::Vector2i> findShortestPath(const Eigen::Vector2i& start, const Eigen::Vector2i& goal, const std::vector<std::vector<bool>>& obstacles,  bool allow_diagonal);

     // --- NEW: Backward Induction Declaration ---
    std::vector<int8_t> runBackwardInduction(
        const std::vector<std::vector<bool>>& obstacles,
        const std::vector<Eigen::Vector2i>& evader_path,
        float gamma,
        float stochasticity,
        float fov_angle,
        float fov_distance
    );

    std::vector<float> runBackwardInductionQ(
    const std::vector<std::vector<bool>>& obstacles,
    const std::vector<Eigen::Vector2i>& evader_path,
    float gamma,
    float stochasticity,
    float fov_angle,
    float fov_distance,
    int start_r,
    int start_c,
    int start_d,
    int fov_lookahead
);

// --- NEW BATCH SIGNATURE ---
    std::vector<float> runBatchBackwardInduction(
        const std::vector<std::vector<bool>>& obstacles,
        const std::vector<std::vector<Eigen::Vector2i>>& evader_paths_batch,
        const std::vector<float>& weights,
        float gamma,
        float stochasticity,
        float fov_angle,
        float fov_distance,
        int start_r, int start_c, int start_d,
        int fov_lookahead
    );

} // namespace gridworld
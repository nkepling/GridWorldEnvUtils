#pragma once

#include <vector>
#include <utility>
#include <Eigen/Dense>
#include <random> 
#include <optional> 
#include <map>     
#include <omp.h>   

// Note: Pybind includes are technically not needed in the header 
// if you only use standard types here, but keeping them to match your setup.
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

namespace gridworld {

    bool checkFOV(const Eigen::Vector2i& agent_pos, const Eigen::Vector2i& target_pos, const Eigen::Vector2i& agent_dir, float fov_angle, float fov_distance, const std::vector<std::vector<bool>>& obstacles);

    std::vector<Eigen::Vector2i> getVisibleCells(const Eigen::Vector2i& agent_pos, const Eigen::Vector2i& agent_dir, float fov_angle, float fov_distance, const std::vector<std::vector<bool>>& obstacles);

    bool checkLineOfSight(const Eigen::Vector2i& agent_pos, const Eigen::Vector2i& target_pos, const std::vector<std::vector<bool>>& obstacles);

    bool is_in_bounds(const int& x_coord, const int& y_coord, const std::vector<std::vector<bool>>& obstacles);

    std::vector<Eigen::Vector2i> findShortestPath(const Eigen::Vector2i& start, const Eigen::Vector2i& goal, const std::vector<std::vector<bool>>& obstacles,  bool allow_diagonal);

    std::vector<int8_t> runBackwardInduction(
        const std::vector<std::vector<bool>>& obstacles,
        const std::vector<Eigen::Vector2i>& evader_path,
        float gamma,
        float stochasticity,
        float fov_angle,
        float fov_distance,
        int fov_lookahead
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

    float getCenteringBonus(const Eigen::Vector2i& p_pos, const Eigen::Vector2i& e_pos, const Eigen::Vector2i& p_dir, float fov_angle, float fov_dist);

    std::vector<int8_t> runBackwardInductionWithBias(
        const std::vector<std::vector<bool>>& obstacles,
        const std::vector<Eigen::Vector2i>& evader_path,
        float gamma,
        float stochasticity,
        float fov_angle,
        float fov_distance
    );


    std::vector<float> runBackwardInductionTube(
        const std::vector<std::vector<bool>>& obstacles,
        const std::vector<float>& reachability_tube, // Changed to float
        float gamma,
        float stochasticity,
        float fov_angle,
        float fov_distance,
        int start_r, int start_c, int start_d,
        int fov_lookahead
    );

} // namespace gridworld
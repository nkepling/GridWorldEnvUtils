#pragma once

#include <vector>
#include <utility> // For std::pair
#include <Eigen/Dense>
#include <random> 
#include <optional> // Required for std::optional



namespace gridworld {

    bool checkFOV(const Eigen::Vector2i& agent_pos, const Eigen::Vector2i& target_pos, const Eigen::Vector2i& agent_dir, float fov_angle, float fov_distance, const std::vector<std::vector<bool>>& obstacles);

    std::vector<Eigen::Vector2i> getVisibleCells(const Eigen::Vector2i& agent_pos, const Eigen::Vector2i& agent_dir, float fov_angle, float fov_distance, const std::vector<std::vector<bool>>& obstacles);

    bool checkLineOfSight(const Eigen::Vector2i& agent_pos, const Eigen::Vector2i& target_pos, const std::vector<std::vector<bool>>& obstacles);

    bool is_in_bounds(const int& x_coord, const int& y_coord, const std::vector<std::vector<bool>>& obstacles);

    std::vector<Eigen::Vector2i> findShortestPath(const Eigen::Vector2i& start, const Eigen::Vector2i& goal, const std::vector<std::vector<bool>>& obstacles,  bool allow_diagonal);

} // namespace gridworld
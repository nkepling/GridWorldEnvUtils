#ifndef ASTAR_HPP
#define ASTAR_HPP

#include <vector>
#include <Eigen/Core>

namespace AStar {

    // Use Eigen::Vector2i to stay consistent with your CityEnv code.
    using Vec2i = Eigen::Vector2i;

    /**
     * @brief Finds the shortest path between two points using the A* algorithm.
     * @param start The starting grid coordinate.
     * @param goal The target grid coordinate.
     * @param obstacle_map A 2D boolean grid where 'true' represents an obstacle.
     * @param allow_diagonal Toggles 8-directional movement.
     * @return A vector of coordinates representing the path from start to goal.
     * Returns an empty vector if no path is found.
     */
    std::vector<Vec2i> findPath(
        const Vec2i& start,
        const Vec2i& goal,
        const std::vector<std::vector<bool>>& obstacle_map,
        bool allow_diagonal = true
    );
}

#endif // ASTAR_HPP
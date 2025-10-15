#include "gridworldenvutils.hpp"
#include "AStar.hpp"

namespace gridworld {

    bool checkFOV(const Eigen::Vector2i& agent_pos, const Eigen::Vector2i& target_pos, const Eigen::Vector2i& agent_dir, float fov_angle, float fov_distance, const std::vector<std::vector<bool>>& obstacles) {

        Eigen::Vector2i vector_to_target = target_pos - agent_pos;

        if (vector_to_target.squaredNorm() > fov_distance * fov_distance) {
            return false;
        }

        Eigen::Vector2i agent_dir_normalized = agent_dir.normalized();
        float angle_to_target = std::atan2(vector_to_target.y(), vector_to_target.x());
        float agent_facing_angle = std::atan2(agent_dir_normalized.y(), agent_dir_normalized.x());

        float angle_diff = std::abs(angle_to_target - agent_facing_angle);

        if (angle_diff > fov_angle / 2) {
            return false;
        }

        if (!checkLineOfSight(agent_pos, target_pos, obstacles)) {
            return false;
        }

        return true;
    }

    bool checkLineOfSight(const Eigen::Vector2i& agent_pos, const Eigen::Vector2i& target_pos, const std::vector<std::vector<bool>>& obstacles) {

        int x0 = agent_pos.x(), y0 = agent_pos.y();
        int x1 = target_pos.x(), y1 = target_pos.y();

        int dx = std::abs(x1 - x0);
        int dy = -std::abs(y1 - y0);

        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        
        int err = dx + dy;

        while (true) {
            // Check if the current cell is an obstacle
            if (is_in_bounds(x0, y0, obstacles) && obstacles[y0][x0]) {
                return false; // Blocked by an obstacle
            }
            if (x0 == x1 && y0 == y1) {
                break; // Reached the end
            }
            
            int e2 = 2 * err;
            if (e2 >= dy) {
                err += dy;
                x0 += sx;
            }
            if (e2 <= dx) {
                err += dx;
                y0 += sy;
            }
        }
        return true; // Line of sight is clear
    }

    std::vector<Eigen::Vector2i> findShortestPath(
        const Eigen::Vector2i& start,
        const Eigen::Vector2i& goal,
        const std::vector<std::vector<bool>>& obstacle_map,
        bool allow_diagonal = true
    ) {
        auto result = AStar::findPath(start, goal, obstacle_map, allow_diagonal);
        return result;
    }

    bool is_in_bounds(const int& x_coord, const int& y_coord, const std::vector<std::vector<bool>>& obstacles) {
        int rows = obstacles.size();
        if (rows == 0) return false;
        int cols = obstacles[0].size();
        return x_coord >= 0 && x_coord < cols && y_coord >= 0 && y_coord < rows;
    }

}
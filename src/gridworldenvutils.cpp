#include "gridworldenvutils.hpp"
#include "AStar.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <map>
#include <vector>
#include <omp.h>
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

namespace py = pybind11;
namespace gridworld {

    // --- Helper Structs ---
    struct Transition {
        float prob;
        int r, c, d;
    };

    std::vector<Transition> getTransitions(int r, int c, int d, int action, const std::vector<std::vector<bool>>& obstacles, float stochasticity) {
        int rows = obstacles.size();
        int cols = obstacles[0].size();
        std::vector<Transition> transitions;

        // Outcomes map: action_code -> probability
        std::map<int, float> outcomes; 
        
        // 1. Success case
        if (stochasticity > 0) {
            outcomes[action] += stochasticity;
        }

        // 2. Failure/Slip cases
        float p_fail = 1.0f - stochasticity;
        if (p_fail > 0) {
            std::vector<int> slips;
            if (action == 0) slips = {0};
            else if (action == 1) slips = {5, 6}; // UP -> UL, UR
            else if (action == 2) slips = {7, 8}; // DOWN -> DL, DR
            else if (action == 3) slips = {5, 7}; // LEFT -> UL, DL
            else if (action == 4) slips = {6, 8}; // RIGHT -> UR, DR
            else if (action == 5) slips = {1, 3}; // UL -> UP, LEFT
            else if (action == 6) slips = {1, 4}; // UR -> UP, RIGHT
            else if (action == 7) slips = {2, 3}; // DL -> DOWN, LEFT
            else if (action == 8) slips = {2, 4}; // DR -> DOWN, RIGHT
            else if (action == 9 || action == 10) slips = {0, 11}; // ROT -> STAY, OVERSTEER
            
            float prob_per_slip = p_fail / slips.size();
            for (int s : slips) outcomes[s] += prob_per_slip;
        }

        for (auto const& [act, prob] : outcomes) {
            if (prob <= 0.0f) continue;

            int nr = r, nc = c, nd = d;
            int intended = action;

            // Physics Logic
            if (act == 1) { nr = std::max(0, r - 1); nd = 6; } // UP (N)
            else if (act == 2) { nr = std::min(rows - 1, r + 1); nd = 2; } // DOWN (S)
            else if (act == 3) { nc = std::max(0, c - 1); nd = 4; } // LEFT (W)
            else if (act == 4) { nc = std::min(cols - 1, c + 1); nd = 0; } // RIGHT (E)
            else if (act == 5) { nr = std::max(0, r - 1); nc = std::max(0, c - 1); nd = 5; } // UL (NW)
            else if (act == 6) { nr = std::max(0, r - 1); nc = std::min(cols - 1, c + 1); nd = 7; } // UR (NE)
            else if (act == 7) { nr = std::min(rows - 1, r + 1); nc = std::max(0, c - 1); nd = 3; } // DL (SW)
            else if (act == 8) { nr = std::min(rows - 1, r + 1); nc = std::min(cols - 1, c + 1); nd = 1; } // DR (SE)
            else if (act == 9) { nd = (d + 1) % 8; } // CW
            else if (act == 10) { nd = (d + 8 - 1) % 8; } // CCW
            else if (act == 11) { // OVERSTEER
                if (intended == 9) nd = (d + 2) % 8;
                else if (intended == 10) nd = (d + 8 - 2) % 8;
            }

            // Wall collision check
            if (is_in_bounds(nc, nr, obstacles) && obstacles[nr][nc]) {
                nr = r; nc = c; // Bounce back
            }

            transitions.push_back({prob, nr, nc, nd});
        }
        return transitions;
    }


    // --- Helper: Check bounds (Raw Pointer) ---
    inline bool is_blocked_flat(const uint8_t* flat_map, int x, int y, int width, int height) {
        if (x < 0 || x >= width || y < 0 || y >= height) return true;
        return flat_map[y * width + x] != 0;
    }

    std::vector<uint8_t> flattenMap(const std::vector<std::vector<bool>>& grid, int& out_rows, int& out_cols) {
        out_rows = grid.size();
        out_cols = (out_rows > 0) ? grid[0].size() : 0;
        std::vector<uint8_t> flat_grid(out_rows * out_cols);
        for (int r = 0; r < out_rows; ++r) {
            for (int c = 0; c < out_cols; ++c) {
                flat_grid[r * out_cols + c] = grid[r][c] ? 1 : 0;
            }
        }
        return flat_grid;
    }

    // --- Helper: Optimized FOV ---
   bool checkFOV_Optimized(
        const Eigen::Vector2i& agent_pos, 
        const Eigen::Vector2i& target_pos, 
        const Eigen::Vector2i& agent_dir, 
        float min_cos_angle, 
        float fov_dist_sq,   
        const uint8_t* flat_obstacles, 
        int map_width,
        int map_height
    ) {
        Eigen::Vector2i to_target = target_pos - agent_pos;
        float dist_sq = (float)to_target.squaredNorm();
        
        // Check distance
        if (dist_sq > fov_dist_sq || dist_sq < 0.1f) return false;

        // Check Angle (Dot Product Optimization)
        float dot = (float)agent_dir.dot(to_target);
        if (dot < 0) return false; // Behind

        float agent_dir_mag_sq = (float)agent_dir.squaredNorm();
        float min_cos_sq = min_cos_angle * min_cos_angle;

        if ((dot * dot) < (min_cos_sq * agent_dir_mag_sq * dist_sq)) return false;

        // Line of Sight (Bresenham)
        int x0 = agent_pos.x(); int y0 = agent_pos.y();
        int x1 = target_pos.x(); int y1 = target_pos.y();
        
        // Save start pos to avoid self-blocking
        int start_x = x0; 
        int start_y = y0;

        int dx = std::abs(x1 - x0); int dy = -std::abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1; int sy = (y0 < y1) ? 1 : -1;
        int err = dx + dy;

        while (true) {
            // 1. Safety Bounds Check
            if (x0 < 0 || x0 >= map_width || y0 < 0 || y0 >= map_height) {
                return false; 
            }

            // 2. Obstacle Check
            if (flat_obstacles[y0 * map_width + x0]) {
                // If we hit a wall...
                
                // A. Is it the target? (We can see the "skin" of a wall)
                if (x0 == x1 && y0 == y1) break; 
                
                // B. Is it the agent itself? (Don't let the agent's own pixel block the view)
                if (x0 != start_x || y0 != start_y) {
                    return false; 
                }
            }

            // 3. Reached Target
            if (x0 == x1 && y0 == y1) break;
            
            // 4. Step
            int e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
        return true;
    }
    // --- Helper: Get Transitions (Raw Pointer) ---
    std::vector<Transition> getTransitionsFlat(
        int r, int c, int d, int a, 
        int rows, int cols, 
        const uint8_t* flat_map,
        float stochasticity
    ) {
        // 0-7: Move, 8: Wait, 9: RotL, 10: RotR
        static const int d_cols[] = {1, 1, 0, -1, -1, -1, 0, 1}; // dx
        static const int d_rows[] = {0, 1, 1, 1, 0, -1, -1, -1}; // dy

        int nr = r, nc = c, nd = d;

        if (a < 8) {
            nc += d_cols[a];
            nr += d_rows[a];
        } else if (a == 8) { /* Wait */ }
        else if (a == 9) nd = (d - 1 + 8) % 8;
        else if (a == 10) nd = (d + 1) % 8;

        // Boundary Check & Obstacle Check
        if (nr < 0 || nr >= rows || nc < 0 || nc >= cols || flat_map[nr * cols + nc] != 0) {
            nr = r; nc = c; // Bump
        } 

        // FIX 1: Correct initialization order {prob, r, c, d}
        return {{1.0f, nr, nc, nd}};
    }

    // --- Legacy Functions (Keep for compatibility if needed) ---
    bool is_in_bounds(const int& x_coord, const int& y_coord, const std::vector<std::vector<bool>>& obstacles) {
        int rows = obstacles.size();
        if (rows == 0) return false;
        int cols = obstacles[0].size();
        return x_coord >= 0 && x_coord < cols && y_coord >= 0 && y_coord < rows;
    }
    
    std::vector<Eigen::Vector2i> getVisibleCells(
        const Eigen::Vector2i& agent_pos,
        const Eigen::Vector2i& agent_dir,
        float fov_angle,
        float fov_distance,
        const std::vector<std::vector<bool>>& obstacles
    ) {
        std::vector<Eigen::Vector2i> visible_cells;

        if (obstacles.empty() || obstacles[0].empty()) {
            return visible_cells; 
        }

        int num_rows = obstacles.size();
        int num_cols = obstacles[0].size();
        
        // --- OPTIMIZATION: Bounding Box Search ---
        // Only iterate cells within the square bounding box of the radius
        int rad = static_cast<int>(std::ceil(fov_distance));
        
        int r_min = std::max(0, agent_pos.y() - rad);
        int r_max = std::min(num_rows - 1, agent_pos.y() + rad);
        int c_min = std::max(0, agent_pos.x() - rad);
        int c_max = std::min(num_cols - 1, agent_pos.x() + rad);

        // Iterate only the relevant subset
        for (int r = r_min; r <= r_max; ++r) {
            for (int c = c_min; c <= c_max; ++c) {
                Eigen::Vector2i target_pos(c, r); 
                if (checkFOV(agent_pos, target_pos, agent_dir, fov_angle, fov_distance, obstacles)) {
                    visible_cells.push_back(target_pos);
                }
            }
        }
        return visible_cells;
    }


    bool checkFOV(const Eigen::Vector2i& agent_pos, const Eigen::Vector2i& target_pos, const Eigen::Vector2i& agent_dir, float fov_angle, float fov_distance, const std::vector<std::vector<bool>>& obstacles) {

        Eigen::Vector2i vector_to_target = target_pos - agent_pos;

        if (vector_to_target.squaredNorm() > fov_distance * fov_distance) {
            return false;
        }

        Eigen::Vector2i agent_dir_normalized = agent_dir.normalized();
        float angle_to_target = std::atan2(vector_to_target.y(), vector_to_target.x());
        float agent_facing_angle = std::atan2(agent_dir_normalized.y(), agent_dir_normalized.x());

        float angle_diff = std::abs(angle_to_target - agent_facing_angle);

        // Handle wrapping
        while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
        angle_diff = std::abs(angle_diff);

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
            if (is_in_bounds(x0, y0, obstacles) && obstacles[y0][x0]) {
                return false; 
            }
            if (x0 == x1 && y0 == y1) {
                break; 
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
        return true; 
    }

    std::vector<Eigen::Vector2i> findShortestPath(
        const Eigen::Vector2i& start,
        const Eigen::Vector2i& goal,
        const std::vector<std::vector<bool>>& obstacle_map,
        bool allow_diagonal
    ) {
        auto result = AStar::findPath(start, goal, obstacle_map, allow_diagonal);
        return result;
    }


    //     // Main Backward Induction Function
    std::vector<int8_t> runBackwardInduction(
        const std::vector<std::vector<bool>>& obstacles,
        const std::vector<Eigen::Vector2i>& evader_path,
        float gamma,
        float stochasticity,
        float fov_angle,
        float fov_distance
    ) {
        int rows = obstacles.size();
        int cols = obstacles[0].size();
        int num_dirs = 8;
        int num_actions = 11;
        int T = evader_path.size();
        int num_states = rows * cols * num_dirs;

        std::vector<Eigen::Vector2i> dir_vecs = {
            {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}
        };

        std::vector<float> V_next(num_states, 0.0f);
        std::vector<float> V_curr(num_states, 0.0f);
        std::vector<int8_t> policy_table(T * num_states, 0);



        

        for (int t = T - 1; t >= 0; --t) {
            
            int er = evader_path[t].y();
            int ec = evader_path[t].x();
            
            int ner = (t < T - 1) ? evader_path[t+1].y() : er;
            int nec = (t < T - 1) ? evader_path[t+1].x() : ec;
            
            Eigen::Vector2i next_evader_pos(nec, ner);

            #pragma omp parallel for collapse(2) 
            for (int r = 0; r < rows; ++r) {
                for (int c = 0; c < cols; ++c) {
                    
                    if (obstacles[r][c]) continue;

                    for (int d = 0; d < num_dirs; ++d) {
                        
                        int state_idx = (r * cols + c) * num_dirs + d;

                        // if (r == er && c == ec) {
                        //     V_curr[state_idx] = 0.0f;
                        //     policy_table[t * num_states + state_idx] = 0;
                        //     continue;
                        // }

                        float max_q = -1e9f;
                        int best_a = 0;

                        for (int a = 0; a < num_actions; ++a) {
                            // Get transitions (internal helper)
                            std::vector<Transition> outcomes = getTransitions(r, c, d, a, obstacles, stochasticity);
                            
                            float q_val = 0.0f;
                            for (const auto& tr : outcomes) {
                                Eigen::Vector2i p_pos(tr.c, tr.r);
                                Eigen::Vector2i p_dir = dir_vecs[tr.d];
                                
                                bool in_view = checkFOV(p_pos, next_evader_pos, p_dir, fov_angle, fov_distance, obstacles);
                                // bool captured = (tr.r == ner && tr.c == nec);
                                
                                float reward = in_view ? 1.0f : 0.0f;
                                // if (captured) reward += 1.0f;
                                float v_future = 0.0f;
                                int next_idx = (tr.r * cols + tr.c) * num_dirs + tr.d;
                                v_future = V_next[next_idx];
                                
                           
                                // if (!captured) {
                                //     int next_idx = (tr.r * cols + tr.c) * num_dirs + tr.d;
                                //     v_future = V_next[next_idx];
                                // }
                                
                                q_val += tr.prob * (reward + gamma * v_future);
                            }

                            if (q_val > max_q) {
                                max_q = q_val;
                                best_a = a;
                            }
                        }

                        V_curr[state_idx] = max_q;
                        policy_table[t * num_states + state_idx] = (int8_t)best_a;
                    }
                }
            }
            
            // Update V_next
            // Safe copy inside the loop
            #pragma omp parallel for
            for(int i=0; i<num_states; ++i) V_next[i] = V_curr[i];
            
            if (t % 10 == 0) {
                 std::cout << "Step " << t << " solved." << std::endl;
            }
        }
        return policy_table;
    }




    std::vector<float> runBackwardInductionQ(
        const std::vector<std::vector<bool>>& obstacles,
        const std::vector<Eigen::Vector2i>& evader_path,
        float gamma,
        float stochasticity,
        float fov_angle,
        float fov_distance,
        int start_r, int start_c, int start_d,
        int fov_lookahead
    ) {
        int rows, cols;
        std::vector<uint8_t> flat_obstacles = flattenMap(obstacles, rows, cols); // Creates std::vector
        
        int num_dirs = 8;
        int num_actions = 11;
        int T = evader_path.size();
        int num_states = rows * cols * num_dirs;

        static const std::vector<Eigen::Vector2i> dir_vecs = {
            {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}
        };

        float min_cos = std::cos(fov_angle / 2.0f);
        float fov_dist_sq = fov_distance * fov_distance;

        std::vector<float> V_curr(num_states, 0.0f);
        std::vector<float> V_next(num_states, 0.0f);
        std::vector<float> Reward_Cache(num_states, 0.0f);
        std::vector<float> start_q_values(num_actions, -1e9f);

        for (int t = T - 1; t >= 0; --t) {

            // Step 1: Pre-calc Rewards
            #pragma omp parallel for collapse(2)
            for (int r = 0; r < rows; ++r) {
                for (int c = 0; c < cols; ++c) {
                    if (flat_obstacles[r * cols + c]) continue;

                    for (int d = 0; d < num_dirs; ++d) {
                        float vis_score = 0.0f;
                        Eigen::Vector2i p_pos(c, r);
                        Eigen::Vector2i p_dir = dir_vecs[d];

                        for (int k = 1; k <= fov_lookahead; ++k) {
                            int target_t = t + k;
                            if (target_t >= T) break;

                            Eigen::Vector2i ev_pos(evader_path[target_t].x(), evader_path[target_t].y());

                            // FIX HERE: Use .data() to get the raw pointer
                            if (checkFOV(p_pos, ev_pos, p_dir, fov_angle, fov_distance, obstacles)) {
                                vis_score += 1.0f;
                            }
                        }
                        Reward_Cache[(r * cols + c) * num_dirs + d] = vis_score;
                    }
                }
            }

            // Step 2: Bellman
            #pragma omp parallel for collapse(2)
            for (int r = 0; r < rows; ++r) {
                for (int c = 0; c < cols; ++c) {
                    // if (flat_obstacles[r * cols + c]) continue;
                    if (obstacles[r][c]) continue;

                    for (int d = 0; d < num_dirs; ++d) {
                        float max_q = -1e9f;
                        for (int a = 0; a < num_actions; ++a) {
                            // Legacy version uses legacy getTransitionsFlat helper
                            auto outcomes = getTransitions(r, c, d, a, obstacles, stochasticity);
                            
                            float q_val = 0.0f;
                            for (const auto& tr : outcomes) {
                                int next_idx = (tr.r * cols + tr.c) * num_dirs + tr.d;
                                q_val += tr.prob * (Reward_Cache[next_idx] + gamma * V_next[next_idx]);
                            }

                            

                            if (t == 0 && r == start_r && c == start_c && d == start_d) {
                                start_q_values[a] = q_val;
                            }
                            if (q_val > max_q) max_q = q_val;
                        }
                        V_curr[(r * cols + c) * num_dirs + d] = max_q;
                    }
                }
            }
            std::swap(V_curr, V_next);
        }
        return start_q_values;
    }

  std::vector<float> runBatchBackwardInduction(
    const std::vector<std::vector<bool>>& obstacles,
        // CHANGED: Batch of paths
        const std::vector<std::vector<Eigen::Vector2i>>& evader_paths_batch, 
        const std::vector<float>& weights,
        float gamma,
        float stochasticity,
        float fov_angle,
        float fov_distance,
        int start_r, int start_c, int start_d,
        int fov_lookahead
    ) {
        int rows, cols;
        // 1. Flatten map ONCE (Shared by all threads)
        std::vector<uint8_t> flat_obstacles = flattenMap(obstacles, rows, cols);
        const uint8_t* map_ptr = flat_obstacles.data(); // Get raw pointer for helpers

        int num_scenarios = evader_paths_batch.size();
        int num_dirs = 8;
        int num_actions = 11;
        int num_states = rows * cols * num_dirs;

        static const std::vector<Eigen::Vector2i> dir_vecs = {
            {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}
        };

        float min_cos = std::cos(fov_angle / 2.0f);
        float fov_dist_sq = fov_distance * fov_distance;

        // Final Accumulator
        std::vector<float> total_weighted_q(num_actions, 0.0f);

        // --- THREADING STRATEGY (For 32-Thread CPU) ---
        // int max_hw_threads = omp_get_max_threads(); 
        
        // // Reserve 4 threads for Python/OS
        // int available_threads = std::max(1, max_hw_threads - 4); 

        // // Split threads: Outer (Scenarios) vs Inner (Grid)
        // int outer_threads = std::min(num_scenarios, available_threads);
        // int inner_threads = std::max(1, available_threads / outer_threads);

        // Allow loops inside loops
        // omp_set_max_active_levels(2); 
        // omp_set_max_active_levels(1);

        // --- OUTER LOOP: Scenarios ---
        // #pragma omp parallel for schedule(dynamic)
        for (int k = 0; k < num_scenarios; ++k) {
            
            // Thread-Local Data (Each scenario gets its own solver memory)
            const auto& evader_path = evader_paths_batch[k];
            int T = evader_path.size();
            float weight = weights[k];

            std::vector<float> V_curr(num_states, 0.0f);
            std::vector<float> V_next(num_states, 0.0f);
            std::vector<float> Reward_Cache(num_states, 0.0f);
            std::vector<float> local_q_values(num_actions, -1e9f);

            for (int t = T - 1; t >= 0; --t) {

                // --- Step 1: Pre-calc Rewards ---
                // #pragma omp parallel for collapse(2) num_threads(inner_threads)
                #pragma omp parallel for collapse(2)
                for (int r = 0; r < rows; ++r) {
                    for (int c = 0; c < cols; ++c) {
                        if (flat_obstacles[r * cols + c]) continue;

                        for (int d = 0; d < num_dirs; ++d) {
                            float vis_score = 0.0f;
                            Eigen::Vector2i p_pos(c, r);
                            Eigen::Vector2i p_dir = dir_vecs[d];

                            for (int k = 1; k <= fov_lookahead; ++k) {
                                int target_t = t + k;
                                if (target_t >= T) break;

                                Eigen::Vector2i ev_pos(evader_path[target_t].x(), evader_path[target_t].y());

                                // FIX HERE: Use .data() to get the raw pointer
                                if (checkFOV_Optimized(p_pos, ev_pos, p_dir, min_cos, fov_dist_sq, flat_obstacles.data(), cols, rows)) {
                                    vis_score += 1.0f;
                                }
                            }
                            Reward_Cache[(r * cols + c) * num_dirs + d] = vis_score;
                        }
                    }
                }

                // --- Step 2: Bellman Update ---
                // #pragma omp parallel for collapse(2) num_threads(inner_threads)
                 for (int r = 0; r < rows; ++r) {
                        for (int c = 0; c < cols; ++c) {
                            if (flat_obstacles[r * cols + c]) continue;

                            for (int d = 0; d < num_dirs; ++d) {
                                float max_q = -1e9f;
                                for (int a = 0; a < num_actions; ++a) {
                                    // Legacy version uses legacy getTransitionsFlat helper
                                    auto outcomes = getTransitionsFlat(r, c, d, a, rows, cols, flat_obstacles.data(), stochasticity);
                                    
                                    float q_val = 0.0f;
                                    for (const auto& tr : outcomes) {
                                        int next_idx = (tr.r * cols + tr.c) * num_dirs + tr.d;
                                        q_val += tr.prob * (Reward_Cache[next_idx] + gamma * V_next[next_idx]);
                                    }

                                    if (t == 0 && r == start_r && c == start_c && d == start_d) {
                                        local_q_values[a] = q_val;
                                    }
                                    if (q_val > max_q) max_q = q_val;
                                }
                                V_curr[(r * cols + c) * num_dirs + d] = max_q;
                            }
                        }
                    }
                std::swap(V_curr, V_next);
            }

            // Accumulate Result Thread-Safely


            for (int a = 0; a < num_actions; ++a) {
                total_weighted_q[a] += weight * local_q_values[a];
            }

        }

        return total_weighted_q;
    }
} 

// #include "gridworldenvutils.hpp"
// #include "AStar.hpp"
// #include <iostream>
// #include <algorithm>
// #include <cmath>
// #include <map>       // Required for std::map
// #include <vector>    // Required for std::vector
// #include <omp.h>     // Required for OpenMP
// #include <vector>
// #include <cmath>
// #include <pybind11/pybind11.h>
// #include <pybind11/eigen.h>        // For Eigen types
// #include <pybind11/stl.h>          // For STL containers like std::vector
// #include <pybind11/numpy.h>        // For NumPy arrays
// #include "gridworldenvutils.hpp"

// namespace py = pybind11;
// namespace gridworld {

//     // --- Helper Structs ---
//     // Must be defined BEFORE any function signatures that use it
//     struct Transition {
//         float prob;
//         int r, c, d;
//     };

//     // --- Forward Declarations ---
//     std::vector<Transition> getTransitions(
//         int r, int c, int d, int action, 
//         const std::vector<std::vector<bool>>& obstacles, 
//         float stochasticity
//     );

//     inline bool is_blocked(const std::vector<uint8_t>& flat_map, int x, int y, int width, int height) {
//     if (x < 0 || x >= width || y < 0 || y >= height) return true; // Treat OOB as blocked
//     return flat_map[y * width + x] != 0;
//     }

//     bool checkFOV_Optimized(
//         const Eigen::Vector2i& agent_pos, 
//         const Eigen::Vector2i& target_pos, 
//         const Eigen::Vector2i& agent_dir, 
//         float min_cos_angle, // Pre-calculated: cos(fov_angle / 2)
//         float fov_dist_sq,   // Pre-calculated: fov_distance^2
//         const std::vector<uint8_t>& flat_obstacles, // USE FLAT UINT8, NOT VECTOR<BOOL>
//         int map_width,
//         int map_height
//     ) {
//         Eigen::Vector2i to_target = target_pos - agent_pos;
//         float dist_sq = (float)to_target.squaredNorm();
//         if (dist_sq > fov_dist_sq || dist_sq < 0.1f) return false;

//         // 3. Angle Check (Dot Product - No Atan2)        
//         float dot = (float)agent_dir.dot(to_target);

//         if (dot < 0) return false; 

//         // b. Squared comparison
        
//         float agent_dir_mag_sq = (float)agent_dir.squaredNorm();
//         float min_cos_sq = min_cos_angle * min_cos_angle;

//         if ((dot * dot) < (min_cos_sq * agent_dir_mag_sq * dist_sq)) {
//             return false;
//         }

//         // 4. Line of Sight (Bresenham)
//         // Inlined for speed (avoid function call overhead)
//         int x0 = agent_pos.x();
//         int y0 = agent_pos.y();
//         int x1 = target_pos.x();
//         int y1 = target_pos.y();

//         int dx = std::abs(x1 - x0);
//         int dy = -std::abs(y1 - y0);
//         int sx = (x0 < x1) ? 1 : -1;
//         int sy = (y0 < y1) ? 1 : -1;
//         int err = dx + dy;

//         while (true) {
//             // Check collision (Current pixel)
//             // Note: Usually we don't check the start/end pixel strictly, 
//             // but for tight obstacles we might.
//             if (flat_obstacles[y0 * map_width + x0]) {
//                 if (x0 == x1 && y0 == y1) break; // If target is ON an obstacle, it's visible?
//                 return false; 
//             }

//             if (x0 == x1 && y0 == y1) break;
            
//             int e2 = 2 * err;
//             if (e2 >= dy) { err += dy; x0 += sx; }
//             if (e2 <= dx) { err += dx; y0 += sy; }
//         }

//         return true;
//     }


//     std::vector<Eigen::Vector2i> getVisibleCells(
//         const Eigen::Vector2i& agent_pos,
//         const Eigen::Vector2i& agent_dir,
//         float fov_angle,
//         float fov_distance,
//         const std::vector<std::vector<bool>>& obstacles
//     ) {
//         std::vector<Eigen::Vector2i> visible_cells;

//         if (obstacles.empty() || obstacles[0].empty()) {
//             return visible_cells; 
//         }

//         int num_rows = obstacles.size();
//         int num_cols = obstacles[0].size();
        
//         // --- OPTIMIZATION: Bounding Box Search ---
//         // Only iterate cells within the square bounding box of the radius
//         int rad = static_cast<int>(std::ceil(fov_distance));
        
//         int r_min = std::max(0, agent_pos.y() - rad);
//         int r_max = std::min(num_rows - 1, agent_pos.y() + rad);
//         int c_min = std::max(0, agent_pos.x() - rad);
//         int c_max = std::min(num_cols - 1, agent_pos.x() + rad);

//         // Iterate only the relevant subset
//         for (int r = r_min; r <= r_max; ++r) {
//             for (int c = c_min; c <= c_max; ++c) {
//                 Eigen::Vector2i target_pos(c, r); 
//                 if (checkFOV(agent_pos, target_pos, agent_dir, fov_angle, fov_distance, obstacles)) {
//                     visible_cells.push_back(target_pos);
//                 }
//             }
//         }
//         return visible_cells;
//     }


//     bool checkFOV(const Eigen::Vector2i& agent_pos, const Eigen::Vector2i& target_pos, const Eigen::Vector2i& agent_dir, float fov_angle, float fov_distance, const std::vector<std::vector<bool>>& obstacles) {

//         Eigen::Vector2i vector_to_target = target_pos - agent_pos;

//         if (vector_to_target.squaredNorm() > fov_distance * fov_distance) {
//             return false;
//         }

//         Eigen::Vector2i agent_dir_normalized = agent_dir.normalized();
//         float angle_to_target = std::atan2(vector_to_target.y(), vector_to_target.x());
//         float agent_facing_angle = std::atan2(agent_dir_normalized.y(), agent_dir_normalized.x());

//         float angle_diff = std::abs(angle_to_target - agent_facing_angle);

//         // Handle wrapping
//         while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
//         while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
//         angle_diff = std::abs(angle_diff);

//         if (angle_diff > fov_angle / 2) {
//             return false;
//         }

//         if (!checkLineOfSight(agent_pos, target_pos, obstacles)) {
//             return false;
//         }

//         return true;
//     }

//     bool checkLineOfSight(const Eigen::Vector2i& agent_pos, const Eigen::Vector2i& target_pos, const std::vector<std::vector<bool>>& obstacles) {

//         int x0 = agent_pos.x(), y0 = agent_pos.y();
//         int x1 = target_pos.x(), y1 = target_pos.y();

//         int dx = std::abs(x1 - x0);
//         int dy = -std::abs(y1 - y0);

//         int sx = (x0 < x1) ? 1 : -1;
//         int sy = (y0 < y1) ? 1 : -1;
        
//         int err = dx + dy;

//         while (true) {
//             if (is_in_bounds(x0, y0, obstacles) && obstacles[y0][x0]) {
//                 return false; 
//             }
//             if (x0 == x1 && y0 == y1) {
//                 break; 
//             }
            
//             int e2 = 2 * err;
//             if (e2 >= dy) {
//                 err += dy;
//                 x0 += sx;
//             }
//             if (e2 <= dx) {
//                 err += dx;
//                 y0 += sy;
//             }
//         }
//         return true; 
//     }

//     std::vector<Eigen::Vector2i> findShortestPath(
//         const Eigen::Vector2i& start,
//         const Eigen::Vector2i& goal,
//         const std::vector<std::vector<bool>>& obstacle_map,
//         bool allow_diagonal
//     ) {
//         auto result = AStar::findPath(start, goal, obstacle_map, allow_diagonal);
//         return result;
//     }

//     bool is_in_bounds(const int& x_coord, const int& y_coord, const std::vector<std::vector<bool>>& obstacles) {
//         int rows = obstacles.size();
//         if (rows == 0) return false;
//         int cols = obstacles[0].size();
//         return x_coord >= 0 && x_coord < cols && y_coord >= 0 && y_coord < rows;
//     }

//     // ==========================================
//     //      BACKWARD INDUCTION IMPLEMENTATION
//     // ==========================================


//     // Overload for Raw Pointer (Fast / Zero-Copy)
// std::vector<Transition> getTransitionsFlat(
//     int r, int c, int d, int a, 
//     int rows, int cols, 
//     const uint8_t* flat_map,  // <--- Takes raw pointer instead of vector<vector>
//     float stochasticity
// ) {
//     // 0-7: Move, 8: Wait, 9: RotL, 10: RotR
//     static const int dr[] = {0, -1, 0, 1, 1, 1, 0, -1}; // Check your Y-axis direction!
//     static const int dc[] = {1, 1, 1, 0, -1, -1, -1, 0}; // Check your X-axis direction!
    
//     // Note: Depending on your dir_vecs definition, verify dr/dc indices match 0-7.
//     // Assuming standard: E, SE, S, SW, W, NW, N, NE or similar.
//     // Ensure this matches your existing getTransitions logic.
//     // Below is a generic implementation assuming 8-connectivity.
    
//     // Example from your previous code:
//     // {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}
//     // dx (cols), dy (rows)
//     static const int d_cols[] = {1, 1, 0, -1, -1, -1, 0, 1};
//     static const int d_rows[] = {0, 1, 1, 1, 0, -1, -1, -1};

//     int nr = r, nc = c, nd = d;

//     if (a < 8) {
//         nc += d_cols[a];
//         nr += d_rows[a];
//     } else if (a == 8) {
//         // Wait
//     } else if (a == 9) nd = (d - 1 + 8) % 8;
//     else if (a == 10) nd = (d + 1) % 8;

//     // Boundary Check
//     if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) {
//         nr = r; nc = c; // Wall bump (Stay)
//     } 
//     // Obstacle Check (Using Raw Pointer math)
//     else if (flat_map[nr * cols + nc] != 0) {
//         nr = r; nc = c; // Wall bump (Stay)
//     }

//     return {{nr, nc, nd, 1.0f}};
// }

//     std::vector<Transition> getTransitions(int r, int c, int d, int action, const std::vector<std::vector<bool>>& obstacles, float stochasticity) {
//         int rows = obstacles.size();
//         int cols = obstacles[0].size();
//         std::vector<Transition> transitions;

//         // Outcomes map: action_code -> probability
//         std::map<int, float> outcomes; 
        
//         // 1. Success case
//         if (stochasticity > 0) {
//             outcomes[action] += stochasticity;
//         }

//         // 2. Failure/Slip cases
//         float p_fail = 1.0f - stochasticity;
//         if (p_fail > 0) {
//             std::vector<int> slips;
//             if (action == 0) slips = {0};
//             else if (action == 1) slips = {5, 6}; // UP -> UL, UR
//             else if (action == 2) slips = {7, 8}; // DOWN -> DL, DR
//             else if (action == 3) slips = {5, 7}; // LEFT -> UL, DL
//             else if (action == 4) slips = {6, 8}; // RIGHT -> UR, DR
//             else if (action == 5) slips = {1, 3}; // UL -> UP, LEFT
//             else if (action == 6) slips = {1, 4}; // UR -> UP, RIGHT
//             else if (action == 7) slips = {2, 3}; // DL -> DOWN, LEFT
//             else if (action == 8) slips = {2, 4}; // DR -> DOWN, RIGHT
//             else if (action == 9 || action == 10) slips = {0, 11}; // ROT -> STAY, OVERSTEER
            
//             float prob_per_slip = p_fail / slips.size();
//             for (int s : slips) outcomes[s] += prob_per_slip;
//         }

//         for (auto const& [act, prob] : outcomes) {
//             if (prob <= 0.0f) continue;

//             int nr = r, nc = c, nd = d;
//             int intended = action;

//             // Physics Logic
//             if (act == 1) { nr = std::max(0, r - 1); nd = 6; } // UP (N)
//             else if (act == 2) { nr = std::min(rows - 1, r + 1); nd = 2; } // DOWN (S)
//             else if (act == 3) { nc = std::max(0, c - 1); nd = 4; } // LEFT (W)
//             else if (act == 4) { nc = std::min(cols - 1, c + 1); nd = 0; } // RIGHT (E)
//             else if (act == 5) { nr = std::max(0, r - 1); nc = std::max(0, c - 1); nd = 5; } // UL (NW)
//             else if (act == 6) { nr = std::max(0, r - 1); nc = std::min(cols - 1, c + 1); nd = 7; } // UR (NE)
//             else if (act == 7) { nr = std::min(rows - 1, r + 1); nc = std::max(0, c - 1); nd = 3; } // DL (SW)
//             else if (act == 8) { nr = std::min(rows - 1, r + 1); nc = std::min(cols - 1, c + 1); nd = 1; } // DR (SE)
//             else if (act == 9) { nd = (d + 1) % 8; } // CW
//             else if (act == 10) { nd = (d + 8 - 1) % 8; } // CCW
//             else if (act == 11) { // OVERSTEER
//                 if (intended == 9) nd = (d + 2) % 8;
//                 else if (intended == 10) nd = (d + 8 - 2) % 8;
//             }

//             // Wall collision check
//             if (is_in_bounds(nc, nr, obstacles) && obstacles[nr][nc]) {
//                 nr = r; nc = c; // Bounce back
//             }

//             transitions.push_back({prob, nr, nc, nd});
//         }
//         return transitions;
//     }

//     std::vector<uint8_t> flattenMap(
//         const std::vector<std::vector<bool>>& grid, 
//         int& out_rows, 
//         int& out_cols
//     ) {
//         out_rows = grid.size();
//         out_cols = (out_rows > 0) ? grid[0].size() : 0;
        
//         // Allocate exact memory once
//         std::vector<uint8_t> flat_grid(out_rows * out_cols);

//         // Flattening loop
//         // Note: We don't use OpenMP here because memory writing 
//         // is usually bandwidth-bound, not compute-bound for small grids.
//         for (int r = 0; r < out_rows; ++r) {
//             const auto& row_vec = grid[r]; // Reference to inner vector
//             for (int c = 0; c < out_cols; ++c) {
//                 // Convert bool to 1 or 0
//                 flat_grid[r * out_cols + c] = row_vec[c] ? 1 : 0;
//             }
//         }
        
//         return flat_grid;
//     }

//     // Main Backward Induction Function
//     std::vector<int8_t> runBackwardInduction(
//         const std::vector<std::vector<bool>>& obstacles,
//         const std::vector<Eigen::Vector2i>& evader_path,
//         float gamma,
//         float stochasticity,
//         float fov_angle,
//         float fov_distance
//     ) {
//         int rows = obstacles.size();
//         int cols = obstacles[0].size();
//         int num_dirs = 8;
//         int num_actions = 11;
//         int T = evader_path.size();
//         int num_states = rows * cols * num_dirs;

//         std::vector<Eigen::Vector2i> dir_vecs = {
//             {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}
//         };

//         std::vector<float> V_next(num_states, 0.0f);
//         std::vector<float> V_curr(num_states, 0.0f);
//         std::vector<int8_t> policy_table(T * num_states, 0);

//         std::cout << "Running Backward Induction (T=" << T << ") with OpenMP..." << std::endl;

        

//         for (int t = T - 1; t >= 0; --t) {
            
//             int er = evader_path[t].y();
//             int ec = evader_path[t].x();
            
//             int ner = (t < T - 1) ? evader_path[t+1].y() : er;
//             int nec = (t < T - 1) ? evader_path[t+1].x() : ec;
            
//             Eigen::Vector2i next_evader_pos(nec, ner);

//             #pragma omp parallel for collapse(2) 
//             for (int r = 0; r < rows; ++r) {
//                 for (int c = 0; c < cols; ++c) {
                    
//                     if (obstacles[r][c]) continue;

//                     for (int d = 0; d < num_dirs; ++d) {
                        
//                         int state_idx = (r * cols + c) * num_dirs + d;

//                         // if (r == er && c == ec) {
//                         //     V_curr[state_idx] = 0.0f;
//                         //     policy_table[t * num_states + state_idx] = 0;
//                         //     continue;
//                         // }

//                         float max_q = -1e9f;
//                         int best_a = 0;

//                         for (int a = 0; a < num_actions; ++a) {
//                             // Get transitions (internal helper)
//                             std::vector<Transition> outcomes = getTransitions(r, c, d, a, obstacles, stochasticity);
                            
//                             float q_val = 0.0f;
//                             for (const auto& tr : outcomes) {
//                                 Eigen::Vector2i p_pos(tr.c, tr.r);
//                                 Eigen::Vector2i p_dir = dir_vecs[tr.d];
                                
//                                 bool in_view = checkFOV(p_pos, next_evader_pos, p_dir, fov_angle, fov_distance, obstacles);
//                                 // bool captured = (tr.r == ner && tr.c == nec);
                                
//                                 float reward = in_view ? 1.0f : 0.0f;
//                                 // if (captured) reward += 1.0f;
//                                 float v_future = 0.0f;
//                                 int next_idx = (tr.r * cols + tr.c) * num_dirs + tr.d;
//                                 v_future = V_next[next_idx];
                                
                           
//                                 // if (!captured) {
//                                 //     int next_idx = (tr.r * cols + tr.c) * num_dirs + tr.d;
//                                 //     v_future = V_next[next_idx];
//                                 // }
                                
//                                 q_val += tr.prob * (reward + gamma * v_future);
//                             }

//                             if (q_val > max_q) {
//                                 max_q = q_val;
//                                 best_a = a;
//                             }
//                         }

//                         V_curr[state_idx] = max_q;
//                         policy_table[t * num_states + state_idx] = (int8_t)best_a;
//                     }
//                 }
//             }
            
//             // Update V_next
//             // Safe copy inside the loop
//             #pragma omp parallel for
//             for(int i=0; i<num_states; ++i) V_next[i] = V_curr[i];
            
//             if (t % 10 == 0) {
//                  std::cout << "Step " << t << " solved." << std::endl;
//             }
//         }
//         return policy_table;
//     }




//     std::vector<float> runBackwardInductionQ(
//     const std::vector<std::vector<bool>>& obstacles,
//     const std::vector<Eigen::Vector2i>& evader_path,
//     float gamma,
//     float stochasticity,
//     float fov_angle,
//     float fov_distance,
//     int start_r,
//     int start_c,
//     int start_d,
//     int fov_lookahead
// ) {
//     int rows, cols;
//     std::vector<uint8_t> flat_obstacles = flattenMap(obstacles, rows, cols);
    
//     int num_dirs = 8;
//     int num_actions = 11;
//     int T = evader_path.size();
//     int num_states = rows * cols * num_dirs;

//     static const std::vector<Eigen::Vector2i> dir_vecs = {
//         {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}
//     };

//     // Pre-calculate FOV constants
//     float min_cos = std::cos(fov_angle / 2.0f);
//     float fov_dist_sq = fov_distance * fov_distance;

//     std::vector<float> V_curr(num_states, 0.0f);
//     std::vector<float> V_next(num_states, 0.0f);
//     std::vector<float> Reward_Cache(num_states, 0.0f); // Scratchpad for rewards
//     std::vector<float> start_q_values(num_actions, -1e9f);

//     for (int t = T - 1; t >= 0; --t) {

//         // --- STEP 1: Pre-calculate Rewards (Geometry Pass) ---
//         // Complexity: States * Lookahead (Independent of Actions)
//         #pragma omp parallel for collapse(2)
//         for (int r = 0; r < rows; ++r) {
//             for (int c = 0; c < cols; ++c) {
//                 if (flat_obstacles[r * cols + c]) continue;

//                 for (int d = 0; d < num_dirs; ++d) {
                    
//                     float vis_score = 0.0f;
//                     Eigen::Vector2i p_pos(c, r); // x=col, y=row
//                     Eigen::Vector2i p_dir = dir_vecs[d];

//                     // Future Trajectory Overlap Check
//                     for (int k = 1; k <= fov_lookahead; ++k) {
//                         int target_t = t + k;
//                         if (target_t >= T) break;

//                         Eigen::Vector2i ev_pos(evader_path[target_t].x(), evader_path[target_t].y());

//                         if (checkFOV_Optimized(p_pos, ev_pos, p_dir, min_cos, fov_dist_sq, flat_obstacles, cols, rows)) {
//                             vis_score += 1.0f;
//                         }
//                     }
                    
//                     int idx = (r * cols + c) * num_dirs + d;
//                     Reward_Cache[idx] = vis_score;
//                 }
//             }
//         }

//         // --- STEP 2: Bellman Update (Graph Pass) ---
//         // Complexity: States * Actions (Fast lookups only)
//         #pragma omp parallel for collapse(2)
//         for (int r = 0; r < rows; ++r) {
//             for (int c = 0; c < cols; ++c) {
//                 if (flat_obstacles[r * cols + c]) continue;

//                 for (int d = 0; d < num_dirs; ++d) {
                    
//                     float max_q = -1e9f;
                    
//                     // Iterate Actions
//                     for (int a = 0; a < num_actions; ++a) {
//                         auto outcomes = getTransitions(r, c, d, a, obstacles, stochasticity);
                        
//                         float q_val = 0.0f;
//                         for (const auto& tr : outcomes) {
//                             int next_idx = (tr.r * cols + tr.c) * num_dirs + tr.d;
                            
//                             // Instant lookup from Step 1
//                             float r_next = Reward_Cache[next_idx]; 
//                             float v_next = V_next[next_idx];
                            
//                             q_val += tr.prob * (r_next + gamma * v_next);
//                         }

//                         // Capture Q-values for the start state at t=0
//                         if (t == 0 && r == start_r && c == start_c && d == start_d) {
//                             start_q_values[a] = q_val;
//                         }

//                         if (q_val > max_q) max_q = q_val;
//                     }
                    
//                     int idx = (r * cols + c) * num_dirs + d;
//                     V_curr[idx] = max_q;
//                 }
//             }
//         }

//         // Pointer swap for next iteration
//         std::swap(V_curr, V_next);
//     }
    
//     return start_q_values;
// }
// //     std::vector<float> runBackwardInductionQ(
// //     const std::vector<std::vector<bool>>& obstacles,
// //     const std::vector<Eigen::Vector2i>& evader_path,
// //     float gamma,
// //     float stochasticity,
// //     float fov_angle,
// //     float fov_distance,
// //     int start_r,
// //     int start_c,
// //     int start_d,
// //     int fov_lookahead
// // ) {
// //         int rows = obstacles.size();
// //         int cols = obstacles[0].size();
// //         int num_dirs = 8;
// //         int num_actions = 11;
// //         int T = evader_path.size();
// //         int num_states = rows * cols * num_dirs;

// //         std::vector<Eigen::Vector2i> dir_vecs = {
// //             {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}
// //         };

// //         std::vector<float> V_next(num_states, 0.0f);
// //         std::vector<float> V_curr(num_states, 0.0f);
// //         std::vector<float> start_q_values(num_actions, -1e9f);

// //         std::vector<uint8_t> flat_obstacles = flattenMap(obstacles, rows, cols);


// //         for (int t = T - 1; t >= 0; --t) {
            
// //             int er = evader_path[t].y();
// //             int ec = evader_path[t].x();
            
// //             int ner = (t < T - 1) ? evader_path[t+1].y() : er;
// //             int nec = (t < T - 1) ? evader_path[t+1].x() : ec;
            
// //             Eigen::Vector2i next_evader_pos(nec, ner);

// //             #pragma omp parallel for collapse(2) 
// //             for (int r = 0; r < rows; ++r) {
// //                 for (int c = 0; c < cols; ++c) {
                    
// //                     if (obstacles[r][c]) continue;

// //                     for (int d = 0; d < num_dirs; ++d) {
                        
// //                         int state_idx = (r * cols + c) * num_dirs + d;

// //                         // if (r == er && c == ec) {
// //                         //     V_curr[state_idx] = 0.0f;
// //                         //     policy_table[t * num_states + state_idx] = 0;
// //                         //     continue;
// //                         // }

// //                         float max_q = -1e9f;
// //                         int best_a = 0;

// //                         for (int a = 0; a < num_actions; ++a) {
// //                             // Get transitions (internal helper)
// //                             std::vector<Transition> outcomes = getTransitions(r, c, d, a, obstacles, stochasticity);
                            
// //                             float q_val = 0.0f;
// //                             for (const auto& tr : outcomes) {
// //                                 Eigen::Vector2i p_pos(tr.c, tr.r);
// //                                 Eigen::Vector2i p_dir = dir_vecs[tr.d];
                                
// //                                 bool in_view = checkFOV(p_pos, next_evader_pos, p_dir, fov_angle, fov_distance, obstacles);
// //                                 bool in_view = checkFOV_Optimized(
// //                                     p_pos, 
// //                                     next_evader_pos, 
// //                                     p_dir, 
// //                                     std::cos(fov_angle / 2.0f), 
// //                                     fov_distance * fov_distance, 
// //                                     flat_obstacles,
// //                                     cols,
// //                                     rows
// //                                 );
// //                                 // bool captured = (tr.r == ner && tr.c == nec);
                                
// //                                 // float reward = in_view ? 1.0f : 0.0f;

// //                                 float reward = 0.0f;
                            
// //                                 // Check overlap for the next 'fov_lookahead' steps
// //                                 for (int k = 0; k < fov_lookahead; ++k) {
// //                                     int target_t = t + 1 + k; // Start looking from next step
                                    
// //                                     // Stop if we exceed the prediction horizon
// //                                     if (target_t >= T) break; 

// //                                     int e_x = evader_path[target_t].x();
// //                                     int e_y = evader_path[target_t].y();
// //                                     Eigen::Vector2i future_ev_pos(e_x, e_y);

// //                                     if (checkFOV_Optimized(
// //                                         p_pos, 
// //                                         future_ev_pos, 
// //                                         p_dir, 
// //                                         std::cos(fov_angle / 2.0f), 
// //                                         fov_distance * fov_distance, 
// //                                         flat_obstacles,
// //                                         cols,
// //                                         rows
// //                                     )) {
// //                                         // You can add decay here if you want: 1.0f * std::pow(0.9, k)
// //                                         reward += 1.0f; 
// //                                     }
// //                                 }
// //                                 // if (captured) reward += 1.0f;
// //                                 float v_future = 0.0f;
// //                                 int next_idx = (tr.r * cols + tr.c) * num_dirs + tr.d;
// //                                 v_future = V_next[next_idx];
                                
                           
// //                                 // if (!captured) {
// //                                 //     int next_idx = (tr.r * cols + tr.c) * num_dirs + tr.d;
// //                                 //     v_future = V_next[next_idx];
// //                                 // }
                                
// //                                 q_val += tr.prob * (reward + gamma * v_future);
// //                             }

// //                             if (t == 0 && r == start_r && c == start_c && d == start_d) {
// //                                 start_q_values[a] = q_val;
// //                             }

// //                             if (q_val > max_q) {
// //                                 max_q = q_val;
// //                                 best_a = a;
// //                             }
// //                         }

// //                         V_curr[state_idx] = max_q;
// //                     }
// //                 }
// //             }
            
// //             // Update V_next
// //             // Safe copy inside the loop
// //             #pragma omp parallel for
// //             for(int i=0; i<num_states; ++i) V_next[i] = V_curr[i];
            
// //             // if (t % 10 == 0) {
// //             //      std::cout << "Step " << t << " solved." << std::endl;
// //             // }
// //         }
// //         return start_q_values;
// //     }

// std::vector<float> runBatchBackwardInduction(
//     py::array_t<uint8_t> obstacles,
//     // Input: List of trajectories (Batch)
//     const std::vector<std::vector<Eigen::Vector2i>>& evader_paths_batch, 
//     // Input: Weights for each trajectory
//     const std::vector<float>& weights,
//     float gamma,
//     float stochasticity,
//     float fov_angle,
//     float fov_distance,
//     int start_r, int start_c, int start_d,
//     int fov_lookahead
// ) {
//     int rows, cols;
//     std::vector<uint8_t> flat_obstacles = flattenMap(obstacles, rows, cols);
    
//     int num_scenarios = evader_paths_batch.size();
//     int num_dirs = 8;
//     int num_actions = 11;
//     int num_states = rows * cols * num_dirs;
    
//     // Constants
//     static const std::vector<Eigen::Vector2i> dir_vecs = {
//         {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}
//     };
//     float min_cos = std::cos(fov_angle / 2.0f);
//     float fov_dist_sq = fov_distance * fov_distance;

//     // Accumulator for the final result
//     std::vector<float> total_weighted_q(num_actions, 0.0f);

//     // --- OUTER PARALLELISM: Process Scenarios in Parallel ---
//     // We disable nested parallelism to prevent thread oversubscription
//     omp_set_max_active_levels(1); 

//     #pragma omp parallel for
//     for (int k = 0; k < num_scenarios; ++k) {
        
//         // Thread-Local Data Structures
//         const auto& evader_path = evader_paths_batch[k];
//         int T = evader_path.size();
//         float weight = weights[k];

//         std::vector<float> V_curr(num_states, 0.0f);
//         std::vector<float> V_next(num_states, 0.0f);
//         std::vector<float> Reward_Cache(num_states, 0.0f);
//         std::vector<float> local_q_values(num_actions, -1e9f);

//         // Backward Induction Loop
//         for (int t = T - 1; t >= 0; --t) {

//             // Step 1: Pre-calculate Rewards (Serial Inner Loop)
//             // Note: Since we parallelize 'k', we keep inner loops serial 
//             // to avoid cache trashing and oversubscription.
//             for (int r = 0; r < rows; ++r) {
//                 for (int c = 0; c < cols; ++c) {
//                     if (flat_obstacles[r * cols + c]) continue;

//                     for (int d = 0; d < num_dirs; ++d) {
//                         float vis_score = 0.0f;
//                         Eigen::Vector2i p_pos(c, r);
//                         Eigen::Vector2i p_dir = dir_vecs[d];

//                         for (int look = 1; look <= fov_lookahead; ++look) {
//                             int target_t = t + look;
//                             if (target_t >= T) break;
//                             Eigen::Vector2i ev_pos(evader_path[target_t].x(), evader_path[target_t].y());
                            
//                             if (checkFOV_Optimized(p_pos, ev_pos, p_dir, min_cos, fov_dist_sq, flat_obstacles, cols)) {
//                                 vis_score += 1.0f;
//                             }
//                         }
//                         Reward_Cache[(r * cols + c) * num_dirs + d] = vis_score;
//                     }
//                 }
//             }

//             // Step 2: Bellman Update
//             for (int r = 0; r < rows; ++r) {
//                 for (int c = 0; c < cols; ++c) {
//                     if (flat_obstacles[r * cols + c]) continue;

//                     for (int d = 0; d < num_dirs; ++d) {
//                         float max_q = -1e9f;
                        
//                         for (int a = 0; a < num_actions; ++a) {
//                             // Inlined transition logic for speed
//                             auto outcomes = getTransitionsFlat(r, c, d, a, obstacles, stochasticity);
                            
//                             float q_val = 0.0f;
//                             for (const auto& tr : outcomes) {
//                                 int next_idx = (tr.r * cols + tr.c) * num_dirs + tr.d;
//                                 q_val += tr.prob * (Reward_Cache[next_idx] + gamma * V_next[next_idx]);
//                             }

//                             if (t == 0 && r == start_r && c == start_c && d == start_d) {
//                                 local_q_values[a] = q_val;
//                             }
//                             if (q_val > max_q) max_q = q_val;
//                         }
//                         V_curr[(r * cols + c) * num_dirs + d] = max_q;
//                     }
//                 }
//             }
//             std::swap(V_curr, V_next);
//         }

//         // Thread-Safe Accumulation
//         #pragma omp critical
//         {
//             for (int a = 0; a < num_actions; ++a) {
//                 total_weighted_q[a] += weight * local_q_values[a];
//             }
//         }
//     }
    
//     return total_weighted_q;
// }

// } // namespace gridworld
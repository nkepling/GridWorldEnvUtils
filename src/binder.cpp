#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>        // For Eigen types
#include <pybind11/stl.h>          // For STL containers like std::vector
#include <pybind11/numpy.h>        // For NumPy arrays
#include "gridworldenvutils.hpp"

namespace py = pybind11;


PYBIND11_MODULE(_gridworldenvutils, m) {
    m.doc() = "Grid World Environment Utilities Module";

    m.def("check_fov", &gridworld::checkFOV, 
          py::arg("agent_pos"), 
          py::arg("target_pos"), 
          py::arg("agent_dir"), 
          py::arg("fov_angle"), 
          py::arg("fov_distance"), 
          py::arg("obstacles"),
          "Check if the target is within the agent's field of view.");

//     m.def("check_line_of_sight", &gridworld::checkLineOfSight, 
//           py::arg("agent_pos"), 
//           py::arg("target_pos"), 
//           py::arg("obstacles"),
//           "Check if there is a clear line of sight between the agent and the target.");

// Use a lambda to wrap checkLineOfSight
    m.def("check_line_of_sight",
        // This lambda function is what gets called from Python
        [](py::array_t<int> agent_pos_py, py::array_t<int> target_pos_py, const std::vector<std::vector<bool>>& obstacles) {
            // It converts the numpy arrays to Eigen vectors...
            Eigen::Vector2i agent_pos(agent_pos_py.at(0), agent_pos_py.at(1));
            Eigen::Vector2i target_pos(target_pos_py.at(0), target_pos_py.at(1));
            // ...and then calls your pure C++ function.
            return gridworld::checkLineOfSight(agent_pos, target_pos, obstacles);
        },
        py::arg("agent_pos"),
        py::arg("target_pos"),
        py::arg("obstacles"),
        "Check if there is a clear line of sight between the agent and the target.");


//     m.def("find_shortest_path", &gridworld::findShortestPath, 
//           py::arg("start"), 
//           py::arg("goal"), 
//           py::arg("obstacles"),
//           py::arg("allow_diagonal") = true,
//           "Find the shortest path from start to goal using A* algorithm.");
// Use a lambda to wrap findShortestPath
    m.def("find_shortest_path",
        [](py::array_t<int> start_py, py::array_t<int> goal_py, const std::vector<std::vector<bool>>& obstacles, bool allow_diagonal) {
            Eigen::Vector2i start(start_py.at(0), start_py.at(1));
            Eigen::Vector2i goal(goal_py.at(0), goal_py.at(1));
            return gridworld::findShortestPath(start, goal, obstacles, allow_diagonal);
        },
        py::arg("start"),
        py::arg("goal"),
        py::arg("obstacles"),
        // Pro-tip: py::arg_v automatically adds the default value to the docstring!
        py::arg_v("allow_diagonal", true),
        "Find the shortest path from start to goal using A* algorithm.");
}







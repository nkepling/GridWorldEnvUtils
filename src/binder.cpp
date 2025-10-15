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

    m.def("check_line_of_sight", &gridworld::checkLineOfSight, 
          py::arg("agent_pos"), 
          py::arg("target_pos"), 
          py::arg("obstacles"),
          "Check if there is a clear line of sight between the agent and the target.");


    m.def("find_shortest_path", &gridworld::findShortestPath, 
          py::arg("start"), 
          py::arg("goal"), 
          py::arg("obstacles"),
          py::arg("allow_diagonal") = true,
          "Find the shortest path from start to goal using A* algorithm.");

}





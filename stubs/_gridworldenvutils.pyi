"""
Grid World Environment Utilities Module
"""
from __future__ import annotations
import numpy
__all__: list[str] = ['check_fov', 'check_line_of_sight', 'find_shortest_path', 'get_visible_cells', 'run_backward_induction']
def check_fov(agent_pos: numpy.ndarray[numpy.int32[2, 1]], target_pos: numpy.ndarray[numpy.int32[2, 1]], agent_dir: numpy.ndarray[numpy.int32[2, 1]], fov_angle: float, fov_distance: float, obstacles: list[list[bool]]) -> bool:
    """
    Check if the target is within the agent's field of view.
    """
def check_line_of_sight(agent_pos: numpy.ndarray[numpy.int32], target_pos: numpy.ndarray[numpy.int32], obstacles: list[list[bool]]) -> bool:
    """
    Check if there is a clear line of sight between the agent and the target.
    """
def find_shortest_path(start: numpy.ndarray[numpy.int32], goal: numpy.ndarray[numpy.int32], obstacles: list[list[bool]], allow_diagonal: bool = True) -> list[numpy.ndarray[numpy.int32[2, 1]]]:
    """
    Find the shortest path from start to goal using A* algorithm.
    """
def get_visible_cells(agent_pos: numpy.ndarray[numpy.int32[2, 1]], agent_dir: numpy.ndarray[numpy.int32[2, 1]], fov_angle: float, fov_distance: float, obstacles: list[list[bool]]) -> list[numpy.ndarray[numpy.int32[2, 1]]]:
    """
    Get all visible cells from the agent's position within its field of view.
    """
def run_backward_induction(obstacles: list[list[bool]], evader_path: list[numpy.ndarray[numpy.int32[2, 1]]], gamma: float, stochasticity: float, fov_angle: float, fov_distance: float) -> list[int]:
    """
    Runs Time-Dependent Backward Induction to compute the optimal policy table.
    """

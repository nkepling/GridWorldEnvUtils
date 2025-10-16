"""
Grid World Environment Utilities Module
"""
from __future__ import annotations
import collections.abc
import numpy
import numpy.typing
import typing
__all__: list[str] = ['check_fov', 'check_line_of_sight', 'find_shortest_path']
def check_fov(agent_pos: typing.Annotated[numpy.typing.ArrayLike, numpy.int32, "[2, 1]"], target_pos: typing.Annotated[numpy.typing.ArrayLike, numpy.int32, "[2, 1]"], agent_dir: typing.Annotated[numpy.typing.ArrayLike, numpy.int32, "[2, 1]"], fov_angle: typing.SupportsFloat, fov_distance: typing.SupportsFloat, obstacles: collections.abc.Sequence[collections.abc.Sequence[bool]]) -> bool:
    """
    Check if the target is within the agent's field of view.
    """
def check_line_of_sight(agent_pos: typing.Annotated[numpy.typing.ArrayLike, numpy.int32], target_pos: typing.Annotated[numpy.typing.ArrayLike, numpy.int32], obstacles: collections.abc.Sequence[collections.abc.Sequence[bool]]) -> bool:
    """
    Check if there is a clear line of sight between the agent and the target.
    """
def find_shortest_path(start: typing.Annotated[numpy.typing.ArrayLike, numpy.int32], goal: typing.Annotated[numpy.typing.ArrayLike, numpy.int32], obstacles: collections.abc.Sequence[collections.abc.Sequence[bool]], allow_diagonal: bool = True) -> list[typing.Annotated[numpy.typing.NDArray[numpy.int32], "[2, 1]"]]:
    """
    Find the shortest path from start to goal using A* algorithm.
    """

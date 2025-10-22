import pytest 
import numpy as np


def test_shortest_path():
    try:
        from gridworldenvutils.path_finding import find_shortest_path
    except ImportError as e:
        pytest.fail(f"Failed to import functions from path_finding.py: {e}")

    # Example grid (0 = free, 1 = obstacle)
    grid = [
        [0, 0, 0, 1, 0],
        [1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0],
        [0, 1, 1, 1, 1],]
    
    start = (0, 0)  
    goal = (4, 2)

    path = find_shortest_path(start, goal, grid, allow_diagonal=False)
    expected_path = np.array([(0, 0), (1, 0), (2, 0), (2, 1), (2, 2), (3, 2), (4, 2)])

    assert np.array_equal(path, expected_path), f"Unexpected path: {expected_path}, got {path}"


    grid = [
        [0, 0, 0, 1, 0],
        [1, 1, 0, 1, 0], 
        [1, 1, 0, 0, 0],
        [1, 1, 1, 1, 1],]
    
    start = (0, 0)
    goal = (4, 0)

    expected_path = np.array([(0,0),(1,0),(2,0),(2,1),(2,2),(3,2),(4,2),(4,1),(4,0)])
    path = find_shortest_path(start, goal, grid, allow_diagonal=False)
    assert np.array_equal(path, expected_path), f"Unexpected path: {expected_path}, got {path}"

    

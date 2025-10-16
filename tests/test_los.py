import pytest
import numpy as np

def test_check_fov():
    try:
        # Assuming your binding code is correct, the import name is what you set in pybind11_add_module
        from gridworldenvutils.los import check_fov
    except ImportError as e:
        pytest.fail(f"Failed to import check_fov: {e}")

    # Example grid (0 = free, 1 = obstacle)
    grid_int = [
        [0, 0, 0, 1, 0],
        [1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0],
        [0, 1, 1, 1, 1],
    ]
    # Convert the integer grid to a boolean numpy array
    obstacles = np.array(grid_int, dtype=bool)

    # --- Test Case 1: Visible Target ---
    # Reshape arrays to be column vectors (2, 1)
    # agent_pos = np.array([[2], [2]])
    agent_pos = np.array([2,2])
    target_pos_visible = np.array([4,2])
    agent_dir = np.array([1,0])  # Facing right is (1, 0) in [x, y] format

    assert check_fov(agent_pos, target_pos_visible, agent_dir, fov_angle=90, fov_distance=5, obstacles=obstacles), "Target should be visible"

    # --- Test Case 2: Obstructed Target ---
    # Reusing the same grid
    agent_pos = np.array([2, 0])
    target_pos_obstructed = np.array([4, 0])
    agent_dir = np.array([1, 0])  # Facing right

    # Note: I renamed fov_distance to match the C++ argument name
    assert not check_fov(agent_pos, target_pos_obstructed, agent_dir, fov_angle=90, fov_distance=10, obstacles=obstacles), "Target should be obstructed and NOT be visible"
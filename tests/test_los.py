import pytest
import numpy as np

def test_check_fov():
    try:
        from gridworldenvutils.los import check_fov
    except ImportError as e:
        pytest.fail(f"Failed to import check_fov: {e}")


    grid_int = [
        [0, 0, 0, 1, 0],
        [1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0],
        [0, 1, 1, 1, 1],
    ]

    obstacles = np.array(grid_int, dtype=bool)


    agent_pos = np.array([2,2])
    target_pos_visible = np.array([4,2])
    agent_dir = np.array([1,0])  # Facing right is (1, 0) in [x, y] format

    assert check_fov(agent_pos, target_pos_visible, agent_dir, fov_angle=90, fov_distance=5, obstacles=obstacles), "Target should be visible"

    # --- Test Case 2: Obstructed Target ---
    agent_pos = np.array([2, 0])
    target_pos_obstructed = np.array([4, 0])
    agent_dir = np.array([1, 0])  # Facing right

    assert not check_fov(agent_pos, target_pos_obstructed, agent_dir, fov_angle=90, fov_distance=10, obstacles=obstacles), "Target should be obstructed and NOT be visible"


def test_visible_cells():
    try:
        from gridworldenvutils.los import get_visible_cells
    except ImportError as e:
        pytest.fail(f"Failed to import get_visible_cells: {e}")

    # Example grid (0 = free, 1 = obstacle)
    grid_int = [
        [0, 0, 0, 1, 0],
        [1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0],
        [0, 1, 1, 1, 1],
    ]

    obstacles = np.array(grid_int, dtype=bool)

    agent_pos = np.array([0,0])
    agent_dir = np.array([1,0])  # Facing right is (1, 0) in [x, y] format

    visible_cells = get_visible_cells(agent_pos, agent_dir, fov_angle=2, fov_distance=5, obstacles=obstacles)

    # Convert list of Eigen::Vector2i to a set of tuples for easier comparison
    visible_set = set((cell[0], cell[1]) for cell in visible_cells)

    expected_visible = {
        (0,0), (1,0), (2,0), # Directly in front
    }

    assert visible_set == expected_visible, f"Visible cells do not match expected. Got {visible_set}, expected {expected_visible}"

    agent_pos = np.array([0,2])
    agent_dir = np.array([1,0]) 

    visible_cells = get_visible_cells(agent_pos, agent_dir, fov_angle=2, fov_distance=10, obstacles=obstacles)

    expected_visible = {
        (0,2), (1,2), (2,2), (3,2), (4,2), # Directly in front
    }
    visible_set = set((cell[0], cell[1]) for cell in visible_cells)
    assert visible_set == expected_visible, f"Visible cells do not match expected. Got {visible_set}, expected {expected_visible}"

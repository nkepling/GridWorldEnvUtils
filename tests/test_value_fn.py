import numpy as np
import pytest
import gridworldenvutils

def test_backward_induction_runs():
    """
    Smoke test for run_backward_induction to ensure it doesn't crash.
    """
    # Simple 5x5 map
    obstacles = [[False] * 5 for _ in range(5)]
    
    # Dummy evader path: (0,0) -> (0,1) -> (0,2)
    evader_path = [np.array([0, 0]), np.array([0, 1]), np.array([0, 2])]

    policy = gridworldenvutils.solvers.run_backward_induction(
        obstacles,
        evader_path,
        0.99, # Gamma
        0.1,  # Stochasticity
        1.57, # FOV Angle
        5.0   # FOV Dist
    )
    
    # Expect policy table size: T * Rows * Cols * Dirs
    # T=3, Rows=5, Cols=5, Dirs=8 -> 3*5*5*8 = 600
    assert len(policy) == 600


def test_backward_induction_q_values():
    """
    Test for run_backward_induction_q to ensure it returns Q-values of correct shape.
    """
    # Simple 5x5 map
    obstacles = [[False] * 5 for _ in range(5)]
    
    # Dummy evader path: (0,0) -> (0,1) -> (0,2)
    evader_path = [np.array([0, 0]), np.array([0, 1]), np.array([0, 2])]

    q_values = gridworldenvutils.solvers.run_backward_induction_q(
        obstacles,
        evader_path,
        0.99, # Gamma
        0.1,  # Stochasticity
        1.57, # FOV Angle
        5.0,  # FOV Dist
        start_r=0,
        start_d=0,
        start_c=0,
        fov_lookahead=1
    )
    
    # Expect Q-values shape: (Num_Actions,)
    num_actions = 11  

    assert len(q_values) == num_actions
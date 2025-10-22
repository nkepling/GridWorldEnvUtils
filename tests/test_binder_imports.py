import pytest
import sys
import os


def test_import_cpp_module():
    try:
        import gridworldenvutils
    except ImportError as e:
        pytest.fail(f"Failed to import C++ module: {e}")    

    try:
        from gridworldenvutils.los import check_fov, check_line_of_sight , get_visible_cells

    except ImportError as e:
        pytest.fail(f"Failed to import functions from los.py: {e}")


def test_import_path_finding():
    try:
        from gridworldenvutils.path_finding import find_shortest_path
    except ImportError as e:
        pytest.fail(f"Failed to import functions from path_finding.py: {e}")




import pytest

from warehouse_mapf.sim.grid import Grid
from warehouse_mapf.sim.validation import PlanError, check_paths, check_step

GRID = Grid.from_lines(["....", ".@..", "...."])


def test_valid_step_passes():
    check_step(GRID, [(0, 0), (0, 2)], [(0, 1), (0, 3)])


def test_following_is_allowed():
    check_step(GRID, [(0, 0), (0, 1)], [(0, 1), (0, 2)])


def test_vertex_conflict():
    with pytest.raises(PlanError, match="vertex"):
        check_step(GRID, [(0, 0), (0, 2)], [(0, 1), (0, 1)])


def test_swap_conflict():
    with pytest.raises(PlanError, match="swap"):
        check_step(GRID, [(0, 0), (0, 1)], [(0, 1), (0, 0)])


def test_obstacle_and_jump():
    with pytest.raises(PlanError, match="blocked"):
        check_step(GRID, [(0, 1)], [(1, 1)])
    with pytest.raises(PlanError, match="jumps"):
        check_step(GRID, [(0, 0)], [(0, 2)])


def test_check_paths_pads_short_paths():
    # Robot 1 finishes at (0, 2) and waits; robot 0 runs into it at t=2.
    with pytest.raises(PlanError, match="step 1->2"):
        check_paths(GRID, [[(0, 0), (0, 1), (0, 2)], [(0, 3), (0, 2)]])

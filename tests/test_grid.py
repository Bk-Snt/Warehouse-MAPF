from pathlib import Path

import numpy as np
import pytest

from warehouse_mapf.sim import Grid

MAPS = Path(__file__).resolve().parent.parent / "maps"


def test_from_lines_marks_obstacles():
    grid = Grid.from_lines(["..@", "...", "@.."])
    assert (grid.height, grid.width) == (3, 3)
    assert grid.is_free((0, 0))
    assert not grid.is_free((0, 2))
    assert not grid.is_free((2, 0))


def test_out_of_bounds_is_not_free():
    grid = Grid.from_lines(["..", ".."])
    assert not grid.is_free((-1, 0))
    assert not grid.is_free((0, 2))


def test_neighbors_skip_obstacles_and_edges():
    grid = Grid.from_lines([".@.", "...", "..."])
    assert grid.neighbors((0, 0)) == [(1, 0)]
    assert grid.neighbors((1, 1)) == [(1, 2), (2, 1), (1, 0)]


def test_rows_must_have_equal_width():
    with pytest.raises(ValueError):
        Grid.from_lines(["...", ".."])


def test_distances_from_goes_around_obstacles():
    grid = Grid.from_lines(["...", ".@.", "..."])
    dist = grid.distances_from((0, 0))
    assert dist[2, 2] == 4
    assert dist[1, 1] == -1


def test_disconnected_map_detected():
    grid = Grid.from_lines([".@.", ".@.", ".@."])
    assert not grid.is_connected()


def test_small_warehouse_map_loads_and_is_connected():
    grid = Grid.from_movingai(MAPS / "warehouse-small-32x32.map")
    assert (grid.height, grid.width) == (32, 32)
    assert grid.is_connected()
    assert np.count_nonzero(~grid.free) > 0

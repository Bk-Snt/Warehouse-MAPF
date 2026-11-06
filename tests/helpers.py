"""Shared test helpers."""

from warehouse_mapf.sim.distances import DistanceCache
from warehouse_mapf.sim.grid import Grid
from warehouse_mapf.sim.simulator import SimView
from warehouse_mapf.sim.validation import check_step


def episode(planner, lines, starts, goals, steps=30, busy=None):
    """One-shot run: robots go from starts to goals; every step is checked."""
    grid = Grid.from_lines(lines)
    distances = DistanceCache(grid)
    pos = list(starts)
    busy = tuple(busy) if busy else (False,) * len(pos)
    for t in range(steps):
        view = SimView(t, grid, distances, tuple(pos), tuple(goals), busy)
        nxt = planner.step(view)
        check_step(grid, pos, nxt)
        pos = nxt
    return pos

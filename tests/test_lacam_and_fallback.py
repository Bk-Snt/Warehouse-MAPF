import itertools
from collections import deque
from pathlib import Path

import numpy as np
from helpers import episode

from warehouse_mapf.planner.fallback import DeadlockFallbackPlanner
from warehouse_mapf.planner.lacam import LaCAM
from warehouse_mapf.planner.pibt import PIBTPlanner
from warehouse_mapf.sim.distances import DistanceCache
from warehouse_mapf.sim.grid import Grid
from warehouse_mapf.sim.simulator import Simulator, SimView
from warehouse_mapf.sim.validation import check_paths, find_conflicts
from warehouse_mapf.sim.warehouse import Warehouse

MAPS = Path(__file__).resolve().parent.parent / "maps"
POCKET = ["@@.@@", "....."]


def solve(lines, starts, goals, **kwargs):
    grid = Grid.from_lines(lines)
    solver = LaCAM(grid, goals, [grid.distances_from(g) for g in goals], **kwargs)
    return grid, solver.solve(starts, lambda q: list(q) == goals)


def reachable_configs(grid, start):
    """Brute force: every configuration reachable from ``start``."""
    seen, queue = {tuple(start)}, deque([tuple(start)])
    while queue:
        config = queue.popleft()
        for nxt in itertools.product(*[[c, *grid.neighbors(c)] for c in config]):
            if nxt not in seen and not find_conflicts(config, nxt):
                seen.add(nxt)
                queue.append(nxt)
    return seen


def test_lacam_solves_the_pocket_corridor():
    starts, goals = [(1, 0), (1, 4)], [(1, 4), (1, 0)]
    grid, result = solve(POCKET, starts, goals)
    assert result.configs is not None
    assert list(result.configs[0]) == starts and list(result.configs[-1]) == goals
    check_paths(grid, [[q[i] for q in result.configs] for i in range(2)])


def test_lacam_is_complete_it_proves_there_is_no_solution():
    # Reversing four robots in a corridor with one pocket is impossible.
    lines = ["@@@@@@@", ".......", "@@.@@@@"]
    starts = [(1, 1), (1, 2), (1, 3), (1, 4)]
    goals = starts[::-1]
    grid, result = solve(lines, starts, goals, stay_at_goal=False)
    assert result.configs is None
    assert result.nodes == len(reachable_configs(grid, starts))  # searched everything


def test_lacam_respects_blocked_cells():
    grid = Grid.from_lines(["...", "..."])
    goals = [(0, 2)]
    solver = LaCAM(grid, goals, [grid.distances_from(goals[0])], blocked={(0, 1)})
    result = solver.solve([(0, 0)], lambda q: q[0] == goals[0])
    assert result.configs is not None
    assert all(q[0] != (0, 1) for q in result.configs)


def test_lacam_stops_at_its_time_limit():
    grid = Grid.from_lines(["@@@@@@@", ".......", "@@.@@@@"])
    starts = [(1, 1), (1, 2), (1, 3), (1, 4)]
    goals = starts[::-1]
    solver = LaCAM(grid, goals, [grid.distances_from(g) for g in goals], stay_at_goal=False)
    result = solver.solve(starts, lambda q: list(q) == goals, time_limit=0.0)
    assert result.configs is None and result.nodes <= 2


def test_fallback_solves_the_pocket_corridor_that_pibt_cannot():
    starts, goals = [(1, 0), (1, 4)], [(1, 4), (1, 0)]
    assert episode(PIBTPlanner(), POCKET, starts, goals, steps=80) != goals
    planner = DeadlockFallbackPlanner(stuck_after=5)
    assert episode(planner, POCKET, starts, goals, steps=80) == goals
    assert planner.solved >= 1


def test_queueing_for_an_occupied_goal_is_not_stuck():
    grid = Grid.from_lines(["....."])
    distances = DistanceCache(grid)
    planner = DeadlockFallbackPlanner(stuck_after=3)
    # Robot 1 is being served at robot 0's goal; robot 0 waits next to it.
    for t in range(10):
        view = SimView(t, grid, distances, ((0, 2), (0, 3)), ((0, 3), (0, 3)), (False, True))
        planner.step(view)
    assert planner.triggers == 0


def test_fixed_moves_and_blocked_cells_are_respected():
    grid = Grid.from_lines(["....."])
    view = SimView(0, grid, DistanceCache(grid), ((0, 0), (0, 4)), ((0, 4), (0, 0)), (False,) * 2)
    planner = PIBTPlanner()
    planner.observe(view)
    nxt = planner.plan_step(view, planner.priorities, fixed={1: (0, 3)}, blocked={(0, 1)})
    assert nxt == [(0, 0), (0, 3)]


def test_fallback_on_narrow_warehouse_runs_safely():
    layout = Warehouse.load(MAPS / "warehouse-narrow-32x32.json")
    sim = Simulator(layout, num_robots=40, seed=1)  # strict: any conflict raises
    planner = DeadlockFallbackPlanner(seed=1, stuck_after=30)
    sim.run(planner, steps=300)
    assert sim.metrics.tasks_completed > 20
    assert np.isfinite(sim.metrics.throughput)

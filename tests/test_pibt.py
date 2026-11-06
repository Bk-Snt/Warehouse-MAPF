import numpy as np
from helpers import episode

from warehouse_mapf.planner.pibt import PIBTPlanner
from warehouse_mapf.sim.distances import DistanceCache
from warehouse_mapf.sim.grid import Grid
from warehouse_mapf.sim.simulator import Simulator, SimView


def test_head_on_in_two_lanes():
    goals = [(1, 4), (1, 0)]
    assert episode(PIBTPlanner(), [".....", "....."], [(1, 0), (1, 4)], goals) == goals


def test_robot_at_its_goal_is_pushed_aside_and_returns():
    # Robot 1 sits on its goal in the corridor; robot 0 has to get past it.
    goals = [(1, 4), (1, 2)]
    assert episode(PIBTPlanner(), ["..@..", "....."], [(1, 0), (1, 2)], goals, steps=20) == goals


def test_busy_robot_is_never_moved():
    starts = [(0, 0), (0, 1)]
    final = episode(PIBTPlanner(), ["..."], starts, [(0, 2), (0, 0)], steps=5, busy=(False, True))
    assert final == starts  # robot 0 waits behind the busy robot


def test_known_limitation_corridor_with_side_pocket():
    """PIBT's guarantees need a biconnected map; a dead-end pocket breaks them.

    Same case as the WHCA* limitation test. The LaCAM fallback (Phase 3) is
    meant to solve it.
    """
    goals = [(1, 4), (1, 0)]
    assert episode(PIBTPlanner(), ["@@.@@", "....."], [(1, 0), (1, 4)], goals, steps=60) != goals


def test_priority_grows_while_working_and_resets_after_errand():
    grid = Grid.from_lines(["....."])
    distances = DistanceCache(grid)
    planner = PIBTPlanner(seed=0)

    def view(t, pos, goal):
        return SimView(t, grid, distances, (pos,), (goal,), (False,))

    planner.step(view(0, (0, 0), (0, 4)))
    start = planner.priorities[0]
    planner.step(view(1, (0, 1), (0, 4)))
    planner.step(view(2, (0, 2), (0, 4)))
    assert planner.priorities[0] == start + 2
    planner.step(view(3, (0, 3), (0, 0)))  # new errand
    assert planner.priorities[0] == start
    assert 0 <= start < 1


def test_lifelong_40_robots_keeps_flowing_without_starvation(small_layout):
    sim = Simulator(small_layout, num_robots=40, seed=0)
    planner = PIBTPlanner(seed=0)
    per_chunk = []
    for _ in range(3):
        before = sim.metrics.tasks_completed
        sim.run(planner, steps=150)  # every step is collision-checked
        per_chunk.append(sim.metrics.tasks_completed - before)
    assert min(per_chunk) >= 40  # no gridlock: tasks keep finishing
    assert sim.metrics.max_errand_time < 120  # no robot waits indefinitely
    assert np.percentile(sim.metrics.errand_times, 99) < 100

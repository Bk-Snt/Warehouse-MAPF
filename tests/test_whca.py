import pytest

from warehouse_mapf.baselines.greedy import GreedyPlanner
from warehouse_mapf.baselines.whca import ReservationTable, WHCAStarPlanner, space_time_astar
from warehouse_mapf.sim.distances import DistanceCache
from warehouse_mapf.sim.grid import Grid
from warehouse_mapf.sim.simulator import Simulator, SimView
from warehouse_mapf.sim.validation import check_paths, check_step


def episode(planner, lines, starts, goals, steps=30):
    """One-shot run: robots go from starts to goals; every step is checked."""
    grid = Grid.from_lines(lines)
    distances = DistanceCache(grid)
    pos = list(starts)
    for t in range(steps):
        view = SimView(t, grid, distances, tuple(pos), tuple(goals), (False,) * len(pos))
        nxt = planner.step(view)
        check_step(grid, pos, nxt)
        pos = nxt
    return pos


def test_astar_goes_straight_then_waits_at_goal():
    grid = Grid.from_lines(["....."])
    path = space_time_astar(
        grid, (0, 0), (0, 4), grid.distances_from((0, 4)), ReservationTable(), window=6
    )
    assert path == [(0, 0), (0, 1), (0, 2), (0, 3), (0, 4), (0, 4), (0, 4)]


def test_astar_avoids_reserved_cells_and_swaps():
    grid = Grid.from_lines(["...", "...", "..."])
    table = ReservationTable()
    other = [(0, 2), (0, 1), (0, 0), (0, 0)]  # moves left along the top row
    table.reserve_path(0, other)
    path = space_time_astar(grid, (0, 0), (0, 2), grid.distances_from((0, 2)), table, window=3)
    assert path is not None
    check_paths(grid, [other, path])


def test_astar_returns_none_when_boxed_in():
    grid = Grid.from_lines(["..."])
    table = ReservationTable()
    table.reserve_path(0, [(0, 1), (0, 0)])  # takes our cell; swapping is not allowed
    assert (
        space_time_astar(grid, (0, 0), (0, 2), grid.distances_from((0, 2)), table, window=2) is None
    )


def test_head_on_in_two_lanes_greedy_deadlocks_whca_solves():
    lanes = [".....", "....."]
    starts, goals = [(1, 0), (1, 4)], [(1, 4), (1, 0)]
    assert episode(GreedyPlanner(), lanes, starts, goals) != goals
    assert episode(WHCAStarPlanner(), lanes, starts, goals) == goals


def test_known_limitation_corridor_with_side_pocket():
    """A solution exists (one robot ducks into the pocket), but WHCA* deadlocks.

    The higher-priority robot books the corridor; the other can't reach the
    pocket in time, so it waits and the two meet head-on. The hybrid
    planner's LaCAM fallback (Phase 3) is meant to solve cases like this.
    """
    pocket = ["@@.@@", "....."]
    starts, goals = [(1, 0), (1, 4)], [(1, 4), (1, 0)]
    assert episode(WHCAStarPlanner(), pocket, starts, goals, steps=60) != goals


def test_lifelong_run_beats_greedy_without_collisions(small_layout):
    def tasks(planner):
        sim = Simulator(small_layout, num_robots=15, seed=2)
        return sim.run(planner, steps=200).tasks_completed  # every step is checked

    assert tasks(WHCAStarPlanner()) > 2 * tasks(GreedyPlanner())


def test_longer_replan_period_runs_safely(small_layout):
    sim = Simulator(small_layout, num_robots=15, seed=4)
    planner = WHCAStarPlanner(window=12, replan_every=4, priority="shuffle", seed=4)
    sim.run(planner, steps=150)
    assert planner.replans < 150


@pytest.mark.parametrize("kwargs", [{"window": 4, "replan_every": 5}, {"priority": "random"}])
def test_bad_parameters_rejected(kwargs):
    with pytest.raises(ValueError):
        WHCAStarPlanner(**kwargs)

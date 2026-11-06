import numpy as np
import pytest

from warehouse_mapf.planner.guidance import CongestionGuidance
from warehouse_mapf.planner.pibt import PIBTPlanner
from warehouse_mapf.sim.grid import Grid
from warehouse_mapf.sim.simulator import Simulator

GRID = Grid.from_lines([".....", ".@.@.", "....."])


def test_without_traffic_tables_equal_shortest_paths():
    guidance = CongestionGuidance(GRID)
    goal = (2, 4)
    expected = GRID.distances_from(goal).astype(float)
    expected[expected < 0] = np.inf
    np.testing.assert_allclose(guidance.table(goal), expected)


def test_moves_are_recorded_by_direction():
    guidance = CongestionGuidance(GRID, decay=0.5)
    guidance.observe([(0, 0)], t=0)
    guidance.observe([(0, 1)], t=1)  # moved east
    assert guidance.flow[0, 0, 1] == pytest.approx(0.5)
    assert guidance.flow[0, 0].sum() == pytest.approx(0.5)
    assert guidance.density[0, 1] > 0


def test_driving_against_traffic_costs_more():
    guidance = CongestionGuidance(GRID, opposing_weight=3.0, density_weight=0.0, decay=0.5)
    for t in range(6):  # a stream of robots driving west along the top row
        guidance.observe([(0, 4 - (t % 4)), (2, 0)], t)
    assert guidance.move_cost((0, 1), (0, 2)) > guidance.move_cost((0, 2), (0, 1))


def test_traffic_steers_route_to_the_quiet_row():
    guidance = CongestionGuidance(GRID, opposing_weight=20.0, density_weight=0.0, decay=0.9)
    for t in range(30):  # steady westbound traffic on the top row
        guidance.observe([(0, 4 - (t % 5))], t)
    table = guidance.table((0, 4))
    start = (0, 0)
    via_top = guidance.move_cost(start, (0, 1)) + table[0, 1]
    via_bottom = guidance.move_cost(start, (1, 0)) + table[1, 0]
    assert via_bottom < via_top  # heading east, the empty bottom row is now cheaper


def test_tables_rebuild_after_refresh_period():
    guidance = CongestionGuidance(GRID, refresh_every=5)
    guidance.table((0, 0))
    guidance.observe([(1, 0)], t=3)
    guidance.table((0, 0))
    assert guidance.rebuilds == 1
    guidance.observe([(1, 0)], t=5)
    guidance.table((0, 0))
    assert guidance.rebuilds == 2


@pytest.mark.parametrize("kwargs", [{"decay": 1.0}, {"refresh_every": 0}])
def test_bad_parameters_rejected(kwargs):
    with pytest.raises(ValueError):
        CongestionGuidance(GRID, **kwargs)


def test_guided_pibt_lifelong_run(small_layout):
    sim = Simulator(small_layout, num_robots=40, seed=1, assignment="balanced")
    planner = PIBTPlanner(seed=1, guidance=True)
    metrics = sim.run(planner, steps=200)  # every step is collision-checked
    assert metrics.tasks_completed > 50
    assert planner.guidance is not None and planner.guidance.rebuilds > 0

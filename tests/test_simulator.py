import pytest

from warehouse_mapf.baselines.greedy import GreedyPlanner
from warehouse_mapf.run import main
from warehouse_mapf.sim.simulator import Simulator
from warehouse_mapf.sim.validation import PlanError


def test_single_robot_completes_tasks(tiny_layout):
    sim = Simulator(tiny_layout, num_robots=1, seed=0)
    metrics = sim.run(GreedyPlanner(), steps=300)
    assert metrics.tasks_completed >= 5
    assert metrics.wait_steps == 0


def test_robot_must_stay_while_served(tiny_layout):
    sim = Simulator(tiny_layout, num_robots=1, seed=0)
    planner = GreedyPlanner()
    while not sim.robots[0].busy:
        sim.step(planner.step(sim.view()))
    r, c = sim.robots[0].pos
    move = next(n for n in sim.grid.neighbors((r, c)))
    with pytest.raises(PlanError, match="while being served"):
        sim.step([move])


def test_service_time_is_respected(tiny_layout):
    sim = Simulator(tiny_layout, num_robots=1, seed=0)
    planner = GreedyPlanner()
    while not sim.robots[0].busy:
        sim.step(planner.step(sim.view()))
    errand = sim.robots[0].errand
    for _ in range(errand.service_time - 1):
        sim.step(planner.step(sim.view()))
        assert sim.robots[0].errand == errand
    sim.step(planner.step(sim.view()))
    assert sim.robots[0].errand != errand


def test_many_robots_run_without_collisions(small_layout):
    sim = Simulator(small_layout, num_robots=30, seed=3, record=True)
    metrics = sim.run(GreedyPlanner(), steps=300)  # every step is collision-checked
    assert metrics.steps == 300
    assert len(sim.history) == 301
    assert metrics.tasks_completed > 0


def test_cli_runs_and_saves_log(tmp_path):
    log = tmp_path / "run.npz"
    result = main(["--robots", "5", "--steps", "50", "--log", str(log)])
    assert result["steps"] == 50
    assert log.exists()


def test_station_load_counts_remaining_station_errands(small_layout):
    sim = Simulator(small_layout, num_robots=12, seed=0)
    load = sim.station_load()
    # Each robot starts with an inbound and a picking/outbound stop ahead of it.
    assert sum(load.values()) == 2 * 12
    assert set(load) <= small_layout.station_cells


def test_errand_load_counts_shelves_too(small_layout):
    sim = Simulator(small_layout, num_robots=12, seed=0)
    assert sum(sim.errand_load().values()) == 3 * 12  # inbound, shelf, delivery

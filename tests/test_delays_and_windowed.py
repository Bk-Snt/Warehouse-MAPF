from warehouse_mapf.planner.pibt import PIBTPlanner, rotation_members
from warehouse_mapf.planner.windowed import WindowedPlanner
from warehouse_mapf.sim.simulator import Simulator
from warehouse_mapf.sim.validation import check_paths, find_conflicts, stop_conflicting


def test_emergency_stop_cascades_to_followers():
    current = [(0, 0), (0, 1), (0, 2)]
    nxt = [(0, 1), (0, 2), (0, 3)]  # a convoy moving east
    fixed, stopped = stop_conflicting(current, nxt, must_stay={2})  # the leader is delayed
    assert fixed == current
    assert stopped == 2
    assert not find_conflicts(current, fixed)


def test_rotation_detection():
    square = [(0, 0), (0, 1), (1, 1), (1, 0)]
    turned = [(0, 1), (1, 1), (1, 0), (0, 0)]
    assert rotation_members(square, turned) == {0, 1, 2, 3}
    assert rotation_members([(0, 0), (0, 1)], [(0, 1), (0, 2)]) == set()


def test_full_delay_means_nobody_moves(small_layout):
    sim = Simulator(small_layout, num_robots=10, seed=0, delay_prob=1.0)
    start = [r.pos for r in sim.robots]
    sim.run(PIBTPlanner(), steps=5)
    assert [r.pos for r in sim.robots] == start
    assert sim.metrics.delayed_moves > 0


def test_delays_do_not_change_the_tasks(small_layout):
    plain = Simulator(small_layout, num_robots=10, seed=3)
    delayed = Simulator(small_layout, num_robots=10, seed=3, delay_prob=0.3)
    assert [r.task for r in plain.robots] == [r.task for r in delayed.robots]


def test_plain_pibt_collides_under_delays(small_layout):
    sim = Simulator(small_layout, num_robots=60, seed=0, delay_prob=0.1)
    sim.run(PIBTPlanner(), steps=150)
    assert sim.metrics.collisions > 0


def test_rollout_is_valid_and_rotation_free(small_layout):
    sim = Simulator(small_layout, num_robots=80, seed=1)
    planner = PIBTPlanner(seed=1)
    sim.run(planner, steps=40)
    planner.observe(sim.view())
    paths = planner.rollout(sim.view(), steps=8)
    assert all(len(p) == 9 for p in paths)
    check_paths(small_layout.grid, paths)
    for k in range(8):
        assert not rotation_members([p[k] for p in paths], [p[k + 1] for p in paths])


def test_naive_window_execution_without_delays_never_collides(small_layout):
    sim = Simulator(small_layout, num_robots=60, seed=0)  # strict: conflicts raise
    sim.run(WindowedPlanner(PIBTPlanner(seed=0), adg=False), steps=200)
    assert sim.metrics.collisions == 0
    assert sim.metrics.tasks_completed > 50


def test_adg_execution_never_collides_under_delays(small_layout):
    # strict=True: any conflicting command would raise PlanError.
    sim = Simulator(small_layout, num_robots=60, seed=2, delay_prob=0.2, stall_prob=0.01)
    metrics = sim.run(WindowedPlanner(PIBTPlanner(seed=2), adg=True), steps=200)
    assert metrics.delayed_moves > 100
    assert metrics.collisions == 0
    assert metrics.tasks_completed > 40

import pytest

from warehouse_mapf.sim.adg import ActionDependencyGraph


def test_waits_are_dropped_and_moves_kept_in_order():
    graph = ActionDependencyGraph([[(0, 0), (0, 0), (0, 1), (0, 2)]])
    assert [(m.frm, m.to, m.time) for m in graph.moves[0]] == [
        ((0, 0), (0, 1), 1),
        ((0, 1), (0, 2), 2),
    ]


def test_follower_waits_for_leader_to_leave():
    # Robot 0 drives east; robot 1 follows one cell behind.
    leader = [(0, 1), (0, 2), (0, 3)]
    follower = [(0, 0), (0, 1), (0, 2)]
    graph = ActionDependencyGraph([leader, follower])
    assert graph.depends_on[1][0] == (0, 0)  # enter (0,1) after leader leaves it
    assert graph.ready(0)
    assert not graph.ready(1)
    graph.done[0] = 1  # leader has left (0, 1)
    assert graph.ready(1)


def test_crossing_order_is_preserved():
    # Robot 0 crosses (1, 1) first, then robot 1 crosses it.
    first = [(1, 0), (1, 1), (1, 2), (1, 2)]
    second = [(0, 1), (0, 1), (1, 1), (2, 1)]
    graph = ActionDependencyGraph([first, second])
    assert graph.depends_on[1][0] == (0, 1)  # wait until robot 0 has left (1, 1)


def test_plan_where_a_robot_never_leaves_is_rejected():
    with pytest.raises(ValueError, match="never leaves"):
        ActionDependencyGraph([[(0, 1), (0, 1)], [(0, 0), (0, 1)]])


def test_finished_after_all_moves():
    graph = ActionDependencyGraph([[(0, 0), (0, 1)], [(1, 0), (1, 0)]])
    assert not graph.finished()
    graph.done[0] = 1
    assert graph.finished()
    assert graph.next_move(0) is None and not graph.ready(0)

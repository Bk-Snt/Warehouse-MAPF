import numpy as np
import pytest

from warehouse_mapf.sim.distances import DistanceCache
from warehouse_mapf.sim.grid import Grid
from warehouse_mapf.sim.warehouse import Station, Warehouse
from warehouse_mapf.tasks import TaskGenerator


def test_layout_loads_with_stations_and_storage(small_layout):
    assert len(small_layout.stations_of("inbound")) == 4
    assert len(small_layout.stations_of("picking")) == 3
    assert len(small_layout.stations_of("outbound")) == 1
    assert small_layout.storage_cells
    assert not set(small_layout.storage_cells) & small_layout.station_cells


def test_station_on_obstacle_rejected():
    grid = Grid.from_lines(["..@", "..."])
    with pytest.raises(ValueError, match="not a free cell"):
        Warehouse(grid, [Station(0, "inbound", (0, 2), 1), Station(1, "picking", (1, 0), 1)])


def test_task_has_inbound_storage_and_delivery(small_layout):
    gen = TaskGenerator(small_layout, DistanceCache(small_layout.grid), np.random.default_rng(0))
    task = gen.next_task((10, 10), t=0)
    kinds = [e.kind for e in task.errands]
    assert kinds[:2] == ["inbound", "storage"]
    assert kinds[2] in ("picking", "outbound")
    assert task.errands[1].cell in small_layout.storage_cells


def test_inbound_is_nearest_to_robot(small_layout):
    gen = TaskGenerator(small_layout, DistanceCache(small_layout.grid), np.random.default_rng(0))
    assert gen.next_task((27, 2), t=0).errands[0].cell == (28, 0)


def test_tasks_are_reproducible_and_skewed(small_layout):
    def products(seed):
        gen = TaskGenerator(
            small_layout,
            DistanceCache(small_layout.grid),
            np.random.default_rng(seed),
            popularity_skew=1.5,
        )
        return [gen.next_task((10, 10), t).errands[1].cell for t in range(300)]

    assert products(1) == products(1)
    counts = sorted((products(1).count(c) for c in set(products(1))), reverse=True)
    assert counts[0] > 300 / len(small_layout.storage_cells) * 10


def test_balanced_assignment_avoids_a_busy_station(small_layout):
    gen = TaskGenerator(
        small_layout,
        DistanceCache(small_layout.grid),
        np.random.default_rng(0),
        assignment="balanced",
    )
    near = gen.next_task((27, 2), t=0).errands[0].cell
    assert near == (28, 0)
    crowded = {(28, 0): 10}  # ten robots already bound for the nearest inbound
    assert gen.next_task((27, 2), t=0, load=crowded).errands[0].cell != (28, 0)


def test_nearest_assignment_ignores_load(small_layout):
    gen = TaskGenerator(small_layout, DistanceCache(small_layout.grid), np.random.default_rng(0))
    assert gen.next_task((27, 2), t=0, load={(28, 0): 10}).errands[0].cell == (28, 0)


def test_unknown_assignment_rejected(small_layout):
    with pytest.raises(ValueError):
        TaskGenerator(
            small_layout,
            DistanceCache(small_layout.grid),
            np.random.default_rng(0),
            assignment="random",
        )

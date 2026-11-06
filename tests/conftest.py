from pathlib import Path

import pytest

from warehouse_mapf.sim.grid import Grid
from warehouse_mapf.sim.warehouse import Station, Warehouse

MAPS = Path(__file__).resolve().parent.parent / "maps"


@pytest.fixture
def small_layout() -> Warehouse:
    return Warehouse.load(MAPS / "warehouse-small-32x32.json")


@pytest.fixture
def tiny_layout() -> Warehouse:
    """7x9 warehouse: one shelf row, inbound on the left, picking on the right."""
    grid = Grid.from_lines(
        [
            ".........",
            ".........",
            "..@@@@@..",
            ".........",
            ".........",
            "..@@@@@..",
            ".........",
        ]
    )
    stations = [
        Station(0, "inbound", (3, 0), service_time=2),
        Station(1, "picking", (3, 8), service_time=3),
    ]
    return Warehouse(grid=grid, stations=stations, shelf_service_time=1)

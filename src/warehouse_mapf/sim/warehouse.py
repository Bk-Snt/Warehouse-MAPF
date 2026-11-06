"""Warehouse layout: the grid plus inbound, picking and outbound stations."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

from warehouse_mapf.sim.grid import Cell, Grid

STATION_KINDS = ("inbound", "picking", "outbound")


@dataclass(frozen=True)
class Station:
    id: int
    kind: str
    cell: Cell
    service_time: int

    def __post_init__(self) -> None:
        if self.kind not in STATION_KINDS:
            raise ValueError(f"unknown station kind {self.kind!r}")
        if self.service_time < 0:
            raise ValueError("service_time must be >= 0")


@dataclass
class Warehouse:
    """A grid with stations. Products live at storage cells beside shelves."""

    grid: Grid
    stations: list[Station]
    shelf_service_time: int = 2
    name: str = "warehouse"
    storage_cells: list[Cell] = field(init=False)

    def __post_init__(self) -> None:
        cells = [s.cell for s in self.stations]
        if len(set(cells)) != len(cells):
            raise ValueError("two stations share a cell")
        for s in self.stations:
            if not self.grid.is_free(s.cell):
                raise ValueError(f"station {s.id} at {s.cell} is not a free cell")
        if not self.stations_of("inbound"):
            raise ValueError("layout needs at least one inbound station")
        if not self.stations_of("picking") and not self.stations_of("outbound"):
            raise ValueError("layout needs at least one picking or outbound station")
        station_cells = set(cells)
        self.storage_cells = [
            cell
            for cell in self.grid.free_cells()
            if cell not in station_cells and self._faces_shelf(cell)
        ]
        if not self.storage_cells:
            raise ValueError("layout has no storage cells (free cells beside a shelf)")

    def _faces_shelf(self, cell: Cell) -> bool:
        r, c = cell
        for dr, dc in ((-1, 0), (0, 1), (1, 0), (0, -1)):
            nb = (r + dr, c + dc)
            if self.grid.in_bounds(nb) and not self.grid.is_free(nb):
                return True
        return False

    def stations_of(self, kind: str) -> list[Station]:
        return [s for s in self.stations if s.kind == kind]

    @property
    def station_cells(self) -> set[Cell]:
        return {s.cell for s in self.stations}

    @classmethod
    def load(cls, path: str | Path) -> Warehouse:
        """Load a JSON layout that points at a MovingAI map in the same folder."""
        path = Path(path)
        spec = json.loads(path.read_text())
        grid = Grid.from_movingai(path.parent / spec["map"])
        stations = [
            Station(
                id=i,
                kind=s["kind"],
                cell=(int(s["cell"][0]), int(s["cell"][1])),
                service_time=int(s["service_time"]),
            )
            for i, s in enumerate(spec["stations"])
        ]
        return cls(
            grid=grid,
            stations=stations,
            shelf_service_time=int(spec.get("shelf_service_time", 2)),
            name=spec.get("name", path.stem),
        )

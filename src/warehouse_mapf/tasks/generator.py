"""Task generator: each task is inbound station -> product -> picking/outbound."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from warehouse_mapf.sim.distances import DistanceCache
from warehouse_mapf.sim.grid import Cell
from warehouse_mapf.sim.warehouse import Station, Warehouse


@dataclass(frozen=True)
class Errand:
    """One stop on a task: go to ``cell`` and stay ``service_time`` steps."""

    kind: str  # "inbound", "storage", "picking" or "outbound"
    cell: Cell
    service_time: int


@dataclass(frozen=True)
class Task:
    id: int
    errands: tuple[Errand, ...]
    created_at: int


class TaskGenerator:
    """Creates tasks for robots as they become free.

    Products sit at storage cells. Popularity follows a Zipf-like law: the
    product with popularity rank k is ordered with weight 1 / k**skew, so a few
    products get most orders, as in real warehouses. Stations are chosen
    greedily by distance (the Phase 1 assignment policy).
    """

    def __init__(
        self,
        warehouse: Warehouse,
        distances: DistanceCache,
        rng: np.random.Generator,
        popularity_skew: float = 1.0,
        outbound_share: float = 0.25,
    ) -> None:
        if popularity_skew < 0:
            raise ValueError("popularity_skew must be >= 0")
        if not 0.0 <= outbound_share <= 1.0:
            raise ValueError("outbound_share must be in [0, 1]")
        self.warehouse = warehouse
        self.distances = distances
        self.rng = rng
        self.outbound_share = outbound_share
        self._next_id = 0

        products = list(warehouse.storage_cells)
        order = rng.permutation(len(products))
        self.products: list[Cell] = [products[i] for i in order]
        ranks = np.arange(1, len(self.products) + 1, dtype=float)
        weights = 1.0 / ranks**popularity_skew
        self.product_weights = weights / weights.sum()

    def _nearest(self, stations: list[Station], cell: Cell) -> Station:
        def key(s: Station) -> tuple[int, int]:
            d = self.distances.dist(cell, s.cell)
            return (d if d >= 0 else 1 << 30, s.id)

        return min(stations, key=key)

    def next_task(self, robot_pos: Cell, t: int) -> Task:
        wh = self.warehouse
        product = self.products[self.rng.choice(len(self.products), p=self.product_weights)]
        inbound = self._nearest(wh.stations_of("inbound"), robot_pos)

        picking, outbound = wh.stations_of("picking"), wh.stations_of("outbound")
        use_outbound = bool(outbound) and (not picking or self.rng.random() < self.outbound_share)
        dest = self._nearest(outbound if use_outbound else picking, product)

        task = Task(
            id=self._next_id,
            errands=(
                Errand("inbound", inbound.cell, inbound.service_time),
                Errand("storage", product, wh.shelf_service_time),
                Errand(dest.kind, dest.cell, dest.service_time),
            ),
            created_at=t,
        )
        self._next_id += 1
        return task

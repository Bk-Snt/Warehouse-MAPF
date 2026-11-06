"""Cached true-distance tables, one per goal cell."""

from __future__ import annotations

import numpy as np

from warehouse_mapf.sim.grid import Cell, Grid


class DistanceCache:
    """Lazily computes and stores a BFS distance table for each goal.

    The grid is undirected, so ``dist(a, goal)`` is the shortest-path length
    from ``a`` to ``goal``. This is the WHCA* / RRA* true-distance heuristic.
    """

    def __init__(self, grid: Grid) -> None:
        self.grid = grid
        self._tables: dict[Cell, np.ndarray] = {}

    def table(self, goal: Cell) -> np.ndarray:
        if goal not in self._tables:
            self._tables[goal] = self.grid.distances_from(goal)
        return self._tables[goal]

    def dist(self, cell: Cell, goal: Cell) -> int:
        """Shortest-path length from ``cell`` to ``goal`` (-1 if unreachable)."""
        return int(self.table(goal)[cell])

    def __len__(self) -> int:
        return len(self._tables)

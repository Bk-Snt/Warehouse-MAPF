"""Congestion guidance: distance-to-goal tables that steer robots around traffic.

The guidance keeps two running averages, updated every step:

* flow[r, c, d]: how often robots recently left cell (r, c) in direction d.
* density[r, c]: how often cell (r, c) was recently occupied.

Both are exponential moving averages in [0, 1]. Moving from u to v then costs

    1 + opposing_weight * flow(v -> u) + density_weight * density(v)

so driving against recent traffic is expensive and robots form lanes, the
main idea behind traffic-flow guidance for lifelong MAPF. Each goal's table is
a Dijkstra search backwards from the goal over these costs. Tables are rebuilt
when they are older than ``refresh_every`` steps. The density map is also what
the dashboard heat map will show.
"""

from __future__ import annotations

import heapq
from collections.abc import Sequence

import numpy as np

from warehouse_mapf.sim.grid import MOVES, Cell, Grid

_DIRECTION = {move: k for k, move in enumerate(MOVES)}


class CongestionGuidance:
    def __init__(
        self,
        grid: Grid,
        opposing_weight: float = 3.0,
        density_weight: float = 1.0,
        decay: float = 0.95,
        refresh_every: int = 20,
    ) -> None:
        if not 0.0 < decay < 1.0:
            raise ValueError("decay must be in (0, 1)")
        if refresh_every < 1:
            raise ValueError("refresh_every must be >= 1")
        self.grid = grid
        self.opposing_weight = opposing_weight
        self.density_weight = density_weight
        self.decay = decay
        self.refresh_every = refresh_every
        self.flow = np.zeros((grid.height, grid.width, len(MOVES)))
        self.density = np.zeros((grid.height, grid.width))
        self.t = 0
        self.rebuilds = 0
        self._tables: dict[Cell, np.ndarray] = {}
        self._built_at: dict[Cell, int] = {}
        self._last: Sequence[Cell] | None = None

    def observe(self, positions: Sequence[Cell], t: int) -> None:
        """Fold the robots' latest moves and positions into the averages."""
        keep, add = self.decay, 1.0 - self.decay
        self.flow *= keep
        self.density *= keep
        if self._last is not None and len(self._last) == len(positions):
            for (r0, c0), (r1, c1) in zip(self._last, positions, strict=True):
                k = _DIRECTION.get((r1 - r0, c1 - c0))
                if k is not None:  # waits (and any non-unit jump) add no flow
                    self.flow[r0, c0, k] += add
        for r, c in positions:
            self.density[r, c] += add
        self._last = list(positions)
        self.t = t

    def move_cost(self, u: Cell, v: Cell) -> float:
        back = _DIRECTION[(u[0] - v[0], u[1] - v[1])]  # direction v -> u
        return (
            1.0
            + self.opposing_weight * float(self.flow[v[0], v[1], back])
            + self.density_weight * float(self.density[v])
        )

    def table(self, goal: Cell) -> np.ndarray:
        """Guided cost-to-go to ``goal`` from every cell (inf = unreachable)."""
        built = self._built_at.get(goal)
        if built is None or self.t - built >= self.refresh_every:
            self._tables[goal] = self._dijkstra(goal)
            self._built_at[goal] = self.t
            self.rebuilds += 1
        return self._tables[goal]

    def _dijkstra(self, goal: Cell) -> np.ndarray:
        dist = np.full((self.grid.height, self.grid.width), np.inf)
        dist[goal] = 0.0
        heap: list[tuple[float, Cell]] = [(0.0, goal)]
        while heap:
            d, v = heapq.heappop(heap)
            if d > dist[v]:
                continue
            for u in self.grid.neighbors(v):  # robots arrive at v from u
                nd = d + self.move_cost(u, v)
                if nd < dist[u]:
                    dist[u] = nd
                    heapq.heappush(heap, (nd, u))
        return dist

"""Windowed Hierarchical Cooperative A* (WHCA*), Silver 2005, for lifelong MAPF.

Robots plan one at a time in priority order. Each robot runs space-time A*
over a window of ``window`` steps, avoiding cells and moves already booked in
a shared reservation table, then books its own path. Beyond the window the
search uses the true distance to the goal (the role RRA* plays in the paper).
Plans are executed for ``replan_every`` steps, or until any robot's goal or
service state changes, and then every robot replans.

Safety: while a robot plans, the current cells of robots not yet planned are
off limits for the first ``replan_every`` steps. That guarantees every later
robot can at least wait in place, so the executed part of the plan never
collides, even when a robot's search fails.
"""

from __future__ import annotations

import heapq
from collections.abc import Sequence

import numpy as np

from warehouse_mapf.sim.grid import Cell, Grid
from warehouse_mapf.sim.simulator import SimView

UNREACHABLE = 1 << 20


class ReservationTable:
    """Space-time bookings: who stands where at each relative time step."""

    def __init__(self) -> None:
        self.vertex: dict[tuple[Cell, int], int] = {}
        self.edge: set[tuple[Cell, Cell, int]] = set()  # (from, to, departure time)

    def reserve_path(self, robot: int, path: Sequence[Cell]) -> None:
        for tau in range(1, len(path)):
            self.vertex[(path[tau], tau)] = robot
            self.edge.add((path[tau - 1], path[tau], tau - 1))

    def can_move(self, frm: Cell, to: Cell, tau: int) -> bool:
        """True if moving ``frm -> to`` between ``tau`` and ``tau + 1`` is free."""
        if (to, tau + 1) in self.vertex:
            return False
        return frm == to or (to, frm, tau) not in self.edge


def space_time_astar(
    grid: Grid,
    start: Cell,
    goal: Cell,
    dist: np.ndarray,
    reservations: ReservationTable,
    window: int,
    keep_out: dict[Cell, int] | None = None,
    keep_out_until: int = 0,
) -> list[Cell] | None:
    """Cheapest ``window``-step path from ``start`` that respects reservations.

    Every action costs 1, except waiting at the goal, which costs 0. A path's
    total cost is its cost inside the window plus the true distance (``dist``)
    from its last cell to the goal. Cells in ``keep_out`` may not be entered at
    times 1..``keep_out_until``. Returns ``window + 1`` cells, or None.
    """
    keep_out = keep_out or {}

    def h(cell: Cell) -> int:
        d = int(dist[cell])
        return d if d >= 0 else UNREACHABLE

    best_g: dict[tuple[Cell, int], int] = {(start, 0): 0}
    parent: dict[tuple[Cell, int], tuple[Cell, int]] = {}
    closed: set[tuple[Cell, int]] = set()
    counter = 0
    open_heap = [(h(start), h(start), 0, counter, start)]  # f, h, -tau, tie, cell

    while open_heap:
        _, _, neg_tau, _, cell = heapq.heappop(open_heap)
        tau = -neg_tau
        node = (cell, tau)
        if node in closed:
            continue
        closed.add(node)
        if tau == window:
            path = [cell]
            while node in parent:
                node = parent[node]
                path.append(node[0])
            return path[::-1]

        g = best_g[node]
        for nxt in (cell, *grid.neighbors(cell)):
            if not reservations.can_move(cell, nxt, tau):
                continue
            if nxt != cell and nxt in keep_out and tau + 1 <= keep_out_until:
                continue
            nxt_node = (nxt, tau + 1)
            g2 = g + (0 if nxt == cell == goal else 1)
            if g2 < best_g.get(nxt_node, UNREACHABLE * 2):
                best_g[nxt_node] = g2
                parent[nxt_node] = node
                counter += 1
                h2 = h(nxt)
                heapq.heappush(open_heap, (g2 + h2, h2, -(tau + 1), counter, nxt))
    return None


class WHCAStarPlanner:
    """WHCA* with a fixed or shuffled priority order.

    Args:
        window: steps each robot plans ahead (w).
        replan_every: steps executed before a full replan (h <= w). The default
            of 1 replans every step: larger values keep robots' current cells
            off limits for longer (see the module docstring), which costs
            throughput in crowded areas such as station approaches.
        priority: "fixed" (robot id order, the classic setting) or "shuffle"
            (a new random order at every replan, which reduces starvation).
        seed: random seed for the shuffled order.
    """

    name = "whca"

    def __init__(
        self,
        window: int = 16,
        replan_every: int = 1,
        priority: str = "fixed",
        seed: int = 0,
    ) -> None:
        if not 1 <= replan_every <= window:
            raise ValueError("need 1 <= replan_every <= window")
        if priority not in ("fixed", "shuffle"):
            raise ValueError("priority must be 'fixed' or 'shuffle'")
        self.window = window
        self.replan_every = replan_every
        self.priority = priority
        self.rng = np.random.default_rng(seed)
        self.plans: list[list[Cell]] = []
        self.replans = 0
        self.failed_searches = 0
        self._plan_t = 0
        self._goals: tuple[Cell, ...] = ()
        self._busy: tuple[bool, ...] = ()

    def step(self, view: SimView) -> list[Cell]:
        if self._needs_replan(view):
            self._replan(view)
        k = view.t - self._plan_t + 1
        return [path[min(k, len(path) - 1)] for path in self.plans]

    def _needs_replan(self, view: SimView) -> bool:
        if not self.plans or view.t - self._plan_t >= self.replan_every:
            return True
        if view.goals != self._goals or view.busy != self._busy:
            return True
        k = view.t - self._plan_t
        return any(path[k] != pos for path, pos in zip(self.plans, view.positions, strict=True))

    def _order(self, view: SimView) -> list[int]:
        n = len(view.positions)
        free = list(range(n)) if self.priority == "fixed" else list(self.rng.permutation(n))
        busy = [i for i in range(n) if view.busy[i]]
        return busy + [int(i) for i in free if not view.busy[i]]

    def _replan(self, view: SimView) -> None:
        self.replans += 1
        n, w = len(view.positions), self.window
        table = ReservationTable()
        plans: list[list[Cell]] = [[] for _ in range(n)]
        unplanned = {pos: i for i, pos in enumerate(view.positions)}

        for i in self._order(view):
            pos = view.positions[i]
            del unplanned[pos]
            path = None
            if not view.busy[i]:
                path = space_time_astar(
                    view.grid,
                    pos,
                    view.goals[i],
                    view.distances.table(view.goals[i]),
                    table,
                    w,
                    keep_out=unplanned,
                    keep_out_until=self.replan_every,
                )
                if path is None:
                    self.failed_searches += 1
            if path is None:
                path = [pos] * (w + 1)  # busy, or no path found: wait in place
            table.reserve_path(i, path)
            plans[i] = path

        self.plans = plans
        self._plan_t = view.t
        self._goals = view.goals
        self._busy = view.busy

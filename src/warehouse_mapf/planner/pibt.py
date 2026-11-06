"""PIBT: Priority Inheritance with Backtracking (Okumura et al., 2019/2022).

PIBT decides one step at a time. Robots act in order of priority. A robot
takes the free neighbouring cell closest to its goal; if a lower-priority
robot stands there, that robot inherits the priority and must move out of the
way first. If it cannot, it reports failure and the first robot backtracks
to its next-best cell.

Priorities fix WHCA*'s starvation problem: a robot's priority grows by one
for every step it spends working on its current errand, and drops back to a
small random tie-breaker once the errand is done. A robot that has waited a
long time eventually outranks everyone near it.
"""

from __future__ import annotations

import sys

import numpy as np

from warehouse_mapf.sim.grid import Cell
from warehouse_mapf.sim.simulator import SimView

UNREACHABLE = 1 << 20


class PIBTPlanner:
    name = "pibt"

    def __init__(self, seed: int = 0) -> None:
        self.rng = np.random.default_rng(seed)
        self.priorities: np.ndarray = np.zeros(0)
        self._tie_breakers: np.ndarray = np.zeros(0)
        self._goals: tuple[Cell, ...] = ()

    def _update_priorities(self, view: SimView) -> None:
        n = len(view.positions)
        if len(self.priorities) != n:
            self._tie_breakers = self.rng.random(n)  # unique fractions break ties
            self.priorities = self._tie_breakers.copy()
            self._goals = view.goals
            return
        for i in range(n):
            if view.goals[i] != self._goals[i]:
                self.priorities[i] = self._tie_breakers[i]  # errand done: reset
            elif not view.busy[i] and view.positions[i] != view.goals[i]:
                self.priorities[i] += 1
        self._goals = view.goals

    def step(self, view: SimView) -> list[Cell]:
        self._update_priorities(view)
        n = len(view.positions)
        sys.setrecursionlimit(max(sys.getrecursionlimit(), 4 * n + 100))

        self._view = view
        self._next: list[Cell | None] = [None] * n
        self._claimed: dict[Cell, int] = {}  # next-step cell -> robot
        self._occupant = {pos: i for i, pos in enumerate(view.positions)}

        for i in range(n):  # robots being served cannot move
            if view.busy[i]:
                self._next[i] = view.positions[i]
                self._claimed[view.positions[i]] = i

        for i in np.argsort(-self.priorities, kind="stable"):
            if self._next[i] is None:
                self._pibt(int(i), None)
        return [cell for cell in self._next if cell is not None]

    def _candidates(self, i: int) -> list[Cell]:
        """Stay or move, closest to the goal first; prefer cells nobody stands on."""
        view = self._view
        pos = view.positions[i]
        dist = view.distances.table(view.goals[i])
        cells = [pos, *view.grid.neighbors(pos)]
        order = self.rng.permutation(len(cells))  # random tie-break between equals
        cells = [cells[k] for k in order]

        def key(c: Cell) -> tuple[int, bool]:
            d = int(dist[c])
            occupied = self._occupant.get(c, i) != i
            return (d if d >= 0 else UNREACHABLE, occupied)

        return sorted(cells, key=key)

    def _pibt(self, i: int, parent: int | None) -> bool:
        """Choose robot i's next cell. Returns False if i has to stay put."""
        pos = self._view.positions[i]
        parent_pos = self._view.positions[parent] if parent is not None else None

        for v in self._candidates(i):
            if v in self._claimed or v == parent_pos:
                continue  # taken next step, or a swap with the robot pushing us
            k = self._occupant.get(v)
            if k is not None and k != i and self._next[k] == pos:
                continue  # k is moving into our cell: swapping is not allowed
            self._next[i] = v
            self._claimed[v] = i
            if k is not None and k != i and self._next[k] is None:
                if not self._pibt(k, i):  # k inherits our priority and must move
                    self._next[i] = None  # k stays on v, so try our next option
                    continue
            return True

        self._next[i] = pos
        self._claimed[pos] = i
        return False

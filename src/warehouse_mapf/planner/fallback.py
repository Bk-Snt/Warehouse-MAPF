"""Deadlock fallback: PIBT for everyday traffic, LaCAM when robots get stuck.

PIBT is fast but can lock up, for example two robots meeting head-on in a
one-lane aisle that only a dead-end pocket could resolve. This planner watches
every robot's progress. Waiting for a goal cell that another robot occupies
(a queue at a station) counts as normal, not stuck. When a robot has gone
``stuck_after`` steps without getting closer to its goal, the planner takes the
robot stuck longest and the free robots around it (up to ``max_group``, within
``radius`` cells) and asks LaCAM for a short plan in which that robot gets
closer to its goal than it has been on this errand. All other robots are
treated as obstacles in that search.

While the plan runs, its robots follow it exactly, and the cells it still
needs are off limits to everyone else; each cell is released once the plan is
done with it, so the rest of the fleet keeps moving with PIBT around it. Each
search gets ``time_limit`` seconds. If LaCAM finds nothing, the next attempt
for that robot uses a group twice as large and twice as wide.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np

from warehouse_mapf.planner.lacam import LaCAM
from warehouse_mapf.planner.pibt import PIBTPlanner
from warehouse_mapf.sim.grid import Cell
from warehouse_mapf.sim.simulator import SimView


@dataclass
class _Fix:
    robots: list[int]
    configs: list[tuple[Cell, ...]]
    k: int = 0  # index of the configuration the robots should be in now


class DeadlockFallbackPlanner:
    name = "pibt+lacam"

    def __init__(
        self,
        base: PIBTPlanner | None = None,
        stuck_after: int = 60,
        radius: int = 4,
        max_group: int = 8,
        max_nodes: int = 2000,
        time_limit: float = 0.2,
        seed: int = 0,
    ) -> None:
        self.base = base or PIBTPlanner(seed=seed)
        self.stuck_after = stuck_after
        self.radius = radius
        self.max_group = max_group
        self.max_nodes = max_nodes
        self.time_limit = time_limit
        self.rng = np.random.default_rng([seed, 2])
        self.fix: _Fix | None = None
        self.triggers = 0
        self.solved = 0
        self.failed = 0
        self._best: list[int] = []
        self._last_progress: list[int] = []
        self._goals: tuple[Cell, ...] = ()
        self._boost: dict[int, int] = {}  # robot -> radius multiplier after a failure

    def step(self, view: SimView) -> list[Cell]:
        self.base.observe(view)
        self._track_progress(view)

        if self.fix is not None and not self._on_track(view):
            self.fix = None
        if self.fix is None:
            stuck = self._stuck(view)
            if stuck:
                self._start_fix(view, stuck)

        fixed: dict[int, Cell] = {}
        blocked: set[Cell] = set()
        if self.fix is not None:
            nxt = self.fix.configs[self.fix.k + 1]
            fixed = dict(zip(self.fix.robots, nxt, strict=True))
            blocked = {cell for config in self.fix.configs[self.fix.k :] for cell in config}
            self.fix.k += 1
            if self.fix.k + 1 >= len(self.fix.configs):
                self.fix = None
        return self.base.plan_step(view, self.base.priorities, fixed=fixed, blocked=blocked)

    # --- progress tracking -------------------------------------------------

    def _dist(self, view: SimView, i: int) -> int:
        return view.distances.dist(view.positions[i], view.goals[i])

    def _track_progress(self, view: SimView) -> None:
        n = len(view.positions)
        if len(self._best) != n:
            self._best = [self._dist(view, i) for i in range(n)]
            self._last_progress = [view.t] * n
            self._goals = view.goals
            return
        occupied = set(view.positions)
        for i in range(n):
            d = self._dist(view, i)
            queueing = view.goals[i] in occupied and view.goals[i] != view.positions[i]
            if view.goals[i] != self._goals[i] or view.busy[i] or queueing or d < self._best[i]:
                self._best[i] = d
                self._last_progress[i] = view.t
                if view.goals[i] != self._goals[i]:
                    self._boost.pop(i, None)
        self._goals = view.goals

    def _stuck(self, view: SimView) -> list[int]:
        stuck = [
            i
            for i in range(len(view.positions))
            if not view.busy[i] and view.t - self._last_progress[i] >= self.stuck_after
        ]
        return sorted(stuck, key=lambda i: self._last_progress[i])  # longest stuck first

    def _on_track(self, view: SimView) -> bool:
        assert self.fix is not None
        expected = self.fix.configs[self.fix.k]
        nxt = self.fix.configs[self.fix.k + 1]
        for r, cell, after in zip(self.fix.robots, expected, nxt, strict=True):
            if view.positions[r] != cell or (view.busy[r] and after != cell):
                return False
        return True

    # --- calling LaCAM -----------------------------------------------------

    def _group(self, view: SimView, seed_robot: int, radius: int, size: int) -> list[int]:
        """Up to ``size`` free robots within ``radius`` steps of the stuck robot, nearest first."""
        occupant = {pos: i for i, pos in enumerate(view.positions)}
        start = view.positions[seed_robot]
        seen = {start}
        queue = deque([(start, 0)])
        group = []
        while queue and len(group) < size:
            cell, d = queue.popleft()
            r = occupant.get(cell)
            if r is not None and not view.busy[r]:
                group.append(r)
            if d == radius:
                continue
            for nb in view.grid.neighbors(cell):
                if nb not in seen:
                    seen.add(nb)
                    queue.append((nb, d + 1))
        return group

    def _start_fix(self, view: SimView, stuck: list[int]) -> None:
        self.triggers += 1
        seed_robot = stuck[0]
        boost = self._boost.get(seed_robot, 1)
        group = self._group(view, seed_robot, self.radius * boost, self.max_group * boost)
        in_group = set(group)
        others = {view.positions[i] for i in range(len(view.positions)) if i not in in_group}
        goals = [view.goals[r] for r in group]
        best = [self._best[r] for r in group]
        tables = [view.distances.table(g) for g in goals]

        def progress_made(config: tuple[Cell, ...]) -> bool:
            return config[0] == goals[0] or int(tables[0][config[0]]) < best[0]

        solver = LaCAM(view.grid, goals, tables, blocked=others, rng=self.rng)
        result = solver.solve(
            [view.positions[r] for r in group], progress_made, self.max_nodes, self.time_limit
        )
        if result.configs is None or len(result.configs) < 2:
            self.failed += 1
            self._last_progress[seed_robot] = view.t  # back off before retrying
            self._boost[seed_robot] = 2
            return
        self.solved += 1
        self.fix = _Fix(group, result.configs)

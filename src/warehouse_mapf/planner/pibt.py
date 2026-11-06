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

With ``guidance=True`` robots rank their moves by congestion-aware cost-to-go
(see ``guidance.py``) instead of plain shortest-path distance.
"""

from __future__ import annotations

import sys

import numpy as np

from warehouse_mapf.planner.guidance import CongestionGuidance
from warehouse_mapf.sim.grid import Cell
from warehouse_mapf.sim.simulator import SimView
from warehouse_mapf.sim.validation import stop_conflicting

UNREACHABLE = 1 << 20


class PIBTPlanner:
    name = "pibt"

    def __init__(
        self,
        seed: int = 0,
        guidance: bool = False,
        guidance_params: dict[str, float] | None = None,
    ) -> None:
        self.rng = np.random.default_rng(seed)
        self.use_guidance = guidance
        self.guidance_params = guidance_params or {}
        self.guidance: CongestionGuidance | None = None
        self.priorities: np.ndarray = np.zeros(0)
        self._tie_breakers: np.ndarray = np.zeros(0)
        self._goals: tuple[Cell, ...] = ()
        self.rotations_broken = 0

    def _update_priorities(self, view: SimView) -> None:
        n = len(view.positions)
        if len(self.priorities) != n:
            self._tie_breakers = self.rng.random(n)  # unique fractions break ties
            self.priorities = self._tie_breakers.copy()
            self._goals = view.goals
            return
        self.priorities = self._next_priorities(self.priorities, self._goals, view)
        self._goals = view.goals

    def _next_priorities(
        self, priorities: np.ndarray, old_goals: tuple[Cell, ...], view: SimView
    ) -> np.ndarray:
        new = priorities.copy()
        for i in range(len(new)):
            if view.goals[i] != old_goals[i]:
                new[i] = self._tie_breakers[i]  # errand done: reset
            elif not view.busy[i] and view.positions[i] != view.goals[i]:
                new[i] += 1
        return new

    def observe(self, view: SimView) -> None:
        """Advance priorities and traffic averages by one real time step."""
        self._update_priorities(view)
        if self.use_guidance:
            if self.guidance is None:
                self.guidance = CongestionGuidance(view.grid, **self.guidance_params)
            self.guidance.observe(view.positions, view.t)

    def step(self, view: SimView) -> list[Cell]:
        self.observe(view)
        return self.plan_step(view, self.priorities)

    def plan_step(
        self,
        view: SimView,
        priorities: np.ndarray,
        forbidden: set[tuple[int, Cell]] | None = None,
    ) -> list[Cell]:
        """One PIBT step from ``view`` with the given priorities.

        ``forbidden`` holds (robot, cell) moves that robot may not make this step.
        """
        n = len(view.positions)
        sys.setrecursionlimit(max(sys.getrecursionlimit(), 4 * n + 100))

        self._view = view
        self._forbidden = forbidden or set()
        self._next: list[Cell | None] = [None] * n
        self._claimed: dict[Cell, int] = {}  # next-step cell -> robot
        self._occupant = {pos: i for i, pos in enumerate(view.positions)}

        for i in range(n):  # robots being served cannot move
            if view.busy[i]:
                self._next[i] = view.positions[i]
                self._claimed[view.positions[i]] = i

        for i in np.argsort(-priorities, kind="stable"):
            if self._next[i] is None:
                self._pibt(int(i), None)
        return [cell for cell in self._next if cell is not None]

    def rollout(self, view: SimView, steps: int) -> list[list[Cell]]:
        """Plan ``steps`` steps ahead by running PIBT on an imagined future.

        Goals stay fixed during the rollout, and a robot that reaches its goal
        (or is already being served) waits there for the rest of the window.
        Rotations, where robots move around a loop in one step, are removed:
        they need every robot in the loop to move at exactly the same moment,
        which a delayed robot breaks. Returns one path of ``steps + 1`` cells per
        robot. Real priorities are not changed.
        """
        positions = list(view.positions)
        busy = list(view.busy)
        priorities = self.priorities.copy() if len(self.priorities) else np.zeros(len(positions))
        paths = [[p] for p in positions]
        for k in range(steps):
            busy = [b or p == g for b, p, g in zip(busy, positions, view.goals, strict=True)]
            future = SimView(
                t=view.t + k,
                grid=view.grid,
                distances=view.distances,
                positions=tuple(positions),
                goals=view.goals,
                busy=tuple(busy),
            )
            if k > 0:
                priorities = self._next_priorities(priorities, view.goals, future)
            nxt = self._plan_step_without_rotations(future, priorities)
            positions = nxt
            for path, p in zip(paths, positions, strict=True):
                path.append(p)
        return paths

    def _plan_step_without_rotations(
        self, view: SimView, priorities: np.ndarray, attempts: int = 5
    ) -> list[Cell]:
        """Plan a step; if robots rotate around a loop, ban one move and replan.

        The banned move is the loop member with the lowest priority, so PIBT
        usually finds another way out. If loops remain after a few attempts,
        the robots in them wait.
        """
        positions = list(view.positions)
        forbidden: set[tuple[int, Cell]] = set()
        for _ in range(attempts):
            nxt = self.plan_step(view, priorities, forbidden)
            rotating = rotation_members(positions, nxt)
            if not rotating:
                return nxt
            self.rotations_broken += 1
            weakest = min(rotating, key=lambda r: priorities[r])
            forbidden.add((weakest, nxt[weakest]))
        nxt, _ = stop_conflicting(positions, nxt, must_stay=rotating)
        return nxt

    def _candidates(self, i: int) -> list[Cell]:
        """Stay or move, cheapest to the goal first; prefer cells nobody stands on."""
        view = self._view
        pos = view.positions[i]
        source = self.guidance if self.guidance is not None else view.distances
        dist = source.table(view.goals[i])
        cells = [pos, *view.grid.neighbors(pos)]
        order = self.rng.permutation(len(cells))  # random tie-break between equals
        cells = [cells[k] for k in order]

        guidance = self.guidance

        def key(c: Cell) -> tuple[float, bool]:
            # Cost of this step (waiting costs 1) plus the cost-to-go from c.
            d = float(dist[c])
            if not 0 <= d < UNREACHABLE:
                return (UNREACHABLE, True)
            step = guidance.move_cost(pos, c) if guidance is not None and c != pos else 1.0
            return (step + d, self._occupant.get(c, i) != i)

        return sorted(cells, key=key)

    def _pibt(self, i: int, parent: int | None) -> bool:
        """Choose robot i's next cell. Returns False if i has to stay put."""
        pos = self._view.positions[i]
        parent_pos = self._view.positions[parent] if parent is not None else None

        for v in self._candidates(i):
            if v in self._claimed or v == parent_pos or (i, v) in self._forbidden:
                continue  # taken next step, a swap with the robot pushing us, or banned
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


def rotation_members(current: list[Cell], nxt: list[Cell]) -> set[int]:
    """Robots that move around a closed loop in this step (A->B->C->A)."""
    occupant = {pos: i for i, pos in enumerate(current)}
    members: set[int] = set()
    done: set[int] = set()
    for start in range(len(current)):
        if start in done or nxt[start] == current[start]:
            continue
        chain, seen, i = [], set(), start
        while i is not None and i not in seen and i not in done and nxt[i] != current[i]:
            seen.add(i)
            chain.append(i)
            i = occupant.get(nxt[i])  # whoever stands where robot i is going
        if i is not None and i in seen:
            members.update(chain[chain.index(i) :])
        done.update(chain)
    return members

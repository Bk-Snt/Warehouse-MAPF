"""Windowed execution: plan several steps ahead, then execute with or without an ADG.

Every ``replan_every`` steps (sooner if a goal or service state changes, or a
robot runs out of plan) the planner rolls PIBT forward ``window`` steps from
the robots' actual positions. Each robot then works through its own moves:

* naive execution: each robot follows its own path step by step, waits
  included, and a delayed robot simply resumes where it stopped. Without
  delays this is exactly the plan; with delays robots fall out of step with
  each other and can run into each other.
* ADG execution: a robot makes its next move only once every robot that must
  use the target cell before it has left. Planned waits are dropped; the
  dependencies alone keep robots apart, so late robots make others wait
  instead of causing collisions.
"""

from __future__ import annotations

from warehouse_mapf.planner.pibt import PIBTPlanner
from warehouse_mapf.sim.adg import ActionDependencyGraph
from warehouse_mapf.sim.grid import Cell
from warehouse_mapf.sim.simulator import SimView


class WindowedPlanner:
    name = "windowed"

    def __init__(
        self,
        base: PIBTPlanner | None = None,
        window: int = 8,
        replan_every: int = 4,
        adg: bool = True,
    ) -> None:
        if not 1 <= replan_every <= window:
            raise ValueError("need 1 <= replan_every <= window")
        self.base = base or PIBTPlanner()
        self.window = window
        self.replan_every = replan_every
        self.use_adg = adg
        self.graph: ActionDependencyGraph | None = None
        self.paths: list[list[Cell]] = []
        self.progress: list[int] = []  # naive execution: index into each path
        self.replans = 0
        self._plan_t = 0
        self._goals: tuple[Cell, ...] = ()
        self._busy: tuple[bool, ...] = ()

    def step(self, view: SimView) -> list[Cell]:
        self.base.observe(view)
        if self.paths:
            self._record_progress(view)
        if self._needs_replan(view):
            self._replan(view)

        commands = list(view.positions)
        for i in range(len(commands)):
            if view.busy[i]:
                continue
            if self.use_adg:
                assert self.graph is not None
                move = self.graph.next_move(i)
                if move is not None and self.graph.ready(i):
                    commands[i] = move.to
            else:
                k = self.progress[i]
                if k + 1 < len(self.paths[i]):
                    commands[i] = self.paths[i][k + 1]
        return commands

    def _record_progress(self, view: SimView) -> None:
        """Advance each robot along its plan once it has reached its next cell."""
        for i, pos in enumerate(view.positions):
            if self.use_adg:
                assert self.graph is not None
                move = self.graph.next_move(i)
                if move is not None and pos == move.to:
                    self.graph.done[i] += 1
            else:
                k = self.progress[i]
                if k + 1 < len(self.paths[i]) and pos == self.paths[i][k + 1]:
                    self.progress[i] += 1

    def _expected_cell(self, i: int) -> Cell | None:
        if self.use_adg:
            assert self.graph is not None
            move = self.graph.next_move(i)
            return move.frm if move is not None else None
        return self.paths[i][self.progress[i]]

    def _needs_replan(self, view: SimView) -> bool:
        if not self.paths or view.t - self._plan_t >= self.replan_every:
            return True
        if view.goals != self._goals or view.busy != self._busy:
            return True
        for i, pos in enumerate(view.positions):
            expected = self._expected_cell(i)
            if expected is not None and pos != expected:
                return True  # robot is not where its plan expects
        return False

    def _replan(self, view: SimView) -> None:
        self.replans += 1
        self.paths = self.base.rollout(view, self.window)
        self.progress = [0] * len(self.paths)
        self.graph = ActionDependencyGraph(self.paths) if self.use_adg else None
        self._plan_t = view.t
        self._goals = view.goals
        self._busy = view.busy

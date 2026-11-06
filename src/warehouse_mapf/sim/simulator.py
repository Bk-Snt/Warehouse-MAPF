"""Lifelong warehouse simulator: robots execute errands forever.

Each timestep the planner returns every robot's next cell. The simulator
checks the step, moves the robots, runs service timers at errand goals and
hands out a new task as soon as a robot finishes one.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
from typing import Protocol

import numpy as np

from warehouse_mapf.sim.distances import DistanceCache
from warehouse_mapf.sim.grid import Cell, Grid
from warehouse_mapf.sim.validation import PlanError, check_step
from warehouse_mapf.sim.warehouse import Warehouse
from warehouse_mapf.tasks.generator import Errand, Task, TaskGenerator


@dataclass
class Robot:
    id: int
    pos: Cell
    task: Task
    errand_idx: int = 0
    busy_until: int | None = None  # set while being served at a goal
    errand_started: int = 0

    @property
    def errand(self) -> Errand:
        return self.task.errands[self.errand_idx]

    @property
    def goal(self) -> Cell:
        return self.errand.cell

    @property
    def busy(self) -> bool:
        return self.busy_until is not None


@dataclass(frozen=True)
class SimView:
    """Read-only snapshot handed to planners each step."""

    t: int
    grid: Grid
    distances: DistanceCache
    positions: tuple[Cell, ...]
    goals: tuple[Cell, ...]
    busy: tuple[bool, ...]  # busy robots must stay where they are


class Planner(Protocol):
    def step(self, view: SimView) -> list[Cell]:
        """Return each robot's position at time ``view.t + 1``."""
        ...


@dataclass
class Metrics:
    steps: int = 0
    tasks_completed: int = 0
    errands_completed: int = 0
    wait_steps: int = 0  # robot not busy, not at its goal, and did not move
    max_errand_time: int = 0  # longest time any robot spent on one errand
    errand_times: list[int] = field(default_factory=list)

    @property
    def throughput(self) -> float:
        """Tasks completed per 1,000 steps."""
        return 1000.0 * self.tasks_completed / self.steps if self.steps else 0.0

    def summary(self) -> dict[str, float]:
        mean_errand = float(np.mean(self.errand_times)) if self.errand_times else 0.0
        return {
            "steps": self.steps,
            "tasks_completed": self.tasks_completed,
            "throughput_per_1000_steps": round(self.throughput, 2),
            "mean_errand_time": round(mean_errand, 2),
            "max_errand_time": self.max_errand_time,
            "wait_steps": self.wait_steps,
        }


class Simulator:
    def __init__(
        self,
        warehouse: Warehouse,
        num_robots: int,
        seed: int = 0,
        record: bool = False,
        popularity_skew: float = 1.0,
        outbound_share: float = 0.25,
        assignment: str = "nearest",
    ) -> None:
        self.warehouse = warehouse
        self.grid = warehouse.grid
        self.distances = DistanceCache(self.grid)
        self.rng = np.random.default_rng(seed)
        self.tasks = TaskGenerator(
            warehouse,
            self.distances,
            self.rng,
            popularity_skew=popularity_skew,
            outbound_share=outbound_share,
            assignment=assignment,
        )
        self.metrics = Metrics()
        self.t = 0
        self.record = record
        self.history: list[list[Cell]] = []

        starts = [c for c in self.grid.free_cells() if c not in warehouse.station_cells]
        if num_robots > len(starts):
            raise ValueError(f"{num_robots} robots do not fit in {len(starts)} start cells")
        picks = self.rng.choice(len(starts), size=num_robots, replace=False)
        self.robots: list[Robot] = []
        for i, k in enumerate(picks):
            task = self.tasks.next_task(starts[k], 0, self.station_load())
            self.robots.append(Robot(id=i, pos=starts[k], task=task))
        self._update_errands()
        self._record()

    def view(self) -> SimView:
        return SimView(
            t=self.t,
            grid=self.grid,
            distances=self.distances,
            positions=tuple(r.pos for r in self.robots),
            goals=tuple(r.goal for r in self.robots),
            busy=tuple(r.busy for r in self.robots),
        )

    def step(self, next_positions: list[Cell]) -> None:
        current = [r.pos for r in self.robots]
        check_step(self.grid, current, next_positions)
        for r, nxt in zip(self.robots, next_positions, strict=True):
            if r.busy and nxt != r.pos:
                raise PlanError(f"robot {r.id} moved while being served at {r.pos}")
            if not r.busy and nxt == r.pos and r.pos != r.goal:
                self.metrics.wait_steps += 1
            r.pos = nxt
        self.t += 1
        self.metrics.steps += 1
        self._update_errands()
        self._record()

    def run(self, planner: Planner, steps: int) -> Metrics:
        for _ in range(steps):
            self.step(planner.step(self.view()))
        return self.metrics

    def _update_errands(self) -> None:
        for r in self.robots:
            if r.busy_until is None and r.pos == r.goal:
                r.busy_until = self.t + r.errand.service_time
            if r.busy_until is not None and self.t >= r.busy_until:
                self._finish_errand(r)

    def _finish_errand(self, r: Robot) -> None:
        elapsed = self.t - r.errand_started
        m = self.metrics
        m.errands_completed += 1
        m.errand_times.append(elapsed)
        m.max_errand_time = max(m.max_errand_time, elapsed)
        r.busy_until = None
        r.errand_started = self.t
        r.errand_idx += 1
        if r.errand_idx == len(r.task.errands):
            m.tasks_completed += 1
            r.task = self.tasks.next_task(r.pos, self.t, self.station_load())
            r.errand_idx = 0

    def station_load(self) -> Counter[Cell]:
        """How many robots still have each station ahead of them in their task."""
        stations = self.warehouse.station_cells
        load: Counter[Cell] = Counter()
        for r in self.robots:
            for errand in r.task.errands[r.errand_idx :]:
                if errand.cell in stations:
                    load[errand.cell] += 1
        return load

    def _record(self) -> None:
        if self.record:
            self.history.append([r.pos for r in self.robots])

    def save_log(self, path: str | Path) -> None:
        """Save the run for replay: positions has shape (steps + 1, robots, 2)."""
        if not self.record:
            raise RuntimeError("create the Simulator with record=True to save a log")
        np.savez_compressed(
            path,
            positions=np.array(self.history, dtype=np.int16),
            free=self.grid.free,
            station_cells=np.array([s.cell for s in self.warehouse.stations], dtype=np.int16),
            station_kinds=np.array([s.kind for s in self.warehouse.stations]),
        )

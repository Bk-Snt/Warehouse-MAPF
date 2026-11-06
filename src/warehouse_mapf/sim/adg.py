"""Action dependency graph (ADG): execute a plan safely when robots run late.

Based on Hoenig et al., "Persistent and Robust Execution of MAPF Schedules in
Warehouses" (2019). A plan is turned into each robot's list of moves (waits are
dropped). If robot A leaves a cell that robot B later enters, B's move into
that cell depends on A's move out of it. At run time a robot starts its next
move only when every move it depends on has finished. Delays then turn into
waiting, never collisions, and every cell is used by robots in the planned order.
"""

from __future__ import annotations

from collections import defaultdict
from collections.abc import Sequence
from dataclasses import dataclass

from warehouse_mapf.sim.grid import Cell


@dataclass(frozen=True)
class Move:
    robot: int
    index: int  # position in this robot's move list
    time: int  # plan step at which the move starts
    frm: Cell
    to: Cell


class ActionDependencyGraph:
    def __init__(self, paths: Sequence[Sequence[Cell]]) -> None:
        self.moves: list[list[Move]] = []
        for robot, path in enumerate(paths):
            moves = []
            for t in range(len(path) - 1):
                if path[t] != path[t + 1]:
                    moves.append(Move(robot, len(moves), t, path[t], path[t + 1]))
            self.moves.append(moves)
        # depends_on[robot][index] = (other robot, its move index) that must finish first
        self.depends_on: list[list[tuple[int, int] | None]] = [[None] * len(m) for m in self.moves]
        self.done = [0] * len(paths)  # moves finished per robot
        self._build(paths)

    def _build(self, paths: Sequence[Sequence[Cell]]) -> None:
        # Each stay in a cell: (enter time, robot, entering move, leaving move);
        # a move index is None for the start cell or for staying until the end.
        visits: dict[Cell, list[tuple[int, int, int | None, int | None]]] = defaultdict(list)
        for robot, path in enumerate(paths):
            if not path:
                continue
            enter_time, enter_move = 0, None
            for move in self.moves[robot]:
                visits[move.frm].append((enter_time, robot, enter_move, move.index))
                enter_time, enter_move = move.time + 1, move.index
            visits[path[-1]].append((enter_time, robot, enter_move, None))

        for cell, stays in visits.items():
            stays.sort()
            for (_, r1, _, leave1), (_, r2, enter2, _) in zip(stays, stays[1:], strict=False):
                if r1 == r2 or enter2 is None:
                    continue
                if leave1 is None:
                    raise ValueError(f"robot {r2} enters {cell} but robot {r1} never leaves it")
                self.depends_on[r2][enter2] = (r1, leave1)

    def next_move(self, robot: int) -> Move | None:
        moves = self.moves[robot]
        return moves[self.done[robot]] if self.done[robot] < len(moves) else None

    def ready(self, robot: int) -> bool:
        """True if the robot's next move may start now."""
        move = self.next_move(robot)
        if move is None:
            return False
        dep = self.depends_on[robot][move.index]
        return dep is None or self.done[dep[0]] > dep[1]

    def finished(self) -> bool:
        return all(d == len(m) for d, m in zip(self.done, self.moves, strict=True))

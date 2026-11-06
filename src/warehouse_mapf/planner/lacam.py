"""LaCAM: Lazy Constraints Addition search for MAPF (Okumura, AAAI 2023).

LaCAM searches over configurations (one cell per robot). From each
configuration it asks a fast generator, PIBT, for a successor. If that
successor leads nowhere, it adds constraints one robot at a time ("robot 3 must
go to cell v") and asks PIBT again, so in the limit every combination of moves
is tried. That makes it complete: given enough nodes it finds a solution if one
exists, which plain PIBT cannot promise. The deadlock fallback uses it on a
small group of stuck robots, so the search stays small.
"""

from __future__ import annotations

import sys
import time
from collections import deque
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field

import numpy as np

from warehouse_mapf.sim.grid import Cell, Grid
from warehouse_mapf.sim.validation import find_conflicts

Config = tuple[Cell, ...]
UNREACHABLE = 1 << 20


@dataclass
class _Constraint:
    """A chain of 'robot who goes to cell where' decisions, built lazily."""

    parent: _Constraint | None
    who: int | None
    where: Cell | None
    depth: int

    def assignments(self) -> dict[int, Cell]:
        out: dict[int, Cell] = {}
        node: _Constraint | None = self
        while node is not None and node.who is not None:
            out[node.who] = node.where  # type: ignore[assignment]
            node = node.parent
        return out


@dataclass
class _Node:
    config: Config
    parent: _Node | None
    order: list[int]
    tree: deque[_Constraint] = field(default_factory=deque)


@dataclass
class LaCAMResult:
    configs: list[Config] | None  # start ... goal, or None if no solution was found
    nodes: int


class LaCAM:
    """Find a sequence of configurations that reaches a goal predicate.

    Args:
        grid: the map.
        goals: each robot's goal cell.
        dist_tables: each robot's distance-to-goal table.
        blocked: cells no robot in this search may enter (other robots).
        stay_at_goal: once a robot reaches its goal it may not leave (robots are
            served at their goal and cannot be moved).
        rng: tie-breaking randomness.
    """

    def __init__(
        self,
        grid: Grid,
        goals: Sequence[Cell],
        dist_tables: Sequence[np.ndarray],
        blocked: set[Cell] | None = None,
        stay_at_goal: bool = True,
        rng: np.random.Generator | None = None,
    ) -> None:
        self.grid = grid
        self.goals = list(goals)
        self.dist = list(dist_tables)
        self.blocked = blocked or set()
        self.stay_at_goal = stay_at_goal
        self.rng = rng or np.random.default_rng(0)
        self.n = len(goals)

    def _d(self, i: int, cell: Cell) -> int:
        d = int(self.dist[i][cell])
        return d if d >= 0 else UNREACHABLE

    def _moves(self, i: int, cell: Cell) -> list[Cell]:
        if self.stay_at_goal and cell == self.goals[i]:
            return [cell]
        return [cell, *(v for v in self.grid.neighbors(cell) if v not in self.blocked)]

    def _order(self, config: Config) -> list[int]:
        """Robots far from their goals act first."""
        return sorted(range(self.n), key=lambda i: (-self._d(i, config[i]), i))

    def solve(
        self,
        starts: Sequence[Cell],
        is_goal: Callable[[Config], bool],
        max_nodes: int = 20000,
        time_limit: float | None = None,
    ) -> LaCAMResult:
        """Search until ``is_goal`` holds, the open list empties (no solution
        exists), or ``max_nodes`` / ``time_limit`` seconds run out."""
        deadline = None if time_limit is None else time.perf_counter() + time_limit
        sys.setrecursionlimit(max(sys.getrecursionlimit(), 4 * self.n + 100))
        start = tuple(starts)
        root = _Node(start, None, self._order(start))
        root.tree.append(_Constraint(None, None, None, 0))
        open_stack = [root]
        explored = {start: root}

        while open_stack and len(explored) <= max_nodes:
            if deadline is not None and time.perf_counter() > deadline:
                break
            node = open_stack[-1]
            if is_goal(node.config):
                return LaCAMResult(self._backtrack(node), len(explored))
            if not node.tree:
                open_stack.pop()
                continue
            constraint = node.tree.popleft()
            if constraint.depth < self.n:  # lazily add the next robot's options
                i = node.order[constraint.depth]
                options = self._moves(i, node.config[i])
                for k in self.rng.permutation(len(options)):
                    node.tree.append(_Constraint(constraint, i, options[k], constraint.depth + 1))
            successor = self._generate(node, constraint.assignments())
            if successor is None:
                continue
            if successor in explored:
                open_stack.append(explored[successor])
                continue
            new = _Node(successor, node, self._order(successor))
            new.tree.append(_Constraint(None, None, None, 0))
            explored[successor] = new
            open_stack.append(new)
        return LaCAMResult(None, len(explored))

    @staticmethod
    def _backtrack(node: _Node) -> list[Config]:
        configs = []
        cur: _Node | None = node
        while cur is not None:
            configs.append(cur.config)
            cur = cur.parent
        return configs[::-1]

    # Configuration generator: PIBT that respects the constraint's fixed moves.

    def _generate(self, node: _Node, fixed: dict[int, Cell]) -> Config | None:
        config = node.config
        self._config = config
        self._next: list[Cell | None] = [None] * self.n
        self._claimed: dict[Cell, int] = {}
        self._occupant = {cell: i for i, cell in enumerate(config)}
        for i, v in fixed.items():
            if v in self._claimed:
                return None
            self._next[i] = v
            self._claimed[v] = i
        for i in node.order:
            if self._next[i] is None and not self._pibt(i, None):
                if self._claimed.get(config[i]) not in (None, i):
                    return None  # i must stay, but its cell is already taken
        nxt = tuple(c for c in self._next if c is not None)
        if len(nxt) != self.n or find_conflicts(config, nxt):
            return None
        return nxt

    def _pibt(self, i: int, parent: int | None) -> bool:
        pos = self._config[i]
        parent_pos = self._config[parent] if parent is not None else None
        options = self._moves(i, pos)
        order = self.rng.permutation(len(options))
        options = sorted((options[k] for k in order), key=lambda v: self._d(i, v))
        for v in options:
            if v in self._claimed or v == parent_pos:
                continue
            k = self._occupant.get(v)
            if k is not None and k != i and self._next[k] == pos:
                continue  # swap
            self._next[i] = v
            self._claimed[v] = i
            if k is not None and k != i and self._next[k] is None and not self._pibt(k, i):
                self._next[i] = None
                continue
            return True
        self._next[i] = pos
        self._claimed.setdefault(pos, i)
        return False

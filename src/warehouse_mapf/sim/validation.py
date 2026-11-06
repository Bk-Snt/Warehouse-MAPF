"""Collision checker: every plan and every executed step goes through here."""

from __future__ import annotations

from collections import defaultdict
from collections.abc import Sequence

from warehouse_mapf.sim.grid import Cell, Grid


class PlanError(Exception):
    """A step or plan breaks a movement or collision rule."""


def check_moves(grid: Grid, current: Sequence[Cell], nxt: Sequence[Cell]) -> None:
    """Physical rules only: each robot waits or moves to a free 4-neighbour."""
    if len(current) != len(nxt):
        raise PlanError(f"expected {len(current)} next positions, got {len(nxt)}")
    for i, (a, b) in enumerate(zip(current, nxt, strict=True)):
        if not grid.is_free(b):
            raise PlanError(f"robot {i} moves onto blocked cell {b}")
        if abs(a[0] - b[0]) + abs(a[1] - b[1]) > 1:
            raise PlanError(f"robot {i} jumps from {a} to {b}")


def find_conflicts(current: Sequence[Cell], nxt: Sequence[Cell]) -> list[str]:
    """Describe every vertex and swap conflict in one synchronous step."""
    problems = []
    occupants: dict[Cell, list[int]] = defaultdict(list)
    for i, b in enumerate(nxt):
        occupants[b].append(i)
    for cell, robots in occupants.items():
        if len(robots) > 1:
            problems.append(f"vertex conflict at {cell}: robots {robots}")
    at_start = {cell: i for i, cell in enumerate(current)}
    for i, (a, b) in enumerate(zip(current, nxt, strict=True)):
        j = at_start.get(b)
        if a != b and j is not None and j > i and nxt[j] == a:
            problems.append(f"swap conflict between robots {i} and {j} on {a}<->{b}")
    return problems


def check_step(grid: Grid, current: Sequence[Cell], nxt: Sequence[Cell]) -> None:
    """Check one synchronous step for all robots.

    Rules: each robot moves to a 4-neighbour or waits, never onto an obstacle,
    no two robots share a cell (vertex conflict) and no two robots swap cells
    (edge conflict). Following another robot into the cell it just left is allowed.
    """
    check_moves(grid, current, nxt)
    problems = find_conflicts(current, nxt)
    if problems:
        raise PlanError(problems[0])


def stop_conflicting(
    current: Sequence[Cell], nxt: Sequence[Cell], must_stay: set[int] | None = None
) -> tuple[list[Cell], int]:
    """Emergency stop: keep robots in place until the step has no conflicts.

    Robots in ``must_stay`` stay first. Then every mover involved in a vertex or
    swap conflict is sent back to its current cell, which can push the conflict
    onto a robot that was following it, so this repeats until the step is clean.
    Returns the fixed step and how many movers had to be stopped.
    """
    result = list(nxt)
    for i in must_stay or ():
        result[i] = current[i]
    stopped = 0
    while True:
        occupants: dict[Cell, list[int]] = defaultdict(list)
        for i, b in enumerate(result):
            occupants[b].append(i)
        at_start = {cell: i for i, cell in enumerate(current)}
        revert = set()
        for robots in occupants.values():
            if len(robots) > 1:
                revert.update(i for i in robots if result[i] != current[i])
        for i, (a, b) in enumerate(zip(current, result, strict=True)):
            j = at_start.get(b)
            if a != b and j is not None and j != i and result[j] == a:
                revert.update((i, j))
        if not revert:
            return result, stopped
        for i in revert:
            result[i] = current[i]
        stopped += len(revert)


def check_paths(grid: Grid, paths: Sequence[Sequence[Cell]]) -> None:
    """Check full paths; a robot whose path ends early waits at its last cell."""
    if not paths:
        return
    if any(len(p) == 0 for p in paths):
        raise PlanError("every path needs at least its start cell")
    horizon = max(len(p) for p in paths)

    def at(p: Sequence[Cell], t: int) -> Cell:
        return p[min(t, len(p) - 1)]

    for t in range(horizon - 1):
        try:
            check_step(grid, [at(p, t) for p in paths], [at(p, t + 1) for p in paths])
        except PlanError as err:
            raise PlanError(f"step {t}->{t + 1}: {err}") from None

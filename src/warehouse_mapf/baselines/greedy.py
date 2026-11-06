"""Greedy one-step planner: a smoke test for the simulator, not a real baseline.

Each step, robots in a fixed order take the neighbouring cell that most reduces
their distance to the goal, if that cell is empty now and not yet claimed.
It never collides, but it has no lookahead and deadlocks easily in narrow
aisles. WHCA* and the hybrid planner should beat it by a wide margin.
"""

from __future__ import annotations

from warehouse_mapf.sim.grid import Cell
from warehouse_mapf.sim.simulator import SimView


class GreedyPlanner:
    name = "greedy"

    def step(self, view: SimView) -> list[Cell]:
        occupied_now = set(view.positions)
        claimed: set[Cell] = set()
        nxt: list[Cell] = list(view.positions)

        # Robots that stay (busy or already at the goal) keep their cell.
        movers = []
        for i, pos in enumerate(view.positions):
            if view.busy[i] or pos == view.goals[i]:
                claimed.add(pos)
            else:
                movers.append(i)

        for i in movers:
            pos, table = view.positions[i], view.distances.table(view.goals[i])
            options = [c for c in view.grid.neighbors(pos) if 0 <= table[c] < table[pos]]
            options.sort(key=lambda c: table[c])
            for cell in options:
                if cell not in occupied_now and cell not in claimed:
                    nxt[i] = cell
                    break
            claimed.add(nxt[i])
        return nxt

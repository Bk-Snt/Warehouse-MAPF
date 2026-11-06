"""Grid map: free cells, obstacles and 4-connected moves.

Cells are (row, col) tuples. Maps load from the MovingAI benchmark format
(https://movingai.com/benchmarks/formats.html) or from plain text lines.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np

Cell = tuple[int, int]

# N, E, S, W as (d_row, d_col). "Wait" is handled by the planner, not the grid.
MOVES: tuple[Cell, ...] = ((-1, 0), (0, 1), (1, 0), (0, -1))

# MovingAI terrain: '.', 'G' and 'S' are passable; '@', 'O', 'T', 'W' are not.
_FREE_CHARS = frozenset(".GS")


@dataclass(frozen=True, eq=False)
class Grid:
    """A 4-connected grid. ``free[r, c]`` is True where a robot may stand."""

    free: np.ndarray

    def __post_init__(self) -> None:
        if self.free.ndim != 2 or self.free.dtype != bool:
            raise ValueError("free must be a 2-D boolean array")

    @property
    def height(self) -> int:
        return int(self.free.shape[0])

    @property
    def width(self) -> int:
        return int(self.free.shape[1])

    @classmethod
    def from_lines(cls, lines: list[str]) -> Grid:
        """Build a grid from text rows, e.g. ``["..@", "..."]``."""
        rows = [line.rstrip("\r\n") for line in lines if line.strip()]
        if not rows:
            raise ValueError("map has no rows")
        width = len(rows[0])
        if any(len(r) != width for r in rows):
            raise ValueError("all map rows must have the same width")
        free = np.array([[ch in _FREE_CHARS for ch in r] for r in rows], dtype=bool)
        return cls(free)

    @classmethod
    def from_movingai(cls, path: str | Path) -> Grid:
        """Load a MovingAI ``.map`` file."""
        lines = Path(path).read_text().splitlines()
        header: dict[str, str] = {}
        for i, line in enumerate(lines):
            if line.strip() == "map":
                body = lines[i + 1 :]
                break
            key, _, value = line.partition(" ")
            header[key] = value.strip()
        else:
            raise ValueError(f"{path}: missing 'map' line")
        grid = cls.from_lines(body)
        if (grid.height, grid.width) != (int(header["height"]), int(header["width"])):
            raise ValueError(f"{path}: size does not match header")
        return grid

    def in_bounds(self, cell: Cell) -> bool:
        r, c = cell
        return 0 <= r < self.height and 0 <= c < self.width

    def is_free(self, cell: Cell) -> bool:
        return self.in_bounds(cell) and bool(self.free[cell])

    def neighbors(self, cell: Cell) -> list[Cell]:
        """Free cells one move away (N, E, S, W order)."""
        r, c = cell
        return [(r + dr, c + dc) for dr, dc in MOVES if self.is_free((r + dr, c + dc))]

    def free_cells(self) -> list[Cell]:
        return [(int(r), int(c)) for r, c in zip(*np.nonzero(self.free), strict=True)]

    def distances_from(self, source: Cell) -> np.ndarray:
        """BFS distance from ``source`` to every cell (-1 = unreachable).

        On a unit-cost grid this equals the true-distance heuristic that
        WHCA* gets from RRA*; the planner caches one table per goal cell.
        """
        if not self.is_free(source):
            raise ValueError(f"source {source} is not a free cell")
        dist = np.full(self.free.shape, -1, dtype=np.int32)
        dist[source] = 0
        queue = deque([source])
        while queue:
            cell = queue.popleft()
            for nxt in self.neighbors(cell):
                if dist[nxt] < 0:
                    dist[nxt] = dist[cell] + 1
                    queue.append(nxt)
        return dist

    def is_connected(self) -> bool:
        """True if every free cell can reach every other free cell."""
        cells = self.free_cells()
        if not cells:
            return True
        return bool((self.distances_from(cells[0])[self.free] >= 0).all())

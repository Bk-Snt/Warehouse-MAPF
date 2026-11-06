"""Phase 3 deadlock experiment on the narrow-aisle warehouse.

Run from the repo root:
    python experiments/phase3_deadlocks.py --robots 60 --seeds 0 1 2 3 4 5 6 7 8 9

A run counts as deadlocked if, after a 200-step warm-up, any 100-step window
finishes 2 tasks or fewer. Prints a Markdown table and appends every run to
results/phase3_deadlocks.csv.
"""

from __future__ import annotations

import argparse
import csv
import statistics
import time
from pathlib import Path

from warehouse_mapf.planner.fallback import DeadlockFallbackPlanner
from warehouse_mapf.planner.pibt import PIBTPlanner
from warehouse_mapf.sim.simulator import Simulator
from warehouse_mapf.sim.warehouse import Warehouse

ROOT = Path(__file__).resolve().parents[1]
VARIANTS = {
    "PIBT, nearest assignment": ("nearest", False),
    "PIBT + LaCAM fallback, nearest assignment": ("nearest", True),
    "PIBT, balanced assignment": ("balanced", False),
    "PIBT + LaCAM fallback, balanced assignment": ("balanced", True),
}


def run_one(warehouse, variant, robots, seed, steps) -> dict:
    assignment, fallback = VARIANTS[variant]
    sim = Simulator(warehouse, robots, seed=seed, assignment=assignment)
    planner = PIBTPlanner(seed=seed)
    if fallback:
        planner = DeadlockFallbackPlanner(planner, seed=seed)
    start = time.perf_counter()
    windows = []
    for _ in range(steps // 100):
        before = sim.metrics.tasks_completed
        sim.run(planner, 100)
        windows.append(sim.metrics.tasks_completed - before)
    return {
        "variant": variant,
        "robots": robots,
        "seed": seed,
        "throughput": sim.metrics.throughput,
        "worst_window": min(windows[2:]),
        "deadlocked": int(min(windows[2:]) <= 2),
        "fixes_solved": getattr(planner, "solved", 0),
        "fixes_failed": getattr(planner, "failed", 0),
        "ms_per_step": 1000 * (time.perf_counter() - start) / steps,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--layout", type=Path, default=ROOT / "maps/warehouse-narrow-32x32.json")
    parser.add_argument("--robots", type=int, default=60)
    parser.add_argument("--seeds", type=int, nargs="+", default=list(range(10)))
    parser.add_argument("--steps", type=int, default=1000)
    parser.add_argument("--variants", nargs="+", choices=list(VARIANTS), default=list(VARIANTS))
    parser.add_argument("--out", type=Path, default=ROOT / "results/phase3_deadlocks.csv")
    args = parser.parse_args()

    warehouse = Warehouse.load(args.layout)
    rows = [
        run_one(warehouse, variant, args.robots, seed, args.steps)
        for variant in args.variants
        for seed in args.seeds
    ]
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("a" if args.out.exists() else "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        if f.tell() == 0:
            writer.writeheader()
        writer.writerows(rows)

    print(f"{warehouse.name}, {args.robots} robots, {args.steps} steps, {len(args.seeds)} seeds\n")
    print("| Variant | Deadlocked seeds | Tasks / 1,000 steps | LaCAM fixes (solved / failed) |")
    print("| --- | --- | --- | --- |")
    for variant in args.variants:
        sel = [r for r in rows if r["variant"] == variant]
        tp = [r["throughput"] for r in sel]
        sd = statistics.stdev(tp) if len(tp) > 1 else 0.0
        fixes = (
            f"{sum(r['fixes_solved'] for r in sel)} / {sum(r['fixes_failed'] for r in sel)}"
            if VARIANTS[variant][1]
            else "–"
        )
        print(
            f"| {variant} | {sum(r['deadlocked'] for r in sel)} of {len(sel)} "
            f"| {statistics.mean(tp):.0f} ± {sd:.0f} | {fixes} |"
        )


if __name__ == "__main__":
    main()

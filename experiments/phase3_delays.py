"""Phase 3 delay experiment: how execution copes when robots run late.

Run from the repo root:
    python experiments/phase3_delays.py --robots 60 --seeds 0 1 2

Each commanded move fails with probability p, and robots occasionally stall
for 5 steps (0.2% chance per move once p > 0). Collisions are moves the
emergency stop had to cancel. Prints a Markdown table and appends every run to
results/phase3_delays.csv.
"""

from __future__ import annotations

import argparse
import csv
import statistics
import time
from pathlib import Path

from warehouse_mapf.planner.pibt import PIBTPlanner
from warehouse_mapf.planner.windowed import WindowedPlanner
from warehouse_mapf.sim.simulator import Simulator
from warehouse_mapf.sim.warehouse import Warehouse

ROOT = Path(__file__).resolve().parents[1]
VARIANTS = {
    "PIBT, replan every step": lambda seed: PIBTPlanner(seed=seed),
    "Window plans, naive execution": lambda seed: WindowedPlanner(
        PIBTPlanner(seed=seed), adg=False
    ),
    "Window plans + ADG": lambda seed: WindowedPlanner(PIBTPlanner(seed=seed), adg=True),
}


def run_one(warehouse, variant, robots, seed, steps, delay) -> dict:
    sim = Simulator(
        warehouse,
        robots,
        seed=seed,
        assignment="balanced",
        delay_prob=delay,
        stall_prob=0.002 if delay > 0 else 0.0,
        strict=False,
    )
    start = time.perf_counter()
    m = sim.run(VARIANTS[variant](seed), steps)
    return {
        "variant": variant,
        "robots": robots,
        "seed": seed,
        "delay": delay,
        "throughput": m.throughput,
        "collisions_per_1000": 1000 * m.collisions / steps,
        "ms_per_step": 1000 * (time.perf_counter() - start) / steps,
    }


def fmt(values: list[float]) -> str:
    sd = statistics.stdev(values) if len(values) > 1 else 0.0
    return f"{statistics.mean(values):.0f} ± {sd:.0f}"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--layout", type=Path, default=ROOT / "maps/warehouse-small-32x32.json")
    parser.add_argument("--robots", type=int, default=60)
    parser.add_argument("--delays", type=float, nargs="+", default=[0.0, 0.05, 0.1, 0.2])
    parser.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2])
    parser.add_argument("--steps", type=int, default=500)
    parser.add_argument("--out", type=Path, default=ROOT / "results/phase3_delays.csv")
    args = parser.parse_args()

    warehouse = Warehouse.load(args.layout)
    rows = [
        run_one(warehouse, variant, args.robots, seed, args.steps, delay)
        for delay in args.delays
        for variant in VARIANTS
        for seed in args.seeds
    ]

    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("a" if args.out.exists() else "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        if f.tell() == 0:
            writer.writeheader()
        writer.writerows(rows)

    print(f"{args.robots} robots, {args.steps} steps, seeds {args.seeds}\n")
    print("| Delay p | Execution | Tasks / 1,000 steps | Collisions / 1,000 steps |")
    print("| --- | --- | --- | --- |")
    for delay in args.delays:
        for variant in VARIANTS:
            sel = [r for r in rows if r["delay"] == delay and r["variant"] == variant]
            print(
                f"| {delay:.0%} | {variant} | {fmt([r['throughput'] for r in sel])} "
                f"| {fmt([r['collisions_per_1000'] for r in sel])} |"
            )


if __name__ == "__main__":
    main()

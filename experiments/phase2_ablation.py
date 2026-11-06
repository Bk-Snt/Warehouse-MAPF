"""Phase 2 ablation: what congestion guidance and balanced assignment each add to PIBT.

Run from the repo root:
    python experiments/phase2_ablation.py --robots 40 80 120 --seeds 0 1 2

Prints a Markdown table (mean +/- std over seeds) and writes every run to
results/phase2_ablation.csv.
"""

from __future__ import annotations

import argparse
import csv
import statistics
import time
from pathlib import Path

import numpy as np

from warehouse_mapf.planner.pibt import PIBTPlanner
from warehouse_mapf.sim.simulator import Simulator
from warehouse_mapf.sim.warehouse import Warehouse

ROOT = Path(__file__).resolve().parents[1]
VARIANTS = {
    "PIBT": {"guidance": False, "assignment": "nearest"},
    "+ balanced assignment": {"guidance": False, "assignment": "balanced"},
    "+ congestion guidance": {"guidance": True, "assignment": "nearest"},
    "+ both": {"guidance": True, "assignment": "balanced"},
}


def run_one(warehouse: Warehouse, variant: str, robots: int, seed: int, steps: int) -> dict:
    cfg = VARIANTS[variant]
    sim = Simulator(warehouse, robots, seed=seed, assignment=cfg["assignment"])
    planner = PIBTPlanner(seed=seed, guidance=cfg["guidance"])
    start = time.perf_counter()
    m = sim.run(planner, steps)
    return {
        "variant": variant,
        "robots": robots,
        "seed": seed,
        "throughput": m.throughput,
        "p99_errand": float(np.percentile(m.errand_times, 99)),
        "max_errand": m.max_errand_time,
        "ms_per_step": 1000 * (time.perf_counter() - start) / steps,
    }


def fmt(values: list[float], digits: int = 0) -> str:
    mean = statistics.mean(values)
    sd = statistics.stdev(values) if len(values) > 1 else 0.0
    return f"{mean:.{digits}f} ± {sd:.{digits}f}"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--layout", type=Path, default=ROOT / "maps/warehouse-small-32x32.json")
    parser.add_argument("--robots", type=int, nargs="+", default=[40, 80, 120])
    parser.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2])
    parser.add_argument("--steps", type=int, default=1000)
    parser.add_argument("--out", type=Path, default=ROOT / "results/phase2_ablation.csv")
    args = parser.parse_args()

    warehouse = Warehouse.load(args.layout)
    rows = [
        run_one(warehouse, variant, n, seed, args.steps)
        for n in args.robots
        for variant in VARIANTS
        for seed in args.seeds
    ]

    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("a" if args.out.exists() else "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        if f.tell() == 0:
            writer.writeheader()
        writer.writerows(rows)

    print("| Robots | Variant | Tasks / 1,000 steps | p99 errand (steps) | ms / step |")
    print("| --- | --- | --- | --- | --- |")
    for n in args.robots:
        for variant in VARIANTS:
            sel = [r for r in rows if r["robots"] == n and r["variant"] == variant]
            print(
                f"| {n} | {variant} | {fmt([r['throughput'] for r in sel])} "
                f"| {fmt([r['p99_errand'] for r in sel])} "
                f"| {fmt([r['ms_per_step'] for r in sel], 1)} |"
            )


if __name__ == "__main__":
    main()

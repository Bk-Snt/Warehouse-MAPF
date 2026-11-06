"""Run one simulation from the command line.

Example:
    python -m warehouse_mapf.run --robots 20 --steps 1000 --planner whca
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from warehouse_mapf.baselines.greedy import GreedyPlanner
from warehouse_mapf.baselines.whca import WHCAStarPlanner
from warehouse_mapf.planner.fallback import DeadlockFallbackPlanner
from warehouse_mapf.planner.pibt import PIBTPlanner
from warehouse_mapf.planner.windowed import WindowedPlanner
from warehouse_mapf.sim.simulator import Simulator
from warehouse_mapf.sim.warehouse import Warehouse

PLANNERS = {"greedy": GreedyPlanner, "whca": WHCAStarPlanner, "pibt": PIBTPlanner}
DEFAULT_LAYOUT = Path(__file__).resolve().parents[2] / "maps" / "warehouse-small-32x32.json"


def main(argv: list[str] | None = None) -> dict[str, float]:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--layout", type=Path, default=DEFAULT_LAYOUT)
    parser.add_argument("--planner", choices=sorted(PLANNERS), default="greedy")
    parser.add_argument("--robots", type=int, default=20)
    parser.add_argument("--steps", type=int, default=1000)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--log", type=Path, help="save a replay log (.npz) here")
    parser.add_argument("--window", type=int, default=16, help="WHCA* window w")
    parser.add_argument("--replan-every", type=int, default=1, help="WHCA* replan period h")
    parser.add_argument("--priority", choices=["fixed", "shuffle"], default="fixed")
    parser.add_argument("--guidance", action="store_true", help="PIBT: congestion guidance")
    parser.add_argument("--assignment", choices=["nearest", "balanced"], default="nearest")
    parser.add_argument(
        "--execution",
        choices=["step", "naive", "adg"],
        default="step",
        help="PIBT: replan every step, or 8-step window plans run naively or with an ADG",
    )
    parser.add_argument(
        "--fallback", action="store_true", help="PIBT: LaCAM fallback for stuck robots"
    )
    parser.add_argument("--delay", type=float, default=0.0, help="chance a move is delayed")
    parser.add_argument("--stall-prob", type=float, default=0.0, help="chance of a 5-step stall")
    args = parser.parse_args(argv)

    warehouse = Warehouse.load(args.layout)
    sim = Simulator(
        warehouse,
        args.robots,
        seed=args.seed,
        record=args.log is not None,
        assignment=args.assignment,
        delay_prob=args.delay,
        stall_prob=args.stall_prob,
        strict=args.delay == 0 and args.stall_prob == 0,
    )
    planner_params: dict[str, object] = {}
    if args.planner == "whca":
        planner_params = {
            "window": args.window,
            "replan_every": args.replan_every,
            "priority": args.priority,
        }
        planner = WHCAStarPlanner(**planner_params, seed=args.seed)
    elif args.planner == "pibt":
        planner_params = {
            "guidance": args.guidance,
            "execution": args.execution,
            "fallback": args.fallback,
        }
        if args.fallback and args.execution != "step":
            parser.error("--fallback currently works with --execution step only")
        planner = PIBTPlanner(seed=args.seed, guidance=args.guidance)
        if args.fallback:
            planner = DeadlockFallbackPlanner(planner, seed=args.seed)
        elif args.execution != "step":
            planner = WindowedPlanner(planner, adg=args.execution == "adg")
    else:
        planner = PLANNERS[args.planner]()

    start = time.perf_counter()
    metrics = sim.run(planner, args.steps)
    elapsed = time.perf_counter() - start

    result = {
        "planner": args.planner,
        "layout": warehouse.name,
        "robots": args.robots,
        "seed": args.seed,
        "assignment": args.assignment,
        **planner_params,
        **metrics.summary(),
        "ms_per_step": round(1000 * elapsed / max(args.steps, 1), 3),
    }
    print(json.dumps(result, indent=2))
    if args.log:
        sim.save_log(args.log)
    return result


if __name__ == "__main__":
    main()

# Warehouse MAPF

[![CI](https://github.com/Bk-Snt/Warehouse-MAPF-/actions/workflows/ci.yml/badge.svg)](https://github.com/Bk-Snt/Warehouse-MAPF-/actions/workflows/ci.yml)

Lifelong multi-agent path finding for warehouse robots: a hybrid PIBT + LNS planner with
congestion guidance, LaCAM deadlock fallback and delay-robust execution, benchmarked against
WHCA\*, PIBT, RHCR and LaCAM. Includes a replay dashboard with a congestion heat map.

> **Status:** Phase 1 done (simulator and WHCA\* baseline). Phase 2 (PIBT and congestion
> guidance) is next.

## The problem

Hundreds of robots share a grid warehouse and never stop. Each robot loops through a task cycle:

1. Go to an inbound station and load a product.
2. Take it to the product's storage location.
3. Deliver to a picking or outbound station.
4. Receive the next task immediately and repeat.

The planner must keep every robot collision-free, avoid deadlocks and starvation, keep
throughput high as the fleet grows, and stay safe when robots run late.

## Approach

| Layer | What it does | Borrowed from |
| --- | --- | --- |
| Rolling horizon | Plan *w* steps ahead, replan every *h* steps | WHCA\*, RHCR |
| Guided heuristic | True distance to the goal plus a traffic cost on busy cells | WHCA\* (RRA\*), traffic-flow guidance |
| Window plan | Priority inheritance with backtracking: fast, no starvation | PIBT |
| Refinement | Anytime large-neighbourhood search with space-time A\* repairs | MAPF-LNS, WHCA\* |
| Deadlock fallback | Solve a stuck area with a complete solver | LaCAM |
| Execution | Action dependency graph: delays cause waits, never collisions | ADG |

## Results so far

Lifelong runs on the 32×32 warehouse, 1,000 steps, seed 0. Every step is collision-checked.

| Robots | Greedy (tasks / 1,000 steps) | WHCA\* (tasks / 1,000 steps) | WHCA\* longest errand (steps) |
| --- | --- | --- | --- |
| 10 | 9 | 106 | 60 |
| 20 | 15 | 212 | 65 |
| 30 | 8 | 247 | 155 |

The greedy planner deadlocks within the first ~100 steps. WHCA\* scales well to 20 robots,
then gains less per robot as queues form at stations, and the longest errand time grows,
a sign of robots waiting a long time for their turn. Reproduce a row with:

```bash
python -m warehouse_mapf.run --planner whca --robots 30 --steps 1000
```

## Roadmap

| Phase | Scope | Gate |
| --- | --- | --- |
| 1 | Simulator and WHCA\* baseline | WHCA\* runs lifelong on the 32×32 map with zero collisions |
| 2 | PIBT and congestion guidance | Higher throughput than WHCA\*, no robot starves |
| 3 | LNS, deadlock fallback, robust execution | No deadlocks across 10 seeds; zero collisions with delays |
| 4 | C++ core and full benchmarks | 500 robots under 1 s per step; every chart reproducible |
| 5 | Dashboard and heat map | Public write-up and demo video |

## Repository layout

```text
src/warehouse_mapf/
  sim/          grid, robots, stations, delay model, ADG executor
  tasks/        task generator, assignment policies
  planner/      guided heuristic, PIBT, LNS, LaCAM fallback
  baselines/    WHCA*, adapters for PIBT / RHCR / LaCAM
experiments/    configs, runner, analysis
dashboard/      replay UI with congestion heat map
maps/           hand-made and MovingAI benchmark maps
tests/
```

## Getting started

Requires Python 3.10+.

```bash
python -m venv .venv
source .venv/bin/activate        # Windows: .venv\Scripts\activate
pip install -e ".[dev]"
pytest
```

## License

MIT. See [LICENSE](LICENSE).

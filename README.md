# Warehouse MAPF

[![CI](https://github.com/Bk-Snt/Warehouse-MAPF-/actions/workflows/ci.yml/badge.svg)](https://github.com/Bk-Snt/Warehouse-MAPF-/actions/workflows/ci.yml)

Lifelong multi-agent path finding for warehouse robots: a hybrid PIBT + LNS planner with
congestion guidance, LaCAM deadlock fallback and delay-robust execution, benchmarked against
WHCA\*, PIBT, RHCR and LaCAM. Includes a replay dashboard with a congestion heat map.

> **Status:** Phase 2 in progress: PIBT done; congestion guidance and assignment policies next.

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
Throughput is tasks completed per 1,000 steps; the longest errand (in steps) shows starvation.

| Robots | Greedy | WHCA\* | PIBT | WHCA\* longest errand | PIBT longest errand |
| --- | --- | --- | --- | --- | --- |
| 10 | 9 | 106 | 109 | 60 | 56 |
| 20 | 15 | 212 | 207 | 65 | 55 |
| 30 | 8 | 247 | 302 | 155 | 56 |
| 40 | – | 146 | 387 | 253 | 65 |
| 60 | – | – | 512 | – | 92 |
| 120 | – | – | 603 | – | 237 |

- Greedy deadlocks within the first ~100 steps.
- WHCA\* matches PIBT up to 20 robots, then robots crowd the stations and throughput falls.
- PIBT keeps scaling and plans in about 1 ms per step against WHCA\*'s 5–30 ms. Beyond ~60
  robots its errand times grow as traffic builds up, which congestion guidance targets next.

Reproduce a row with:

```bash
python -m warehouse_mapf.run --planner pibt --robots 40 --steps 1000
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

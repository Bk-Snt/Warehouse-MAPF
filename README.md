# Warehouse MAPF

[![CI](https://github.com/Bk-Snt/Warehouse-MAPF-/actions/workflows/ci.yml/badge.svg)](https://github.com/Bk-Snt/Warehouse-MAPF-/actions/workflows/ci.yml)

Lifelong multi-agent path finding for warehouse robots: a hybrid PIBT + LNS planner with
congestion guidance, LaCAM deadlock fallback and delay-robust execution, benchmarked against
WHCA\*, PIBT, RHCR and LaCAM. Includes a replay dashboard with a congestion heat map.

> **Status:** Phase 3 in progress: robust execution (delays + ADG) and the LaCAM deadlock
> fallback are done; LNS refinement is next.

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
  robots its errand times grow as traffic builds up; the Phase 2 ablation below tackles this.

Reproduce a row with:

```bash
python -m warehouse_mapf.run --planner pibt --robots 40 --steps 1000
```

### Phase 2 ablation: what each part adds to PIBT

Mean ± standard deviation over seeds 0–2, 1,000 steps each. p99 errand = the 99th-percentile
time a robot needs for one errand; lower means less waiting and no starvation.

| Robots | Variant | Tasks / 1,000 steps | p99 errand (steps) | ms / step |
| --- | --- | --- | --- | --- |
| 40 | PIBT | 382 ± 5 | 58 ± 4 | 0.8 |
| 40 | + balanced assignment | 388 ± 1 | 54 ± 2 | 0.9 |
| 40 | + congestion guidance | **411 ± 5** | 54 ± 5 | 4.4 |
| 40 | + both | 408 ± 1 | **49 ± 1** | 5.0 |
| 80 | PIBT | 505 ± 56 | 170 ± 46 | 1.4 |
| 80 | + balanced assignment | 609 ± 11 | 82 ± 2 | 1.5 |
| 80 | + congestion guidance | 536 ± 60 | 188 ± 53 | 5.8 |
| 80 | + both | **638 ± 5** | **80 ± 1** | 7.2 |
| 120 | PIBT | 541 ± 61 | 280 ± 66 | 1.8 |
| 120 | + balanced assignment | 621 ± 19 | **144 ± 3** | 2.0 |
| 120 | + congestion guidance | 586 ± 61 | 305 ± 63 | 6.7 |
| 120 | + both | **645 ± 15** | 151 ± 3 | 8.3 |

- **Balanced assignment is the biggest win in crowded runs:** at 80–120 robots it adds
  15–21% throughput, halves the p99 errand time and removes most of the seed-to-seed variance.
  It picks the station with the lowest travel plus expected queueing time, and sends at most
  one robot to a shelf at a time. Sending every robot to its nearest station overloads a few.
- **Congestion guidance adds 5–8% throughput** by charging extra for driving against recent
  traffic, so robots form lanes. It costs 4–5× more planning time, mostly Python Dijkstra
  searches, which the Phase 4 C++ port targets.
- With both, PIBT completes 645 tasks per 1,000 steps with 120 robots, against WHCA\*'s peak of
  247 at 30 robots.

Reproduce with `python experiments/phase2_ablation.py --robots 40 80 120 --seeds 0 1 2`, or
a single run with `python -m warehouse_mapf.run --planner pibt --guidance --assignment balanced`.

### Phase 3: robust execution when robots run late

Real robots slip, brake and pause. In these runs each commanded move fails with probability
*p* and robots occasionally stall for 5 steps. A collision is a move the simulator's emergency
stop had to cancel. 60 robots, balanced assignment, 500 steps, mean ± std over seeds 0–2.

| Delay p | Execution | Tasks / 1,000 steps | Collisions / 1,000 steps |
| --- | --- | --- | --- |
| 0% | PIBT, replan every step | 503 ± 6 | 0 |
| 0% | Window plans + ADG | 417 ± 25 | 0 |
| 5% | PIBT, replan every step | 473 ± 6 | 570 ± 82 |
| 5% | Window plans, naive execution | 474 ± 5 | 663 ± 141 |
| 5% | Window plans + ADG | 395 ± 16 | **0** |
| 10% | PIBT, replan every step | 445 ± 4 | 1,037 ± 48 |
| 10% | Window plans, naive execution | 449 ± 3 | 1,027 ± 45 |
| 10% | Window plans + ADG | 385 ± 8 | **0** |
| 20% | PIBT, replan every step | 395 ± 9 | 1,687 ± 86 |
| 20% | Window plans, naive execution | 397 ± 7 | 1,721 ± 72 |
| 20% | Window plans + ADG | 352 ± 10 | **0** |

- **Without an ADG, every delay rate causes hundreds of collisions.** PIBT packs robots
  nose to tail, so when a leader is delayed the robot behind drives into it.
- **With the action dependency graph there are zero collisions at every delay rate**, also
  in strict mode, where a single conflicting command would stop the run. A robot only enters
  a cell once the robot planned before it has left.
- **Safety costs 11–17% throughput**, because robots keep a one-cell gap instead of moving
  nose to tail. The gap shrinks as delays grow (17% at 0% delay, 11% at 20%).
- Building window plans uncovered a deadlock: some knots of robots can only be untangled by
  rotating around a loop, which delays make unsafe. The planner now bans one move in the loop
  and lets PIBT find another way out, instead of freezing the whole loop.

Reproduce with `python experiments/phase3_delays.py --robots 60 --seeds 0 1 2`, or a single
run with `python -m warehouse_mapf.run --planner pibt --execution adg --delay 0.1`.

### Phase 3: deadlocks on a narrow-aisle warehouse

`maps/warehouse-narrow-32x32` has one-lane aisles, and every other aisle is split into two
dead ends. A run counts as deadlocked if, after a 200-step warm-up, any 100-step window
finishes 2 tasks or fewer. 60 robots, 1,000 steps, seeds 0–9.

| Variant | Deadlocked seeds | Tasks / 1,000 steps | LaCAM fixes (solved / failed) |
| --- | --- | --- | --- |
| PIBT, nearest assignment | 3 of 10 | 179 ± 76 | – |
| PIBT + LaCAM fallback, nearest assignment | 2 of 10 | 188 ± 60 | 141 / 179 |
| PIBT, balanced assignment | **0 of 10** | **279 ± 15** | – |
| PIBT + LaCAM fallback, balanced assignment | **0 of 10** | 274 ± 16 | 18 / 19 |

- **The deadlocks came from task assignment, not path planning.** With nearest assignment, a
  popular product in a dead-end aisle drew a dozen robots at once and they jammed the aisle.
  Sending at most one robot to a shelf at a time removes the deadlocks in all 10 seeds and
  raises throughput by 56%.
- **The LaCAM fallback solves small local deadlocks that PIBT and WHCA\* cannot**, such as
  two robots meeting head-on in a corridor that only a side pocket can resolve. It watches
  each robot's progress and, after 60 steps without progress, plans a short escape for the
  stuck robot and its neighbours with LaCAM, a complete search. It cannot untangle a jam of
  a dozen robots within its 0.2 s budget, and firing it during ordinary congestion cost
  throughput, so it now triggers only on clear deadlocks and costs about 2% when unused.

Reproduce with `python experiments/phase3_deadlocks.py --robots 60`.

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

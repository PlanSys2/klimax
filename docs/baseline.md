# Baseline: the existing HTN planners

What Klimax has to beat. Every planner below was built from source and run on the benchmark of
the HTN tracks of IPC 2023, on one machine, under the same limits.

## Method

- **Benchmark**: `ipc2023-htn/ipc2023-domains` (commit `46a5665`): 761 total-order problems in
  23 domains and 283 partial-order problems in 11 domains.
- **Limits**: one core (each run is pinned to its own), 8 GB of address space, and the time
  limit. The time includes parsing and grounding; it does not include converting the plan back
  to the input model or verifying it.
- **Verification**: a problem counts as solved only if the IPC verifier (`pandaPIparser -v`)
  accepts the plan. Its lenient mode is the fallback for planners that normalize names.
- **Time limits**: 100 ms, 1 s and 30 s. The official competition ran 30 minutes; those results
  are at <https://ipc2023-htn.github.io/results>.
- **Machine**: Intel(R) Core(TM) Ultra 9 285H, 10 runs in parallel on 16 cores.
- The entries of IPC 2023 were built with their own recipe (in the recipe's base image) and run
  with the options of that recipe.

## Coverage

### Total order: 761 problems

| Planner | 100 ms | 1 s | 30 s |
|---|---|---|---|
| PandaDealer agile-1 | 155 | 318 | 494 |
| PandaDealer agile-lama | 162 | 308 | 494 |
| PANDA lamda (TO) | 140 | 278 | 487 |
| PANDA ppro (TO) | 138 | 275 | 450 |
| Linear-Complex agile-2 | 147 | 303 | 449 |
| Linear-Simple agile-2 | 148 | 260 | 390 |
| TOAD io-ff | 60 | 215 | 350 |
| HyperTensioN | 52 | 298 | 332 |
| Lilotane | 73 | 173 | 309 |
| pandaPIengine defaults | 102 | 170 | 252 |
| PDDL4J LiftedTreePath | 11 | 140 | 244 |
| SIADEX | 16 | 83 | 120 |
| Aries | 8 | 29 | 60 |
| *Solved by at least one* | | | 589 |

### Partial order: 283 problems

| Planner | 100 ms | 1 s | 30 s |
|---|---|---|---|
| Linear-Simple agile-2 | 128 | 161 | 207 |
| Linear-Complex agile-2 | 123 | 162 | 204 |
| PANDA ppro (PO) | 101 | 118 | 156 |
| PANDA lamda (PO) | 90 | 113 | 151 |
| pandaPIengine defaults | 89 | 110 | 149 |
| Aries | 34 | 62 | 106 |
| SIADEX | 28 | 85 | 93 |
| *Solved by at least one* | | | 219 |

The order agrees with the official results of IPC 2023: PandaDealer and the PANDA entry lead
total order, ahead of TOAD, LiftedTreePath, SIADEX and Aries; the Linear planners lead partial
order, ahead of PANDA, Aries and SIADEX. HyperTensioN and Lilotane did not take part in 2023.

## Planners

| Name here | Planner | Configuration |
|---|---|---|
| PandaDealer agile-1, agile-lama | PandaDealer, IPC 2023 | `rc2(add)` with dead-end pruning; `lama(lmc, add, gbfs)` |
| PANDA ppro, lamda | PANDA entry, IPC 2023 | Total order: `ppro-to-agl-gbfs-add`, `lamda-to-agl-gbfs-lmc`. Partial order: `ppro-po-agl-gas-ff`, `lamda-po-agl-gas-lmc` |
| Linear-Simple, Linear-Complex | IPC 2023 | Agile configuration 2 of each |
| TOAD io-ff | IPC 2023 | `toad-io-ff` |
| PDDL4J LiftedTreePath | IPC 2023 | `LiftedTreePath` |
| SIADEX | IPC 2023 | Its only configuration |
| Aries | `plaans/aries` | `aries-plan`, defaults |
| HyperTensioN | `Maumagnaguagno/HyperTensioN` | IPC 2020 options: `typredicate pullup dejavu` |
| Lilotane | `domschrei/lilotane` | Defaults, CaDiCaL |
| pandaPIengine defaults | `panda-planner-dev` | Greedy best-first with RC-FF |

Total-order-only planners: PandaDealer, TOAD, LiftedTreePath, HyperTensioN and Lilotane.

## By domain, at 30 s

Problems solved. `#` is the number of problems, `Any` the number solved by at least one planner.

### Total order

| Domain | # | Dealer | Dealer lama | lamda | ppro | Lin-C | Lin-S | TOAD | Hype | Lilotane | defaults | PDDL4J | SIADEX | Aries | Any |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| AssemblyHierarchical | 30 | **30** | 29 | **30** | **30** | 22 | 23 | **30** | 3 | 4 | 4 | 2 | 0 | 2 | 30 |
| Barman-BDI | 20 | 16 | 16 | 16 | 15 | 18 | 17 | 12 | 18 | 16 | 8 | 16 | **19** | 3 | 19 |
| Blocksworld-GTOHP | 30 | 27 | 27 | **29** | **29** | 26 | 25 | 19 | 14 | 23 | 25 | 22 | 10 | 3 | 30 |
| Blocksworld-HPDDL | 30 | **24** | 20 | 20 | 23 | 22 | 21 | 19 | 21 | 1 | 5 | 0 | 0 | 0 | 24 |
| Depots | 30 | 22 | 25 | 25 | 22 | **26** | **26** | 23 | 23 | 22 | 18 | 22 | 21 | 7 | 26 |
| Factories-simple | 20 | 8 | **9** | **9** | 7 | 6 | 6 | 4 | 3 | 4 | 5 | 4 | 0 | 1 | 9 |
| Freecell-Learned-ECAI-16 | 60 | 5 | 5 | 0 | 1 | **7** | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 7 |
| Hiking | 30 | 22 | 22 | 21 | 22 | 23 | 22 | 11 | **25** | 22 | 5 | 15 | 0 | 1 | 25 |
| Lamps | 30 | **17** | **17** | 16 | 16 | 14 | 16 | 16 | 3 | 0 | 14 | 0 | 0 | 10 | 17 |
| Logistics-Learned-ECAI-16 | 80 | **74** | 69 | 69 | 41 | 71 | 39 | 32 | 22 | 26 | 28 | 1 | 0 | 0 | 80 |
| Minecraft-Player | 20 | 1 | 1 | 1 | 1 | 1 | 1 | 0 | **5** | 2 | 1 | 1 | 3 | 0 | 5 |
| Minecraft-Regular | 59 | 35 | 36 | 34 | 34 | 36 | 34 | 34 | **49** | 30 | 34 | 25 | 20 | 0 | 50 |
| Monroe-Fully-Observable | 20 | 15 | 15 | 14 | 12 | 17 | 14 | 0 | **20** | **20** | 10 | 17 | 6 | 6 | 20 |
| Monroe-Partially-Observable | 20 | 7 | 8 | 7 | 7 | 7 | 7 | 0 | 0 | **19** | 6 | 16 | 0 | 6 | 20 |
| Multiarm-Blocksworld | 74 | **74** | **74** | 72 | 73 | 40 | 30 | 70 | 8 | 2 | 11 | 0 | 1 | 0 | 74 |
| Robot | 20 | **20** | **20** | **20** | **20** | **20** | **20** | 19 | 14 | 11 | 11 | 4 | 0 | 1 | 20 |
| Rover-GTOHP | 30 | 18 | 17 | 19 | 19 | 17 | 18 | 8 | **27** | 17 | 8 | 18 | 26 | 5 | 27 |
| Satellite-GTOHP | 20 | 13 | 12 | 12 | 15 | 12 | 12 | 10 | **16** | 11 | 7 | 12 | 0 | 5 | 16 |
| SharpSAT | 21 | 10 | 10 | 10 | 10 | 10 | 9 | 0 | **11** | 7 | 8 | 4 | 0 | 0 | 12 |
| Snake | 27 | **3** | **3** | 2 | 2 | **3** | 2 | 2 | 2 | 1 | 2 | 0 | 0 | 0 | 3 |
| Towers | 20 | **11** | **11** | **11** | **11** | **11** | 10 | 10 | 9 | 8 | 10 | 4 | 10 | 3 | 11 |
| Transport | 40 | 22 | 30 | **33** | 23 | 21 | 19 | 7 | 32 | **33** | 14 | 31 | 1 | 7 | 34 |
| Woodworking | 30 | 20 | 18 | 17 | 17 | 19 | 19 | 24 | 7 | **30** | 18 | **30** | 3 | 0 | 30 |

### Partial order

| Domain | # | Lin-S | Lin-C | ppro | lamda | defaults | Aries | SIADEX | Any |
|---|---|---|---|---|---|---|---|---|---|
| Barman-BDI | 20 | 16 | 17 | 2 | 1 | 1 | 1 | **19** | 19 |
| Colouring | 30 | **5** | **5** | **5** | **5** | **5** | 4 | 0 | 5 |
| Monroe-Fully-Observable | 25 | **21** | 18 | 18 | 16 | 15 | 8 | 8 | 23 |
| Monroe-Partially-Observable | 25 | 14 | **15** | 11 | 9 | 10 | 8 | 1 | 20 |
| PCP | 17 | **14** | **14** | **14** | **14** | **14** | 9 | 0 | 14 |
| Rover | 20 | **20** | **20** | 6 | 7 | 4 | 12 | 14 | 20 |
| Satellite | 25 | **25** | **25** | **25** | **25** | **25** | **25** | **25** | 25 |
| Transport | 40 | **23** | 22 | 13 | 11 | 11 | 12 | 1 | 24 |
| UM-Translog | 22 | **22** | **22** | **22** | **22** | **22** | 0 | **22** | 22 |
| Ultralight-Cockpit | 29 | **29** | 28 | **29** | **29** | **29** | 16 | 0 | 29 |
| Woodworking | 30 | **18** | **18** | 11 | 12 | 13 | 11 | 3 | 18 |

## Notes

- **Aries** returns plans the verifier rejects in three problems of Robot (total order): the plan
  crosses a door that does not connect the two rooms. They count as unsolved.
- **TOAD** writes an empty plan in four problems (Hiking, Blocksworld-GTOHP): its own
  verification step discards every candidate. They count as unsolved.
- **PandaDealer agile-lama** found a plan for one problem of Factories-simple whose conversion to
  the input model did not finish in 300 s. It counts as unsolved.
- **SIADEX** and **LiftedTreePath** run out of memory often (220 and 97 total-order problems).
- All configurations but the PANDA entry were run in two passes (1 s, then 30 s for what was
  left), so a run close to 1 s had two chances at the 100 ms and 1 s columns. The PANDA entry was
  run once at 30 s.
- Not measured: optimal configurations, the satisficing configurations with plan improvement,
  PDDL4J OptiPlan and the lifted Linear planner.

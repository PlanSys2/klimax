# Results

Klimax against the planners of [the baseline](baseline.md), on the benchmark of the HTN tracks of
IPC 2023. Same machine, same limits (one core, 8 GB), every plan verified with the IPC verifier.
The method is that of the baseline.

## Total order: 761 problems

| Planner | 100 ms | 1 s | 30 s |
|---|---|---|---|
| **Klimax** | 346 | 492 | 620 |
| PandaDealer agile-1 | 155 | 318 | 494 |
| PandaDealer agile-lama | 162 | 308 | 494 |
| PANDA lamda | 140 | 278 | 487 |
| PANDA ppro | 138 | 275 | 450 |
| Linear-Complex | 147 | 303 | 449 |
| Linear-Simple | 148 | 260 | 390 |
| TOAD | 60 | 215 | 350 |
| HyperTensioN | 52 | 298 | 332 |
| Lilotane | 73 | 173 | 309 |
| pandaPIengine defaults | 102 | 170 | 252 |
| PDDL4J LiftedTreePath | 11 | 140 | 244 |
| SIADEX | 16 | 83 | 120 |
| Aries | 8 | 29 | 60 |

By domain at 30 s, against the best of the other configurations in each domain (not one planner: the best of each row). Klimax is ahead in 12 domains, level in 8 and behind in 3.

| Domain | # | Klimax | Best other | Which |
|---|---|---|---|---|
| AssemblyHierarchical | 30 | **30** | **30** | PANDA lamda, PANDA ppro, PandaDealer agile-1, TOAD |
| Barman-BDI | 20 | **20** | 19 | SIADEX |
| Blocksworld-GTOHP | 30 | **30** | 29 | PANDA lamda, PANDA ppro |
| Blocksworld-HPDDL | 30 | **29** | 24 | PandaDealer agile-1 |
| Depots | 30 | **28** | 26 | Linear-Complex, Linear-Simple |
| Factories-simple | 20 | **9** | **9** | PANDA lamda, PandaDealer agile-lama |
| Freecell-Learned-ECAI-16 | 60 | **14** | 7 | Linear-Complex |
| Hiking | 30 | 22 | **25** | HyperTensioN |
| Lamps | 30 | **17** | **17** | PandaDealer agile-1, PandaDealer agile-lama |
| Logistics-Learned-ECAI-16 | 80 | 71 | **74** | PandaDealer agile-1 |
| Minecraft-Player | 20 | **5** | **5** | HyperTensioN |
| Minecraft-Regular | 59 | **54** | 49 | HyperTensioN |
| Monroe-Fully-Observable | 20 | **20** | **20** | HyperTensioN, Lilotane |
| Monroe-Partially-Observable | 20 | 17 | **19** | Lilotane |
| Multiarm-Blocksworld | 74 | **74** | **74** | PandaDealer agile-1, PandaDealer agile-lama |
| Robot | 20 | **20** | **20** | Linear-Complex, Linear-Simple, PANDA lamda, PANDA ppro,... |
| Rover-GTOHP | 30 | **30** | 27 | HyperTensioN |
| Satellite-GTOHP | 20 | **20** | 16 | HyperTensioN |
| SharpSAT | 21 | **17** | 11 | HyperTensioN |
| Snake | 27 | **5** | 3 | Linear-Complex, PandaDealer agile-1, PandaDealer agile-... |
| Towers | 20 | **18** | 11 | Linear-Complex, PANDA lamda, PANDA ppro, PandaDealer ag... |
| Transport | 40 | **40** | 33 | Lilotane, PANDA lamda |
| Woodworking | 30 | **30** | **30** | Lilotane, PDDL4J LiftedTreePath |

## Partial order: 283 problems

| Planner | 100 ms | 1 s | 30 s |
|---|---|---|---|
| **Klimax** | 147 | 193 | 251 |
| Linear-Simple | 128 | 161 | 207 |
| Linear-Complex | 123 | 162 | 204 |
| PANDA ppro | 101 | 118 | 156 |
| PANDA lamda | 90 | 113 | 151 |
| pandaPIengine defaults | 89 | 110 | 149 |
| Aries | 34 | 62 | 106 |
| SIADEX | 28 | 85 | 93 |

By domain at 30 s, against the best of the other configurations in each domain (not one planner: the best of each row). Klimax is ahead in 6 domains, level in 5 and behind in 0.

| Domain | # | Klimax | Best other | Which |
|---|---|---|---|---|
| Barman-BDI | 20 | **20** | 19 | SIADEX |
| Colouring | 30 | **8** | 5 | Linear-Complex, Linear-Simple, PANDA lamda, PANDA ppro,... |
| Monroe-Fully-Observable | 25 | **23** | 21 | Linear-Simple |
| Monroe-Partially-Observable | 25 | **20** | 15 | Linear-Complex |
| PCP | 17 | **14** | **14** | Linear-Complex, Linear-Simple, PANDA lamda, PANDA ppro,... |
| Rover | 20 | **20** | **20** | Linear-Complex, Linear-Simple |
| Satellite | 25 | **25** | **25** | Aries, Linear-Complex, Linear-Simple, PANDA lamda, PAND... |
| Transport | 40 | **40** | 23 | Linear-Simple |
| UM-Translog | 22 | **22** | **22** | Linear-Complex, Linear-Simple, PANDA lamda, PANDA ppro,... |
| Ultralight-Cockpit | 29 | **29** | **29** | Linear-Simple, PANDA lamda, PANDA ppro, pandaPIengine d... |
| Woodworking | 30 | **30** | 18 | Linear-Complex, Linear-Simple |

## Length of the plans

The score of the satisficing tracks of the IPC: for each problem solved, the length of the
shortest plan that any planner found divided by the length of its own; the sum over the track.

| | Klimax | Best other |
|---|---|---|
| Total order | 569.6 | 475.0 (PandaDealer agile-lama) |
| Partial order | 245.2 | 196.2 (Linear-Simple) |

The search of the ground problem gives plans close to the shortest ones. The search that does
not ground gives long ones where it has to explore (Lamps, Satellite, Rover, Barman): after its
first plan it looks, for a moment, for one of half the length.

## Optimal: the shortest plan, proved

Klimax with `--anytime` counts when it proves that no plan has fewer actions. Against the
optimal recipes of IPC 2023 that need no LP solver, one run at 30 s per track, on the same cores
as the tables above:

| Total order (761) | 100 ms | 1 s | 30 s |
|---|---|---|---|
| **Klimax** | 163 | 239 | 319 |
| PandaDealer optimal | 79 | 128 | 206 |
| PANDA optimal (`rc2(lmc)`) | 57 | 118 | 192 |

| Partial order (283) | 100 ms | 1 s | 30 s |
|---|---|---|---|
| **Klimax** | 78 | 106 | 142 |
| PANDA optimal (`rc2(lmc)`) | 73 | 95 | 119 |

Where Klimax and another planner both prove a plan the shortest, the lengths agree: in 210 of
210 total-order problems and in 114 of 114 partial-order ones.

In total order Klimax proves at least as many as the others in every domain; the widest gaps
are Minecraft-Regular (52 against 23), Hiking (16 against 1), Satellite (20 against 5),
Transport (16 against 6) and Monroe-Fully-Observable (20 against 11). In partial order the
proof needs the search that interleaves subtasks. It runs first without the relaxation, cut
by the bound on the length alone: where the space under the bound is small, the relaxation
costs more at each node than it saves (Monroe: 35 proved, none with it). Klimax proves more
in Monroe (35 against 12), Ultralight-Cockpit (29 against 25) and Transport (4 against 3),
PANDA in Satellite (23 against 21) and Woodworking (10 against 7). The search keeps every node
in memory and stops at its limit in Colouring, Rover and Transport.

## Notes

- The Klimax rows come from one run at 30 s per track, as those of the baseline do, on the same
  cores.
- A totally ordered problem is searched in three ways that share the time: over the ground
  problem with heuristics; depth first over the problem as written, without grounding it; and
  as a satisfiability problem, by layers of decomposition over the ground problem.
  - The second one solves the problems whose ground form is too large (Snake, SharpSAT, Rover)
    and domains written as a recipe (Blocksworld, Hiking). It tries first the methods whose
    actions undo less of the goal: Blocksworld-GTOHP goes from 23 to 30 with that.
  - The third one solves the domains whose methods leave many parameters free and where only
    the goal tells which values were right (Woodworking: 30 against 16 without it; Monroe,
    partially observable: 17 against 8).
  - The share costs the first one problems that it solved near the limit (Logistics).
- A partially ordered problem is searched over the ground problem and by satisfiability, which
  takes each task network in one of the orders it allows (Woodworking: 30 against 16).
- Klimax answers "unsolvable" on 8 total-order problems of Minecraft-Player: the grounding proves
  that the initial tasks cannot be decomposed into reachable actions. No planner of the baseline
  solves them. They count as not solved in the tables.
- It also answers "unsolvable" on problems 19 and 20 of Towers, and they are: both files lack
  `(smallerThan r3 r18)`, `(smallerThan r12 r18)` and `(smallerThan r15 r18)` (three other facts
  are written twice instead), and the methods of the domain leave no other way to move those
  rings. No planner of the baseline solves them either.
- Plans of megabytes (Towers) run the IPC verifier out of its time; Klimax's verifier decides
  then, for any planner. The two agree on every plan compared ([semantics](semantics.md),
  section 7).
- 44 total-order problems are not grounded in 10 s (Snake, SharpSAT, Minecraft-Player, Hiking):
  tens of millions of instances. The search that does not ground solves some of them (Snake
  from 2 to 7, SharpSAT from 10 to 17); it has no heuristic, and the rest need one.
- Over the ground problem, a partially ordered problem is searched both with each task network
  in one order it allows and interleaving the subtasks of different tasks. In Colouring and PCP the second runs out of its
  memory on the larger problems.

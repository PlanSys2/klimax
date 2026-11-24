# Results

Klimax against the planners of [the baseline](baseline.md), on the benchmark of the HTN tracks of
IPC 2023. Same machine, same limits (one core, 8 GB), every plan verified with the IPC verifier.
The method is that of the baseline.

## Total order: 761 problems

| Planner | 100 ms | 1 s | 30 s |
|---|---|---|---|
| **Klimax** | 220 | 383 | 554 |
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

By domain at 30 s, against the best of the other configurations in each domain (not one planner: the best of each row). Klimax is ahead in 7 domains, level in 5 and behind in 11.

| Domain | # | Klimax | Best other | Which |
|---|---|---|---|---|
| AssemblyHierarchical | 30 | **30** | **30** | PANDA lamda, PANDA ppro, PandaDealer agile-1, TOAD |
| Barman-BDI | 20 | **20** | 19 | SIADEX |
| Blocksworld-GTOHP | 30 | 22 | **29** | PANDA lamda, PANDA ppro |
| Blocksworld-HPDDL | 30 | 23 | **24** | PandaDealer agile-1 |
| Depots | 30 | **29** | 26 | Linear-Complex, Linear-Simple |
| Factories-simple | 20 | 7 | **9** | PANDA lamda, PandaDealer agile-lama |
| Freecell-Learned-ECAI-16 | 60 | **12** | 7 | Linear-Complex |
| Hiking | 30 | 17 | **25** | HyperTensioN |
| Lamps | 30 | 16 | **17** | PandaDealer agile-1, PandaDealer agile-lama |
| Logistics-Learned-ECAI-16 | 80 | **74** | **74** | PandaDealer agile-1 |
| Minecraft-Player | 20 | **5** | **5** | HyperTensioN |
| Minecraft-Regular | 59 | **53** | 49 | HyperTensioN |
| Monroe-Fully-Observable | 20 | **20** | **20** | HyperTensioN, Lilotane |
| Monroe-Partially-Observable | 20 | 8 | **19** | Lilotane |
| Multiarm-Blocksworld | 74 | **74** | **74** | PandaDealer agile-1, PandaDealer agile-lama |
| Robot | 20 | 18 | **20** | Linear-Complex, Linear-Simple, PANDA lamda, PANDA ppro,... |
| Rover-GTOHP | 30 | 22 | **27** | HyperTensioN |
| Satellite-GTOHP | 20 | **20** | 16 | HyperTensioN |
| SharpSAT | 21 | 10 | **11** | HyperTensioN |
| Snake | 27 | 2 | **3** | Linear-Complex, PandaDealer agile-1, PandaDealer agile-... |
| Towers | 20 | **17** | 11 | Linear-Complex, PANDA lamda, PANDA ppro, PandaDealer ag... |
| Transport | 40 | **40** | 33 | Lilotane, PANDA lamda |
| Woodworking | 30 | 15 | **30** | Lilotane, PDDL4J LiftedTreePath |

## Partial order: 283 problems

| Planner | 100 ms | 1 s | 30 s |
|---|---|---|---|
| **Klimax** | 148 | 193 | 234 |
| Linear-Simple | 128 | 161 | 207 |
| Linear-Complex | 123 | 162 | 204 |
| PANDA ppro | 101 | 118 | 156 |
| PANDA lamda | 90 | 113 | 151 |
| pandaPIengine defaults | 89 | 110 | 149 |
| Aries | 34 | 62 | 106 |
| SIADEX | 28 | 85 | 93 |

By domain at 30 s, against the best of the other configurations in each domain (not one planner: the best of each row). Klimax is ahead in 4 domains, level in 6 and behind in 1.

| Domain | # | Klimax | Best other | Which |
|---|---|---|---|---|
| Barman-BDI | 20 | **20** | 19 | SIADEX |
| Colouring | 30 | **8** | 5 | Linear-Complex, Linear-Simple, PANDA lamda, PANDA ppro,... |
| Monroe-Fully-Observable | 25 | **25** | 21 | Linear-Simple |
| Monroe-Partially-Observable | 25 | **15** | **15** | Linear-Complex |
| PCP | 17 | **14** | **14** | Linear-Complex, Linear-Simple, PANDA lamda, PANDA ppro,... |
| Rover | 20 | **20** | **20** | Linear-Complex, Linear-Simple |
| Satellite | 25 | **25** | **25** | Aries, Linear-Complex, Linear-Simple, PANDA lamda, PAND... |
| Transport | 40 | **40** | 23 | Linear-Simple |
| UM-Translog | 22 | **22** | **22** | Linear-Complex, Linear-Simple, PANDA lamda, PANDA ppro,... |
| Ultralight-Cockpit | 29 | **29** | **29** | Linear-Simple, PANDA lamda, PANDA ppro, pandaPIengine d... |
| Woodworking | 30 | 16 | **18** | Linear-Complex, Linear-Simple |

## Optimal: the shortest plan, proved

Klimax with `--anytime` counts when it proves that no plan has fewer actions. Against the
optimal recipes of IPC 2023 that need no LP solver, at 1 s (longer limits are still to be
measured):

| Planner | Total order (761) | Partial order (283) |
|---|---|---|
| **Klimax** | 240 | 91 |
| PandaDealer optimal | 125 | - |
| PANDA optimal (`rc2(lmc)`) | 118 | 96 |

Where Klimax and another planner both prove a plan the shortest, the lengths agree: in 124 of
124 total-order problems and in 81 of 81 partial-order ones.

## Notes

- The Klimax rows come from one run at 30 s per track, as those of the baseline do. The
  total-order run was made on a machine about twice as slow as usual (every problem took about
  twice as long as in a run at 1 s made the same day), which lowers its 100 ms and 1 s columns: a
  separate run at 1 s solves 255 at 100 ms and 437 at 1 s. To be measured again.
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
  tens of millions of instances. They need a search that does not ground.
- A partially ordered problem is searched both with each task network in one order it allows and
  interleaving the subtasks of different tasks. In Colouring and PCP the second runs out of its
  memory on the larger problems.

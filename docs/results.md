# Results

Klimax against the planners of [the baseline](baseline.md), on the benchmark of the HTN tracks of
IPC 2023. Same machine, same limits (one core, 8 GB), every plan verified with the IPC verifier.
The method is that of the baseline.

## Total order: 761 problems

| Planner | 100 ms | 1 s | 30 s |
|---|---|---|---|
| **Klimax** | 341 | 484 | 626 |
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

By domain at 30 s, against the best of the other configurations in each domain (not one planner: the best of each row). Klimax is ahead in 13 domains, level in 7 and behind in 3.

| Domain | # | Klimax | Best other | Which |
|---|---|---|---|---|
| AssemblyHierarchical | 30 | **30** | **30** | PANDA lamda, PANDA ppro, PandaDealer agile-1, TOAD |
| Barman-BDI | 20 | **20** | 19 | SIADEX |
| Blocksworld-GTOHP | 30 | **30** | 29 | PANDA lamda, PANDA ppro |
| Blocksworld-HPDDL | 30 | **30** | 24 | PandaDealer agile-1 |
| Depots | 30 | **29** | 26 | Linear-Complex, Linear-Simple |
| Factories-simple | 20 | **9** | **9** | PANDA lamda, PandaDealer agile-lama |
| Freecell-Learned-ECAI-16 | 60 | **13** | 7 | Linear-Complex |
| Hiking | 30 | 22 | **25** | HyperTensioN |
| Lamps | 30 | 16 | **17** | PandaDealer agile-1, PandaDealer agile-lama |
| Logistics-Learned-ECAI-16 | 80 | **77** | 74 | PandaDealer agile-1 |
| Minecraft-Player | 20 | **5** | **5** | HyperTensioN |
| Minecraft-Regular | 59 | **54** | 49 | HyperTensioN |
| Monroe-Fully-Observable | 20 | **20** | **20** | HyperTensioN, Lilotane |
| Monroe-Partially-Observable | 20 | 18 | **19** | Lilotane |
| Multiarm-Blocksworld | 74 | **74** | **74** | PandaDealer agile-1, PandaDealer agile-lama |
| Robot | 20 | **20** | **20** | Linear-Complex, Linear-Simple, PANDA lamda, PANDA ppro,... |
| Rover-GTOHP | 30 | **29** | 27 | HyperTensioN |
| Satellite-GTOHP | 20 | **20** | 16 | HyperTensioN |
| SharpSAT | 21 | **17** | 11 | HyperTensioN |
| Snake | 27 | **5** | 3 | Linear-Complex, PandaDealer agile-1, PandaDealer agile-... |
| Towers | 20 | **18** | 11 | Linear-Complex, PANDA lamda, PANDA ppro, PandaDealer ag... |
| Transport | 40 | **40** | 33 | Lilotane, PANDA lamda |
| Woodworking | 30 | **30** | **30** | Lilotane, PDDL4J LiftedTreePath |

## Partial order: 283 problems

| Planner | 100 ms | 1 s | 30 s |
|---|---|---|---|
| **Klimax** | 144 | 189 | 251 |
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
| Total order | 573.8 | 475.0 (PandaDealer agile-lama) |
| Partial order | 245.3 | 196.2 (Linear-Simple) |

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

## Numbers

The numeric HTN benchmark of Quenard et al. ([HTN-Numerical-Benchmarks](https://github.com/gaspard-quenard/HTN-Numerical-Benchmarks)):
seven families of 20 problems with numeric fluents. At 30 s per problem, one core and 8 GB:

| Family | Klimax | Aries |
|---|---|---|
| Alchemy | 20 | 8 |
| Backpack | 18 | 3 |
| Gripper_colors | 16 | 8 |
| Minecraft | 20 | 3 |
| Overcooked | 19 | 0 |
| Transport_fuel | 13 | 1 |
| Transvasement | 20 | 6 |
| **Total** | **126 of 140** | **29 of 140** |

Within 100 ms and within 1 s Klimax solves 92 and 105. Aries solves 2 and 12 counting only
its solver, since it is run from Python (below), which takes half a second more.

- Aries is `up-aries` 0.5.0, through Unified Planning: its own HDDL reader takes no numbers,
  and it takes no real fluents, so the numeric fluents are declared as integers (every value
  in the benchmark is one). It does not always stop at its time limit: it is stopped then.
  Of the 111 it does not solve, 96 run out of time and 15 end without an answer (11 of them
  in Overcooked).
- Klimax solves every problem that Aries solves. In those 29 the plans of Aries add up to
  647 actions and those of Klimax to 689.
- The paper that presents the benchmark (Quenard et al., HPlan 2026) reports 91 for its
  planner SibylSmt, 48 for Aries and 42 for SIADEX, with 10 minutes per problem on another
  machine: those figures are theirs. No public code of SibylSmt was found to measure it here.
- No other verifier reads hierarchical plans with numbers: the plans of both are checked by
  Klimax's own, which is independent of its searches.
- In Transport_fuel the plan depends on where each truck fills up. No heuristic of Klimax
  reads the numbers; what finds those plans is that a task still to do always counts at
  least the fewest actions it takes, so going to fill up once more is never free. One of the
  methods of that domain leaves its subtasks without an order: by the semantics of HDDL it
  is a partially ordered domain.

## Time

A first measure, on the forty problems of
[klimax-benchmarks](https://github.com/PlanSys2/klimax-benchmarks) (four families of ten, of
growing size, with their generators and the harness that ran this). Each family has durative
actions and tasks that the initial network does not order:

- **Transport**: trucks deliver packages over roads of some length.
- **Satellite**: a satellite takes images of sites that are visible only for a while (timed
  initial literals).
- **Workshop**: pieces go through two machines in order; a machine does one piece at a time.
- **Cellar**: fuses are mended by the light of matches that burn for a while: mending has to
  overlap the match.

Both planners get the same problem through Unified Planning, at 30 s per problem, one core and
8 GB, with every plan checked by Klimax's verifier and, for Klimax, by Kairos' validator:

| Family | Klimax | Aries |
|---|---|---|
| Transport | 10 | 0 |
| Satellite | 10 | 10 |
| Workshop | 10 | 10 |
| Cellar | 10 | 10 |
| **Total** | **40 of 40** | **30 of 40** |

- Aries is `up-aries` 0.5.0. In Transport it runs out of time in five problems and gives a
  plan in the five smallest, which the verifier rejects: each `drive` lasts 0.1 more than the
  problem says (the length of its road). They are counted as not solved; with durations taken
  loosely they would be five more.
- In the thirty that both solve the plans of Klimax end at 3218 in all and those of Aries at
  3314. Klimax's is the shorter in fifteen and Aries' in seven.
- Both are run from Python. Within 1 s of the whole process Klimax solves 34 and Aries 29.
- Klimax finds a plan with one durative action after another and then lets Kairos' scheduler
  overlap what the hierarchy and the actions allow. In Satellite, where the scheduler does not
  read the timed initial literals, the plans stay with one action after another. In Cellar no
  such plan exists, and the start and the end of each action are planned as two steps.

## By feature

What each planner solves of the problems that use each feature of HDDL 2.1, as `klimax --check`
reports them (`docs/semantics.md`): all the problems above (IPC 2023, numeric and temporal) by
the features of their domain, and the families by feature of
[klimax-benchmarks](https://github.com/PlanSys2/klimax-benchmarks) (`features/`: object
fluents, action costs, conditional effects, orderings between time points, durative methods and
constraints on states; five problems with a plan in each). Same limits as above; a problem a
planner does not take (one it cannot read) counts as not solved. The plans of the families are
verified by Klimax's verifier, the IPC verifier reading only HDDL 1.0 (it ignores the
constraints on states that the plans of `rooms` must meet).

At 30 s:

| Feature | Problems | Klimax | Aries | PANDA | PANDA ppro | PANDA lamda | PandaDealer | PandaDealer lama | Linear-Simple | Linear-Complex | TOAD | SIADEX | Lilotane | HyperTensioN | PDDL4J |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| hierarchy | 1254 | **1066** | 225 | 407 | 614 | 646 | 494 | 494 | 605 | 661 | 350 | 214 | 309 | 332 | 244 |
| partial-order-initial | 274 | **248** | 104 | 101 | 106 | 105 | 0 | 0 | 151 | 151 | 0 | 85 | 0 | 0 | 0 |
| partial-order-methods | 141 | **127** | 43 | 76 | 80 | 76 | 0 | 0 | 86 | 83 | 0 | 31 | 0 | 0 | 0 |
| recursion | 1030 | **843** | 127 | 306 | 506 | 536 | 458 | 460 | 458 | 513 | 314 | 130 | 263 | 307 | 198 |
| method-preconditions | 1117 | **932** | 172 | 343 | 539 | 563 | 472 | 464 | 524 | 579 | 343 | 187 | 276 | 300 | 213 |
| free-method-parameters | 1202 | **1017** | 213 | 393 | 600 | 632 | 494 | 494 | 591 | 647 | 350 | 214 | 309 | 332 | 244 |
| empty-methods | 616 | **465** | 38 | 153 | 275 | 301 | 298 | 291 | 254 | 309 | 218 | 87 | 108 | 165 | 59 |
| method-constraints | 137 | **127** | 53 | 88 | 95 | 93 | 22 | 23 | 103 | 104 | 0 | 62 | 39 | 20 | 33 |
| goal | 718 | **647** | 149 | 271 | 439 | 433 | 330 | 325 | 398 | 409 | 259 | 112 | 197 | 201 | 166 |
| durative-actions | 50 | **50** | 30 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| timed-initial-literals | 10 | **10** | **10** | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| durative-methods | 5 | **5** | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| method-durations | 5 | **5** | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| time-point-orderings | 5 | **5** | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| state-constraints | 5 | **5** | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| numeric-fluents | 195 | **180** | 59 | 3 | 5 | 5 | 0 | 0 | 5 | 5 | 0 | 0 | 0 | 0 | 0 |
| action-costs | 5 | **5** | 0 | 3 | **5** | **5** | 0 | 0 | **5** | **5** | 0 | 0 | 0 | 0 | 0 |
| metric | 5 | **5** | 0 | 3 | **5** | **5** | 0 | 0 | **5** | **5** | 0 | 0 | 0 | 0 | 0 |
| object-fluents | 5 | **5** | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| conditional-effects | 5 | **5** | 0 | 3 | 3 | 3 | 0 | 0 | 3 | 3 | 0 | 1 | 0 | 0 | 0 |
| negative-preconditions | 942 | **767** | 126 | 299 | 497 | 520 | 441 | 435 | 474 | 528 | 309 | 170 | 238 | 284 | 179 |
| equality | 362 | **300** | 76 | 109 | 148 | 143 | 113 | 111 | 178 | 181 | 75 | 59 | 119 | 91 | 106 |
| universal-preconditions | 221 | **188** | 28 | 59 | 146 | 140 | 123 | 120 | 109 | 122 | 91 | 16 | 43 | 51 | 33 |
| metric-fluents | 5 | **5** | 0 | 3 | **5** | **5** | 0 | 0 | **5** | **5** | 0 | 0 | 0 | 0 | 0 |
| **all** | 1254 | 1066 | 225 | 407 | 614 | 646 | 494 | 494 | 605 | 661 | 350 | 214 | 309 | 332 | 244 |

At 1 s (the two PANDA configurations of IPC 2023 were measured at 30 s only):

| Feature | Problems | Klimax | Aries | PANDA | PandaDealer | PandaDealer lama | Linear-Simple | Linear-Complex | TOAD | SIADEX | Lilotane | HyperTensioN | PDDL4J |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| hierarchy | 1254 | **837** | 119 | 279 | 307 | 299 | 414 | 455 | 203 | 165 | 164 | 294 | 135 |
| partial-order-initial | 274 | **207** | 69 | 89 | 0 | 0 | 142 | 142 | 0 | 82 | 0 | 0 | 0 |
| partial-order-methods | 141 | **94** | 29 | 45 | 0 | 0 | 45 | 46 | 0 | 26 | 0 | 0 | 0 |
| recursion | 1030 | **655** | 42 | 197 | 279 | 269 | 284 | 321 | 183 | 88 | 127 | 270 | 100 |
| method-preconditions | 1117 | **719** | 86 | 224 | 285 | 270 | 340 | 379 | 197 | 138 | 139 | 262 | 109 |
| free-method-parameters | 1202 | **788** | 116 | 265 | 307 | 299 | 400 | 441 | 203 | 165 | 164 | 294 | 135 |
| empty-methods | 616 | **364** | 19 | 103 | 175 | 162 | 176 | 212 | 112 | 63 | 48 | 144 | 32 |
| method-constraints | 137 | **83** | 31 | 46 | 0 | 0 | 47 | 47 | 0 | 53 | 8 | 17 | 6 |
| goal | 718 | **522** | 80 | 189 | 211 | 202 | 261 | 277 | 160 | 85 | 112 | 182 | 88 |
| durative-actions | 50 | **44** | 29 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| timed-initial-literals | 10 | **10** | 9 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| durative-methods | 5 | **5** | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| method-durations | 5 | **5** | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| time-point-orderings | 5 | **5** | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| state-constraints | 5 | **5** | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| numeric-fluents | 195 | **155** | 37 | 1 | 0 | 0 | 5 | 5 | 0 | 0 | 0 | 0 | 0 |
| action-costs | 5 | **5** | 0 | 1 | 0 | 0 | **5** | **5** | 0 | 0 | 0 | 0 | 0 |
| metric | 5 | **5** | 0 | 1 | 0 | 0 | **5** | **5** | 0 | 0 | 0 | 0 | 0 |
| object-fluents | 5 | **5** | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| conditional-effects | 5 | **4** | 0 | 2 | 0 | 0 | 2 | 2 | 0 | 1 | 0 | 0 | 0 |
| negative-preconditions | 942 | **603** | 61 | 191 | 263 | 246 | 298 | 336 | 175 | 123 | 109 | 247 | 83 |
| equality | 362 | **205** | 24 | 48 | 64 | 65 | 94 | 99 | 40 | 43 | 64 | 86 | 57 |
| universal-preconditions | 221 | **130** | 6 | 12 | 59 | 51 | 36 | 48 | 42 | 7 | 9 | 45 | 6 |
| metric-fluents | 5 | **5** | 0 | 1 | 0 | 0 | **5** | **5** | 0 | 0 | 0 | 0 | 0 |
| **all** | 1254 | 837 | 119 | 279 | 307 | 299 | 414 | 455 | 203 | 165 | 164 | 294 | 135 |

- No planner gives a plan for the problems of the families that have none (`p06` of each).
  Klimax proves three of them unsolvable and reports the other three as not supported.
- Aries' invalid plans: three of the IPC (a method's precondition not applied, plaans/aries#284)
  and five of the temporal benchmark (durations 0.1 longer than the problem's). TOAD's four are
  of the IPC.
- Aries' numeric and temporal results are through Unified Planning (`up-aries` 0.5.0), which
  costs it about 0.65 s of Python at start: at 1 s that weighs; at 30 s it does not. Its
  executable reads none of the families by feature, which use what its HDDL reader does not
  take; through Unified Planning they cannot be expressed (orderings between time points,
  durative methods and constraints on states are not in it).
- Planners that ignore the metric still solve `couriers`: their plans are valid, not the
  cheapest.

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
    partially observable: 18 against 8).
  - The search by satisfiability gives up most of its part when it is not the way: where it
    is, the plan comes within a dozen layers and a few hundred conflicts.
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

# Roadmap

Klimax is being built in phases, each with a criterion it has to meet before the next one
starts. The measurements are in [results.md](results.md); the planners it is measured against,
in [baseline.md](baseline.md).

| Phase | State | Content | Criterion |
|---|---|---|---|
| H0 | done | The semantics of HDDL 2.1 in writing ([semantics.md](semantics.md)); the existing planners built and measured | The baseline reproduces the ranking of IPC 2023 |
| H1 | done | Parser of HDDL 1.0, model, grounding over the decomposition graph, plan verifier | Reads the whole IPC benchmark; the verifier agrees with the one of the IPC |
| H2 | done | Total-order search | Coverage at least that of the best planner of the baseline at 1 s and 30 s |
| H3 | done | Partial order: one linearization of each network, and a search that interleaves | Coverage at least that of the best planner of the baseline |
| H3b | done | Python module and Unified Planning engine ([up-klimax](https://github.com/PlanSys2/up-klimax)) | Hierarchical problems solved from Unified Planning, with the plans verified |
| H4 | done | Anytime search, optimal plans, finer heuristics | Best satisficing score; optimal coverage at least that of the baseline |
| H5 | in progress | Numbers | More problems of the numeric HTN benchmark than its authors' planner |
| H6 | planned | Time: the whole of HDDL 2.1, with a reference benchmark | Solves the benchmark with validated plans |
| H7 | planned | Robustness, fuzzing, PlanSys2 plugin, version 1.0.0 | As Kairos |

## Where it stands

- **Total and partial order** (H2, H3): ahead of every planner of the baseline in coverage at
  100 ms, 1 s and 30 s. By domain it is still behind some planner in several of them
  ([results.md](results.md) lists which).
- **Without grounding** (H4): a totally ordered problem is also searched as written, depth
  first, with the parameters that a method leaves free bound when a precondition needs them.
  The searches share the time.
- **By satisfiability** (H4): a problem is also searched as a formula, by layers
  of decomposition, with a SAT solver of its own. It is what solves the domains whose methods
  leave many parameters free (Woodworking, Monroe).
- **Numbers** (H5): numeric fluents in the conditions and effects of actions and methods and
  in the goal, in the verifier and in two of the three searches (the one by satisfiability
  leaves those problems to the others); in the Python module and in Unified Planning too.
  The heuristics do not see the numbers: where a resource decides the plan (fuel), what
  finds it is the search, which counts every pending task at least by the fewest actions it
  takes, so that repeating a detour is never free.
- **Anytime and optimal** (H4): after the first plan Klimax looks for shorter ones and proves
  when none exists, with total and with partial order.
- **Unified Planning** (H3b): a planner (oneshot and anytime), a validator of hierarchical plans
  and a compiler for hierarchical problems.

## Open in the current phase

- A heuristic for the search that does not ground: it is depth first in the order of the
  domain, and most of the total-order problems whose ground form is too large (Snake,
  Minecraft-Player) are still out of its reach.
- The search by satisfiability encodes the ground problem: its formula grows with the number
  of ground methods, and the sharing of the time costs the other searches problems that they
  solved near the limit. Encoding the problem as written, as Lilotane does, would make it
  smaller.
- Domains where a planner of the baseline solves more: Hiking, Lamps, Monroe (partially
  observable, total order), by one to three problems each.
- An optimal search with an admissible heuristic. With partial order, a proof that does not
  keep every node in memory: it is what stops Klimax in Colouring, Rover and Transport, though
  it proves more plans the shortest than the planners of the baseline in both tracks.

## Known limits

- Durative actions and temporal constraints are read by neither the planner nor the verifier
  yet: they are reported as not supported, never accepted. Of numbers, object fluents and the
  metric are not read either.
- With partial order, "unsolvable" is only answered when the search that interleaves exhausts
  its space; otherwise the answer is "not supported", with the reason.

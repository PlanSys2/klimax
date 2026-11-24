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
| H4 | in progress | Anytime search, optimal plans, finer heuristics | Best satisficing score; optimal coverage at least that of the baseline |
| H5 | next | Numbers | The numeric HTN benchmark |
| H6 | planned | Time: the whole of HDDL 2.1, with a reference benchmark | Solves the benchmark with validated plans |
| H7 | planned | Robustness, fuzzing, PlanSys2 plugin, version 1.0.0 | As Kairos |

## Where it stands

- **Total and partial order** (H2, H3): ahead of every planner of the baseline in coverage at
  100 ms, 1 s and 30 s. By domain it is still behind some planner in several of them
  ([results.md](results.md) lists which).
- **Anytime and optimal** (H4): after the first plan Klimax looks for shorter ones and proves
  when none exists, with total and with partial order.
- **Unified Planning** (H3b): a planner (oneshot and anytime), a validator of hierarchical plans
  and a compiler for hierarchical problems.

## Open in the current phase

- A search that does not ground, for the problems whose ground form is too large (44 of the
  total-order benchmark).
- Domains where a planner of the baseline solves more: Woodworking, Hiking, Rover,
  Blocksworld-GTOHP, Monroe (partially observable).
- An optimal search with an admissible heuristic, and the measure of the optimal track at
  longer time limits.

## Known limits

- Numbers, durative actions and temporal constraints are read by neither the planner nor the
  verifier yet: they are reported as not supported, never accepted.
- With partial order, "unsolvable" is only answered when the search that interleaves exhausts
  its space; otherwise the answer is "not supported", with the reason.

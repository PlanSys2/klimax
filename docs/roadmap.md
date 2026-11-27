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
| H5 | done | Numbers | More problems of the numeric HTN benchmark than its authors' planner |
| H6 | done | Time: the whole of HDDL 2.1, with a reference benchmark ([klimax-benchmarks](https://github.com/PlanSys2/klimax-benchmarks)) | Solves the benchmark with validated plans |
| H7 | done | Robustness, fuzzing, PlanSys2 plugin, version 1.0.0 | As Kairos |

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
- **Metric and action costs** (H5): the value of the problem's metric is given with every plan
  and by the verifier. When the metric adds up action costs (`(increase (total-cost) ...)` by
  constants or static functions, or any sum of fluents that nothing reads), the anytime search
  bounds the cost instead of the length and proves the cheapest plan, with total and with
  partial order. A fluent that nothing reads is no longer part of the state.
- **Object fluents** (H5): fluents whose value is an object, in conditions, effects and the
  goal. Kairos grounds them and Klimax turns them into atoms, so that the searches and their
  heuristics see them; the verifier follows them with the evaluator of Kairos' validator.
- **Time** (H6, first steps): durative actions, planned with each one as a single step, as
  Kairos does with temporally simple problems, or as its start and its end where actions must
  overlap. The plan is then timed by Kairos' scheduler:
  each action as early as what it reads and writes and the orderings of the hierarchy allow,
  so actions that neither orders run at once. Kairos' validator checks every plan as a temporal
  plan of PDDL 2.1. The verifier reads plans with times and durations, with actions that
  overlap, and checks all that HDDL 2.1 says of time in a method: orderings between the starts
  and the ends of subtasks, durative methods with their conditions at start, over all and at
  end, and constraints on the durations of a task and of its subtasks. Timed initial literals: a step waits for the one it needs (a clock that every step
  moves on, and each literal as the intervals of that clock in which it holds).
- **Constraints on the states of the plan** (H6, level L4): `before`, `after` and `between` a
  subtask, and along the task of a method `at start`, `at end`, `always`, `sometime`,
  `at-most-once`, `sometime-before`, `sometime-after`, `within`, `hold-after`, `hold-during`
  and `always-within`. The verifier checks them all. Of the searches, what must hold before a
  subtask is tested as a precondition is; the rest is met by the plan found or the problem is
  reported as not supported.
- **Anytime and optimal** (H4): after the first plan Klimax looks for better ones and proves
  when none exists, with total and with partial order.
- **Unified Planning** (H3b): a planner (oneshot and anytime), a validator of hierarchical plans
  and a compiler for hierarchical problems.

## Open

Of time (H6):

- The search with the start and the end of each durative action apart does not look at
  durations: it finds an order of starts and ends, and Kairos then gives it times or not.
- Timed initial literals together with actions that must overlap; Kairos' scheduler does not
  read timed initial literals, so plans with them stay with one action after another.
- Planning with the constraints on time points, on durations and on states that the verifier
  checks and the searches do not (all but `before` and the end of `between`).
- The reference benchmark has the temporal level only: numbers (L2) and constraints (L4) are
  to come.

From before:

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

- Of time: plans are found with one durative action after another, and only then made
  shorter. Where that gives no plan (an action needs what another gives only while it runs),
  the start and the end of each durative action are planned as two steps, and Kairos says when
  each takes place; but that search does not look at durations, so an order of starts and ends
  that no times fit is reported as not supported, as is a problem with no plan of either
  kind, never as unsolvable. Not with timed initial literals. The searches order subtasks only by "ends before it starts": the other
  constraints on time points and on durations, and what a durative method asks over all and at
  end, are met by the times given to the plan found or the problem is reported as not
  supported. Timed initial values of fluents and continuous change (`#t`) are not read yet;
  `?duration` inside conditions and effects is, with the duration the action is planned with
  (what it is equal to, or else the least it allows). An action that reads an atom of a timed
  initial
  literal is planned as needing it from its start to its end, whatever the domain says: some
  plans are not found. The verifier does follow
  plans whose actions overlap. So two actions that must start at once and last differently,
  with something after both, are reported as not supported: four of the five problems of the
  `meetings` family of [klimax-benchmarks](https://github.com/PlanSys2/klimax-benchmarks).
- Where a resource decides the plan the heuristics do not see it: the seven largest problems
  of Transport_fuel (58 to 82 places) run out of time, and take 0.15 s without the fuel.
- Of object fluents: two that must differ, or that must be equal in the goal, are not planned
  with (reported as not supported).
- The first plan is found without looking at the metric: only the anytime search uses it. With
  actions that cost nothing the cheapest plan is looked for among plans up to about twice as
  long as the first, and is not proved the cheapest. A metric that is not a sum of action
  costs (one that reads the state, or to maximize) is improved among the plans that are no
  longer than the last, without a proof.
- With partial order, "unsolvable" is only answered when the search that interleaves exhausts
  its space; otherwise the answer is "not supported", with the reason.

# The language of Klimax and its semantics

Klimax supports **HDDL 1.0 in full** and **HDDL 2.1**. This document says what Klimax accepts
and what a solution is. HDDL 2.1 is a proposal, not a closed standard: where it is silent or
ambiguous, this document takes a decision and marks it **[decision]**. Klimax implements these
decisions; they are proposed to the authors once the whole language works, and corrected then
if needed.

Sources:

- HDDL 1.0: Höller et al., *HDDL: An Extension to PDDL for Expressing Hierarchical Planning
  Problems*, AAAI 2020. The language of the HTN tracks of IPC 2020 and IPC 2023.
- HDDL 2.1: Pellier et al., in two papers, and the grammar in `github.com/pellierd/HDDL2.1`
  (`extened-bnf.tex`, commit of 2025-01-09):
  - arXiv 2206.01822 (2022), *Towards Defining an HTN Formalism with Time*: syntax, and
    semantics only for the method constraints. It leaves durations and time constraints across
    decompositions as open questions.
  - arXiv 2306.07353 (2023), *Towards Defining a Formalism and a Semantics for Temporal HTN
    Planning*: adds definitions of executable temporal task network, decomposition and solution.
    Where the two differ, this document follows the second.
- PDDL 2.1, 2.2 and 3.0 for everything HDDL 2.1 borrows. Klimax follows Kairos wherever a
  construct is plain PDDL.

## 1. Levels

A problem belongs to the lowest level whose features it uses. Klimax reports the level.

| Level | Language | Adds |
|---|---|---|
| L1 | HDDL 1.0 | Abstract tasks, methods, task networks with total or partial order, method preconditions, variable constraints, an optional state goal |
| L2 | HDDL 2.1, numeric | Numeric fluents in conditions and effects of actions and methods, metric |
| L3 | HDDL 2.1, temporal | Durative actions and methods, duration constraints, ordering over start and end points, timed initial literals |
| L4 | HDDL 2.1, constraints | Decomposition constraints (`before`, `after`, `between`, and the timed ones) |

L1 is what the competitions measure and it is implemented first. L2 to L4 are each a superset of
the level before; a problem may also be L2 and L4 without being L3.

## 2. Model

- **State**: as in PDDL. Truth values of ground atoms and, from L2, values of ground numeric
  fluents. Closed world.
- **Primitive task**: an action (L1) or a durative action (L3), with PDDL semantics.
- **Abstract task**: a name with typed parameters. It has no conditions or effects of its own.
- **Method**: decomposes one abstract task into a task network. It may have a precondition.
- **Task network**: a set of labelled task occurrences, ordering constraints between them, and
  constraints on variables and (L4) on states.
- **Problem**: objects, an initial state, an initial task network and, optionally, a state goal.

## 3. Solutions

### 3.1 L1 and L2: sequential

A solution is a sequence of ground actions together with a decomposition tree such that:

1. The roots of the tree are the tasks of the initial task network. Every inner node is an
   abstract task with the method applied to it, and its children are the subtasks of that method.
   Every leaf is an action.
2. The sequence contains exactly the leaves, in an order that respects every ordering constraint
   of the initial network and of every applied method. Orderings are inherited: if a method orders
   subtask `a` before subtask `b`, every leaf below `a` comes before every leaf below `b`.
3. The sequence is executable from the initial state, and the state goal, if any, holds at the end.
4. Variable constraints hold for the bindings chosen.
5. **Method precondition**: it holds in some state before the first leaf below that method
   occurrence, after everything the orderings place before the occurrence, and not before the
   precondition of the method above it. This is the semantics of the IPC verifier (a method
   precondition is a primitive task with no effect, ordered before all the subtasks). With a
   total order that state is the one just before the first leaf. A method with no subtasks
   tests its precondition at any point the orderings allow.

**No task insertion**: the plan contains only leaves of the tree. **[decision]** A problem with a
state goal and no `:htn` section is a PDDL problem; Klimax passes it to Kairos.

In L2, numeric conditions and effects follow PDDL 2.1 for sequential plans. A numeric method
precondition is tested as in point 5. **[decision]** A fluent has no value until the initial
state or an effect gives it one: a comparison that reads it does not hold, and an action with
an effect that reads it (or that divides by zero) is not applicable. All the effects of an
action read the state before it.

A fluent whose value is an object (`:object-fluents`) may have no value (`undefined`): an
atom or a comparison that reads it then does not hold. **[decision]** A function term is not
allowed as an argument of a task (in a method or in the initial task network): HDDL 2.1 does not
say when it would be evaluated. A parameter of the method bound by its precondition, as
`(= (in ?b) ?from)`, says it.

The metric of the problem (`(:metric minimize e)` or `maximize`) is the value of `e` in the
state the plan ends in; it does not change which plans are valid. **[decision]** As in the
action costs of PDDL, `(total-cost)` starts at 0 when the initial state does not say.

### 3.2 L3: temporal

Every task occurrence `t` has two time points, `start(t)` and `end(t)`, with
`start(t) ≤ end(t)`. A solution assigns a time to every point and is valid when:

1. **Durative actions** follow PDDL 2.1: conditions `at start`, `over all` and `at end`, effects
   `at start` and `at end`, and the duration constraint. The leaves, taken as a PDDL 2.1 plan,
   are valid, including its rules on concurrent actions (no moving targets; `ε` separation).
   Kairos' validator checks this part.
2. An **instantaneous action** in a temporal problem has `start = end`.
3. **Envelope**: for an abstract task `t` decomposed by a method with subtasks `s1 … sn`,
   `start(t) = min start(si)` and `end(t) = max end(si)`. With no subtasks, `start(t) = end(t)`.
   **[decision]** The 2023 paper only requires the task to contain its subtasks
   (`start(t) ≤ start(si)` and `end(t) ≥ end(si)`, in its definition of decomposition) and calls
   the duration of an abstract task "generally unbounded". The 2022 paper says "the sum" of the
   durations in one section and "the max" in another. Klimax takes the tightest interval the 2023
   constraints allow: with containment alone, a constraint such as `(>= ?duration 10)` would be
   met by any decomposition, since the task could always be stretched.
4. **Ordering constraints** compare time points with `<`, `<=`, `=`, `>=`, `>`. A bare task id
   stands for a point by its side: `(< a b)` means `end(a) < start(b)`, as in HDDL 1.0.
   `:ordered-subtasks` means `end(si) ≤ start(si+1)` **[decision]** (not strict: two consecutive
   instantaneous actions at the same instant are ordered by the sequence, as in PDDL 2.1 plans
   with `ε`). `(not (< a b))` is the complement comparison.
5. **Method duration constraints** compare `?duration`, which is `end(t) − start(t)` of the task
   decomposed, and `(duration id)`, which is `end − start` of a subtask, with numeric
   expressions. Expressions are evaluated in the state at `start(t)` **[decision]**, as PDDL 2.1
   does for actions.
6. **Durative method condition**: `at start` holds at `start(t)`, `at end` at `end(t)`, `over all`
   in the open interval between them. A plain `:precondition` on a durative method means
   `at start`.
7. **Timed initial literals** follow PDDL 2.2.
8. **Time**: rational, with the `ε` of PDDL 2.1 (Kairos' default, 0.001). **[decision]** This
   departs from the 2023 paper, which defines dates and durations as natural numbers to keep the
   problem decidable. Klimax keeps rationals for compatibility with PDDL plans and validators,
   and does not promise termination on problems whose only solutions need unbounded precision.
   A problem whose durations are integers has integer solutions under both readings whenever no
   `ε` separation is needed.
9. **Delays** between tasks (at least `d` after the end of another) are not in the language. They
   are modelled with an auxiliary durative action, as the proposal suggests.

### 3.3 L4: decomposition constraints

`before`, `after` and `between` take task ids and a condition:

- `(before id φ)`: `φ` holds in the state just before `start(id)`.
- `(after id φ)`: `φ` holds in the state just after `end(id)`.
- `(between id1 id2 φ)`: `φ` holds in every state from just after `end(id1)` to just before
  `start(id2)`.

The timed constraints (`at`, `hold-during`, `hold-after`, `always`, `sometime`, `at-most-once`,
`sometime-before`, `sometime-after`, `always-within`, `within`) have the semantics of the PDDL 3.0
trajectory constraints **restricted to the interval of the method that states them**
**[decision]**: from `start(t)` to `end(t)` of the decomposed task. In the initial task network
the interval is the whole plan. The 2023 paper evaluates them on the trajectory of "the task
network", and its decomposition merges the constraints of a method into those of the whole
network; read literally, an `always` stated in a method would then hold over the whole plan.

## 4. Syntax accepted

Klimax accepts the grammar of the HDDL 2.1 repository and the spellings found in the competition
domains and in the proposal's own examples:

- `:subtasks`, `:tasks`, `:ordered-subtasks`, `:ordered-tasks`; `:ordering` and `:order`.
- Subtasks with or without an id; a single subtask without `and`.
- `:precondition` and `:condition` on durative actions and methods (the examples of the proposal
  use both).
- Requirement `:numeric-fluents`, and also `:numeric-fluent` and `:fluents`.
- `:htn` in the problem, with optional `:parameters`. `:goal` is optional.

Points of the published grammar that Klimax reads differently, each one a mistake or an omission
in the text:

| Grammar | Klimax |
|---|---|
| `<durative-method-def>` has no opening parenthesis | `(:durative-method …)` |
| `<assign-op>` is used but only `increase` and `decrease` are listed | Also `assign`, `scale-up`, `scale-down`, as in PDDL 2.1 |
| `<d-op>` for durations lacks `<` and `>` while methods use `<binary-comp>` | Actions: `<=`, `>=`, `=`. Methods: all five |
| `(within <timed-task-id> <gd>)` has no deadline | `(within <number> <gd>)` as in PDDL 3.0; the form of the grammar is rejected with a message |
| `:duration` of a method is under `:duration-inequalities` | Accepted with `:durative-actions` when it is an equality |
| No `:derived`, no preferences | Derived predicates accepted (Kairos supports them); preferences rejected |
| No continuous effects | Rejected with a message |

Anything outside this is an error with file, line and column, never a silent approximation.

## 5. Plan format

- L1 and L2: the format of the IPC HTN tracks (`==>`, the numbered primitive plan, `root` and
  the decomposition lines), so that the IPC verifier can check Klimax's plans.
  As that verifier requires, the subtask ids of a decomposition line, and those of `root`, are
  written in an order the task network allows (each id after the ids ordered before it).
- L3 and L4: **[decision]** the same format with a time and a duration for every primitive step,
  `id time: (action args) [duration]`: after the id, the line of a PDDL 2.1 plan. The
  decomposition lines are the same. No standard exists; with the ids taken off, the steps are a
  plan that the PDDL temporal validators read, and with the times taken off, one that the IPC HTN
  tools read. The verifier also takes a plan without times, and gives its actions one after
  another.

## 6. Open questions

To propose to the authors of HDDL 2.1 once Klimax implements all of it, with the planner, the
benchmark and the verifier as evidence:

1. The interval of an abstract task: exactly its subtasks (3.2, point 3) or any interval that
   contains them, as the 2023 definition of decomposition allows.
2. Whether `:ordered-subtasks` forbids gaps between consecutive subtasks.
3. The interval of the timed constraints of a method (3.3).
4. Where the numeric expressions of a method duration constraint are evaluated.
5. A plan format for temporal hierarchical plans, and a verifier.
6. Natural numbers against rationals for time (3.2, point 8).

## 7. The plan verifier

`klimax --verify DOMAIN PROBLEM PLAN` checks a plan against section 3.1. It interprets the syntax
tree directly and shares no code with the search. It covers L1; a plan for anything else is
reported as unsupported, never accepted.

It was compared with the IPC verifier (`pandaPIparser -v`) on 1786 plans found by seven planners
on the IPC 2023 benchmark and on 17222 variations of them, each with one change (two actions
swapped or moved, an argument or a method replaced, a step removed, subtasks or roots
reordered):

| | Both accept | Both reject | Differ |
|---|---|---|---|
| Plans as found | 1565 | 3 | 0 |
| Variations | 462 | 16756 | 1 |

- The 3 plans both reject are Aries' invalid plans of the baseline.
- 218 more plans (and 3 variations) are accepted only by the lenient mode of the IPC verifier,
  because the planner changed the names: Klimax accepts the 59 that differ only in case (HDDL is
  case-insensitive) and rejects the 159 with other characters replaced.
- The one difference is a variation in which a method with one subtask is applied with none
  (Rover, `m-navigate_abs-3`). The IPC verifier accepts it; Klimax rejects it.

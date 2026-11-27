^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package klimax
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Time: among methods that are otherwise equal, the searches try first the one whose first
  action ends soonest in a schedule of the plan so far, with the orderings of the hierarchy.
  The work goes to whoever is free first, and plans overlap more: the factory of PlanSys2's
  behavior tree example, three cars with three robots, went from a makespan of 90 to 45 (one
  robot did nearly every errand); the workshop family of klimax-benchmarks, 10% shorter in all.
* Contributors: Francisco Martín Rico

1.0.0 (2026-11-27)
------------------
The first release: an HTN planner for HDDL 1.0 and HDDL 2.1, built on Kairos.

* Language: HDDL 1.0 in full, and HDDL 2.1: numbers and a metric, time (durative actions and
  methods, timed initial literals, orderings between time points, durations of methods), object
  fluents and constraints on the states of a plan. Where HDDL 2.1 is silent or ambiguous,
  ``docs/semantics.md`` says what Klimax takes, including what executing a plan means.
* Planning: total and partial order, grounded and lifted searches and a search by
  satisfiability, sharing the time; the first plan, better ones as they are found (anytime), or
  the shortest, proved. Every plan is verified, independently of the search, before it is
  returned.
* Time: plans with one durative action after another, given times that overlap what the
  hierarchy and the actions allow, and, where actions must overlap, their starts and ends planned
  apart.
* A verifier of hierarchical plans, temporal ones included (``klimax --verify``), and the plan
  format for them: that of the IPC HTN tracks with a time and a duration on each action.
* Interfaces: the ``klimax`` executable (``--tree`` and ``--timeline`` views), the C++ library,
  the Python module (``pip install klimax-planner``) and, through up-klimax, the Unified Planning
  Framework. In PlanSys2 it is the planner of HDDL mode.
* Results: against the planners of IPC 2023 on its benchmark, the numeric one of Quenard et al.
  and klimax-benchmarks, in ``docs/results.md``.
* Contributors: Francisco Martín Rico

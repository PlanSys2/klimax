# Klimax in Python

Klimax is an HTN planner for HDDL. This module gives its library to Python: planning, checking
a domain and a problem, and verifying plans.

```python
import klimax

domain = open("domain.hddl").read()
problem = open("problem.hddl").read()

result = klimax.solve(domain, problem, timeout=30)
if result.status == klimax.Status.SOLVED:
    print(result.plan)                 # the format of the IPC HTN tracks
    for step in result.steps_of_plan:  # each action; each task with its method and subtasks
        print(step.id, step.task, step.args, step.method, step.subtasks)
```

- `solve(domain, problem, timeout=0, decomposition=True, anytime=False, on_plan=None,
  verify=True)`: `status` is `SOLVED`, `UNSOLVABLE`
  (proved), `TIMEOUT`, `UNSUPPORTED`, `INVALID_INPUT` (see `diagnostics`) or `INTERNAL_ERROR`.
  Every plan is verified, independently of the search, before it is returned (`verify`).
  With `decomposition` it is also given as steps: of each task, its method, the values of the method's parameters and which
  step is each of its subtasks.
  With `anytime=True` it goes on after the first plan looking for shorter ones, calling
  `on_plan(plan, steps)` with each, until none is left or the time runs out; `optimal` says
  whether the last one was proved the shortest.
- `check(domain, problem)`: every error found, with file, line and column.
- `verify(domain, problem, plan, decomposition=False)`: whether a plan in the IPC format
  solves the problem, and why not; with `decomposition`, a valid plan also as steps.

Hierarchical problems with total or partial order and numeric fluents; without time. To build from
source, Kairos must be installed with `-DKAIROS_INSTALL_CORE=ON`:

```
cmake -S . -B build -DKLIMAX_BUILD_PYTHON=ON -DCMAKE_PREFIX_PATH=<kairos prefix>
cmake --build build
PYTHONPATH=build/python python -c "import klimax; print(klimax.__version__)"
```

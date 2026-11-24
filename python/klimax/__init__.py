# Copyright 2026 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Klimax, an HTN planner for HDDL, in Python.

    import klimax
    result = klimax.solve(open("domain.hddl").read(), open("problem.hddl").read(), timeout=30)
    if result.status == klimax.Status.SOLVED:
        print(result.plan)             # the format of the IPC HTN tracks
        for step in result.steps_of_plan:
            ...                        # each action, and each task with its method and subtasks

`solve` gives the plan verified, independently of the search. `check` reports every error of a
domain and a problem; `verify` checks a plan in the IPC format.
"""

from ._klimax import (
    CheckResult,
    Diagnostic,
    PlanStep,
    Severity,
    SolveResult,
    Status,
    Summary,
    VerifyResult,
    __version__,
    check,
    solve,
    verify,
)

__all__ = [
    "CheckResult", "Diagnostic", "PlanStep", "Severity", "SolveResult", "Status", "Summary",
    "VerifyResult", "__version__", "check", "solve", "verify",
]

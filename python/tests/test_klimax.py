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

import os
import pathlib

import klimax
import pytest

DATA = pathlib.Path(os.environ.get(
    "KLIMAX_TEST_DATA", pathlib.Path(__file__).parents[2] / "tests" / "data")) / "house"
DOMAIN = (DATA / "domain.hddl").read_text()


@pytest.mark.parametrize("name", ["p1-total-order", "p2-partial-order"])
def test_solves_and_the_plan_is_valid(name):
    problem = (DATA / f"{name}.hddl").read_text()
    result = klimax.solve(DOMAIN, problem, timeout=10)
    assert result.status == klimax.Status.SOLVED, result.message
    assert klimax.verify(DOMAIN, problem, result.plan).valid
    assert result.steps == len(result.sequence) > 0


def test_decomposition():
    problem = (DATA / "p1-total-order.hddl").read_text()
    result = klimax.solve(DOMAIN, problem)
    steps = {s.id: s for s in result.steps_of_plan}
    # Every step is reached from the tasks of the initial network, each action once
    seen = []

    def walk(step_id):
        step = steps[step_id]
        if step.primitive:
            assert not step.method and not step.subtasks
            seen.append(step_id)
            return
        assert step.method and len(step.subtasks) == len(step.labels)
        for child in step.subtasks:
            walk(child)
    assert len(result.roots) == len(result.root_labels)
    for root in result.roots:
        walk(root)
    assert sorted(seen) == sorted(result.sequence)
    # Without it, only the text
    plain = klimax.solve(DOMAIN, problem, decomposition=False)
    assert plain.status == klimax.Status.SOLVED and not plain.steps_of_plan


def test_decomposition_of_a_given_plan():
    problem = (DATA / "p1-total-order.hddl").read_text()
    solved = klimax.solve(DOMAIN, problem)
    report = klimax.verify(DOMAIN, problem, solved.plan, decomposition=True)
    assert report.valid and report.sequence == solved.sequence
    assert report.roots == solved.roots
    assert [s.id for s in report.steps_of_plan] == [s.id for s in solved.steps_of_plan]
    assert not klimax.verify(DOMAIN, problem, solved.plan).steps_of_plan


def test_a_wrong_plan_is_not_valid():
    problem = (DATA / "p1-total-order.hddl").read_text()
    plan = klimax.solve(DOMAIN, problem).plan
    lines = plan.splitlines()
    broken = "\n".join(lines[:1] + lines[2:]) + "\n"  # without its first action
    report = klimax.verify(DOMAIN, problem, broken)
    assert not report.valid and report.reason


def test_errors_in_the_input():
    problem = (DATA / "p1-total-order.hddl").read_text()
    report = klimax.check(DOMAIN.replace("(:task", "(:tusk", 1), problem)
    assert not report.ok
    assert report.diagnostics[0].severity == klimax.Severity.ERROR
    assert report.diagnostics[0].line > 0 and "domain.hddl" in str(report.diagnostics[0])
    result = klimax.solve(DOMAIN.replace("(:task", "(:tusk", 1), problem)
    assert result.status == klimax.Status.INVALID_INPUT and result.diagnostics


def test_summary():
    problem = (DATA / "p1-total-order.hddl").read_text()
    report = klimax.check(DOMAIN, problem)
    assert report.ok and report.summary.total_order
    assert report.summary.tasks > 0 and report.summary.methods > 0
    assert not klimax.check(DOMAIN, (DATA / "p2-partial-order.hddl").read_text()).summary.total_order


def test_anytime():
    problem = (DATA / "p1-total-order.hddl").read_text()
    seen = []
    result = klimax.solve(DOMAIN, problem, timeout=10, anytime=True,
                          on_plan=lambda plan, steps: seen.append((plan, steps)))
    assert result.status == klimax.Status.SOLVED and result.optimal
    assert result.plans == len(seen) >= 1
    assert [s for _, s in seen] == sorted({s for _, s in seen}, reverse=True)
    assert seen[-1][1] == result.steps and seen[-1][0] == result.plan
    assert all(klimax.verify(DOMAIN, problem, plan).valid for plan, _ in seen)
    first = klimax.solve(DOMAIN, problem)
    assert first.plans == 1 and not first.optimal and first.steps >= result.steps


TRIP_DOMAIN = """
(define (domain trip) (:requirements :typing :hierarchy :numeric-fluents)
 (:types place) (:predicates (at ?l - place) (road ?a ?b - place) (station ?l - place))
 (:functions (fuel) (need ?a ?b - place))
 (:task go :parameters (?to - place))
 (:method there :parameters (?to - place) :task (go ?to) :precondition (at ?to))
 (:method step :parameters (?to ?a ?b - place) :task (go ?to) :precondition (at ?a)
  :ordered-subtasks (and (move ?a ?b) (go ?to)))
 (:method refuel :parameters (?to ?l - place) :task (go ?to) :precondition (< (fuel) 3)
  :ordered-subtasks (and (fill ?l) (go ?to)))
 (:action move :parameters (?a ?b - place)
  :precondition (and (at ?a) (road ?a ?b) (>= (fuel) (need ?a ?b)))
  :effect (and (not (at ?a)) (at ?b) (decrease (fuel) (need ?a ?b))))
 (:action fill :parameters (?l - place)
  :precondition (and (at ?l) (station ?l) (< (fuel) 5)) :effect (assign (fuel) 5)))
"""
TRIP = """
(define (problem p) (:domain trip) (:objects a b c d - place)
 (:htn :ordered-subtasks (and (go d)))
 (:init (at a) (road a b) (road b c) (road c d) (= (fuel) 5) %s
  (= (need a b) 3) (= (need b c) 2) (= (need c d) 3)))
"""


def test_numbers():
    # 5 - 3 = 2 at b: enough to reach c but not d from there, so it fills up at b
    problem = TRIP % "(station b)"
    assert klimax.check(TRIP_DOMAIN, problem).ok
    result = klimax.solve(TRIP_DOMAIN, problem, timeout=10)
    assert result.status == klimax.Status.SOLVED, result.message
    assert result.steps == 4 and "fill b" in result.plan
    assert klimax.verify(TRIP_DOMAIN, problem, result.plan).valid
    # The same plan where b has no station is not one, and there is none
    dry = TRIP % ""
    assert not klimax.verify(TRIP_DOMAIN, dry, result.plan).valid
    assert klimax.solve(TRIP_DOMAIN, dry, timeout=10).status == klimax.Status.UNSOLVABLE


def test_version():
    assert klimax.__version__.count(".") == 2

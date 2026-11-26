// Copyright 2026 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.



#include <gtest/gtest.h>

#include <cstdio>
#include <string>

#include "klimax/check.hpp"
#include "klimax/solve.hpp"

namespace klimax
{
namespace
{

// A kiln that must be fired before it bakes; baking takes as long as the piece says
const char * kDomain =
  "(define (domain d) (:requirements :typing :hierarchy :durative-actions :numeric-fluents)\n"
  " (:types piece)\n"
  " (:predicates (hot) (baked ?p - piece) (free))\n"
  " (:functions (time-of ?p - piece))\n"
  " (:task make :parameters (?p - piece))\n"
  " (:method make-cold :parameters (?p - piece) :task (make ?p)\n"
  "  :precondition (not (hot)) :ordered-subtasks (and (fire) (bake ?p)))\n"
  " (:method make-hot :parameters (?p - piece) :task (make ?p)\n"
  "  :precondition (hot) :ordered-subtasks (and (bake ?p)))\n"
  " (:durative-action fire :parameters () :duration (= ?duration 8)\n"
  "  :condition (and (at start (free)))\n"
  "  :effect (and (at start (not (free))) (at end (free)) (at end (hot))))\n"
  " (:durative-action bake :parameters (?p - piece) :duration (= ?duration (time-of ?p))\n"
  "  :condition (and (at start (free)) (over all (hot)))\n"
  "  :effect (and (at start (not (free))) (at end (free)) (at end (baked ?p)))))";

std::string problem(const std::string & extra = "")
{
  return "(define (problem p) (:domain d) (:objects cup bowl - piece)\n"
         "(:htn :ordered-subtasks (and (make cup) (make bowl)))\n"
         "(:init (free) (= (time-of cup) 3) (= (time-of bowl) 4.5) " + extra + ")\n"
         "(:goal (and (baked cup) (baked bowl))))";
}

TEST(TemporalTest, OneActionAfterAnother)
{
  SolveResult r = solve(kDomain, problem());
  ASSERT_EQ(r.status, Status::Solved) << r.message;
  EXPECT_EQ(r.steps, 3u) << r.plan;
  // Each action with its time and its duration; the next starts once the one before ends
  EXPECT_NE(r.plan.find(" 0.000: (fire) [8.000]"), std::string::npos) << r.plan;
  EXPECT_NE(r.plan.find(" 8.001: (bake cup) [3.000]"), std::string::npos) << r.plan;
  EXPECT_NE(r.plan.find(" 11.002: (bake bowl) [4.500]"), std::string::npos) << r.plan;
  ASSERT_TRUE(r.makespan.has_value());
  EXPECT_NEAR(*r.makespan, 15.502, 1e-9);
  VerifyResult v = verify(kDomain, problem(), r.plan);
  EXPECT_TRUE(v.valid) << v.reason;
  ASSERT_TRUE(v.makespan.has_value());
  EXPECT_NEAR(*v.makespan, 15.502, 1e-9);
}

const char * kTree = "root 4 5\n4 make cup -> make-cold 1 2\n5 make bowl -> make-hot 3\n<==\n";

TEST(TemporalTest, TheVerifierTimesAPlanThatGivesNoTimes)
{
  const std::string plan = std::string("==>\n1 fire\n2 bake cup\n3 bake bowl\n") + kTree;
  VerifyResult v = verify(kDomain, problem(), plan);
  EXPECT_TRUE(v.valid) << v.reason;
  ASSERT_TRUE(v.makespan.has_value());
  EXPECT_NEAR(*v.makespan, 15.502, 1e-9);
}

TEST(TemporalTest, TheVerifierChecksTimesAndDurations)
{
  auto plan = [](const std::string & second) {
      return "==>\n1 0.000: (fire) [8.000]\n" + second + "\n3 20: (bake bowl) [4.5]\n" + kTree;
    };
  // Later than needed is fine
  EXPECT_TRUE(verify(kDomain, problem(), plan("2 10: (bake cup) [3]")).valid);
  // A duration that is not the action's
  VerifyResult v = verify(kDomain, problem(), plan("2 10: (bake cup) [2]"));
  EXPECT_FALSE(v.valid);
  EXPECT_NE(v.reason.find("duration"), std::string::npos) << v.reason;
  // Baking while the kiln is still being fired
  v = verify(kDomain, problem(), plan("2 4: (bake cup) [3]"));
  EXPECT_FALSE(v.valid);
  // A time on some actions only
  EXPECT_FALSE(verify(kDomain, problem(), plan("2 bake cup")).valid);
}

// A light that is on only while its switch is held: reading needs both at once
std::string light(const std::string & subtasks)
{
  return "(define (domain d) (:requirements :typing :hierarchy :durative-actions)\n"
         " (:predicates (light) (read))\n"
         " (:task study :parameters ())\n"
         " (:method m :parameters () :task (study) " + subtasks + " (and (hold) (look)))\n"
         " (:durative-action hold :parameters () :duration (= ?duration 5)\n"
         "  :condition (and) :effect (and (at start (light)) (at end (not (light)))))\n"
         " (:durative-action look :parameters () :duration (= ?duration 2)\n"
         "  :condition (and (over all (light))) :effect (at end (read))))";
}

const char * kStudy = "(define (problem p) (:domain d)\n"
  "(:htn :subtasks (and (study))) (:init) (:goal (read)))";

TEST(TemporalTest, DurationInConditionsAndEffects)
{
  // A battery that working drains by twice what the work lasts, and that must last for it
  const char * domain =
    "(define (domain d) (:requirements :typing :hierarchy :durative-actions :numeric-fluents)\n"
    " (:types worker)\n"
    " (:predicates (done ?w - worker))\n"
    " (:functions (battery) (pace ?w - worker))\n"
    " (:task job :parameters (?w - worker))\n"
    " (:method m :parameters (?w - worker) :task (job ?w) :ordered-subtasks (and (work ?w)))\n"
    " (:durative-action work :parameters (?w - worker) :duration (= ?duration (pace ?w))\n"
    "  :condition (and (at start (>= (battery) (* 2 ?duration))))\n"
    "  :effect (and (at end (decrease (battery) (* 2 ?duration))) (at end (done ?w)))))";
  auto problem = [](const std::string & battery) {
      return "(define (problem p) (:domain d) (:objects ann bob - worker)\n"
             "(:htn :ordered-subtasks (and (job ann) (job bob)))\n"
             "(:init (= (battery) " + battery + ") (= (pace ann) 3) (= (pace bob) 2))\n"
             "(:goal (and (done ann) (done bob))))";
    };
  // 2 * 3 + 2 * 2 = 10
  SolveResult r = solve(domain, problem("10"));
  ASSERT_EQ(r.status, Status::Solved) << r.message;
  EXPECT_TRUE(verify(domain, problem("10"), r.plan).valid);
  EXPECT_NE(solve(domain, problem("9")).status, Status::Solved);
}

TEST(TemporalTest, ActionsThatMustOverlap)
{
  // With one action after another there is no plan: looking needs the light, which is on
  // only while the switch is held. With the start and the end of each as two steps there is
  SolveResult r = solve(light(":subtasks"), kStudy);
  ASSERT_EQ(r.status, Status::Solved) << r.message;
  ASSERT_TRUE(r.makespan.has_value());
  EXPECT_NEAR(*r.makespan, 5.0, 1e-9) << r.plan;
  EXPECT_NE(r.plan.find("(hold) [5.000]"), std::string::npos) << r.plan;
  EXPECT_NE(r.plan.find("(look) [2.000]"), std::string::npos) << r.plan;
  EXPECT_EQ(r.plan.find("#end"), std::string::npos) << r.plan;
  EXPECT_TRUE(verify(light(":subtasks"), kStudy, r.plan).valid);
  // Not where the method orders them: holding ends before looking starts
  r = solve(light(":ordered-subtasks"), kStudy);
  EXPECT_NE(r.status, Status::Solved) << r.plan;
  EXPECT_NE(r.status, Status::Unsolvable);
}

TEST(TemporalTest, WhatIsNotPlannedYetIsSaid)
{
  // Looking takes longer than the light lasts: no times fit, and that is not "no plan" said
  // by a search that does not look at durations
  std::string longer = light(":subtasks");
  longer.replace(longer.find("(= ?duration 2)"), 15, "(= ?duration 7)");
  SolveResult r = solve(longer, kStudy);
  EXPECT_EQ(r.status, Status::Unsupported) << r.message << r.plan;
  EXPECT_FALSE(r.message.empty());
}

TEST(TemporalTest, TheVerifierFollowsActionsThatOverlap)
{
  auto plan = [](const std::string & look) {
      return "==>\n1 0: (hold) [5]\n2 " + look + ": (look) [2]\nroot 3\n3 study -> m 1 2\n<==\n";
    };
  VerifyResult v = verify(light(":subtasks"), kStudy, plan("1"));
  EXPECT_TRUE(v.valid) << v.reason;
  ASSERT_TRUE(v.makespan.has_value());
  EXPECT_NEAR(*v.makespan, 5.0, 1e-9);
  // Looking once the light is off, or still looking when it goes off
  EXPECT_FALSE(verify(light(":subtasks"), kStudy, plan("6")).valid);
  EXPECT_FALSE(verify(light(":subtasks"), kStudy, plan("4")).valid);
  // The method orders them: looking cannot start before holding ends
  v = verify(light(":ordered-subtasks"), kStudy, plan("1"));
  EXPECT_FALSE(v.valid);
  EXPECT_NE(v.reason.find("too early"), std::string::npos) << v.reason;
}

TEST(TemporalTest, ActionsOverlapWhereTheHierarchyAllows)
{
  // Two workers, a job each: at the same time if the task network does not order them
  const char * domain =
    "(define (domain d) (:requirements :typing :hierarchy :durative-actions)\n"
    " (:types worker)\n"
    " (:predicates (idle ?w - worker) (done ?w - worker))\n"
    " (:task job :parameters (?w - worker))\n"
    " (:method m :parameters (?w - worker) :task (job ?w) :subtasks (and (work ?w)))\n"
    " (:durative-action work :parameters (?w - worker) :duration (= ?duration 5)\n"
    "  :condition (and (at start (idle ?w)))\n"
    "  :effect (and (at start (not (idle ?w))) (at end (idle ?w)) (at end (done ?w)))))";
  auto problem = [](const std::string & subtasks) {
      return "(define (problem p) (:domain d) (:objects ann bob - worker)\n"
             "(:htn " + subtasks + " (and (job ann) (job bob))) (:init (idle ann) (idle bob))\n"
             "(:goal (and (done ann) (done bob))))";
    };
  SolveResult r = solve(domain, problem(":subtasks"));
  ASSERT_EQ(r.status, Status::Solved) << r.message;
  ASSERT_TRUE(r.makespan.has_value());
  EXPECT_NEAR(*r.makespan, 5.0, 1e-9) << r.plan;
  EXPECT_TRUE(verify(domain, problem(":subtasks"), r.plan).valid);
  // The same plan is not one of the network that orders them
  EXPECT_FALSE(verify(domain, problem(":ordered-subtasks"), r.plan).valid);
  r = solve(domain, problem(":ordered-subtasks"));
  ASSERT_EQ(r.status, Status::Solved) << r.message;
  ASSERT_TRUE(r.makespan.has_value());
  EXPECT_NEAR(*r.makespan, 10.0, 1e-9) << r.plan;
}

// Sites that can be looked at only while they are visible, which the problem says with timed
// initial literals
const char * kSites =
  "(define (domain d)\n"
  " (:requirements :typing :hierarchy :durative-actions :timed-initial-literals)\n"
  " (:types site)\n"
  " (:predicates (visible ?s - site) (seen ?s - site))\n"
  " (:task observe :parameters (?s - site))\n"
  " (:method m :parameters (?s - site) :task (observe ?s) :ordered-subtasks (and (look ?s)))\n"
  " (:durative-action look :parameters (?s - site) :duration (= ?duration 2)\n"
  "  :condition (and (over all (visible ?s))) :effect (at end (seen ?s))))";

std::string sites(const std::string & b_until)
{
  return "(define (problem p) (:domain d) (:objects a b - site)\n"
         "(:htn :ordered-subtasks (and (observe a) (observe b)))\n"
         "(:init (at 10 (visible a)) (at 20 (not (visible a))) (visible b)\n"
         " (at " + b_until + " (not (visible b))))\n"
         "(:goal (and (seen a) (seen b))))";
}

TEST(TemporalTest, AStepWaitsForATimedInitialLiteral)
{
  // a is visible from 10: looking at it starts then, and b is looked at afterwards
  SolveResult r = solve(kSites, sites("30"));
  ASSERT_EQ(r.status, Status::Solved) << r.message;
  EXPECT_NE(r.plan.find(" 10.001: (look a) [2.000]"), std::string::npos) << r.plan;
  ASSERT_TRUE(r.makespan.has_value());
  EXPECT_NEAR(*r.makespan, 14.0, 0.01) << r.plan;
  EXPECT_TRUE(verify(kSites, sites("30"), r.plan).valid);
  // By then b is no longer visible if it stops being so at 13: no plan in that order
  r = solve(kSites, sites("13"));
  EXPECT_NE(r.status, Status::Solved) << r.plan;
}

TEST(TemporalTest, TheVerifierFollowsTimedInitialLiterals)
{
  auto plan = [](const std::string & a, const std::string & b) {
      return "==>\n1 " + a + ": (look a) [2]\n2 " + b + ": (look b) [2]\nroot 3 4\n"
             "3 observe a -> m 1\n4 observe b -> m 2\n<==\n";
    };
  EXPECT_TRUE(verify(kSites, sites("30"), plan("11", "14")).valid);
  // Before a is visible, and while it stops being so
  EXPECT_FALSE(verify(kSites, sites("30"), plan("5", "14")).valid);
  EXPECT_FALSE(verify(kSites, sites("30"), plan("19", "22")).valid);
  // b after it stops being visible
  EXPECT_FALSE(verify(kSites, sites("30"), plan("11", "29")).valid);
  // Without times the verifier waits as the plan needs
  VerifyResult v = verify(kSites, sites("30"),
      "==>\n1 look a\n2 look b\nroot 3 4\n3 observe a -> m 1\n4 observe b -> m 2\n<==\n");
  EXPECT_TRUE(v.valid) << v.reason;
  ASSERT_TRUE(v.makespan.has_value());
  EXPECT_NEAR(*v.makespan, 14.002, 1e-9);
}

// Two workers and a job each, as the subtasks a and b of a network that says something of
// their times
const char * kJobs =
  "(define (domain d) (:requirements :typing :hierarchy :durative-actions)\n"
  " (:types worker)\n"
  " (:predicates (idle ?w - worker) (done ?w - worker))\n"
  " (:task job :parameters (?w - worker))\n"
  " (:method m :parameters (?w - worker) :task (job ?w) :subtasks (and (work ?w)))\n"
  " (:durative-action work :parameters (?w - worker) :duration (= ?duration 5)\n"
  "  :condition (and (at start (idle ?w)))\n"
  "  :effect (and (at start (not (idle ?w))) (at end (idle ?w)) (at end (done ?w)))))";

std::string jobs(const std::string & ordering)
{
  return "(define (problem p) (:domain d) (:objects ann bob - worker)\n"
         "(:htn :subtasks (and (a (job ann)) (b (job bob))) :ordering " + ordering + ")\n"
         "(:init (idle ann) (idle bob)) (:goal (and (done ann) (done bob))))";
}

std::string jobs_plan(const std::string & ann, const std::string & bob)
{
  return "==>\n1 " + ann + ": (work ann) [5]\n2 " + bob + ": (work bob) [5]\nroot 3 4\n"
         "3 job ann -> m 1\n4 job bob -> m 2\n<==\n";
}

TEST(TemporalTest, OrderingsBetweenTimePoints)
{
  auto valid = [](const std::string & ordering, const std::string & ann, const std::string & bob) {
      return verify(kJobs, jobs(ordering), jobs_plan(ann, bob)).valid;
    };
  // Both start at once
  EXPECT_TRUE(valid("(= (start a) (start b))", "0", "0"));
  EXPECT_FALSE(valid("(= (start a) (start b))", "0", "1"));
  // b starts later, however little; or not before
  EXPECT_TRUE(valid("(< (start a) (start b))", "0", "1"));
  EXPECT_FALSE(valid("(< (start a) (start b))", "0", "0"));
  EXPECT_TRUE(valid("(<= (start a) (start b))", "0", "0"));
  // b starts while a runs
  const char * during = "(and (< (start a) (start b)) (< (start b) (end a)))";
  EXPECT_TRUE(valid(during, "0", "3"));
  EXPECT_FALSE(valid(during, "0", "5"));
  // Between tasks: one ends, then the other starts, with no time asked in between
  EXPECT_TRUE(valid("(< a b)", "0", "5"));
  EXPECT_FALSE(valid("(< a b)", "0", "4"));
  EXPECT_TRUE(valid("(> b a)", "0", "5"));
  EXPECT_FALSE(valid("(> a b)", "0", "5"));
  // The end of one written as a sum and the start of the other as a number are the same time
  for (const char * start : {"0.002", "7.002", "0.1", "1.3"}) {
    char end[32];
    std::snprintf(end, sizeof(end), "%.3f", std::stod(start) + 5.0);
    EXPECT_TRUE(valid("(< a b)", start, end)) << start;
  }
  // Its negation: b has started when a ends
  EXPECT_TRUE(valid("(not (< a b))", "0", "4"));
  EXPECT_FALSE(valid("(not (< a b))", "0", "6"));
  // With the points written, a comparison of times: strictly before
  EXPECT_FALSE(valid("(< (end a) (start b))", "0", "5"));
  EXPECT_TRUE(valid("(< (end a) (start b))", "0", "5.001"));
}

TEST(TemporalTest, ThePlanMeetsOrderingsBetweenTimePoints)
{
  // One action after another does not start them at once: the times of the plan do
  SolveResult r = solve(kJobs, jobs("(= (start a) (start b))"));
  ASSERT_EQ(r.status, Status::Solved) << r.message;
  ASSERT_TRUE(r.makespan.has_value());
  EXPECT_NEAR(*r.makespan, 5.0, 1e-9) << r.plan;
  EXPECT_TRUE(verify(kJobs, jobs("(= (start a) (start b))"), r.plan).valid);
  // What the times found do not meet is said, not answered
  r = solve(kJobs, jobs("(and (< (start a) (start b)) (< (start b) (end a)))"));
  EXPECT_NE(r.status, Status::Unsolvable);
  if (r.status == Status::Solved) {
    EXPECT_TRUE(verify(kJobs,
      jobs("(and (< (start a) (start b)) (< (start b) (end a)))"), r.plan).valid);
  }
}

// A shift of two jobs, by a durative method that says what the whole may last
std::string shifts(const std::string & duration, const std::string & condition = "")
{
  return "(define (domain d)\n"
         " (:requirements :typing :hierarchy :durative-actions :durative-methods)\n"
         " (:types worker)\n"
         " (:predicates (idle ?w - worker) (done ?w - worker) (open))\n"
         " (:task shift :parameters (?v ?w - worker))\n"
         " (:durative-method both :parameters (?v ?w - worker) :task (shift ?v ?w)\n"
         "  :duration " + duration + " " + condition + "\n"
         "  :subtasks (and (x (work ?v)) (y (work ?w))))\n"
         " (:durative-action work :parameters (?w - worker) :duration (= ?duration 5)\n"
         "  :condition (and (at start (idle ?w)))\n"
         "  :effect (and (at start (not (idle ?w))) (at end (idle ?w)) (at end (done ?w))))\n"
         " (:action close :parameters () :effect (not (open))))";
}

const char * kShift = "(define (problem p) (:domain d) (:objects ann bob - worker)\n"
  "(:htn :subtasks (and (shift ann bob))) (:init (idle ann) (idle bob) (open))\n"
  "(:goal (and (done ann) (done bob))))";

std::string shift_plan(const std::string & bob)
{
  return "==>\n1 0: (work ann) [5]\n2 " + bob + ": (work bob) [5]\nroot 3\n"
         "3 shift ann bob -> both 1 2\n<==\n";
}

TEST(TemporalTest, DurationsOfMethods)
{
  // The whole in six units at most: the jobs overlap
  EXPECT_TRUE(verify(shifts("(<= ?duration 6)"), kShift, shift_plan("1")).valid);
  VerifyResult v = verify(shifts("(<= ?duration 6)"), kShift, shift_plan("5"));
  EXPECT_FALSE(v.valid);
  EXPECT_NE(v.reason.find("duration"), std::string::npos) << v.reason;
  // The durations of subtasks, and several constraints
  EXPECT_TRUE(verify(shifts("(= (duration x) (duration y))"), kShift, shift_plan("1")).valid);
  EXPECT_FALSE(verify(shifts("(> (duration x) (duration y))"), kShift, shift_plan("1")).valid);
  EXPECT_TRUE(verify(shifts("(and (>= ?duration 8) (<= ?duration (* 2 (duration x))))"), kShift,
    shift_plan("3")).valid);
  EXPECT_FALSE(verify(shifts("(and (>= ?duration 8) (<= ?duration (* 2 (duration x))))"), kShift,
    shift_plan("1")).valid);
  // The planner: one job after the other lasts ten, at once five
  SolveResult r = solve(shifts("(<= ?duration 6)"), kShift);
  ASSERT_EQ(r.status, Status::Solved) << r.message;
  ASSERT_TRUE(r.makespan.has_value());
  EXPECT_NEAR(*r.makespan, 5.0, 1e-9) << r.plan;
}

TEST(TemporalTest, ConditionsOfDurativeMethods)
{
  // The shop is open while the shift lasts; closing it is another task of the problem
  const char * p = "(define (problem p) (:domain d) (:objects ann bob - worker)\n"
    "(:htn :subtasks (and (shift ann bob) (close))) (:init (idle ann) (idle bob) (open))\n"
    "(:goal (and (done ann) (done bob))))";
  auto plan = [](const std::string & close) {
      return "==>\n1 0: (work ann) [5]\n2 1: (work bob) [5]\n4 " + close + ": (close) [0]\n"
             "root 3 4\n3 shift ann bob -> both 1 2\n<==\n";
    };
  const std::string open = shifts("()", ":condition (and (at start (idle ?v)) (over all (open)))");
  VerifyResult v = verify(open, p, plan("7"));
  EXPECT_TRUE(v.valid) << v.reason;
  v = verify(open, p, plan("3"));
  EXPECT_FALSE(v.valid);
  EXPECT_NE(v.reason.find("over all"), std::string::npos) << v.reason;
  // At end: when the last of its actions has ended
  EXPECT_TRUE(verify(shifts("()", ":condition (at end (done ?w))"), p, plan("7")).valid);
  v = verify(shifts("()", ":condition (at end (open))"), p, plan("3"));
  EXPECT_FALSE(v.valid);
  EXPECT_NE(v.reason.find("at end"), std::string::npos) << v.reason;
  // The planner closes after the shift
  SolveResult r = solve(open, p);
  EXPECT_NE(r.status, Status::Unsolvable);
  if (r.status == Status::Solved) {
    EXPECT_TRUE(verify(open, p, r.plan).valid) << r.plan;
  }
}

TEST(TemporalTest, TheDecompositionSaysOrdersAndPreconditions)
{
  // What an executor needs to follow the plan by its hierarchy
  SolveOptions options;
  options.decomposition = true;
  SolveResult r = solve(kSites, sites("30"), options);
  ASSERT_EQ(r.status, Status::Solved) << r.message;
  ASSERT_EQ(r.root_order.size(), 1u);  // observe a, then observe b
  EXPECT_EQ(r.root_order[0], (std::pair<uint32_t, uint32_t>(0, 1)));
  for (const PlanStep & s : r.steps_of_plan) {
    if (!s.primitive) {
      EXPECT_EQ(s.method, "m");
      EXPECT_TRUE(s.precondition.empty());
    }
  }
  r = solve(kJobs, jobs("(< a b)"), options);
  ASSERT_EQ(r.status, Status::Solved) << r.message;
  EXPECT_EQ(r.root_order.size(), 1u);
  // When each method was applied: the second job's once the first has ended
  for (const PlanStep & s : r.steps_of_plan) {
    if (!s.primitive && s.args == std::vector<std::string>{"bob"}) {
      EXPECT_EQ(s.applied_after, 1u);
      ASSERT_TRUE(s.applied_at.has_value());
      EXPECT_GE(*s.applied_at, 5.0);
    }
  }
}

}  // namespace
}  // namespace klimax

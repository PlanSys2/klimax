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

#include <chrono>
#include <optional>
#include <string>
#include <string_view>

#include "hddl/model.hpp"
#include "hddl/parser.hpp"
#include "search/lifted.hpp"
#include "verify/verifier.hpp"

namespace klimax::search
{
namespace
{

// A robot that carries items along a line of rooms a - b - c - d. `bring-it` leaves the robot
// free, `go-step` the next room and `tour-two` two rooms that nothing fixes until `go` does;
// `fly` needs wings nobody has; `walk` is written from the end, so it comes first from itself
const char * kDomain =
  "(define (domain d) (:requirements :typing :hierarchy :negative-preconditions)\n"
  " (:types robot room item)\n"
  " (:predicates (at ?r - robot ?l - room) (in ?i - item ?l - room) (holding ?r - robot ?i - item)\n"
  "  (free ?r - robot) (link ?a ?b - room) (winged ?r - robot) (seen ?l - room))\n"
  " (:task go :parameters (?r - robot ?to - room))\n"
  " (:task bring :parameters (?i - item ?to - room))\n"
  " (:task tour :parameters (?r - robot))\n"
  " (:task fly :parameters (?r - robot ?to - room))\n"
  " (:task walk :parameters (?r - robot ?to - room))\n"
  " (:task pace :parameters (?r - robot))\n"
  " (:task hop :parameters (?r - robot))\n"
  " (:method pace-more :parameters (?r - robot ?x ?y - room) :task (pace ?r)\n"
  "  :precondition (and (at ?r ?x) (link ?x ?y))\n"
  "  :ordered-subtasks (and (move ?r ?x ?y) (move ?r ?y ?x) (pace ?r) (look ?r ?x)))\n"
  " (:method pace-stop :parameters (?r - robot) :task (pace ?r))\n"
  " (:method hop-on :parameters (?r - robot ?from ?to - room) :task (hop ?r)\n"
  "  :ordered-subtasks (and (look ?r ?from) (move ?r ?from ?to)))\n"
  " (:method walk-more :parameters (?r - robot ?mid ?to - room) :task (walk ?r ?to)\n"
  "  :precondition (link ?mid ?to) :ordered-subtasks (and (walk ?r ?mid) (move ?r ?mid ?to)))\n"
  " (:method walk-here :parameters (?r - robot ?to - room) :task (walk ?r ?to)\n"
  "  :precondition (at ?r ?to))\n"
  " (:method go-here :parameters (?r - robot ?to - room) :task (go ?r ?to)\n"
  "  :precondition (at ?r ?to))\n"
  " (:method go-step :parameters (?r - robot ?from ?mid ?to - room) :task (go ?r ?to)\n"
  "  :precondition (and (at ?r ?from) (link ?from ?mid))\n"
  "  :ordered-subtasks (and (move ?r ?from ?mid) (go ?r ?to)))\n"
  " (:method bring-done :parameters (?i - item ?to - room) :task (bring ?i ?to)\n"
  "  :precondition (in ?i ?to))\n"
  " (:method bring-it :parameters (?i - item ?from ?to - room ?r - robot) :task (bring ?i ?to)\n"
  "  :precondition (in ?i ?from)\n"
  "  :ordered-subtasks (and (go ?r ?from) (pick ?r ?i ?from) (go ?r ?to) (drop ?r ?i ?to)))\n"
  " (:method tour-two :parameters (?r - robot ?x ?y - room) :task (tour ?r)\n"
  "  :ordered-subtasks (and (look ?r ?x) (go ?r ?y) (look ?r ?y)))\n"
  " (:method fly-there :parameters (?r - robot ?from ?to - room) :task (fly ?r ?to)\n"
  "  :precondition (winged ?r) :ordered-subtasks (soar ?r ?from ?to))\n"
  " (:action move :parameters (?r - robot ?from ?to - room)\n"
  "  :precondition (and (at ?r ?from) (link ?from ?to))\n"
  "  :effect (and (not (at ?r ?from)) (at ?r ?to)))\n"
  " (:action pick :parameters (?r - robot ?i - item ?l - room)\n"
  "  :precondition (and (at ?r ?l) (in ?i ?l) (free ?r))\n"
  "  :effect (and (not (in ?i ?l)) (not (free ?r)) (holding ?r ?i)))\n"
  " (:action drop :parameters (?r - robot ?i - item ?l - room)\n"
  "  :precondition (and (at ?r ?l) (holding ?r ?i))\n"
  "  :effect (and (in ?i ?l) (free ?r) (not (holding ?r ?i))))\n"
  " (:action look :parameters (?r - robot ?l - room)\n"
  "  :precondition (and (at ?r ?l) (not (seen ?l))) :effect (seen ?l))\n"
  " (:action soar :parameters (?r - robot ?from ?to - room)\n"
  "  :precondition (and (at ?r ?from) (winged ?r))\n"
  "  :effect (and (not (at ?r ?from)) (at ?r ?to))))";

struct Solved
{
  Result result;
  verifier::Outcome checked;
  int pieces = 0;
};

// With `polls`, the search runs in pieces: each stops after that many looks at the clock
Solved solve(std::string_view htn, std::string_view goal = "", int polls = 0)
{
  auto d = hddl::parse_domain(kDomain, "d.hddl");
  auto p = hddl::parse_problem(
    "(define (problem p) (:domain d) (:objects r1 r2 - robot a b c d - room cup book - item)\n" +
    std::string(htn) + "\n(:init (at r1 a) (at r2 d) (free r1) (free r2) (link a b) (link b a)"
    " (link b c) (link c b) (link c d) (link d c) (in cup c) (in book b))" + std::string(goal) +
    ")", "p.hddl");
  EXPECT_TRUE(d.ok() && p.ok());
  auto m = hddl::analyze(std::move(d).value(), std::move(p).value());
  EXPECT_TRUE(m.ok());
  Solved s;
  if (polls == 0) {
    s.result = lifted(m.value());
  } else {
    using Clock = kairos::BudgetConfig::Clock;
    Clock::time_point now{};
    LiftedSearch search(m.value());
    for (s.pieces = 0; s.pieces < 100000; ++s.pieces) {
      kairos::BudgetConfig config;
      config.clock = [&]() {return now += std::chrono::milliseconds(1);};
      config.deadline = now + std::chrono::milliseconds(polls);
      config.check_interval = 1;
      config.check_period = std::chrono::microseconds(0);
      config.memory_probe = []() {return std::optional<size_t>();};
      kairos::Budget budget(config);
      s.result = search.run(&budget);
      if (s.result.outcome != Outcome::OutOfBudget) {
        break;
      }
    }
  }
  if (s.result.outcome == Outcome::Solved) {
    s.checked = verifier::verify(m.value(), s.result.plan);
  }
  return s;
}

TEST(LiftedTest, PlansWithoutGrounding)
{
  Solved s = solve("(:htn :ordered-subtasks (and (bring cup a) (bring book d)))");
  ASSERT_EQ(s.result.outcome, Outcome::Solved) << s.result.message;
  EXPECT_TRUE(s.checked.valid) << s.checked.reason << "\n" << s.result.plan;
  EXPECT_GT(s.result.steps, 4u);
  EXPECT_EQ(s.result.plans, 1u);
}

TEST(LiftedTest, ParametersLeftForLater)
{
  // The rooms of the tour are those where the robot is when it looks, which nothing says
  // before; the second must be one not seen yet
  Solved s = solve("(:htn :ordered-subtasks (and (tour r1) (tour r2)))");
  ASSERT_EQ(s.result.outcome, Outcome::Solved) << s.result.message;
  EXPECT_TRUE(s.checked.valid) << s.checked.reason << "\n" << s.result.plan;
  // A variable of the initial task network
  s = solve("(:htn :parameters (?where - room) :ordered-subtasks (and (bring cup ?where)))",
      "(:goal (in cup d))");
  ASSERT_EQ(s.result.outcome, Outcome::Solved) << s.result.message;
  EXPECT_TRUE(s.checked.valid) << s.checked.reason << "\n" << s.result.plan;
  EXPECT_NE(s.result.plan.find("drop r"), std::string::npos);
}

TEST(LiftedTest, GoalAndDeadEnds)
{
  // The first way of bringing the cup leaves it where it is: the goal sends the search back
  Solved s = solve("(:htn :parameters (?x - room) :ordered-subtasks (and (bring cup ?x)))",
      "(:goal (and (in cup b) (at r2 b)))");
  ASSERT_EQ(s.result.outcome, Outcome::Solved) << s.result.message;
  EXPECT_TRUE(s.checked.valid) << s.checked.reason << "\n" << s.result.plan;
  EXPECT_EQ(solve("(:htn :ordered-subtasks (and (fly r1 c)))").result.outcome,
    Outcome::Unsolvable);
  EXPECT_EQ(solve("(:htn :ordered-subtasks (and (go r1 c)))", "(:goal (seen a))").result.outcome,
    Outcome::Unsolvable);
}

TEST(LiftedTest, GoesOnAfterTheBudgetRanOut)
{
  const char * htn = "(:htn :parameters (?x - room) :ordered-subtasks (and (tour r1) (bring cup ?x)"
    " (tour r2)))";
  const char * goal = "(:goal (and (in cup b) (at r2 b)))";
  const Solved whole = solve(htn, goal);
  ASSERT_EQ(whole.result.outcome, Outcome::Solved) << whole.result.message;
  for (int polls : {2, 3, 7}) {
    const Solved s = solve(htn, goal, polls);
    ASSERT_EQ(s.result.outcome, Outcome::Solved) << polls;
    EXPECT_GT(s.pieces, 1) << polls;
    EXPECT_EQ(s.result.plan, whole.result.plan) << polls;
    EXPECT_EQ(s.result.expansions, whole.result.expansions) << polls;
  }
  EXPECT_EQ(solve("(:htn :ordered-subtasks (and (go r1 c)))", "(:goal (seen a))", 2).result.outcome,
    Outcome::Unsolvable);
}

TEST(LiftedTest, RecursionThatDoesNotProgress)
{
  // `walk` comes first from itself, before any action: to a from b, to b from a...
  Solved s = solve("(:htn :ordered-subtasks (and (walk r1 d) (walk r2 a)))");
  ASSERT_EQ(s.result.outcome, Outcome::Solved) << s.result.message;
  EXPECT_TRUE(s.checked.valid) << s.checked.reason << "\n" << s.result.plan;
  EXPECT_EQ(s.result.steps, 6u);
  // Without a plan, how many times `walk` should come from itself has no end: it gives up
  EXPECT_EQ(solve("(:htn :ordered-subtasks (and (walk r1 d)))", "(:goal (seen a))").result.outcome,
    Outcome::OutOfBudget);
}

TEST(LiftedTest, RecursionThatComesBackToTheSameState)
{
  // `pace` goes and comes back before it calls itself: the state is the same, with more to do
  Solved s = solve("(:htn :ordered-subtasks (and (pace r1) (pace r2)))");
  ASSERT_EQ(s.result.outcome, Outcome::Solved) << s.result.message;
  EXPECT_TRUE(s.checked.valid) << s.checked.reason << "\n" << s.result.plan;
  EXPECT_EQ(s.result.steps, 0u);  // the second time is cut, and the first one stops instead
}

TEST(LiftedTest, StatesAlreadySeen)
{
  // Going on from a room already left, with the same tasks pending, is the same search again
  Solved s = solve("(:htn :ordered-subtasks (and (go r1 d) (fly r1 a)))");
  EXPECT_EQ(s.result.outcome, Outcome::Unsolvable) << s.result.message;
  EXPECT_LT(s.result.expansions, 100u);
}

TEST(LiftedTest, WhatLaterActionsRequire)
{
  // `hop-on` has no precondition: its rooms must be linked for `move`, which never changes
  Solved s = solve("(:htn :ordered-subtasks (and (hop r1) (hop r1) (hop r2)))");
  ASSERT_EQ(s.result.outcome, Outcome::Solved) << s.result.message;
  EXPECT_TRUE(s.checked.valid) << s.checked.reason << "\n" << s.result.plan;
  EXPECT_EQ(s.result.steps, 6u);
}

TEST(LiftedTest, TotalOrderOnly)
{
  Solved s = solve("(:htn :subtasks (and (bring cup a) (bring book d)))");
  EXPECT_EQ(s.result.outcome, Outcome::Unsupported);
  EXPECT_NE(s.result.message.find("totally ordered"), std::string::npos);
}

}  // namespace
}  // namespace klimax::search

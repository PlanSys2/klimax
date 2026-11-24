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

#include <cstdlib>
#include <string>
#include <string_view>
#include <vector>

#include "hddl/model.hpp"
#include "hddl/parser.hpp"
#include "search/total_order.hpp"
#include "tdg/grounder.hpp"
#include "verify/verifier.hpp"

namespace klimax::search
{
namespace
{

// A robot that carries items along a line of rooms a - b - c - d; `tour` has two independent
// parameters (split by the grounder); `fly` needs wings nobody has
const char * kDomain =
  "(define (domain d) (:requirements :typing :hierarchy :negative-preconditions)\n"
  " (:types robot room item)\n"
  " (:predicates (at ?r - robot ?l - room) (in ?i - item ?l - room) (holding ?r - robot ?i - item)\n"
  "  (free ?r - robot) (link ?a ?b - room) (winged ?r - robot) (seen ?l - room))\n"
  " (:task go :parameters (?r - robot ?to - room))\n"
  " (:task bring :parameters (?i - item ?to - room))\n"
  " (:task tour :parameters (?r - robot))\n"
  " (:task visit :parameters (?r - robot ?l - room))\n"
  " (:task fly :parameters (?r - robot ?to - room))\n"
  " (:method go-here :parameters (?r - robot ?to - room) :task (go ?r ?to)\n"
  "  :precondition (at ?r ?to))\n"
  " (:method go-step :parameters (?r - robot ?from ?mid ?to - room) :task (go ?r ?to)\n"
  "  :precondition (and (at ?r ?from) (link ?from ?mid))\n"
  "  :ordered-subtasks (and (move ?r ?from ?mid) (go ?r ?to)))\n"
  " (:method bring-it :parameters (?i - item ?from ?to - room ?r - robot) :task (bring ?i ?to)\n"
  "  :precondition (in ?i ?from)\n"
  "  :ordered-subtasks (and (go ?r ?from) (pick ?r ?i ?from) (go ?r ?to) (drop ?r ?i ?to)))\n"
  " (:method bring-done :parameters (?i - item ?to - room) :task (bring ?i ?to)\n"
  "  :precondition (in ?i ?to))\n"
  " (:method tour-two :parameters (?r - robot ?x ?y - room) :task (tour ?r)\n"
  "  :ordered-subtasks (and (visit ?r ?x) (visit ?r ?y)))\n"
  " (:method visit-it :parameters (?r - robot ?l - room) :task (visit ?r ?l)\n"
  "  :ordered-subtasks (and (go ?r ?l) (look ?r ?l)))\n"
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
  "  :precondition (at ?r ?l) :effect (seen ?l))\n"
  " (:action soar :parameters (?r - robot ?from ?to - room)\n"
  "  :precondition (and (at ?r ?from) (winged ?r))\n"
  "  :effect (and (not (at ?r ?from)) (at ?r ?to))))";

std::string problem(std::string_view htn, std::string_view goal = "")
{
  return "(define (problem p) (:domain d) (:objects r1 - robot a b c d - room cup book - item)\n" +
         std::string(htn) + "\n(:init (at r1 b) (free r1) (link a b) (link b a) (link b c)"
         " (link c b) (link c d) (link d c) (in cup d) (in book a))" + std::string(goal) + ")";
}

struct Solved
{
  Result result;
  bool verified = false;
  std::string why;
};

Solved solve(const std::string & text)
{
  auto d = hddl::parse_domain(kDomain, "d.hddl");
  auto p = hddl::parse_problem(text, "p.hddl");
  EXPECT_TRUE(d.ok() && p.ok());
  auto m = hddl::analyze(std::move(d).value(), std::move(p).value());
  EXPECT_TRUE(m.ok());
  auto g = tdg::ground(m.value());
  EXPECT_TRUE(g.ok());
  Solved out;
  out.result = total_order(g.value());
  if (out.result.outcome == Outcome::Solved) {
    // The plan is for the model as written, not for the one with parameters split
    const verifier::Outcome v = verifier::verify(m.value(), out.result.plan);
    out.verified = v.valid;
    out.why = v.reason;
  }
  return out;
}

size_t lines_with(const std::string & text, std::string_view word)
{
  size_t n = 0;
  for (size_t at = text.find(word); at != std::string::npos; at = text.find(word, at + 1)) {
    ++n;
  }
  return n;
}

TEST(TotalOrderTest, SolvesAndThePlanIsValid)
{
  Solved s = solve(problem("(:htn :ordered-subtasks (and (bring cup a) (bring book d)))"));
  ASSERT_EQ(s.result.outcome, Outcome::Solved);
  EXPECT_TRUE(s.verified) << s.why << "\n" << s.result.plan;
  // b -> d (2 moves), pick, d -> a (3), drop, pick, a -> d (3), drop: no detours
  EXPECT_EQ(s.result.steps, 12u);
  EXPECT_EQ(lines_with(s.result.plan, " move "), 8u);
}

TEST(TotalOrderTest, NothingToDo)
{
  Solved s = solve(problem("(:htn :ordered-subtasks (bring book a))"));
  ASSERT_EQ(s.result.outcome, Outcome::Solved);
  EXPECT_TRUE(s.verified) << s.why;
  EXPECT_EQ(s.result.steps, 0u);
}

// The tasks are done and the state goal must hold too
TEST(TotalOrderTest, StateGoal)
{
  Solved ok = solve(problem("(:htn :ordered-subtasks (bring cup a))", "(:goal (at r1 a))"));
  ASSERT_EQ(ok.result.outcome, Outcome::Solved);
  EXPECT_TRUE(ok.verified) << ok.why;
  // Bringing the cup to a leaves the robot in a: it cannot also end in d
  Solved no = solve(problem("(:htn :ordered-subtasks (bring cup a))", "(:goal (at r1 d))"));
  EXPECT_EQ(no.result.outcome, Outcome::Unsolvable);
}

// Parameters split by the grounder do not show in the plan
TEST(TotalOrderTest, SplitParametersAreInvisible)
{
  Solved s = solve(problem("(:htn :ordered-subtasks (tour r1))"));
  ASSERT_EQ(s.result.outcome, Outcome::Solved);
  EXPECT_TRUE(s.verified) << s.why << "\n" << s.result.plan;
  EXPECT_EQ(s.result.plan.find("split:"), std::string::npos);
  EXPECT_EQ(lines_with(s.result.plan, "-> tour-two"), 1u);
  EXPECT_EQ(lines_with(s.result.plan, "-> visit-it"), 2u);
}

TEST(TotalOrderTest, Unsolvable)
{
  EXPECT_EQ(solve(problem("(:htn :ordered-subtasks (fly r1 d))")).result.outcome,
    Outcome::Unsolvable);
}

// A partial order is searched in one order it allows: the plan is valid for the problem
TEST(TotalOrderTest, PartialOrderInOneOrder)
{
  Solved s = solve(problem("(:htn :subtasks (and (bring cup a) (bring book d)))"));
  ASSERT_EQ(s.result.outcome, Outcome::Solved);
  EXPECT_TRUE(s.verified) << s.why << "\n" << s.result.plan;
  // With an order between them, also when it is not the one written
  Solved o = solve(problem("(:htn :subtasks (and (t1 (bring cup a)) (t2 (bring book d)))"
      " :ordering (< t2 t1))"));
  ASSERT_EQ(o.result.outcome, Outcome::Solved);
  EXPECT_TRUE(o.verified) << o.why;
  EXPECT_LT(o.result.plan.find("drop r1 book d"), o.result.plan.find("drop r1 cup a"));
}

// a1 enables b1 and b1 enables a2: the two tasks must be interleaved, which no order of them does
TEST(TotalOrderTest, InterleavedTasks)
{
  auto d = hddl::parse_domain(
    "(define (domain Turns) (:requirements :hierarchy)\n"
    " (:predicates (first) (second))\n"
    " (:task A :parameters ()) (:task B :parameters ())\n"
    " (:method mA :parameters () :task (A) :ordered-subtasks (and (a1) (a2)))\n"
    " (:method mB :parameters () :task (B) :ordered-subtasks (b1))\n"
    " (:action a1 :parameters () :effect (first))\n"
    " (:action b1 :parameters () :precondition (first) :effect (second))\n"
    " (:action a2 :parameters () :precondition (second) :effect (not (first))))", "d.hddl");
  auto p = hddl::parse_problem(
    "(define (problem p) (:domain Turns) (:htn :subtasks (and (A) (B))) (:init))", "p.hddl");
  ASSERT_TRUE(d.ok() && p.ok());
  auto m = hddl::analyze(std::move(d).value(), std::move(p).value());
  ASSERT_TRUE(m.ok());
  auto g = tdg::ground(m.value());
  ASSERT_TRUE(g.ok());
  Result r = total_order(g.value());
  ASSERT_EQ(r.outcome, Outcome::Solved) << r.message;
  EXPECT_EQ(r.message, "best first, partial order");
  const verifier::Outcome v = verifier::verify(m.value(), r.plan);
  EXPECT_TRUE(v.valid) << v.reason << "\n" << r.plan;
  EXPECT_LT(r.plan.find(" a1"), r.plan.find(" b1"));
  EXPECT_LT(r.plan.find(" b1"), r.plan.find(" a2"));
}

// With the tasks ordered, the same problem has no plan: proved, every interleaving being none
TEST(TotalOrderTest, OrderedTasksThatNeedInterleaving)
{
  auto d = hddl::parse_domain(
    "(define (domain Turns) (:requirements :hierarchy)\n"
    " (:predicates (first) (second))\n"
    " (:task A :parameters ()) (:task B :parameters ())\n"
    " (:method mA :parameters () :task (A) :ordered-subtasks (and (a1) (a2)))\n"
    " (:method mB :parameters () :task (B) :ordered-subtasks (b1))\n"
    " (:action a1 :parameters () :effect (first))\n"
    " (:action b1 :parameters () :precondition (first) :effect (second))\n"
    " (:action a2 :parameters () :precondition (second) :effect (not (first))))", "d.hddl");
  auto p = hddl::parse_problem(
    "(define (problem p) (:domain Turns) (:htn :ordered-subtasks (and (A) (B))) (:init))",
    "p.hddl");
  auto m = hddl::analyze(std::move(d).value(), std::move(p).value());
  ASSERT_TRUE(m.ok());
  auto g = tdg::ground(m.value());
  ASSERT_TRUE(g.ok());
  EXPECT_EQ(total_order(g.value()).outcome, Outcome::Unsolvable);
}

// Each way of searching alone finds a valid plan
TEST(TotalOrderTest, EveryModeAlone)
{
  const std::string text = problem("(:htn :ordered-subtasks (and (bring cup a) (bring book d)))");
  for (const char * variable : {"KLIMAX_NO_HEURISTIC", "KLIMAX_FREE_ACTIONS", "KLIMAX_BF_NEWEST",
      "KLIMAX_BF_LOOK_AHEAD"})
  {
    setenv(variable, "1", 1);
    Solved s = solve(text);
    unsetenv(variable);
    ASSERT_EQ(s.result.outcome, Outcome::Solved) << variable;
    EXPECT_TRUE(s.verified) << variable << ": " << s.why;
  }
}

// With either way of valuing what is left in each of the two best-first searches
TEST(TotalOrderTest, SumsAndRelaxedPlans)
{
  const std::string text = problem("(:htn :ordered-subtasks (and (bring cup a) (bring book d)))");
  for (const char * value : {"00", "01", "10", "11"}) {
    setenv("KLIMAX_PLAN_COSTS", value, 1);
    Solved s = solve(text);
    unsetenv("KLIMAX_PLAN_COSTS");
    ASSERT_EQ(s.result.outcome, Outcome::Solved) << value;
    EXPECT_TRUE(s.verified) << value << ": " << s.why;
    EXPECT_EQ(s.result.steps, 12u) << value;
  }
}

// What the tours have seen is read by nothing that follows and is not in the goal: the states
// they leave count as one when only that differs, and the proof that the goal cannot hold is
// shorter for it
TEST(TotalOrderTest, StatesThatDifferInWhatNothingReads)
{
  const std::string text = problem(
    "(:htn :ordered-subtasks (and (tour r1) (tour r1) (tour r1) (bring cup a)))",
    "(:goal (at r1 d))");
  Solved with = solve(text);
  setenv("KLIMAX_NO_RELEVANCE", "1", 1);
  Solved without = solve(text);
  unsetenv("KLIMAX_NO_RELEVANCE");
  EXPECT_EQ(with.result.outcome, Outcome::Unsolvable);
  EXPECT_EQ(without.result.outcome, Outcome::Unsolvable);
  EXPECT_LT(with.result.expansions * 2, without.result.expansions);
}

// With anytime the search goes on to the shortest plan, and says that it is
TEST(TotalOrderTest, AnytimeFindsTheShortestPlan)
{
  auto d = hddl::parse_domain(kDomain, "d.hddl");
  // Two rooms to visit, any two: the shortest tour looks at b, where the robot is, twice
  auto p = hddl::parse_problem(problem("(:htn :ordered-subtasks (tour r1))"), "p.hddl");
  auto m = hddl::analyze(std::move(d).value(), std::move(p).value());
  ASSERT_TRUE(m.ok());
  auto g = tdg::ground(m.value());
  ASSERT_TRUE(g.ok());
  std::vector<size_t> lengths;
  Options options;
  options.anytime = true;
  options.on_plan = [&](const std::string & plan, size_t steps) {
      EXPECT_TRUE(verifier::verify(m.value(), plan).valid);
      lengths.push_back(steps);
    };
  const Result result = total_order(g.value(), nullptr, options);
  ASSERT_EQ(result.outcome, Outcome::Solved);
  EXPECT_TRUE(result.optimal);
  EXPECT_EQ(result.steps, 2u);
  EXPECT_EQ(result.plans, lengths.size());
  ASSERT_FALSE(lengths.empty());
  EXPECT_EQ(lengths.back(), 2u);
  for (size_t i = 1; i < lengths.size(); ++i) {
    EXPECT_LT(lengths[i], lengths[i - 1]);
  }
  EXPECT_TRUE(verifier::verify(m.value(), result.plan).valid);
  // Without it, one plan and no claim
  const Result first = total_order(g.value());
  EXPECT_EQ(first.plans, 1u);
  EXPECT_FALSE(first.optimal);
}

TEST(TotalOrderTest, Budget)
{
  auto d = hddl::parse_domain(kDomain, "d.hddl");
  auto p = hddl::parse_problem(
    problem("(:htn :ordered-subtasks (and (bring cup a) (bring book d)))"), "p.hddl");
  auto m = hddl::analyze(std::move(d).value(), std::move(p).value());
  ASSERT_TRUE(m.ok());
  auto g = tdg::ground(m.value());
  ASSERT_TRUE(g.ok());
  kairos::BudgetConfig config;
  config.deadline = kairos::BudgetConfig::Clock::now() - std::chrono::seconds(1);
  config.check_interval = 1;
  kairos::Budget budget(config);
  EXPECT_EQ(total_order(g.value(), &budget).outcome, Outcome::OutOfBudget);
}

}  // namespace
}  // namespace klimax::search

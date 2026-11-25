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

#include <string>
#include <string_view>

#include "hddl/model.hpp"
#include "hddl/parser.hpp"
#include "search/lifted.hpp"
#include "search/sat.hpp"
#include "search/total_order.hpp"
#include "tdg/grounder.hpp"
#include "verify/verifier.hpp"

namespace klimax::search
{
namespace
{

// A truck on a line of places a - b - c - d. Each stretch takes fuel, the tank holds 5 and only
// some places sell it: where to fill up is the problem
const char * kDomain =
  "(define (domain d) (:requirements :typing :hierarchy :negative-preconditions :fluents)\n"
  " (:types place)\n"
  " (:predicates (at ?l - place) (road ?a ?b - place) (station ?l - place))\n"
  " (:functions (fuel) (need ?a ?b - place) (bought))\n"
  " (:task go :parameters (?to - place))\n"
  " (:method go-here :parameters (?to - place) :task (go ?to) :precondition (at ?to))\n"
  " (:method go-on :parameters (?from ?mid ?to - place) :task (go ?to)\n"
  "  :precondition (and (at ?from) (road ?from ?mid) (>= (fuel) (need ?from ?mid)))\n"
  "  :ordered-subtasks (and (drive ?from ?mid) (go ?to)))\n"
  " (:method go-fill :parameters (?here ?to - place) :task (go ?to)\n"
  "  :precondition (and (at ?here) (station ?here) (< (fuel) 5))\n"
  "  :ordered-subtasks (and (fill ?here) (go ?to)))\n"
  " (:action drive :parameters (?a ?b - place)\n"
  "  :precondition (and (at ?a) (road ?a ?b) (>= (fuel) (need ?a ?b)))\n"
  "  :effect (and (not (at ?a)) (at ?b) (decrease (fuel) (need ?a ?b))))\n"
  " (:action fill :parameters (?l - place)\n"
  "  :precondition (and (at ?l) (station ?l) (< (fuel) 5))\n"
  "  :effect (and (increase (bought) (- 5 (fuel))) (assign (fuel) 5))))";

struct Solved
{
  Result lifted;
  Result ground;
  Result by_sat;
  bool lifted_valid = false;
  bool ground_valid = false;
};

Solved solve(std::string_view stations, std::string_view goal = "")
{
  auto d = hddl::parse_domain(kDomain, "d.hddl");
  auto p = hddl::parse_problem(
    "(define (problem p) (:domain d) (:objects a b c d - place)\n"
    "(:htn :ordered-subtasks (and (go d)))\n"
    "(:init (at a) (road a b) (road b c) (road c d) (= (fuel) 5) (= (bought) 0)"
    " (= (need a b) 3) (= (need b c) 2) (= (need c d) 3) " + std::string(stations) + ")" +
    std::string(goal) + ")", "p.hddl");
  EXPECT_TRUE(d.ok() && p.ok());
  auto m = hddl::analyze(std::move(d).value(), std::move(p).value());
  EXPECT_TRUE(m.ok());
  Solved s;
  s.lifted = lifted(m.value());
  s.lifted_valid = s.lifted.outcome == Outcome::Solved &&
    verifier::verify(m.value(), s.lifted.plan).valid;
  auto g = tdg::ground(m.value());
  EXPECT_TRUE(g.ok());
  s.ground = total_order(g.value());
  s.ground_valid = s.ground.outcome == Outcome::Solved &&
    verifier::verify(m.value(), s.ground.plan).valid;
  s.by_sat = sat(g.value());
  return s;
}

TEST(NumericTest, FuelDecidesThePlan)
{
  // 5 - 3 = 2 at b, enough for c but not for d from there: it fills up at b
  Solved s = solve("(station b)");
  EXPECT_EQ(s.lifted.outcome, Outcome::Solved) << s.lifted.message;
  EXPECT_TRUE(s.lifted_valid) << s.lifted.plan;
  EXPECT_NE(s.lifted.plan.find("fill b"), std::string::npos) << s.lifted.plan;
  EXPECT_EQ(s.ground.outcome, Outcome::Solved) << s.ground.message;
  EXPECT_TRUE(s.ground_valid) << s.ground.plan;
  EXPECT_NE(s.ground.plan.find("fill b"), std::string::npos) << s.ground.plan;
  EXPECT_EQ(s.lifted.steps, 4u);
  EXPECT_EQ(s.ground.steps, 4u);
}

TEST(NumericTest, NoFuelNoPlan)
{
  Solved s = solve("");
  EXPECT_EQ(s.lifted.outcome, Outcome::Unsolvable) << s.lifted.message;
  EXPECT_NE(s.ground.outcome, Outcome::Solved);
  // A station too early does not help: at a the tank is full
  s = solve("(station a)");
  EXPECT_EQ(s.lifted.outcome, Outcome::Unsolvable) << s.lifted.message;
  EXPECT_NE(s.ground.outcome, Outcome::Solved);
}

TEST(NumericTest, NumbersInTheGoal)
{
  // Filling at b buys 3; at c it would buy 5, which the goal does not accept
  Solved s = solve("(station b) (station c)", "(:goal (and (at d) (<= (bought) 3)))");
  EXPECT_TRUE(s.lifted_valid) << s.lifted.message << s.lifted.plan;
  EXPECT_TRUE(s.ground_valid) << s.ground.message << s.ground.plan;
  EXPECT_EQ(s.lifted.plan.find("fill c"), std::string::npos) << s.lifted.plan;
  EXPECT_EQ(s.ground.plan.find("fill c"), std::string::npos) << s.ground.plan;
}

TEST(NumericTest, NotEqual)
{
  // The same with a negated equality: buying 5 is what the goal refuses
  Solved s = solve("(station b) (station c)", "(:goal (and (at d) (not (= (bought) 5))))");
  EXPECT_TRUE(s.lifted_valid) << s.lifted.message << s.lifted.plan;
  EXPECT_TRUE(s.ground_valid) << s.ground.message << s.ground.plan;
  EXPECT_EQ(s.ground.plan.find("fill c"), std::string::npos) << s.ground.plan;
}

TEST(NumericTest, SatisfiabilityLeavesNumbersToTheOthers)
{
  Solved s = solve("(station b)");
  EXPECT_EQ(s.by_sat.outcome, Outcome::Unsupported);
  EXPECT_FALSE(s.by_sat.message.empty());
}

}  // namespace
}  // namespace klimax::search

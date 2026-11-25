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
#include "search/sat.hpp"
#include "tdg/grounder.hpp"
#include "verify/verifier.hpp"

namespace klimax::search
{
namespace
{

// A workshop: `make` paints an item with any paint in any booth, after sanding it or not;
// which paint, booth and finish were right only the goal tells. `ship` needs the van at the
// door, which `fetch` brings by a road of three stretches
const char * kDomain =
  "(define (domain w) (:requirements :typing :hierarchy :negative-preconditions)\n"
  " (:types item paint booth place)\n"
  " (:constants door - place)\n"
  " (:predicates (raw ?i - item) (smooth ?i - item) (coloured ?i - item ?p - paint)\n"
  "  (has ?b - booth ?p - paint) (busy ?b - booth) (van ?l - place) (road ?a ?b - place)\n"
  "  (shipped ?i - item))\n"
  " (:task make :parameters (?i - item))\n"
  " (:task prepare :parameters (?i - item))\n"
  " (:task fetch :parameters (?to - place))\n"
  " (:task ship :parameters (?i - item))\n"
  " (:method make-it :parameters (?i - item ?p - paint ?b - booth) :task (make ?i)\n"
  "  :ordered-subtasks (and (prepare ?i) (spray ?i ?p ?b)))\n"
  " (:method prepare-sand :parameters (?i - item) :task (prepare ?i)\n"
  "  :ordered-subtasks (sand ?i))\n"
  " (:method prepare-nothing :parameters (?i - item) :task (prepare ?i))\n"
  " (:method fetch-here :parameters (?to - place) :task (fetch ?to) :precondition (van ?to))\n"
  " (:method fetch-far :parameters (?from ?mid ?to - place) :task (fetch ?to)\n"
  "  :precondition (and (van ?from) (road ?from ?mid))\n"
  "  :ordered-subtasks (and (drive ?from ?mid) (fetch ?to)))\n"
  " (:method ship-it :parameters (?i - item) :task (ship ?i)\n"
  "  :ordered-subtasks (and (fetch door) (load ?i)))\n"
  " (:action sand :parameters (?i - item) :precondition (raw ?i)\n"
  "  :effect (and (not (raw ?i)) (smooth ?i)))\n"
  " (:action spray :parameters (?i - item ?p - paint ?b - booth)\n"
  "  :precondition (and (has ?b ?p) (not (busy ?b)))\n"
  "  :effect (and (coloured ?i ?p) (busy ?b)))\n"
  " (:action drive :parameters (?from ?to - place)\n"
  "  :precondition (and (van ?from) (road ?from ?to))\n"
  "  :effect (and (not (van ?from)) (van ?to)))\n"
  " (:action load :parameters (?i - item) :precondition (van door) :effect (shipped ?i)))";

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
    "(define (problem p) (:domain w) (:objects chair table shelf - item red blue green - paint\n"
    " b1 b2 b3 - booth yard gate lane - place)\n" + std::string(htn) +
    "\n(:init (raw chair) (raw table) (raw shelf) (has b1 red) (has b1 blue) (has b2 blue)"
    " (has b3 green) (has b3 red) (van yard) (road yard gate) (road gate lane) (road lane door)"
    " (road gate yard))" + std::string(goal) + ")", "p.hddl");
  EXPECT_TRUE(d.ok() && p.ok());
  auto m = hddl::analyze(std::move(d).value(), std::move(p).value());
  EXPECT_TRUE(m.ok());
  auto g = tdg::ground(m.value());
  EXPECT_TRUE(g.ok());
  Solved s;
  if (polls == 0) {
    s.result = sat(g.value());
  } else {
    using Clock = kairos::BudgetConfig::Clock;
    Clock::time_point now{};
    SatSearch search(g.value());
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

TEST(SatSearchTest, TheGoalChoosesTheParameters)
{
  // Each booth serves once: blue for the table leaves b1 or b2, and red and green must fit
  Solved s = solve("(:htn :ordered-subtasks (and (make chair) (make table) (make shelf)))",
      "(:goal (and (coloured chair red) (coloured table blue) (coloured shelf green)"
      " (smooth table) (raw chair)))");
  ASSERT_EQ(s.result.outcome, Outcome::Solved) << s.result.message;
  EXPECT_TRUE(s.checked.valid) << s.checked.reason << "\n" << s.result.plan;
  EXPECT_EQ(s.result.steps, 4u) << s.result.plan;
  EXPECT_NE(s.result.plan.find("spray chair red b1"), std::string::npos) << s.result.plan;
  EXPECT_NE(s.result.plan.find("spray table blue b2"), std::string::npos) << s.result.plan;
  EXPECT_NE(s.result.plan.find("sand table"), std::string::npos) << s.result.plan;
}

TEST(SatSearchTest, RecursionTakesMoreLayers)
{
  Solved s = solve("(:htn :ordered-subtasks (and (make chair) (ship chair)))");
  ASSERT_EQ(s.result.outcome, Outcome::Solved) << s.result.message;
  EXPECT_TRUE(s.checked.valid) << s.checked.reason << "\n" << s.result.plan;
  EXPECT_NE(s.result.plan.find("drive lane door"), std::string::npos) << s.result.plan;
  EXPECT_NE(s.result.plan.find("load chair"), std::string::npos) << s.result.plan;
}

TEST(SatSearchTest, GoesOnAfterTheBudgetRanOut)
{
  const char * htn = "(:htn :ordered-subtasks (and (make chair) (make table) (ship chair)))";
  const char * goal = "(:goal (and (coloured chair red) (coloured table blue) (smooth table)))";
  const Solved whole = solve(htn, goal);
  ASSERT_EQ(whole.result.outcome, Outcome::Solved) << whole.result.message;
  for (int polls : {2, 3, 7}) {
    const Solved s = solve(htn, goal, polls);
    ASSERT_EQ(s.result.outcome, Outcome::Solved) << polls;
    EXPECT_GT(s.pieces, 1) << polls;
    EXPECT_TRUE(s.checked.valid) << polls << s.checked.reason;
    EXPECT_EQ(s.result.plan, whole.result.plan) << polls;
  }
}

TEST(SatSearchTest, NothingToDo)
{
  Solved s = solve("(:htn :ordered-subtasks (and (fetch yard)))");
  ASSERT_EQ(s.result.outcome, Outcome::Solved) << s.result.message;
  EXPECT_TRUE(s.checked.valid) << s.checked.reason << "\n" << s.result.plan;
  EXPECT_EQ(s.result.steps, 0u);
}

TEST(SatSearchTest, PartialOrderInOneOrder)
{
  // The tasks may go in any order: it takes one, and the plan is valid for the problem
  Solved s = solve("(:htn :subtasks (and (make chair) (make table)))",
      "(:goal (and (coloured chair green) (coloured table red)))");
  ASSERT_EQ(s.result.outcome, Outcome::Solved) << s.result.message;
  EXPECT_TRUE(s.checked.valid) << s.checked.reason << "\n" << s.result.plan;
}

}  // namespace
}  // namespace klimax::search

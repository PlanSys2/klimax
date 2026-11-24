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

#include <algorithm>
#include <string>
#include <string_view>

#include "hddl/model.hpp"
#include "hddl/parser.hpp"
#include "tdg/grounder.hpp"

namespace klimax::tdg
{
namespace
{

// Rooms in a line a - b - c; `go` is recursive; `fly` has no usable method
const char * kDomain =
  "(define (domain d) (:requirements :typing :hierarchy :negative-preconditions)\n"
  " (:types robot room)\n"
  " (:predicates (at ?r - robot ?l - room) (link ?a ?b - room) (winged ?r - robot))\n"
  " (:task go :parameters (?r - robot ?to - room))\n"
  " (:task fly :parameters (?r - robot ?to - room))\n"
  " (:method go-here :parameters (?r - robot ?to - room) :task (go ?r ?to)\n"
  "  :precondition (at ?r ?to))\n"
  " (:method go-step :parameters (?r - robot ?from ?mid ?to - room) :task (go ?r ?to)\n"
  "  :precondition (and (at ?r ?from) (link ?from ?mid))\n"
  "  :ordered-subtasks (and (move ?r ?from ?mid) (go ?r ?to)))\n"
  " (:method fly-there :parameters (?r - robot ?from ?to - room) :task (fly ?r ?to)\n"
  "  :precondition (winged ?r) :ordered-subtasks (soar ?r ?from ?to))\n"
  " (:action move :parameters (?r - robot ?from ?to - room)\n"
  "  :precondition (and (at ?r ?from) (link ?from ?to))\n"
  "  :effect (and (not (at ?r ?from)) (at ?r ?to)))\n"
  " (:action soar :parameters (?r - robot ?from ?to - room)\n"
  "  :precondition (and (at ?r ?from) (winged ?r))\n"
  "  :effect (and (not (at ?r ?from)) (at ?r ?to))))";

Problem grounded(std::string_view htn)
{
  auto d = hddl::parse_domain(kDomain, "d.hddl");
  auto p = hddl::parse_problem(
    "(define (problem p) (:domain d) (:objects r1 - robot a b c - room)\n" + std::string(htn) +
    "\n(:init (at r1 a) (link a b) (link b a) (link b c) (link c b)))", "p.hddl");
  EXPECT_TRUE(d.ok() && p.ok());
  auto m = hddl::analyze(std::move(d).value(), std::move(p).value());
  EXPECT_TRUE(m.ok());
  auto g = ground(m.value());
  EXPECT_TRUE(g.ok());
  return g.ok() ? std::move(g).value() : Problem{};
}

TEST(GrounderTest, ReachableTasksAndMethods)
{
  Problem p = grounded("(:htn :subtasks (go r1 c))");
  EXPECT_FALSE(p.unsolvable);
  ASSERT_EQ(p.initial.size(), 1u);
  // Only (go r1 c) is ever asked for; the four moves along the line
  EXPECT_EQ(p.tasks.size() - p.primitive_tasks, 1u);
  EXPECT_EQ(p.primitive_tasks, 4u);
  // go-here once (at c) and go-step once per move
  EXPECT_EQ(p.methods.size(), 1u + 1u + 4u);
  for (const Method & m : p.methods) {
    EXPECT_EQ(m.has_precondition, m.method != kNone);
    EXPECT_EQ(m.precondition.empty(), !m.has_precondition);
    for (uint32_t t : m.subtasks) {
      EXPECT_LT(t, p.tasks.size());
    }
  }
}

// No robot is winged: Kairos reaches no `soar`, so `fly` cannot be decomposed
TEST(GrounderTest, Unsolvable)
{
  Problem p = grounded("(:htn :subtasks (fly r1 c))");
  EXPECT_TRUE(p.unsolvable);
  EXPECT_TRUE(p.initial.empty());
  EXPECT_TRUE(p.methods.empty());
  // One dead task makes the whole initial network dead
  EXPECT_TRUE(grounded("(:htn :subtasks (and (go r1 c) (fly r1 b)))").unsolvable);
}

// A parameter of :htn takes every value that leaves something to do
TEST(GrounderTest, InitialParameters)
{
  Problem p = grounded("(:htn :parameters (?x - room) :subtasks (go r1 ?x))");
  EXPECT_EQ(p.initial.size(), 3u);
  EXPECT_EQ(p.tasks.size() - p.primitive_tasks, 3u);
}

TEST(GrounderTest, Budget)
{
  auto d = hddl::parse_domain(kDomain, "d.hddl");
  auto p = hddl::parse_problem(
    "(define (problem p) (:domain d) (:objects r1 - robot a - room) (:htn :subtasks (go r1 a))"
    " (:init (at r1 a)))", "p.hddl");
  auto m = hddl::analyze(std::move(d).value(), std::move(p).value());
  ASSERT_TRUE(m.ok());
  kairos::BudgetConfig config;
  config.deadline = kairos::BudgetConfig::Clock::now() - std::chrono::seconds(1);
  config.check_interval = 1;
  kairos::Budget budget(config);
  auto g = ground(m.value(), &budget);
  ASSERT_FALSE(g.ok());
  EXPECT_EQ(g.diagnostics().at(0).code, "ground.budget");
}

}  // namespace
}  // namespace klimax::tdg

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

#include "klimax/check.hpp"
#include "klimax/solve.hpp"

namespace klimax
{
namespace
{

// A robot crosses a corridor of zones and may not enter one where it knows of an enemy (the
// combat example of PlanSys2); it may only scout a zone if some sensor covers it
const char * kDomain =
  "(define (domain d) (:requirements :typing :hierarchy :negative-preconditions\n"
  "  :existential-preconditions :method-preconditions)\n"
  " (:types zone robot sensor)\n"
  " (:predicates (at ?r - robot ?z - zone) (road ?a ?b - zone) (hostile ?e - robot)\n"
  "  (covers ?s - sensor ?z - zone) (scouted ?z - zone))\n"
  " (:task reach :parameters (?r - robot ?z - zone))\n"
  " (:method here :parameters (?r - robot ?z - zone) :task (reach ?r ?z)\n"
  "  :precondition (at ?r ?z) :ordered-subtasks ())\n"
  " (:method step :parameters (?r - robot ?z ?a ?b - zone) :task (reach ?r ?z)\n"
  "  :precondition (and (at ?r ?a) (road ?a ?b) (not (at ?r ?z)))\n"
  "  :ordered-subtasks (and (scout ?b) (move ?r ?a ?b) (reach ?r ?z)))\n"
  " (:action scout :parameters (?z - zone)\n"
  "  :precondition (exists (?s - sensor) (covers ?s ?z)) :effect (scouted ?z))\n"
  " (:action move :parameters (?r - robot ?a ?b - zone)\n"
  "  :precondition (and (at ?r ?a) (road ?a ?b) (scouted ?b)\n"
  "   (not (exists (?e - robot) (and (hostile ?e) (at ?e ?b)))))\n"
  "  :effect (and (not (at ?r ?a)) (at ?r ?b))))";

std::string problem(const std::string & enemy_at)
{
  return "(define (problem p) (:domain d)\n"
         " (:objects z1 z2 z3 - zone me foe - robot s1 s2 - sensor)\n"
         " (:htn :ordered-subtasks (and (reach me z3)))\n"
         " (:init (at me z1) (road z1 z2) (road z2 z3) (covers s1 z2) (covers s2 z2)\n"
         "  (covers s1 z3) (hostile foe) (at foe " + enemy_at + ")))";
}

TEST(AxiomsTest, QuantifiedConditionsArePlannedWith)
{
  // Kairos makes axioms of both conditions; they are replaced by what they say
  SolveResult r = solve(kDomain, problem("z1"));
  ASSERT_EQ(r.status, Status::Solved) << r.message;
  EXPECT_EQ(r.steps, 4u) << r.plan;
  EXPECT_TRUE(verify(kDomain, problem("z1"), r.plan).valid) << r.plan;
}

TEST(AxiomsTest, AnEnemyOnTheWayLeavesNoPlan)
{
  // The enemy in z2 closes the only road: no plan, never one that goes through it
  SolveResult r = solve(kDomain, problem("z2"));
  EXPECT_NE(r.status, Status::Solved) << r.plan;
  EXPECT_NE(r.status, Status::InternalError) << r.message;
}

}  // namespace
}  // namespace klimax

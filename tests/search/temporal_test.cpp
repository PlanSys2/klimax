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

TEST(TemporalTest, WhatIsNotPlannedYetIsSaid)
{
  // Timed initial literals
  SolveResult r = solve(kDomain, problem("(at 5 (hot))"));
  EXPECT_EQ(r.status, Status::Unsupported) << r.plan;
  // A light that is on only while its switch is held: reading needs both at once, and
  // "no plan" would be false
  const char * domain =
    "(define (domain d) (:requirements :typing :hierarchy :durative-actions)\n"
    " (:predicates (light) (read))\n"
    " (:task study :parameters ())\n"
    " (:method m :parameters () :task (study) :subtasks (and (hold) (look)))\n"
    " (:durative-action hold :parameters () :duration (= ?duration 5)\n"
    "  :condition (and) :effect (and (at start (light)) (at end (not (light)))))\n"
    " (:durative-action look :parameters () :duration (= ?duration 2)\n"
    "  :condition (and (over all (light))) :effect (at end (read))))";
  r = solve(domain, "(define (problem p) (:domain d)\n"
      "(:htn :subtasks (and (study))) (:init) (:goal (read)))");
  EXPECT_EQ(r.status, Status::Unsupported) << r.message << r.plan;
  EXPECT_FALSE(r.message.empty());
}

}  // namespace
}  // namespace klimax

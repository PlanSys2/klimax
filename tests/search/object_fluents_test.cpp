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

// A truck that takes boxes from where they are to where they must be. Where the truck is, where
// each box is and what the truck carries are fluents whose value is an object
const char * kDomain =
  "(define (domain d)\n"
  " (:requirements :typing :hierarchy :object-fluents :negative-preconditions :equality)\n"
  " (:types place truck box)\n"
  " (:predicates (road ?a ?b - place))\n"
  " (:functions (loc ?t - truck) - place (in ?b - box) - place (carried ?t - truck) - box)\n"
  " (:task deliver :parameters (?b - box ?to - place))\n"
  " (:task go :parameters (?t - truck ?to - place))\n"
  " (:method go-here :parameters (?t - truck ?to - place) :task (go ?t ?to)\n"
  "  :precondition (= (loc ?t) ?to))\n"
  " (:method go-on :parameters (?t - truck ?mid ?to - place) :task (go ?t ?to)\n"
  "  :precondition (and (road (loc ?t) ?mid) (not (= (loc ?t) ?to)))\n"
  "  :ordered-subtasks (and (drive ?t ?mid) (go ?t ?to)))\n"
  " (:method deliver-it :parameters (?b - box ?to ?from - place ?t - truck)\n"
  "  :task (deliver ?b ?to) :precondition (= (in ?b) ?from)\n"
  "  :ordered-subtasks (and (go ?t ?from) (load ?t ?b) (go ?t ?to) (unload ?t ?b)))\n"
  " (:action drive :parameters (?t - truck ?to - place)\n"
  "  :precondition (road (loc ?t) ?to)\n"
  "  :effect (assign (loc ?t) ?to))\n"
  " (:action load :parameters (?t - truck ?b - box)\n"
  "  :precondition (= (in ?b) (loc ?t))\n"
  "  :effect (and (assign (carried ?t) ?b) (assign (in ?b) undefined)))\n"
  " (:action unload :parameters (?t - truck ?b - box)\n"
  "  :precondition (= (carried ?t) ?b)\n"
  "  :effect (and (assign (in ?b) (loc ?t)) (assign (carried ?t) undefined))))";

std::string problem(const std::string & roads, const std::string & goal = "(= (in b1) a)")
{
  return "(define (problem p) (:domain d) (:objects a b c - place t1 - truck b1 - box)\n"
         "(:htn :ordered-subtasks (and (deliver b1 a)))\n"
         "(:init " + roads + " (= (loc t1) a) (= (in b1) c))\n(:goal " + goal + "))";
}

const char * kRoads = "(road a b) (road b c) (road c b) (road b a)";

TEST(ObjectFluentsTest, ThePlanFollowsTheirValues)
{
  SolveOptions options;
  options.anytime = true;
  SolveResult r = solve(kDomain, problem(kRoads), options);
  ASSERT_EQ(r.status, Status::Solved) << r.message;
  EXPECT_EQ(r.steps, 6u) << r.plan;
  EXPECT_TRUE(r.optimal);
  EXPECT_NE(r.plan.find("load t1 b1"), std::string::npos) << r.plan;
  EXPECT_TRUE(verify(kDomain, problem(kRoads), r.plan).valid);
}

TEST(ObjectFluentsTest, NoWayBackNoPlan)
{
  // The truck reaches the box but cannot bring it
  SolveResult r = solve(kDomain, problem("(road a b) (road b c)"));
  EXPECT_EQ(r.status, Status::Unsolvable) << r.message << r.plan;
}

TEST(ObjectFluentsTest, TheGoalReadsThem)
{
  // The box ends at a, not at b
  SolveResult r = solve(kDomain, problem(kRoads, "(= (in b1) b)"));
  EXPECT_EQ(r.status, Status::Unsolvable) << r.message << r.plan;
  // A value that the goal refuses, and one that nothing holds
  r = solve(kDomain, problem(kRoads, "(and (not (= (in b1) c)) (not (= (carried t1) b1)))"));
  EXPECT_EQ(r.status, Status::Solved) << r.message;
}

TEST(ObjectFluentsTest, TheVerifierFollowsThem)
{
  const std::string p = problem(kRoads);
  const std::string tree = "root 7\n7 deliver b1 a -> deliver-it 8 3 11 6\n"
    "8 go t1 c -> go-on 1 9\n9 go t1 c -> go-on 2 10\n10 go t1 c -> go-here\n"
    "11 go t1 a -> go-on 4 12\n12 go t1 a -> go-on 5 13\n13 go t1 a -> go-here\n<==\n";
  const std::string good = "==>\n1 drive t1 b\n2 drive t1 c\n3 load t1 b1\n4 drive t1 b\n"
    "5 drive t1 a\n6 unload t1 b1\n" + tree;
  VerifyResult v = verify(kDomain, p, good);
  EXPECT_TRUE(v.valid) << v.reason;
  // Back to a instead of on to c: the truck is not where the box is
  std::string bad = good;
  bad.replace(bad.find("2 drive t1 c"), 12, "2 drive t1 a");
  EXPECT_FALSE(verify(kDomain, p, bad).valid);
  // Straight to c, where no road goes from a
  bad = good;
  bad.replace(bad.find("1 drive t1 b"), 12, "1 drive t1 c");
  EXPECT_FALSE(verify(kDomain, p, bad).valid);
  // The same plan where the box starts elsewhere: the method's precondition never holds
  std::string other = p;
  other.replace(other.find("(= (in b1) c)"), 13, "(= (in b1) b)");
  v = verify(kDomain, other, good);
  EXPECT_FALSE(v.valid);
}

TEST(ObjectFluentsTest, AFunctionTermAsTheArgumentOfATask)
{
  // (go ?t (in ?b)): where the box is when the method is applied
  std::string domain = kDomain;
  const std::string written = "(:method deliver-it :parameters (?b - box ?to ?from - place ?t - truck)\n"
    "  :task (deliver ?b ?to) :precondition (= (in ?b) ?from)\n"
    "  :ordered-subtasks (and (go ?t ?from)";
  ASSERT_NE(domain.find(written), std::string::npos);
  domain.replace(domain.find(written), written.size(),
    "(:method deliver-it :parameters (?b - box ?to - place ?t - truck)\n"
    "  :task (deliver ?b ?to)\n"
    "  :ordered-subtasks (and (go ?t (in ?b))");
  SolveResult r = solve(domain, problem(kRoads));
  ASSERT_EQ(r.status, Status::Solved) << r.message;
  EXPECT_EQ(r.steps, 6u) << r.plan;
  EXPECT_NE(r.plan.find("load t1 b1"), std::string::npos) << r.plan;
  EXPECT_TRUE(verify(domain, problem(kRoads), r.plan).valid);
  // Not in the initial task network, which has no moment before it
  const std::string p = "(define (problem p) (:domain d) (:objects a b c - place t1 - truck b1 - box)\n"
    "(:htn :ordered-subtasks (and (go t1 (in b1))))\n"
    "(:init (road a b) (= (loc t1) a) (= (in b1) b)))";
  EXPECT_EQ(solve(domain, p).status, Status::InvalidInput);
}

}  // namespace
}  // namespace klimax

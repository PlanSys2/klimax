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
#include "verify/verifier.hpp"

namespace klimax::verifier
{
namespace
{

// A robot that carries items between rooms; `tidy` leaves two items in the hall in any order
const char * kDomain =
  "(define (domain d) (:requirements :typing :hierarchy :negative-preconditions)\n"
  " (:types robot room item)\n"
  " (:predicates (at ?r - robot ?l - room) (in ?i - item ?l - room) (holding ?r - robot ?i - item)\n"
  "  (free ?r - robot) (link ?a ?b - room))\n"
  " (:task go :parameters (?r - robot ?to - room))\n"
  " (:task bring :parameters (?i - item ?to - room))\n"
  " (:task tidy :parameters (?a ?b - item))\n"
  " (:method go-here :parameters (?r - robot ?to - room) :task (go ?r ?to)\n"
  "  :precondition (at ?r ?to))\n"
  " (:method go-step :parameters (?r - robot ?from ?to - room) :task (go ?r ?to)\n"
  "  :precondition (and (at ?r ?from) (link ?from ?to)) :ordered-subtasks (move ?r ?from ?to))\n"
  " (:method bring-it :parameters (?i - item ?from ?to - room ?r - robot) :task (bring ?i ?to)\n"
  "  :precondition (in ?i ?from)\n"
  "  :subtasks (and (g1 (go ?r ?from)) (p (pick ?r ?i ?from)) (g2 (go ?r ?to)) (d (drop ?r ?i ?to)))\n"
  "  :ordering (and (< g1 p) (< p g2) (< g2 d)) :constraints (not (= ?from ?to)))\n"
  " (:method tidy-both :parameters (?a ?b - item) :task (tidy ?a ?b)\n"
  "  :subtasks (and (ta (bring ?a hall)) (tb (bring ?b hall))))\n"
  " (:constants hall - room)\n"
  " (:action move :parameters (?r - robot ?from ?to - room)\n"
  "  :precondition (and (at ?r ?from) (link ?from ?to))\n"
  "  :effect (and (not (at ?r ?from)) (at ?r ?to)))\n"
  " (:action pick :parameters (?r - robot ?i - item ?l - room)\n"
  "  :precondition (and (at ?r ?l) (in ?i ?l) (free ?r))\n"
  "  :effect (and (not (in ?i ?l)) (not (free ?r)) (holding ?r ?i)))\n"
  " (:action drop :parameters (?r - robot ?i - item ?l - room)\n"
  "  :precondition (and (at ?r ?l) (holding ?r ?i))\n"
  "  :effect (and (in ?i ?l) (free ?r) (not (holding ?r ?i)))))";

std::string problem(std::string_view htn, std::string_view goal = "")
{
  return "(define (problem p) (:domain d) (:objects r1 r2 - robot k - room cup book - item)\n" +
         std::string(htn) + "\n(:init (at r1 hall) (at r2 k) (free r1) (free r2) (link hall k)"
         " (link k hall) (in cup k) (in book k))" + std::string(goal) + ")";
}

Outcome check(const std::string & problem_text, std::string_view plan)
{
  auto d = hddl::parse_domain(kDomain, "d.hddl");
  auto p = hddl::parse_problem(problem_text, "p.hddl");
  EXPECT_TRUE(d.ok() && p.ok());
  auto m = hddl::analyze(std::move(d).value(), std::move(p).value());
  EXPECT_TRUE(m.ok());
  return verify(m.value(), plan);
}

const std::string kBring = problem("(:htn :subtasks (bring cup hall))");

// r1 goes to the kitchen, takes the cup and comes back
const char * kPlan =
  "==>\n"
  "1 move r1 hall k\n"
  "2 pick r1 cup k\n"
  "3 move r1 k hall\n"
  "4 drop r1 cup hall\n"
  "root 10\n"
  "10 bring cup hall -> bring-it 11 2 12 4\n"
  "11 go r1 k -> go-step 1\n"
  "12 go r1 hall -> go-step 3\n"
  "<==\n";

std::string replaced(std::string text, std::string_view from, std::string_view to)
{
  const size_t at = text.find(from);
  EXPECT_NE(at, std::string::npos) << from;
  return text.replace(at, from.size(), to);
}

TEST(VerifierTest, ValidPlan)
{
  Outcome o = check(kBring, kPlan);
  EXPECT_TRUE(o.valid) << o.reason;
  // Case, parentheses and text around the plan do not matter
  EXPECT_TRUE(check(kBring, replaced(replaced(kPlan, "1 move r1 hall k", "1 (MOVE R1 hall k)"),
      "==>", "Found a plan\n==>")).valid);
}

TEST(VerifierTest, GoalAtTheEnd)
{
  EXPECT_TRUE(check(problem("(:htn :subtasks (bring cup hall))", "(:goal (in cup hall))"),
    kPlan).valid);
  Outcome o = check(problem("(:htn :subtasks (bring cup hall))", "(:goal (in book hall))"), kPlan);
  EXPECT_FALSE(o.valid);
  EXPECT_NE(o.reason.find("goal"), std::string::npos) << o.reason;
}

void expect_invalid(const std::string & plan, std::string_view fragment)
{
  Outcome o = check(kBring, plan);
  EXPECT_FALSE(o.valid) << plan;
  EXPECT_NE(o.reason.find(fragment), std::string::npos) << o.reason;
}

TEST(VerifierTest, InvalidPlans)
{
  // The sequence
  expect_invalid(replaced(kPlan, "2 pick r1 cup k", "2 pick r1 book k"), "bring-it");
  expect_invalid(replaced(kPlan, "1 move r1 hall k", "1 move r1 k hall"), "go-step");
  expect_invalid(replaced(kPlan, "1 move r1 hall k\n2 pick r1 cup k", "2 pick r1 cup k\n1 move r1 hall k"),
    "ordering");
  expect_invalid(replaced(kPlan, "4 drop r1 cup hall\n", ""), "not in the plan");
  expect_invalid(replaced(kPlan, "1 move r1 hall k", "1 move r1 hall attic"), "unknown object");
  expect_invalid(replaced(kPlan, "1 move r1 hall k", "1 move cup hall k"), "wrong type");
  expect_invalid(replaced(kPlan, "1 move r1 hall k", "1 fly r1 hall k"), "unknown task");
  // The tree
  expect_invalid(replaced(kPlan, "root 10", "root 11"), "also a subtask");
  expect_invalid(replaced(replaced(kPlan, "root 10", "root 11"), "bring-it 11 2 12 4",
    "bring-it 12 2 12 4"), "more than one");
  expect_invalid(replaced(kPlan, "-> bring-it 11 2 12 4", "-> bring-it 11 2 12"), "not part of");
  expect_invalid(replaced(kPlan, "-> bring-it 11 2 12 4", "-> bring-it 2 11 12 4"), "bring-it");
  expect_invalid(replaced(kPlan, "-> bring-it", "-> tidy-both"), "does not decompose");
  expect_invalid(replaced(kPlan, "-> go-step 1", "-> go-here"), "not part of");
  expect_invalid(replaced(kPlan, "12 go r1 hall -> go-step 3", "12 go r1 hall -> go-step 1"),
    "more than one");
  expect_invalid(replaced(kPlan, "12 go r1 hall -> go-step 3\n", ""), "not in the plan");
  expect_invalid(replaced(kPlan, "root 10\n", ""), "root");
  expect_invalid("nothing here", "==>");
}

// An action applicable where it is, but its method's precondition does not hold
TEST(VerifierTest, MethodPrecondition)
{
  // go-here for a robot that is elsewhere
  const std::string p = problem("(:htn :subtasks (go r1 k))");
  Outcome o = check(p, "==>\nroot 1\n1 go r1 k -> go-here\n<==\n");
  EXPECT_FALSE(o.valid);
  EXPECT_NE(o.reason.find("precondition of method go-here"), std::string::npos) << o.reason;
  EXPECT_TRUE(check(problem("(:htn :subtasks (go r1 hall))"),
    "==>\nroot 1\n1 go r1 hall -> go-here\n<==\n").valid);
  // The constraint of bring-it: from and to must differ
  o = check(problem("(:htn :subtasks (bring cup k))"),
      "==>\n2 pick r2 cup k\n4 drop r2 cup k\nroot 10\n10 bring cup k -> bring-it 11 2 12 4\n"
      "11 go r2 k -> go-here\n12 go r2 k -> go-here\n<==\n");
  EXPECT_FALSE(o.valid);
  EXPECT_NE(o.reason.find("constraints"), std::string::npos) << o.reason;
}

// Two unordered tasks: their actions may interleave, and each precondition is tested at a point
// the orderings allow
TEST(VerifierTest, PartialOrder)
{
  const std::string p = problem("(:htn :subtasks (tidy cup book))");
  const char * tree =
    "root 20\n"
    "20 tidy cup book -> tidy-both 21 22\n"
    "21 bring cup hall -> bring-it 30 2 31 4\n"
    "22 bring book hall -> bring-it 32 6 33 8\n"
    "30 go r1 k -> go-step 1\n"
    "31 go r1 hall -> go-step 3\n"
    "32 go r2 k -> go-here\n"
    "33 go r2 hall -> go-step 7\n"
    "<==\n";
  const std::string interleaved = std::string(
    "==>\n1 move r1 hall k\n6 pick r2 book k\n2 pick r1 cup k\n7 move r2 k hall\n"
    "3 move r1 k hall\n8 drop r2 book hall\n4 drop r1 cup hall\n") + tree;
  Outcome o = check(p, interleaved);
  EXPECT_TRUE(o.valid) << o.reason;
  // The children of tidy-both in the other order: the method does not order them
  EXPECT_TRUE(check(p, replaced(interleaved, "tidy-both 21 22", "tidy-both 22 21")).valid);
  // Dropping before coming back breaks the order inside bring-it
  o = check(p, replaced(interleaved, "3 move r1 k hall\n8 drop r2 book hall\n",
      "8 drop r2 book hall\n3 move r1 k hall\n"));
  EXPECT_TRUE(o.valid) << o.reason;  // still fine: these two belong to different tasks
  o = check(p, replaced(interleaved, "7 move r2 k hall\n3 move r1 k hall\n8 drop r2 book hall\n",
      "8 drop r2 book hall\n7 move r2 k hall\n3 move r1 k hall\n"));
  EXPECT_FALSE(o.valid);
}

TEST(VerifierTest, Numbers)
{
  // `a` needs one unit of (f) and takes it; `b` puts back what (g) says, which has no value
  // until `c` gives it one
  auto outcome = [](std::string_view steps, std::string_view plan) {
      auto d = hddl::parse_domain(
        "(define (domain d) (:requirements :hierarchy :numeric-fluents) (:predicates (p))\n"
        " (:functions (f) (g)) (:task t :parameters ())\n"
        " (:method m :parameters () :task (t) :ordered-subtasks (and " + std::string(steps) +
        "))\n"
        " (:action a :parameters () :precondition (>= (f) 1) :effect (and (p) (decrease (f) 1)))\n"
        " (:action b :parameters () :effect (increase (f) (* 2 (g))))\n"
        " (:action c :parameters () :precondition (< (f) 1) :effect (assign (g) (+ (f) 0.5))))",
        "d.hddl");
      auto p = hddl::parse_problem(
        "(define (problem p) (:domain d) (:htn :subtasks (t)) (:init (= (f) 2))"
        " (:goal (and (p) (= (f) 0))))", "p.hddl");
      EXPECT_TRUE(d.ok() && p.ok());
      auto m = hddl::analyze(std::move(d).value(), std::move(p).value());
      EXPECT_TRUE(m.ok());
      return verify(m.value(), std::string(plan));
    };
  Outcome o = outcome("(a) (a)", "==>\n1 a\n2 a\nroot 3\n3 t -> m 1 2\n<==\n");
  EXPECT_TRUE(o.valid) << o.reason;
  // The third one finds nothing left
  o = outcome("(a) (a) (a)", "==>\n1 a\n2 a\n4 a\nroot 3\n3 t -> m 1 2 4\n<==\n");
  EXPECT_FALSE(o.valid);
  EXPECT_NE(o.reason.find("not applicable"), std::string::npos) << o.reason;
  // One left: the goal wants none
  o = outcome("(a)", "==>\n1 a\nroot 3\n3 t -> m 1\n<==\n");
  EXPECT_FALSE(o.valid);
  // (g) has no value yet
  o = outcome("(a) (b)", "==>\n1 a\n2 b\nroot 3\n3 t -> m 1 2\n<==\n");
  EXPECT_FALSE(o.valid);
  EXPECT_NE(o.reason.find("no value"), std::string::npos) << o.reason;
  // 2 - 1 - 1 = 0; (g) = 0.5; back to 1; and to 0
  o = outcome("(a) (a) (c) (b) (a)",
      "==>\n1 a\n2 a\n4 c\n5 b\n6 a\nroot 3\n3 t -> m 1 2 4 5 6\n<==\n");
  EXPECT_TRUE(o.valid) << o.reason;
}

TEST(VerifierTest, UnsupportedIsNeverAccepted)
{
  auto d = hddl::parse_domain(
    "(define (domain d) (:requirements :hierarchy :timed-initial-literals :numeric-fluents)\n"
    " (:predicates (p)) (:functions (f))\n"
    " (:task t :parameters ())\n"
    " (:method m :parameters () :task (t) :ordered-subtasks (a))\n"
    " (:action a :parameters () :effect (p)))", "d.hddl");
  auto p = hddl::parse_problem(
    "(define (problem p) (:domain d) (:htn :subtasks (t)) (:init (at 5 (= (f) 3))))", "p.hddl");
  ASSERT_TRUE(d.ok() && p.ok());
  auto m = hddl::analyze(std::move(d).value(), std::move(p).value());
  ASSERT_TRUE(m.ok());
  Outcome o = verify(m.value(), "==>\n1 a\nroot 2\n2 t -> m 1\n<==\n");
  EXPECT_FALSE(o.valid);
  EXPECT_NE(o.reason.find("unsupported"), std::string::npos) << o.reason;
}

TEST(VerifierTest, ManyValuesForFreeParameters)
{
  // The method leaves four parameters free: 20736 values, chosen from what its precondition
  // requires in the state. The initial task network leaves five, and any value will do
  const char * domain =
    "(define (domain d) (:requirements :typing :hierarchy :negative-preconditions)\n"
    "(:types n)\n"
    "(:predicates (next ?a ?b - n) (bad ?a - n) (seen ?a - n))\n"
    "(:task walk :parameters ())\n"
    "(:method chain :parameters (?a ?b ?c ?d - n) :task (walk)\n"
    "  :precondition (and (next ?a ?b) (next ?b ?c) (next ?c ?d) (not (bad ?d)))\n"
    "  :ordered-subtasks (and (see ?a)))\n"
    "(:action see :parameters (?a - n) :precondition (and) :effect (seen ?a)))";
  auto outcome = [&](std::string_view init, std::string_view plan) {
      auto d = hddl::parse_domain(domain, "d.hddl");
      auto p = hddl::parse_problem(
        "(define (problem p) (:domain d)\n"
        "(:objects n0 n1 n2 n3 n4 n5 n6 n7 n8 n9 n10 n11 - n)\n"
        "(:htn :parameters (?v ?w ?x ?y ?z - n) :subtasks (walk))\n"
        "(:init " + std::string(init) + "))", "p.hddl");
      EXPECT_TRUE(d.ok() && p.ok());
      auto m = hddl::analyze(std::move(d).value(), std::move(p).value());
      EXPECT_TRUE(m.ok());
      return verify(m.value(), plan);
    };
  const char * plan = "==>\n1 see n4\nroot 2\n2 walk -> chain 1\n<==\n";
  // Two chains from n4: the one through n9 ends in a bad object
  const std::string init = "(next n4 n9) (next n9 n1) (next n1 n2) (bad n2) (next n4 n5) "
    "(next n5 n6) (next n6 n7) (next n0 n1)";
  Outcome o = outcome(init, plan);
  EXPECT_TRUE(o.valid) << o.reason;
  EXPECT_FALSE(outcome(init + " (bad n7)", plan).valid);
  EXPECT_FALSE(outcome(init, "==>\n1 see n0\nroot 2\n2 walk -> chain 1\n<==\n").valid);
}

}  // namespace
}  // namespace klimax::verifier

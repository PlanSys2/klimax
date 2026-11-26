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

// Going through a door: open it, walk, close it; a lamp that can be switched on and off at any
// moment. `constraints` is what the method says of the states
std::string domain(const std::string & constraints)
{
  return "(define (domain d) (:requirements :hierarchy :method-constraints)\n"
         " (:predicates (open) (inside) (lit) (key))\n"
         " (:task enter :parameters ())\n"
         " (:method m :parameters () :task (enter)\n"
         "  :subtasks (and (o (unlock)) (w (walk)) (c (lock)))\n"
         "  :ordering (and (< o w) (< w c))\n"
         "  :constraints (and " + constraints + "))\n"
         " (:action unlock :parameters () :effect (open))\n"
         " (:action walk :parameters () :precondition (open) :effect (inside))\n"
         " (:action lock :parameters () :effect (not (open)))\n"
         " (:action switch :parameters () :effect (lit))\n"
         " (:action dim :parameters () :effect (not (lit))))";
}

// The plan: the steps of the method, with switching the lamp on and off where `order` says
// ("s": switch, "d": dim, "o", "w", "c")
std::string plan(const std::string & order)
{
  std::string actions;
  std::string roots = "root 9";
  for (char step : order) {
    switch (step) {
      case 'o': actions += "1 unlock\n"; break;
      case 'w': actions += "2 walk\n"; break;
      case 'c': actions += "3 lock\n"; break;
      case 's': actions += "4 switch\n"; roots += " 4"; break;
      default: actions += "5 dim\n"; roots += " 5"; break;
    }
  }
  return "==>\n" + actions + roots + "\n9 enter -> m 1 2 3\n<==\n";
}

std::string problem(const std::string & order, const std::string & init = "")
{
  std::string tasks = "(enter)";
  for (char step : order) {
    tasks += step == 's' ? " (switch)" : step == 'd' ? " (dim)" : "";
  }
  return "(define (problem p) (:domain d) (:htn :subtasks (and " + tasks + "))\n"
         "(:init " + init + "))";
}

bool valid(const std::string & constraints, const std::string & order, const std::string & init = "")
{
  return verify(domain(constraints), problem(order, init), plan(order)).valid;
}

TEST(ConstraintsTest, BeforeAfterBetween)
{
  // Lit just before walking: switched on before that, whenever
  EXPECT_TRUE(valid("(before w (lit))", "sowc"));
  EXPECT_TRUE(valid("(before w (lit))", "oswc"));
  EXPECT_FALSE(valid("(before w (lit))", "owsc"));
  // And not dimmed in between
  EXPECT_FALSE(valid("(before w (lit))", "sodwc"));
  // Just after: its own effects count
  EXPECT_TRUE(valid("(after w (inside))", "owc"));
  EXPECT_FALSE(valid("(after w (lit))", "owsc"));
  EXPECT_TRUE(valid("(after w (lit))", "sowc"));
  // (at (start id) ...) and (at (end id) ...) are the same two
  EXPECT_FALSE(valid("(at (start w) (lit))", "owsc"));
  EXPECT_TRUE(valid("(at (end w) (inside))", "owc"));
  // From the end of one to the start of the other, in every state
  EXPECT_TRUE(valid("(between o c (open))", "owc"));
  EXPECT_TRUE(valid("(between o c (lit))", "sowc"));
  EXPECT_FALSE(valid("(between o c (lit))", "sowdc"));
  EXPECT_FALSE(valid("(between o c (lit))", "oswc"));
  EXPECT_TRUE(valid("(between o c (lit))", "sowcd"));
}

TEST(ConstraintsTest, AlongTheTask)
{
  // The task lasts from its first action to its last: what happens outside does not count
  EXPECT_TRUE(valid("(always (lit))", "sowcd"));
  EXPECT_FALSE(valid("(always (lit))", "sowdc"));
  EXPECT_FALSE(valid("(always (lit))", "oswc"));
  EXPECT_TRUE(valid("(sometime (lit))", "oswc"));
  EXPECT_FALSE(valid("(sometime (lit))", "owcs"));
  EXPECT_TRUE(valid("(at start (key))", "owc", "(key)"));
  EXPECT_FALSE(valid("(at start (open))", "owc"));
  EXPECT_TRUE(valid("(at end (inside))", "owc"));
  EXPECT_FALSE(valid("(at end (open))", "owc"));
  // It becomes true once, however long it stays
  EXPECT_TRUE(valid("(at-most-once (lit))", "oswc"));
  EXPECT_FALSE(valid("(at-most-once (lit))", "osdwsc"));
  // If inside ever holds, lit held before
  EXPECT_TRUE(valid("(sometime-before (inside) (lit))", "oswc"));
  EXPECT_FALSE(valid("(sometime-before (inside) (lit))", "owsc"));
  // Whenever lit holds, the door is closed then or later
  EXPECT_TRUE(valid("(sometime-after (open) (inside))", "owc"));
  EXPECT_FALSE(valid("(sometime-after (inside) (open))", "owc"));
}

TEST(ConstraintsTest, OfTheInitialNetwork)
{
  // About the whole plan
  const char * free = "(define (domain d) (:requirements :hierarchy :method-constraints)\n"
    " (:predicates (lit))\n"
    " (:action switch :parameters () :effect (lit))\n"
    " (:action dim :parameters () :effect (not (lit))))";
  auto check = [&](const std::string & constraints) {
      return verify(free, "(define (problem p) (:domain d)\n"
               "(:htn :subtasks (and (a (switch)) (b (dim))) :ordering (< a b)\n"
               " :constraints (and " + constraints + ")) (:init))",
               "==>\n1 switch\n2 dim\nroot 1 2\n<==\n").valid;
    };
  EXPECT_TRUE(check("(sometime (lit))"));
  EXPECT_FALSE(check("(always (lit))"));
  EXPECT_TRUE(check("(after a (lit))"));
  EXPECT_FALSE(check("(after b (lit))"));
  EXPECT_TRUE(check("(at end (not (lit)))"));
}

TEST(ConstraintsTest, ThePlannerTestsWhatMustHoldBefore)
{
  // Lit before walking: the lamp is switched on first, though nothing else asks for it
  const std::string p = problem("s");
  SolveResult r = solve(domain("(before w (lit))"), p);
  ASSERT_EQ(r.status, Status::Solved) << r.message;
  EXPECT_LT(r.plan.find("switch"), r.plan.find("walk")) << r.plan;
  EXPECT_TRUE(verify(domain("(before w (lit))"), p, r.plan).valid);
  // The end of a between is tested too
  r = solve(domain("(between o c (lit))"), p);
  if (r.status == Status::Solved) {
    EXPECT_TRUE(verify(domain("(between o c (lit))"), p, r.plan).valid);
  } else {
    EXPECT_EQ(r.status, Status::Unsupported) << r.message;
  }
  // With no lamp to switch on there is no plan
  r = solve(domain("(before w (lit))"), problem(""));
  EXPECT_NE(r.status, Status::Solved) << r.plan;
}

TEST(ConstraintsTest, ThePlannerSaysWhatItDoesNotMeet)
{
  // The searches do not look at these constraints: a plan that meets them is a plan, one
  // that does not is not given
  SolveResult r = solve(domain("(after w (inside))"), problem(""));
  EXPECT_EQ(r.status, Status::Solved) << r.message;
  r = solve(domain("(always (lit))"), problem(""));
  EXPECT_EQ(r.status, Status::Unsupported) << r.plan;
  EXPECT_NE(r.message.find("always"), std::string::npos) << r.message;
}

TEST(ConstraintsTest, WithTimes)
{
  // Two jobs of five units, one from 0 and the other from 5: times count from the start of
  // the task
  auto check = [](const std::string & constraints) {
      const std::string d =
        "(define (domain d)\n"
        " (:requirements :typing :hierarchy :durative-actions :method-constraints)\n"
        " (:types worker)\n"
        " (:predicates (done ?w - worker))\n"
        " (:task shift :parameters (?v ?w - worker))\n"
        " (:method both :parameters (?v ?w - worker) :task (shift ?v ?w)\n"
        "  :subtasks (and (x (work ?v)) (y (work ?w)))\n"
        "  :constraints (and " + constraints + "))\n"
        " (:durative-action work :parameters (?w - worker) :duration (= ?duration 5)\n"
        "  :condition (and) :effect (at end (done ?w))))";
      return verify(d, "(define (problem p) (:domain d) (:objects ann bob - worker)\n"
               "(:htn :subtasks (and (shift ann bob))) (:init))",
               "==>\n1 0: (work ann) [5]\n2 5: (work bob) [5]\nroot 3\n"
               "3 shift ann bob -> both 1 2\n<==\n").valid;
    };
  EXPECT_TRUE(check("(within 6 (done ?v))"));
  EXPECT_FALSE(check("(within 4 (done ?v))"));
  EXPECT_TRUE(check("(hold-after 5 (done ?v))"));
  EXPECT_FALSE(check("(hold-after 3 (done ?v))"));
  EXPECT_TRUE(check("(hold-during 5 10 (done ?v))"));
  EXPECT_FALSE(check("(hold-during 2 10 (done ?v))"));
  EXPECT_TRUE(check("(hold-during 0 4 (not (done ?v)))"));
  EXPECT_TRUE(check("(always-within 6 (done ?v) (done ?w))"));
  EXPECT_FALSE(check("(always-within 3 (done ?v) (done ?w))"));
}

TEST(ConstraintsTest, Errors)
{
  auto errors = [&](const std::string & constraints) {
      return !check(domain(constraints), problem("")).ok;
    };
  EXPECT_FALSE(errors("(before w (lit))"));
  EXPECT_TRUE(errors("(before nobody (lit))"));
  EXPECT_TRUE(errors("(before w (unknown))"));
  EXPECT_TRUE(errors("(hold-during 5 2 (lit))"));
  // The form of the grammar, with a task where PDDL 3.0 has a time
  EXPECT_TRUE(errors("(within (start w) (lit))"));
  EXPECT_FALSE(errors("(within 3 (lit))"));
}

}  // namespace
}  // namespace klimax

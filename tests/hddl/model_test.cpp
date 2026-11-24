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
#include <vector>

#include "hddl/model.hpp"
#include "hddl/parser.hpp"

namespace klimax::hddl
{
namespace
{

const char * kDomain =
  "(define (domain d) (:requirements :typing :hierarchy :negative-preconditions)\n"
  " (:types robot room - object kitchen - room) (:constants base - room)\n"
  " (:predicates (at ?r - robot ?l - room))\n"
  " (:task go-to :parameters (?r - robot ?to - room))\n"
  " (:action move :parameters (?r - robot ?from ?to - room)\n"
  "  :precondition (at ?r ?from) :effect (and (not (at ?r ?from)) (at ?r ?to)))\n";

const char * kProblem =
  "(define (problem p) (:domain d) (:objects r1 - robot hall - room k - kitchen)\n"
  " (:htn :ordered-subtasks (go-to r1 hall)) (:init (at r1 k)))";

kairos::Result<Model> model(std::string_view methods, std::string_view problem = kProblem)
{
  auto d = parse_domain(std::string(kDomain) + std::string(methods) + ")", "d.hddl");
  auto p = parse_problem(problem, "p.hddl");
  EXPECT_TRUE(d.ok() && p.ok());
  return analyze(std::move(d).value(), std::move(p).value());
}

std::vector<std::string> errors(std::string_view methods, std::string_view problem = kProblem)
{
  auto r = model(methods, problem);
  EXPECT_FALSE(r.ok()) << "expected an error";
  std::vector<std::string> out;
  for (const auto & d : r.diagnostics()) {
    if (d.severity == kairos::Severity::Error) {
      out.push_back(d.code);
    }
  }
  return out;
}

const char * kStep =
  "(:method step :parameters (?r - robot ?a ?b ?to - room) :task (go-to ?r ?to)\n"
  " :precondition (at ?r ?a) :ordered-subtasks (and (move ?r ?a ?b) (go-to ?r ?to)))\n";
const char * kArrived =
  "(:method arrived :parameters (?r - robot ?to - room) :task (go-to ?r ?to)\n"
  " :precondition (at ?r ?to))\n";

TEST(HddlModelTest, ResolvesNames)
{
  auto r = model(std::string(kStep) + kArrived);
  ASSERT_TRUE(r.ok());
  const Model & m = r.value();
  EXPECT_EQ(m.user_actions, 1u);
  EXPECT_EQ(m.pddl.actions.size(), 3u);  // move and one per method precondition
  ASSERT_EQ(m.tasks.size(), 2u);
  EXPECT_TRUE(m.tasks[TaskId(0)].primitive);
  EXPECT_EQ(m.tasks[TaskId(1)].name, "go-to");
  ASSERT_EQ(m.methods.size(), 2u);
  const MethodInfo & step = m.methods[0];
  EXPECT_EQ(step.task, TaskId(1));
  EXPECT_EQ(step.task_args, (std::vector<Arg>{{true, 0}, {true, 3}}));
  EXPECT_EQ(step.precondition_action, 1);
  ASSERT_EQ(step.network.steps.size(), 2u);
  EXPECT_EQ(step.network.steps[0].task, TaskId(0));
  EXPECT_EQ(step.network.steps[0].args, (std::vector<Arg>{{true, 0}, {true, 1}, {true, 2}}));
  EXPECT_EQ(step.network.order, (std::vector<std::pair<uint32_t, uint32_t>>{{0, 1}}));
  EXPECT_TRUE(step.network.total_order);
  EXPECT_EQ(m.methods_of[1], (std::vector<uint32_t>{0, 1}));
  EXPECT_TRUE(m.has_htn);
  ASSERT_EQ(m.initial.steps.size(), 1u);
  EXPECT_FALSE(m.initial.steps[0].args[0].variable);
  EXPECT_TRUE(m.total_order);
}

TEST(HddlModelTest, PartialOrder)
{
  auto r = model(
    "(:method m :parameters (?r - robot ?to - room) :task (go-to ?r ?to)\n"
    " :subtasks (and (a (move ?r base ?to)) (b (move ?r ?to base)) (c (go-to ?r base)))\n"
    " :ordering (and (< a c) (< b c)) :constraints (and (not (= ?to base)) (sortof ?to - kitchen)))");
  ASSERT_TRUE(r.ok());
  const Network & n = r->methods[0].network;
  EXPECT_FALSE(n.total_order);
  EXPECT_FALSE(r->total_order);
  EXPECT_EQ(n.order.size(), 2u);
  ASSERT_EQ(n.constraints.size(), 1u);
  EXPECT_FALSE(n.constraints[0].equal);
  EXPECT_FALSE(n.constraints[0].rhs.variable);
  ASSERT_EQ(n.sorts.size(), 1u);
  EXPECT_EQ(n.sorts[0].param, 1u);
  // A chain written with :ordering is a total order too
  auto chain = model(
    "(:method m :parameters (?r - robot ?to - room) :task (go-to ?r ?to)\n"
    " :subtasks (and (a (move ?r base ?to)) (b (go-to ?r base))) :ordering (< b a))");
  ASSERT_TRUE(chain.ok());
  EXPECT_TRUE(chain->methods[0].network.total_order);
}

TEST(HddlModelTest, Errors)
{
  using V = std::vector<std::string>;
  const std::string head = "(:method m :parameters (?r - robot ?to - room) ";
  EXPECT_EQ(errors(head + ":task (fly ?r ?to))"), V{"hddl.unknown_task"});
  EXPECT_EQ(errors(head + ":task (move ?r ?to ?to))"), V{"hddl.method_for_action"});
  EXPECT_EQ(errors(head + ":task (go-to ?r))"), V{"hddl.arity"});
  EXPECT_EQ(errors(head + ":task (go-to ?to ?r))"), (V{"hddl.type_mismatch", "hddl.type_mismatch"}));
  EXPECT_EQ(errors(head + ":task (go-to ?r ?x))"), V{"hddl.undeclared_variable"});
  EXPECT_EQ(errors(head + ":task (go-to ?r nowhere))"), V{"hddl.unknown_object"});
  EXPECT_EQ(errors(head + ":task (go-to ?r ?to) :subtasks (jump ?r))"), V{"hddl.unknown_task"});
  EXPECT_EQ(errors(head + ":task (go-to ?r ?to) :subtasks (move ?r ?to))"), V{"hddl.arity"});
  EXPECT_EQ(errors(head + ":task (go-to ?r ?to) :subtasks (and (a (go-to ?r ?to))"
    " (a (go-to ?r ?to))))"), V{"hddl.duplicate_subtask_id"});
  EXPECT_EQ(errors(head + ":task (go-to ?r ?to) :subtasks (a (go-to ?r ?to)) :ordering (< a z))"),
    V{"hddl.unknown_subtask_id"});
  EXPECT_EQ(errors(head + ":task (go-to ?r ?to) :subtasks (and (a (go-to ?r ?to))"
    " (b (go-to ?r ?to))) :ordering (and (< a b) (< b a)))"), V{"hddl.cyclic_ordering"});
  EXPECT_EQ(errors(head + ":task (go-to ?r ?to) :constraints (sortof ?to - nothing))"),
    V{"hddl.unknown_type"});
  EXPECT_EQ(errors("(:method m :parameters (?r ?r - robot ?to - room) :task (go-to ?r ?to))"),
    V{"hddl.duplicate_parameter"});
  EXPECT_EQ(errors("(:method m :parameters (?r - robot ?to - cave) :task (go-to ?r ?to))"),
    V{"hddl.unknown_type"});
  EXPECT_EQ(errors(std::string(kArrived) + kArrived), V{"hddl.duplicate_method"});
  EXPECT_EQ(errors("(:task go-to :parameters ())"), V{"hddl.duplicate_task"});
  EXPECT_EQ(errors("(:task move :parameters ())"), V{"hddl.task_is_action"});
  // A constant of the wrong type is an error; a variable of a compatible type is not
  EXPECT_EQ(errors(head + ":task (go-to ?r ?to) :subtasks (go-to base ?to))"),
    V{"hddl.type_mismatch"});
}

// Kairos checks the precondition of a method as it checks that of an action
TEST(HddlModelTest, PreconditionIsChecked)
{
  auto r = model(
    "(:method m :parameters (?r - robot ?to - room) :task (go-to ?r ?to)\n"
    " :precondition (near ?r ?to))");
  ASSERT_FALSE(r.ok());
  ASSERT_TRUE(r.diagnostics().at(0).location.has_value());
  EXPECT_EQ(r.diagnostics()[0].location->line, 8u);
}

TEST(HddlModelTest, ProblemErrors)
{
  using V = std::vector<std::string>;
  auto problem = [](const std::string & htn) {
      return "(define (problem p) (:domain d) (:objects r1 - robot hall - room)\n " + htn +
             " (:init (at r1 hall)))";
    };
  EXPECT_EQ(errors(kArrived, problem("(:htn :subtasks (go-to r1 attic))")),
    V{"hddl.unknown_object"});
  EXPECT_EQ(errors(kArrived, problem("(:htn :subtasks (go-to hall r1))")),
    (V{"hddl.type_mismatch", "hddl.type_mismatch"}));
  EXPECT_EQ(errors(kArrived, problem("(:htn :subtasks ())")), V{"hddl.nothing_to_do"});
  EXPECT_EQ(errors(kArrived, problem("")), V{"hddl.nothing_to_do"});
  // Constants of the domain and parameters of :htn are usable in the initial network
  auto ok = model(kArrived, problem("(:htn :parameters (?x - room) :subtasks (and (go-to r1 base)"
      " (go-to r1 ?x)))"));
  ASSERT_TRUE(ok.ok());
  EXPECT_TRUE(ok->initial.steps[1].args[1].variable);
  EXPECT_FALSE(ok->initial.total_order);
  // A goal and no :htn: a classical problem
  auto classical = model(kArrived,
      "(define (problem p) (:domain d) (:objects r1 - robot) (:init) (:goal (at r1 base)))");
  ASSERT_TRUE(classical.ok());
  EXPECT_FALSE(classical->has_htn);
}

}  // namespace
}  // namespace klimax::hddl

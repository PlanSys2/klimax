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
#include <vector>

#include "hddl/parser.hpp"
#include "pddl/printer.hpp"

namespace klimax::hddl
{
namespace
{

std::string dump(const std::vector<kairos::Diagnostic> & diagnostics)
{
  std::string out;
  for (const auto & d : diagnostics) {
    out += kairos::to_string(d) + "\n";
  }
  return out;
}

std::vector<std::string> codes(const std::vector<kairos::Diagnostic> & diagnostics)
{
  std::vector<std::string> out;
  for (const auto & d : diagnostics) {
    out.push_back(d.code);
  }
  return out;
}

Domain domain_ok(std::string_view text)
{
  auto r = parse_domain(text, "d.hddl");
  EXPECT_TRUE(r.ok()) << dump(r.diagnostics());
  return r.ok() ? std::move(r).value() : Domain{};
}

Problem problem_ok(std::string_view text)
{
  auto r = parse_problem(text, "p.hddl");
  EXPECT_TRUE(r.ok()) << dump(r.diagnostics());
  return r.ok() ? std::move(r).value() : Problem{};
}

std::vector<std::string> domain_errors(std::string_view text)
{
  auto r = parse_domain(text, "d.hddl");
  EXPECT_FALSE(r.ok()) << "expected a parse failure";
  return codes(r.diagnostics());
}

std::string name(const Domain & d, Symbol s) {return std::string(d.pddl.symbols.name(s));}
std::string name(const Problem & p, Symbol s) {return std::string(p.pddl.symbols.name(s));}

std::string term(const Domain & d, ast::TermId id)
{
  return name(d, d.pddl.nodes.terms[id].name);
}

const char * kHead = "(define (domain d) (:requirements :typing :hierarchy :method-preconditions)"
  " (:types a b) (:predicates (p ?x - a))\n";

std::string in_domain(std::string_view body) {return std::string(kHead) + std::string(body) + ")";}

TEST(HddlParserTest, TaskAndMethod)
{
  Domain d = domain_ok(in_domain(
      "(:task Deliver :parameters (?i - a ?to - b))\n"
      "(:method m-deliver :parameters (?i - a ?to - b ?r - a)\n"
      "  :task (deliver ?i ?to) :precondition (and (p ?i) (not (p ?r)))\n"
      "  :ordered-subtasks (and (go ?r ?to) (fetch ?r ?i)))\n"
      "(:action go :parameters (?r - a ?to - b) :precondition (p ?r) :effect (not (p ?r)))"));
  ASSERT_EQ(d.tasks.size(), 1u);
  EXPECT_EQ(name(d, d.tasks[0].name), "deliver");
  EXPECT_EQ(d.tasks[0].params.size(), 2u);
  EXPECT_EQ(d.tasks[0].loc, (ast::Loc{2, 2}));

  ASSERT_EQ(d.methods.size(), 1u);
  const Method & m = d.methods[0];
  EXPECT_EQ(name(d, m.name), "m-deliver");
  EXPECT_EQ(m.params.size(), 3u);
  EXPECT_EQ(name(d, m.task), "deliver");
  ASSERT_EQ(m.task_args.size(), 2u);
  EXPECT_EQ(term(d, m.task_args[1]), "?to");
  EXPECT_EQ(kairos::pddl::to_pddl(d.pddl.nodes, d.pddl.symbols, m.precondition),
    "(and (p ?i) (not (p ?r)))");
  EXPECT_TRUE(m.network.ordered);
  ASSERT_EQ(m.network.subtasks.size(), 2u);
  EXPECT_EQ(name(d, m.network.subtasks[1].task), "fetch");
  EXPECT_FALSE(m.network.subtasks[1].id.valid());
  EXPECT_EQ(term(d, m.network.subtasks[1].args[0]), "?r");
  EXPECT_EQ(d.pddl.actions.size(), 1u);
}

TEST(HddlParserTest, PartialOrderWithIdsAndConstraints)
{
  Domain d = domain_ok(in_domain(
      "(:task t :parameters (?x ?y - a))\n"
      "(:method m :parameters (?x ?y ?z - a) :task (t ?x ?y)\n"
      "  :subtasks (and (t1 (u ?x)) (t2 (u ?y)) (t3 (v)))\n"
      "  :ordering (and (< t1 t2) (< t1 t3))\n"
      "  :constraints (and (not (= ?x ?y)) (= ?z ?x) (sortof ?z - a) (not (sortof ?y - b))))"));
  const TaskNetwork & n = d.methods.at(0).network;
  EXPECT_FALSE(n.ordered);
  ASSERT_EQ(n.subtasks.size(), 3u);
  EXPECT_EQ(name(d, n.subtasks[0].id), "t1");
  EXPECT_EQ(name(d, n.subtasks[2].task), "v");
  EXPECT_TRUE(n.subtasks[2].args.empty());
  ASSERT_EQ(n.ordering.size(), 2u);
  EXPECT_EQ(name(d, n.ordering[1].before), "t1");
  EXPECT_EQ(name(d, n.ordering[1].after), "t3");
  ASSERT_EQ(n.constraints.size(), 4u);
  EXPECT_EQ(n.constraints[0].kind, ConstraintKind::Different);
  EXPECT_EQ(n.constraints[1].kind, ConstraintKind::Equal);
  EXPECT_EQ(term(d, n.constraints[1].lhs), "?z");
  EXPECT_EQ(n.constraints[2].kind, ConstraintKind::Sort);
  EXPECT_EQ(name(d, n.constraints[2].types.at(0)), "a");
  EXPECT_EQ(n.constraints[3].kind, ConstraintKind::NotSort);
}

// The spellings found in the competition domains
TEST(HddlParserTest, Spellings)
{
  Domain d = domain_ok(in_domain(
      "(:task t :parameters ())\n"
      "(:method empty :parameters () :task (t) :subtasks ())\n"
      "(:method none :parameters () :task (t))\n"
      "(:method single :parameters () :task (t) :subtasks (u))\n"
      "(:method single-id :parameters () :task (t) :tasks (t1 (u)) :order (< t1 t1))\n"
      "(:method ordered-tasks :parameters () :task (t) :ordered-tasks (and (u) (v))\n"
      "  :ordering () :constraints ())\n"
      "(:method prec :parameters () :task (t) :precondition () :subtasks (and))"));
  ASSERT_EQ(d.methods.size(), 6u);
  EXPECT_TRUE(d.methods[0].network.subtasks.empty());
  EXPECT_TRUE(d.methods[1].network.subtasks.empty());
  EXPECT_EQ(d.methods[2].network.subtasks.size(), 1u);
  EXPECT_EQ(name(d, d.methods[3].network.subtasks.at(0).id), "t1");
  EXPECT_EQ(d.methods[3].network.ordering.size(), 1u);
  EXPECT_TRUE(d.methods[4].network.ordered);
  EXPECT_EQ(d.methods[4].network.subtasks.size(), 2u);
  EXPECT_TRUE(d.methods[5].precondition.valid());
}

TEST(HddlParserTest, HddlRequirementsAreKnown)
{
  auto r = parse_domain(
    "(define (domain d) (:requirements :hierarchy :method-preconditions :typing :bogus))", "d");
  ASSERT_TRUE(r.ok());
  ASSERT_EQ(r.diagnostics().size(), 1u) << dump(r.diagnostics());
  EXPECT_NE(r.diagnostics()[0].message.find(":bogus"), std::string::npos);
}

TEST(HddlParserTest, Errors)
{
  using V = std::vector<std::string>;
  EXPECT_EQ(domain_errors(in_domain("(:method m :parameters ())")), V{"hddl.method_without_task"});
  EXPECT_EQ(domain_errors(in_domain("(:method m :task (t) :task (t))")), V{"parse.duplicate_key"});
  EXPECT_EQ(domain_errors(in_domain("(:method m :task (t) :subtasks (u) :ordered-subtasks (v))")),
    V{"parse.duplicate_key"});
  EXPECT_EQ(domain_errors(in_domain("(:method m :task (t) :effect (p))")), V{"parse.unknown_key"});
  EXPECT_EQ(domain_errors(in_domain("(:method m :task (t) :ordering (> a b))")),
    V{"hddl.ordering"});
  EXPECT_EQ(domain_errors(in_domain("(:method m :task (t) :constraints (p ?x))")),
    V{"hddl.constraint"});
  EXPECT_EQ(domain_errors(in_domain("(:method m :task (t) :causallinks (and))")),
    V{"hddl.causal_links"});
  EXPECT_EQ(domain_errors(in_domain("(:task t :precondition (p ?x))")),
    V{"hddl.task_with_precondition"});
  // One bad method does not hide the next error
  EXPECT_EQ(domain_errors(in_domain("(:method m :task (t) :subtasks (?x))\n(:method n)")),
    (V{"hddl.subtask", "hddl.method_without_task"}));
}

TEST(HddlParserTest, ErrorPosition)
{
  auto r = parse_domain(in_domain("(:method m :parameters ()\n   :task (t) :ordering (> a b))"),
      "d.hddl");
  ASSERT_FALSE(r.ok());
  ASSERT_TRUE(r.diagnostics().at(0).location.has_value());
  EXPECT_EQ(r.diagnostics()[0].location->file, "d.hddl");
  EXPECT_EQ(r.diagnostics()[0].location->line, 3u);
  EXPECT_EQ(r.diagnostics()[0].location->column, 25u);
}

TEST(HddlParserTest, Problem)
{
  Problem p = problem_ok(
    "(define (problem p) (:domain d) (:objects o1 o2 - a)\n"
    " (:htn :parameters (?x - a) :subtasks (and (t1 (deliver o1 ?x)) (t2 (go o2)))\n"
    "  :ordering (< t1 t2) :constraints (not (= ?x o1)))\n"
    " (:init (p o1)))");
  EXPECT_TRUE(p.has_htn);
  EXPECT_EQ(p.htn_params.size(), 1u);
  ASSERT_EQ(p.htn.subtasks.size(), 2u);
  EXPECT_EQ(name(p, p.htn.subtasks[0].task), "deliver");
  EXPECT_EQ(p.htn.ordering.size(), 1u);
  EXPECT_EQ(p.htn.constraints.size(), 1u);
  EXPECT_FALSE(p.pddl.goal.valid());
  EXPECT_EQ(p.pddl.init.size(), 1u);

  Problem classical = problem_ok("(define (problem p) (:domain d) (:init) (:goal (p o1)))");
  EXPECT_FALSE(classical.has_htn);
  EXPECT_TRUE(classical.pddl.goal.valid());

  auto twice = parse_problem("(define (problem p) (:domain d) (:htn :tasks ()) (:htn :tasks ()))");
  EXPECT_FALSE(twice.ok());
}

}  // namespace
}  // namespace klimax::hddl

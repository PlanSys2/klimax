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

/// \file
/// \brief HDDL syntax tree: Kairos' PDDL tree plus tasks, methods and task networks.

#ifndef KLIMAX__HDDL__AST_HPP_
#define KLIMAX__HDDL__AST_HPP_

#include <cstdint>
#include <vector>

#include "pddl/ast.hpp"

namespace klimax::hddl
{

namespace ast = kairos::ast;
using kairos::Symbol;

/// @brief "(id (task t1 ... tn))" or "(task t1 ... tn)".
struct Subtask
{
  Symbol id;  // invalid if not given
  Symbol task;
  std::vector<ast::TermId> args;
  ast::Loc loc;
};

/// @brief What an ordering constraint says of a subtask: the task (the point depends on the
/// side it is on), or when it starts or ends.
enum class Point : uint8_t {Task, Start, End};

/// @brief "(< before after)", and in HDDL 2.1 "(op x y)" with op one of <, <=, =, >=, > and
/// each side an id, "(start id)" or "(end id)"; or "(not (op x y))".
struct Order
{
  Symbol before;
  Symbol after;
  ast::Loc loc;
  ast::CompareOp op = ast::CompareOp::Lt;
  Point before_point = Point::Task;
  Point after_point = Point::Task;
  bool negated = false;
};

/// @brief "(op x y)" in the :duration of a method: each side ?duration (of the task
/// decomposed), "(duration id)" (of a subtask) or a numeric expression.
struct MethodDuration
{
  ast::CompareOp op = ast::CompareOp::Eq;
  ast::ExprId lhs;
  ast::ExprId rhs;
  ast::Loc loc;
};

enum class ConstraintKind : uint8_t
{
  Equal,      // (= t1 t2)
  Different,  // (not (= t1 t2))
  Sort,       // (sortof ?v - type)
  NotSort,    // (not (sortof ?v - type))
  // HDDL 2.1, on the states of the plan: a condition `first` (and `second`) and, for some,
  // subtask ids `a` and `b` or times `from` and `to`
  Before,          // (before id gd): just before the subtask starts
  After,           // (after id gd): just after it ends
  Between,         // (between id1 id2 gd): from the end of one to the start of the other
  AtStart,         // (at start gd): when the task decomposed starts
  AtEnd,           // (at end gd)
  Always,          // (always gd): while it lasts
  Sometime,        // (sometime gd)
  AtMostOnce,      // (at-most-once gd)
  SometimeBefore,  // (sometime-before gd1 gd2): if gd1 ever holds, gd2 did before
  SometimeAfter,   // (sometime-after gd1 gd2): whenever gd1 holds, gd2 does then or later
  Within,          // (within t gd): by t after it starts
  HoldAfter,       // (hold-after t gd): from t on
  HoldDuring,      // (hold-during t1 t2 gd)
  AlwaysWithin     // (always-within t gd1 gd2): whenever gd1 holds, gd2 does within t
};

struct Constraint
{
  ConstraintKind kind = ConstraintKind::Equal;
  ast::TermId lhs;
  ast::TermId rhs;            // Equal, Different
  std::vector<Symbol> types;  // Sort, NotSort
  ast::Loc loc;
  Symbol a;                   // subtask ids
  Symbol b;
  ast::FormulaId first;
  ast::FormulaId second;
  double from = 0.0;
  double to = 0.0;
};

struct TaskNetwork
{
  std::vector<Subtask> subtasks;
  bool ordered = false;  // :ordered-subtasks: a total order in the order written
  std::vector<Order> ordering;
  std::vector<Constraint> constraints;
};

/// @brief An abstract task: "(:task name :parameters (...))".
struct Task
{
  Symbol name;
  std::vector<ast::TypedName> params;
  ast::Loc loc;
};

struct Method
{
  Symbol name;
  std::vector<ast::TypedName> params;
  Symbol task;
  std::vector<ast::TermId> task_args;
  ast::Loc task_loc;
  ast::FormulaId precondition;  // invalid if absent
  TaskNetwork network;
  ast::Loc loc;
  // (:durative-method ...): a condition with at start, over all and at end in `precondition`,
  // and constraints on durations
  bool durative = false;
  std::vector<MethodDuration> duration;
};

/// @brief Symbols and nodes of the HTN part live in `pddl`.
struct Domain
{
  ast::Domain pddl;
  std::vector<Task> tasks;
  std::vector<Method> methods;
};

struct Problem
{
  ast::Problem pddl;
  bool has_htn = false;
  std::vector<ast::TypedName> htn_params;
  TaskNetwork htn;
  ast::Loc htn_loc;
};

}  // namespace klimax::hddl

#endif  // KLIMAX__HDDL__AST_HPP_

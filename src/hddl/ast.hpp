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

/// @brief "(< before after)".
struct Order
{
  Symbol before;
  Symbol after;
  ast::Loc loc;
};

enum class ConstraintKind : uint8_t
{
  Equal,      // (= t1 t2)
  Different,  // (not (= t1 t2))
  Sort,       // (sortof ?v - type)
  NotSort     // (not (sortof ?v - type))
};

struct Constraint
{
  ConstraintKind kind = ConstraintKind::Equal;
  ast::TermId lhs;
  ast::TermId rhs;            // Equal, Different
  std::vector<Symbol> types;  // Sort, NotSort
  ast::Loc loc;
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

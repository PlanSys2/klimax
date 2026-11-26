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
/// \brief The checked model of an HDDL domain and problem: names resolved, types verified.

#ifndef KLIMAX__HDDL__MODEL_HPP_
#define KLIMAX__HDDL__MODEL_HPP_

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "core/id.hpp"
#include "hddl/ast.hpp"
#include "kairos/result.hpp"
#include "semantic/model.hpp"

namespace klimax::hddl
{

namespace sem = kairos::sem;

using TaskId = kairos::Id<struct TaskTag>;

/// @brief An argument of a task: a parameter of the enclosing method (or of :htn), or an object.
struct Arg
{
  bool variable = false;
  uint32_t index = 0;  // parameter position, or sem::ObjectId value

  bool operator==(const Arg &) const = default;
};

/// @brief A task, primitive (an action) or abstract.
struct TaskInfo
{
  std::string name;
  std::vector<sem::TypeSpec> params;
  bool primitive = false;
  bool durative = false;  // primitive only
  uint32_t action = 0;    // primitive: index in pddl.actions or pddl.durative_actions
  ast::Loc loc;
  bool artificial = false;  // added by parameter splitting: not part of a plan
};

struct Step
{
  TaskId task;
  std::vector<Arg> args;
  std::string id;  // empty if not given
  ast::Loc loc;
};

struct VarConstraint
{
  bool equal = true;
  Arg lhs;
  Arg rhs;
};

/// @brief (sortof ?v - t) and its negation.
struct SortConstraint
{
  uint32_t param = 0;
  sem::TypeSpec types;
  bool negated = false;
};

/// @brief When step `a` starts or ends, compared with when step `b` does: <, <= or =.
struct TimeConstraint
{
  uint32_t a = 0;
  bool a_end = false;
  uint32_t b = 0;
  bool b_end = false;
  ast::CompareOp op = ast::CompareOp::Le;
};

/// @brief A duration constraint of a method: two expressions of `pddl` over ?duration (of the
/// task decomposed), "(duration id)" (of a step: MethodInfo::duration_of) and the numeric
/// functions of the domain.
struct DurationConstraint
{
  ast::CompareOp op = ast::CompareOp::Eq;
  ast::ExprId lhs;
  ast::ExprId rhs;
};

/// @brief A constraint of HDDL 2.1 on the states of the plan: what must hold just before a
/// step, just after it or between two, or along the task that the network decomposes.
struct StateConstraint
{
  ConstraintKind kind = ConstraintKind::Always;
  uint32_t a = 0;          // steps (Before, After, Between)
  uint32_t b = 0;
  ast::FormulaId first;    // in the nodes of the domain; of the problem for the initial network
  ast::FormulaId second;
  double from = 0.0;       // times after the start of the task
  double to = 0.0;
  // `first` of a method as an action of `pddl` with no effect, as its precondition is (-1:
  // none, for the initial network), and the parameters of the method that are that action's
  int32_t action = -1;
  std::vector<uint32_t> action_params;
};

struct Network
{
  std::vector<Step> steps;
  // (a, b): step a before step b, that is, a has ended when b starts; acyclic. What the
  // searches order by
  std::vector<std::pair<uint32_t, uint32_t>> order;
  // HDDL 2.1: what the ordering constraints say of the times and `order` does not: a strict
  // comparison of time points, or one that is not "ends before it starts". Checked on the
  // times of a plan
  std::vector<TimeConstraint> times;
  std::vector<StateConstraint> states;
  std::vector<VarConstraint> constraints;
  std::vector<SortConstraint> sorts;
  bool total_order = false;  // the order leaves a single sequence
};

struct Param
{
  std::string name;  // with the '?'
  sem::TypeSpec types;
};

struct MethodInfo
{
  std::string name;
  std::vector<Param> params;
  TaskId task;
  std::vector<Arg> task_args;
  // The precondition as an action of `pddl` with no effect, so that Kairos checks and grounds
  // it; -1 if the method has none
  int32_t precondition_action = -1;
  // The parameters of that action: those the precondition mentions, as positions in `params`
  std::vector<uint32_t> precondition_params;
  bool artificial = false;  // added by parameter splitting: not part of a plan
  Network network;
  ast::Loc loc;
  // (:durative-method ...). Its condition at start is the precondition above; the whole of
  // it, with what must hold over all and at end, is this formula of `pddl` (invalid: none)
  ast::FormulaId condition;
  std::vector<DurationConstraint> durations;
  // The step that each "(duration id)" in them names, by expression
  std::vector<std::pair<ast::ExprId, uint32_t>> duration_of;
};

struct Model
{
  ast::Domain pddl;      // with one extra action per method precondition
  ast::Problem problem;
  sem::DomainModel domain;
  sem::ProblemModel objects;
  size_t user_actions = 0;  // pddl.actions[0 .. user_actions) are the domain's own

  kairos::IdVector<TaskId, TaskInfo> tasks;
  std::vector<MethodInfo> methods;
  std::vector<std::vector<uint32_t>> methods_of;  // by abstract TaskId value

  bool has_htn = false;  // false: a classical problem (a goal and no :htn)
  bool has_goal = false;  // the problem states one
  std::vector<Param> initial_params;
  Network initial;

  bool total_order = false;  // every network, the initial one included
};

/// @brief Checks a parsed domain and problem and resolves every name.
kairos::Result<Model> analyze(Domain domain, Problem problem);

}  // namespace klimax::hddl

#endif  // KLIMAX__HDDL__MODEL_HPP_

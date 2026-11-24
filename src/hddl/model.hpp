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

struct Network
{
  std::vector<Step> steps;
  std::vector<std::pair<uint32_t, uint32_t>> order;  // (a, b): step a before step b; acyclic
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
  // The precondition as an action of `pddl` with the method's parameters and no effect, so
  // that Kairos checks and grounds it; -1 if the method has none
  int32_t precondition_action = -1;
  Network network;
  ast::Loc loc;
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
  std::vector<Param> initial_params;
  Network initial;

  bool total_order = false;  // every network, the initial one included
};

/// @brief Checks a parsed domain and problem and resolves every name.
kairos::Result<Model> analyze(Domain domain, Problem problem);

}  // namespace klimax::hddl

#endif  // KLIMAX__HDDL__MODEL_HPP_

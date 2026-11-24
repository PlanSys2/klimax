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
/// \brief Grounding: the reachable ground tasks and methods (task decomposition graph).

#ifndef KLIMAX__TDG__GROUNDER_HPP_
#define KLIMAX__TDG__GROUNDER_HPP_

#include <cstdint>
#include <limits>
#include <vector>

#include "core/budget.hpp"
#include "ground/task.hpp"
#include "hddl/model.hpp"
#include "kairos/result.hpp"

namespace klimax::tdg
{

constexpr uint32_t kNone = std::numeric_limits<uint32_t>::max();

/// @brief A ground task: an action instance or an abstract task with its arguments.
struct Task
{
  hddl::TaskId task;
  std::vector<uint32_t> args;     // objects
  std::vector<uint32_t> actions;  // primitive: its instances in `pddl.actions` (several if Kairos
                                  // split a disjunctive precondition)
  std::vector<uint32_t> methods;  // abstract: the ground methods that decompose it
};

struct Method
{
  uint32_t method = kNone;             // index in the model; kNone for the initial task network
  uint32_t task = kNone;               // the ground task it decomposes
  std::vector<uint32_t> params;        // objects, by parameter
  std::vector<uint32_t> subtasks;      // ground tasks, by step of the method's network
  bool has_precondition = false;
  std::vector<uint32_t> precondition;  // instances in `pddl.actions`: holds if any is applicable
};

struct Stats
{
  size_t kairos_actions = 0;   // action instances reachable ignoring the hierarchy
  size_t tasks_built = 0;      // derivable, before keeping what the initial network reaches
  size_t methods_built = 0;
  double milliseconds = 0.0;
};

/**
 * @brief The ground problem: facts and action instances from Kairos, and over them the tasks and
 * methods reachable from the initial task network.
 */
struct Problem
{
  kairos::ground::Task pddl;
  std::vector<Task> tasks;
  std::vector<Method> methods;
  std::vector<uint32_t> initial;  // the ways of instantiating the initial task network (methods)
  bool unsolvable = false;        // no way of decomposing the initial tasks into actions
  size_t primitive_tasks = 0;
  Stats stats;
};

/**
 * @brief Grounds a checked model.
 *
 * - Kairos grounds the actions and the method preconditions (as actions with no effect) by
 *   relaxed reachability, ignoring the hierarchy: a superset of what any plan can use.
 * - Top-down, which tasks the initial network can ask for, with the arguments known at that
 *   point; and only for those:
 * - Bottom-up, in Datalog (Kairos' engine): a method instance is derivable when its
 *   precondition is reachable and its subtasks are derivable; an abstract task, when some method
 *   of it is. So every instance kept can be decomposed into reachable actions.
 * - From the initial task network down, only what it reaches through derivable instances is kept.
 *
 * Error "ground.budget" when the budget runs out.
 */
kairos::Result<Problem> ground(const hddl::Model & model, kairos::Budget * budget = nullptr);

}  // namespace klimax::tdg

#endif  // KLIMAX__TDG__GROUNDER_HPP_

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
  size_t split_subtasks = 0;   // subtasks moved by parameter splitting
  size_t kairos_actions = 0;   // action instances reachable ignoring the hierarchy
  size_t tasks_built = 0;      // derivable, before keeping what the initial network reaches
  size_t methods_built = 0;
  double milliseconds = 0.0;
  double kairos_ms = 0.0;    // of them, Kairos' grounding of the actions
  double datalog_ms = 0.0;   // the Datalog program of the hierarchy
};

/**
 * @brief The ground problem: facts and action instances from Kairos, and over them the tasks and
 * methods reachable from the initial task network.
 */
struct Problem
{
  hddl::Model model;  // the model that was grounded: the one given, with parameters split
  kairos::ground::Task pddl;
  std::vector<Task> tasks;
  std::vector<Method> methods;
  std::vector<uint32_t> initial;  // the ways of instantiating the initial task network (methods)
  // By instance in `pddl.actions`, with each durative action as one step: what it lasts (its
  // duration if a constant, or the least it allows; 1 otherwise), 0 if instantaneous. Empty
  // without durative actions.
  std::vector<double> durations;
  bool unsolvable = false;        // no way of decomposing the initial tasks into actions
  size_t primitive_tasks = 0;
  Stats stats;
};

/**
 * @brief Grounds a checked model.
 *
 * - Parameters that only one subtask of a method uses are split off (hddl/split.hpp).
 * - One Datalog program (Kairos' engine) computes together, so that each part limits the others:
 *   which tasks the initial network can ask for, top-down, with the arguments known at that
 *   point; which action instances are asked for and have their positive preconditions reachable;
 *   the facts those actions add; and which method instances and tasks can be decomposed into
 *   such actions, bottom-up. An action that no task asks for adds nothing: the reachability is
 *   that of the hierarchy, not that of the actions alone.
 * - Kairos then grounds exactly the action instances derived (and the method preconditions, as
 *   actions with no effect), and what the initial network reaches through them is kept.
 *
 * `unsolvable` is a proof: the relaxation only ever adds possibilities.
 *
 * With `start_and_end`, every durative action is planned as two steps, its start and its end
 * (hddl::start_and_end, tdg/compress.hpp), instead of one: for problems where actions must
 * overlap.
 *
 * Error "ground.budget" when the budget runs out.
 */
kairos::Result<Problem> ground(
  const hddl::Model & model, kairos::Budget * budget = nullptr, bool start_and_end = false);

}  // namespace klimax::tdg

#endif  // KLIMAX__TDG__GROUNDER_HPP_

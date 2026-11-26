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
/// \brief Verifier of hierarchical plans in the format of the IPC HTN tracks.

#ifndef KLIMAX__VERIFY__VERIFIER_HPP_
#define KLIMAX__VERIFY__VERIFIER_HPP_

#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "hddl/model.hpp"

namespace klimax::verifier
{

/// @brief When an action of a plan starts and what it lasts.
struct Timed
{
  double time = 0.0;
  double duration = 0.0;
};

struct Outcome
{
  bool valid = false;
  std::string reason;  // why not, in one sentence; empty if valid
  // Valid, and the problem has a metric that does not read time: its value at the end of the
  // plan (none if it reads a fluent with no value)
  std::optional<double> metric;
  // Valid, with durative actions: when each action of the plan starts and what it lasts, in
  // plan order. Those the plan gives, or one after another if it gives none.
  std::vector<Timed> times;
  // Not valid, and only because its times break a constraint on times or durations: with
  // other times the same actions and decomposition may be a plan
  bool only_times = false;
  // Not valid because a constraint of HDDL 2.1 on the states of the plan does not hold
  // (before, after, between, always...): the searches do not look at those
  bool constraint = false;
};

/// @brief A step of a verified plan: an action, or a task with the method that decomposes it.
struct Step
{
  std::string id;                 // as in the plan
  std::string task;
  std::vector<std::string> args;  // objects
  bool primitive = false;
  // Abstract tasks only
  std::string method;
  std::vector<std::string> params;    // objects, by parameter of the method
  std::vector<std::string> subtasks;  // ids, by subtask of the method as written in the domain
  std::vector<std::string> labels;    // of those subtasks in the domain; "" where not given
  // In a plan with times: when it starts and when it ends. A task, when the first of its
  // actions starts and the last one ends
  std::optional<double> start;
  std::optional<double> end;
};

/// @brief The decomposition of a verified plan.
struct Tree
{
  std::vector<Step> steps;            // one per line of the plan
  std::vector<std::string> sequence;  // ids of the actions, in the order of execution
  std::vector<std::string> roots;     // ids, by task of the initial task network
  std::vector<std::string> root_labels;
};

/// @brief The actions of a plan and what the hierarchy says of their order.
struct Order
{
  struct Action
  {
    hddl::TaskId task;
    std::vector<uint32_t> args;  // objects
  };
  std::vector<Action> actions;  // in plan order
  // Pairs of positions in `actions`: the first has ended when the second starts, because a
  // task network orders the tasks they belong to. None if there would be more than `limit`
  // (`complete` is then false).
  std::vector<std::pair<uint32_t, uint32_t>> before;
  size_t limit = 300000;
  bool complete = false;
};

/**
 * @brief Checks that `plan` solves the problem of `model` (docs/semantics.md, section 3.1).
 *
 * The plan is the primitive sequence and its decomposition tree. Checked: every step is an
 * action with objects of the right types; the tree starts at the initial task network and each
 * node applies a method of its task whose subtasks are its children; the sequence respects every
 * ordering constraint; it is executable from the initial state; every method precondition holds
 * at a point allowed by the orderings; variable constraints hold; and the goal, if any, holds at
 * the end.
 *
 * Independent of the searches: the states and what holds in them come from the evaluator of
 * Kairos' validator, which interprets the syntax tree and knows nothing of the ground problem
 * the searches work on. Sequential plans, with numeric and object fluents; anything else is
 * reported as unsupported, never accepted.
 *
 * With `tree`, a valid plan is also given as its decomposition: which subtask of its method each
 * step is, and the values of the method's parameters (those with which its precondition held).
 *
 * With `order`, of a valid plan: its actions and the orderings between them that come from
 * the hierarchy.
 */
Outcome verify(
  const hddl::Model & model, std::string_view plan, Tree * tree = nullptr,
  Order * order = nullptr);

}  // namespace klimax::verifier

#endif  // KLIMAX__VERIFY__VERIFIER_HPP_

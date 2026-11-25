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
/// \brief Search for totally ordered problems: depth-first progression over the ground problem.

#ifndef KLIMAX__SEARCH__TOTAL_ORDER_HPP_
#define KLIMAX__SEARCH__TOTAL_ORDER_HPP_

#include <cstdint>
#include <functional>
#include <string>

#include "core/budget.hpp"
#include "tdg/grounder.hpp"

namespace klimax::search
{

enum class Outcome : uint8_t
{
  Solved,
  Unsolvable,   // the whole space was searched
  OutOfBudget,
  Unsupported   // a partial order, axioms, object fluents: `message` says which
};

struct Options
{
  // Once a plan is found, look for shorter ones until none is left or the budget runs out
  bool anytime = false;
  // Called with every plan found, each shorter than the one before (the last one is the result)
  std::function<void(const std::string & plan, size_t steps)> on_plan;
};

struct Result
{
  Outcome outcome = Outcome::Unsupported;
  std::string plan;     // Solved: in the format of the IPC HTN tracks
  std::string message;  // Unsupported: what; otherwise, which way of searching answered
  size_t steps = 0;         // actions of the plan
  uint64_t expansions = 0;  // tasks taken from the stack
  size_t plans = 0;         // found, each shorter than the one before
  bool optimal = false;     // proved: no plan has fewer actions
};

/**
 * @brief Depth-first progression: the first task of the stack is executed if it is an action,
 * or replaced by the subtasks of one of its methods; a state and stack already seen with as few
 * actions done are not searched again.
 *
 * Methods are tried from the one that can be decomposed into fewest actions. The search is
 * repeated with a bound on the length of the plan that doubles each time: the actions done plus
 * the fewest the pending tasks can take (a lower bound from the decomposition graph) must not
 * exceed it. So the plan found is at most twice as long as needed by that measure, and a
 * recursion that does not progress is cut.
 *
 * The values of the numeric fluents are part of the state; the bounds and the relaxations do
 * not look at numeric conditions, which only makes them more optimistic.
 *
 * Complete on a finite space when every task network is totally ordered. A partially ordered
 * network is searched in one of the orders it allows: a plan found is a plan, but none found
 * proves nothing (Unsupported, with a message).
 *
 * With `Options::anytime`, the search goes on after the first plan: depth first under a bound
 * of one action fewer than the best plan so far, until the space under the bound is exhausted or
 * the budget runs out. With partial order, the search that interleaves then does the same.
 * Exhausted, the plan is the shortest (`Result::optimal`).
 */
Result total_order(
  const tdg::Problem & problem, kairos::Budget * budget = nullptr, const Options & options = {});

}  // namespace klimax::search

#endif  // KLIMAX__SEARCH__TOTAL_ORDER_HPP_

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
  Unsupported   // a partial order, numbers, axioms: `message` says which
};

struct Result
{
  Outcome outcome = Outcome::Unsupported;
  std::string plan;     // Solved: in the format of the IPC HTN tracks
  std::string message;
  size_t steps = 0;         // actions of the plan
  uint64_t expansions = 0;  // tasks taken from the stack
};

/**
 * @brief Depth-first progression: the first task of the stack is executed if it is an action,
 * or replaced by the subtasks of one of its methods, tried in the order written; a state and
 * stack already seen are not searched again.
 *
 * Complete on a finite space; finds the first plan, not a short one. Every task network must be
 * totally ordered.
 */
Result total_order(const tdg::Problem & problem, kairos::Budget * budget = nullptr);

}  // namespace klimax::search

#endif  // KLIMAX__SEARCH__TOTAL_ORDER_HPP_

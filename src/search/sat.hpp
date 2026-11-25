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


#ifndef KLIMAX__SEARCH__SAT_HPP_
#define KLIMAX__SEARCH__SAT_HPP_

#include <memory>

#include "core/budget.hpp"
#include "search/total_order.hpp"
#include "tdg/grounder.hpp"

namespace klimax::search
{

/**
 * @brief Planning as satisfiability, by layers of decomposition.
 *
 * Layer 0 is the initial task network. Each layer has a sequence of positions; a position
 * holds one task or nothing, and becomes in the next layer as many positions as the longest
 * method of the tasks it may hold: an action stays in the first one, a compound task puts the
 * subtasks of one of its methods there. The state before each position is part of the formula:
 * actions and method preconditions read it, actions change it, and what no action at a position
 * can change is the same variable before and after. The formula of a layer is satisfiable when
 * every position can hold an action or nothing: that is a plan. Layers are added until then,
 * keeping the clauses and what the solver learnt. Tasks that can be decomposed in the same ways
 * (one whose methods ignore an argument, in all the values of it) are one task in the formula.
 *
 * For domains whose methods leave many parameters free and where only the goal tells which
 * values were right: the solver reasons on all the positions at once. Totally ordered problems
 * without conditional effects only (Unsupported otherwise, with a message). It does not prove
 * that there is no plan.
 */
class SatSearch
{
public:
  /// @param problem Must outlive the search.
  explicit SatSearch(const tdg::Problem & problem);
  ~SatSearch();

  /// @brief Searches until it ends or the budget runs out (OutOfBudget, with no message). After
  /// the budget ran out, another call goes on from where that one stopped.
  Result run(kairos::Budget * budget = nullptr);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

/// @brief One run of SatSearch.
Result sat(const tdg::Problem & problem, kairos::Budget * budget = nullptr);

}  // namespace klimax::search

#endif  // KLIMAX__SEARCH__SAT_HPP_

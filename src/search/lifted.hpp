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


#ifndef KLIMAX__SEARCH__LIFTED_HPP_
#define KLIMAX__SEARCH__LIFTED_HPP_

#include <memory>
#include <string>

#include "core/budget.hpp"
#include "hddl/model.hpp"
#include "search/total_order.hpp"

namespace klimax::search
{

/**
 * @brief Depth-first progression over the problem as written, without grounding it.
 *
 * The first pending task is executed if it is an action, or replaced by the subtasks of one of
 * its methods. The methods are tried in the order of the domain; the parameters that the task
 * leaves free take the values with which the precondition holds in the current state, found by
 * matching the atoms it requires against the state. A task and state already seen, with the
 * same tasks pending after it, is not searched again. A task that comes from itself with the
 * same arguments and in the same state is a recursion that does not progress: it is cut, and
 * if no plan is found the search starts again allowing it once more, up to a few times. The
 * atoms of predicates that no action changes, which the actions of a method will require, are
 * checked when the method is chosen.
 *
 * For problems whose ground form is too large, and for domains written as a recipe, where the
 * first method that applies is the right one. Totally ordered problems only (Unsupported
 * otherwise, with a message). Unsolvable when the whole space was searched. OutOfBudget with a
 * message when it gives up, for its memory or for the recursion it cut: it does not go on then.
 */
class LiftedSearch
{
public:
  /// @param model Must outlive the search.
  explicit LiftedSearch(const hddl::Model & model);
  ~LiftedSearch();

  /// @brief Searches until it ends or the budget runs out (OutOfBudget). After the budget ran
  /// out, another call goes on from where that one stopped.
  Result run(kairos::Budget * budget = nullptr);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
  std::string unsupported_;
};

/// @brief One run of LiftedSearch.
Result lifted(const hddl::Model & model, kairos::Budget * budget = nullptr);

}  // namespace klimax::search

#endif  // KLIMAX__SEARCH__LIFTED_HPP_

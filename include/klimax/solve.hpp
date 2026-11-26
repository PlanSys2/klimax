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
/// \brief Planning.

#ifndef KLIMAX__SOLVE_HPP_
#define KLIMAX__SOLVE_HPP_

#include <cstdint>
#include <functional>
#include <string>
#include <string_view>
#include <vector>

#include "klimax/check.hpp"
#include "klimax/export.hpp"

namespace klimax
{

enum class Status : uint8_t
{
  Solved,
  Unsolvable,    // proved: there is no plan
  Timeout,
  Unsupported,   // the problem uses something Klimax does not plan with yet
  InvalidInput,  // errors in the domain or the problem: see `diagnostics`
  InternalError  // a bug in Klimax: `message` says what
};

struct SolveOptions
{
  double seconds = 0.0;  // time limit for grounding and search together; 0: none
  // Every plan is checked by the verifier, which shares nothing with the search, before it is
  // returned: one that it rejects becomes Status::InternalError, never a plan. Turning it off
  // saves that time (a third of the whole on plans of tens of thousands of actions).
  bool verify = true;
  // Also give the plan as its decomposition (`SolveResult::steps_of_plan` and what follows it).
  // The verifier works it out, so this verifies the plan whatever `verify` says.
  bool decomposition = false;
  // Go on after the first plan, looking for better ones, until none is left or the time runs
  // out. The result is the best found; `SolveResult::optimal` says if it was proved so.
  // Better is cheaper by the :metric of the problem when it adds up action costs (as
  // `(increase (total-cost) 3)`), with fewer actions when there is no metric, and with a better
  // value of any other metric among the plans that are no longer.
  bool anytime = false;
  // Called with every plan found (IPC format), each better than the one before
  std::function<void(const std::string & plan, size_t steps)> on_plan;
};

struct SolveResult
{
  Status status = Status::InvalidInput;
  std::string plan;        // Solved: the actions and their decomposition, in the IPC format
  // With `SolveOptions::decomposition`: every step, the actions in the order of execution and
  // the tasks of the initial task network (ids, with the labels the problem gives them)
  std::vector<PlanStep> steps_of_plan;
  std::vector<std::string> sequence;
  std::vector<std::string> roots;
  std::vector<std::string> root_labels;
  std::string message;     // Unsupported: what is not supported
  std::vector<Diagnostic> diagnostics;
  size_t steps = 0;        // actions of the plan
  size_t plans = 0;        // found, each better than the one before (1 unless anytime)
  // Proved: no plan is cheaper by the metric, if it adds up action costs that are all above
  // zero; no plan has fewer actions, if there is no metric. Never with another metric.
  bool optimal = false;
  // The value of the problem's :metric at the end of the plan (none: no metric, or one over
  // time)
  std::optional<double> cost;
  // With durative actions: when the plan ends. Its actions then have a time and a duration,
  // "id time: (action args) [duration]", one after another
  std::optional<double> makespan;
  double ground_ms = 0.0;
  double search_ms = 0.0;
  uint64_t expansions = 0;
};

/// @brief Plans for an HDDL domain and problem.
KLIMAX_API SolveResult solve(
  std::string_view domain, std::string_view problem, const SolveOptions & options = {},
  std::string_view domain_file = "domain.hddl", std::string_view problem_file = "problem.hddl");

/// @brief "solved", "unsolvable", "timeout", "unsupported", "invalid input" or "internal error".
KLIMAX_API std::string_view to_string(Status status);

}  // namespace klimax

#endif  // KLIMAX__SOLVE_HPP_

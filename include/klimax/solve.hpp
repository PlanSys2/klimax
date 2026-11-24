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
  // Also give the plan as its decomposition (`SolveResult::steps_of_plan` and what follows it).
  // The plan is verified for it, independently of the search.
  bool decomposition = false;
};

/// @brief A step of a plan: an action, or a task with the method that decomposes it.
struct PlanStep
{
  std::string id;                 // as in `SolveResult::plan`
  std::string task;
  std::vector<std::string> args;  // objects
  bool primitive = false;
  // Abstract tasks only
  std::string method;
  std::vector<std::string> params;    // objects, by parameter of the method
  std::vector<std::string> subtasks;  // ids, by subtask of the method as written in the domain
  std::vector<std::string> labels;    // of those subtasks in the domain; "" where not given
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

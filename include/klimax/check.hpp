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
/// \brief Reading and checking HDDL input, and verifying plans.

#ifndef KLIMAX__CHECK_HPP_
#define KLIMAX__CHECK_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "klimax/export.hpp"

namespace klimax
{

enum class Severity : uint8_t {Error, Warning, Note};

/// @brief A message about the input. `code` is stable; `message` is for humans.
struct Diagnostic
{
  Severity severity = Severity::Error;
  std::string code;
  std::string message;
  std::string file;      // empty if unknown
  uint32_t line = 0;     // 1-based; 0 if unknown
  uint32_t column = 0;
  std::string rendered;  // "file:line:column: severity: message [code]" with the source line
};

/// @brief What was read.
struct Summary
{
  size_t types = 0;
  size_t predicates = 0;
  size_t actions = 0;
  size_t tasks = 0;
  size_t methods = 0;
  size_t objects = 0;
  size_t initial_tasks = 0;
  bool total_order = false;  // every task network is totally ordered
  bool has_goal = false;
};

struct CheckResult
{
  bool ok = false;
  std::vector<Diagnostic> diagnostics;
  Summary summary;
};

/// @brief Reads an HDDL domain and problem and reports every error found.
KLIMAX_API CheckResult check(
  std::string_view domain, std::string_view problem,
  std::string_view domain_file = "domain.hddl", std::string_view problem_file = "problem.hddl");

/// @brief Sizes of the ground problem.
struct GroundResult
{
  bool ok = false;
  std::vector<Diagnostic> diagnostics;
  bool unsolvable = false;   // proved: the initial tasks cannot be decomposed into actions
  size_t facts = 0;
  size_t actions = 0;        // ground primitive tasks
  size_t abstract_tasks = 0;
  size_t methods = 0;
  double milliseconds = 0.0;
  std::string missing;       // with a plan: the first step of it that is not in the ground problem
};

/**
 * @brief Grounds a problem and reports its size. `seconds` limits the time (0: no limit).
 *
 * With a `plan` (IPC format), also checks that every action and every decomposition of it is in
 * the ground problem: a valid plan must be, or the grounding prunes too much.
 */
KLIMAX_API GroundResult ground(
  std::string_view domain, std::string_view problem, double seconds = 0.0,
  std::string_view domain_file = "domain.hddl", std::string_view problem_file = "problem.hddl",
  std::string_view plan = {});

/// @brief A step of a plan: an action, or a task with the method that decomposes it.
struct PlanStep
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
};

struct VerifyResult
{
  bool valid = false;
  std::string reason;                   // why the plan is not valid, in one sentence
  std::vector<Diagnostic> diagnostics;  // errors in the domain or the problem, if any
  // Of a valid plan, the value of the problem's :metric at its end (none: no metric, one over
  // time, or one that reads a fluent with no value)
  std::optional<double> metric;
  // With `decomposition`, of a valid plan: every step, the actions in the order of execution
  // and the tasks of the initial task network (ids, with the labels the problem gives them)
  std::vector<PlanStep> steps_of_plan;
  std::vector<std::string> sequence;
  std::vector<std::string> roots;
  std::vector<std::string> root_labels;
};

/**
 * @brief Verifies a hierarchical plan in the format of the IPC HTN tracks: the primitive
 * sequence and its decomposition tree, between "==>" and "<==".
 *
 * Independent of the planner. Sequential plans, with numeric fluents; anything else is reported
 * as unsupported in `reason`, never accepted.
 */
KLIMAX_API VerifyResult verify(
  std::string_view domain, std::string_view problem, std::string_view plan,
  std::string_view domain_file = "domain.hddl", std::string_view problem_file = "problem.hddl",
  bool decomposition = false);

}  // namespace klimax

#endif  // KLIMAX__CHECK_HPP_

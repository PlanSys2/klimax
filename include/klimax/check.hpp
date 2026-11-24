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
/// \brief Reading and checking HDDL input.

#ifndef KLIMAX__CHECK_HPP_
#define KLIMAX__CHECK_HPP_

#include <cstdint>
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

}  // namespace klimax

#endif  // KLIMAX__CHECK_HPP_

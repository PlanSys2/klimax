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
  InvalidInput   // errors in the domain or the problem: see `diagnostics`
};

struct SolveOptions
{
  double seconds = 0.0;  // time limit for grounding and search together; 0: none
};

struct SolveResult
{
  Status status = Status::InvalidInput;
  std::string plan;        // Solved: the actions and their decomposition, in the IPC format
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

/// @brief "solved", "unsolvable", "timeout", "unsupported" or "invalid input".
KLIMAX_API std::string_view to_string(Status status);

}  // namespace klimax

#endif  // KLIMAX__SOLVE_HPP_

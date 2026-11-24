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
/// \brief HDDL parser: Kairos' PDDL parser extended with the hierarchical sections.

#ifndef KLIMAX__HDDL__PARSER_HPP_
#define KLIMAX__HDDL__PARSER_HPP_

#include <string_view>

#include "hddl/ast.hpp"
#include "kairos/result.hpp"

namespace klimax::hddl
{

/**
 * @brief Parses an HDDL domain.
 *
 * Everything that is PDDL is parsed by Kairos, with its diagnostics and error recovery. Added
 * here: `:task`, `:method` and the spellings of the competition domains (`:subtasks`, `:tasks`,
 * `:ordered-subtasks`, `:ordered-tasks`, `:ordering`, `:order`). Names are checked later.
 */
kairos::Result<Domain> parse_domain(
  std::string_view text, std::string_view filename = "domain.hddl");

/// @brief Parses an HDDL problem: a PDDL problem with an optional `:htn` section.
kairos::Result<Problem> parse_problem(
  std::string_view text, std::string_view filename = "problem.hddl");

}  // namespace klimax::hddl

#endif  // KLIMAX__HDDL__PARSER_HPP_

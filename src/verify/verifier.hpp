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
/// \brief Verifier of hierarchical plans in the format of the IPC HTN tracks.

#ifndef KLIMAX__VERIFY__VERIFIER_HPP_
#define KLIMAX__VERIFY__VERIFIER_HPP_

#include <string>
#include <string_view>

#include "hddl/model.hpp"

namespace klimax::verifier
{

struct Outcome
{
  bool valid = false;
  std::string reason;  // why not, in one sentence; empty if valid
};

/**
 * @brief Checks that `plan` solves the problem of `model` (docs/semantics.md, section 3.1).
 *
 * The plan is the primitive sequence and its decomposition tree. Checked: every step is an
 * action with objects of the right types; the tree starts at the initial task network and each
 * node applies a method of its task whose subtasks are its children; the sequence respects every
 * ordering constraint; it is executable from the initial state; every method precondition holds
 * at a point allowed by the orderings; variable constraints hold; and the goal, if any, holds at
 * the end.
 *
 * Independent of the planner: it interprets the syntax tree directly. Sequential plans without
 * numbers (level L1); anything else is reported as unsupported, never accepted.
 */
Outcome verify(const hddl::Model & model, std::string_view plan);

}  // namespace klimax::verifier

#endif  // KLIMAX__VERIFY__VERIFIER_HPP_

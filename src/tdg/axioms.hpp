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

#ifndef KLIMAX__TDG__AXIOMS_HPP_
#define KLIMAX__TDG__AXIOMS_HPP_

#include <cstddef>
#include <string>

#include "ground/task.hpp"

namespace klimax::tdg
{

/**
 * @brief Derived atoms replaced by their definitions where they are read, so that searches that
 * know nothing of axioms plan with them: an action that reads one becomes a copy per way of
 * meeting its definition (as Kairos does with a disjunction), one that reads its negation the
 * conjunction of what each of its rules does not have. Kairos makes them of quantified
 * conditions: (not (exists (?e) (and (hostile ?e) (at ?e ?z)))), with hostile static, is
 * "at" false of each hostile. Goals and the conditions of durative actions too.
 *
 * @return Empty if every axiom went; else why not (axioms that read each other in a cycle, more
 * than `max_copies` copies of an action, derived atoms in conditional effects or a goal of
 * several cases), and the task is left as it was.
 */
std::string inline_axioms(kairos::ground::Task & task, size_t max_copies = 64);

}  // namespace klimax::tdg

#endif  // KLIMAX__TDG__AXIOMS_HPP_

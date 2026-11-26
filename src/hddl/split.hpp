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
/// \brief Parameter splitting: keeps independent choices of a method from multiplying.

#ifndef KLIMAX__HDDL__SPLIT_HPP_
#define KLIMAX__HDDL__SPLIT_HPP_

#include <cstddef>

#include "hddl/model.hpp"

namespace klimax::hddl
{

/**
 * @brief Moves the parameters of a method that only one subtask uses into a method of their own.
 *
 * A method with subtasks (a ?x) and (b ?y), where nothing else mentions ?x or ?y, has one
 * instance per pair of values, although the two choices are independent. Here (a ?x) becomes a
 * new task with no such parameter, decomposed by a new method that chooses ?x; the same for
 * (b ?y). The instances are then the sum, not the product. The new tasks and methods are marked
 * `artificial`: a plan leaves them out, hanging their subtask from the original method.
 *
 * @return how many subtasks were moved.
 */
size_t split_parameters(Model & model);

/**
 * @brief What a method asks to hold just before a subtask, (before id gd) and the end of
 * (between id1 id2 gd), as something the searches test: the subtask becomes a new task whose
 * only method has that condition as its precondition and the subtask as its only one.
 *
 * A precondition is tested at some point before the subtask starts, not just before: where
 * the order of the tasks leaves room for something in between, the verifier still decides.
 * The new tasks and methods are `artificial`. Call it before split_parameters().
 *
 * @return how many subtasks were wrapped.
 */
size_t test_before(Model & model);

}  // namespace klimax::hddl

#endif  // KLIMAX__HDDL__SPLIT_HPP_

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


#ifndef KLIMAX__TDG__OBJECT_FLUENTS_HPP_
#define KLIMAX__TDG__OBJECT_FLUENTS_HPP_

#include "ground/task.hpp"
#include "hddl/model.hpp"

namespace klimax::tdg
{

/**
 * @brief Rewrites the object fluents of a ground task as atoms: one per fluent and object it
 * may hold, true when it holds it (none true: no value). A comparison with an object is that
 * atom; one between two fluents, an instance of the action per object they may share; an
 * assignment deletes the atoms of the other objects and adds that of the new one.
 *
 * After it the task has no object fluents, and whatever plans with atoms plans with them.
 * False, and the task as it was, if something cannot be written so: two fluents that must
 * differ, or two that must be equal in the goal.
 */
bool object_fluents_as_atoms(kairos::ground::Task & task, const hddl::Model & model);

}  // namespace klimax::tdg

#endif  // KLIMAX__TDG__OBJECT_FLUENTS_HPP_

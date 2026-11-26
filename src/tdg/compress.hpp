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


#ifndef KLIMAX__TDG__COMPRESS_HPP_
#define KLIMAX__TDG__COMPRESS_HPP_

#include <string>

#include "normalize/task.hpp"

namespace klimax::tdg
{

/**
 * @brief Every durative action as an instantaneous one with its name, as Kairos' engine for
 * temporally simple tasks does on the ground task (temporal/compress.hpp), here before
 * grounding, where the hierarchy is grounded:
 * - precondition: the conditions at start, and those over all and at end that its own effects
 *   at start do not give;
 * - effects: those at start and then those at end; an atom added at start and deleted at end
 *   is deleted.
 *
 * A plan of the result, with each such action lasting what its duration says and none
 * overlapping another, is a plan of the task when no action needs what another gives only
 * while it runs. Kairos' validator says so of each plan.
 *
 * The reason, and the task as it was, for what cannot be written so: ?duration in a condition
 * or an effect, continuous effects, a fluent that an action changes at start and at end or
 * reads later than it changes, or a conditional effect at end.
 */
std::string compress(kairos::lifted::Task & task);

}  // namespace klimax::tdg

#endif  // KLIMAX__TDG__COMPRESS_HPP_

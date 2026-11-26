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


#ifndef KLIMAX__TDG__TIMED_HPP_
#define KLIMAX__TDG__TIMED_HPP_

#include <string>

#include "ground/task.hpp"

namespace klimax::tdg
{

/**
 * @brief Timed initial literals as conditions on the clock (tdg/compress.hpp): an atom that
 * only they change holds in some intervals of time, and an action that needs it becomes one
 * instance per interval it fits in: either the clock is already inside (and the action ends
 * before the interval does), or it is before and the action starts when the interval opens.
 * The search then waits by choosing the second.
 *
 * An action that reads such an atom needs it from its start to its end, also if the domain
 * asks for it only at start or at end: plans are lost, none is wrong.
 *
 * After it the task has no timed literals. The reason, and the task as it was, for what
 * cannot be written so: timed values of fluents, an atom that actions change too, or one read
 * by the goal or by the condition of an effect.
 */
std::string timed_literals(kairos::ground::Task & task);

}  // namespace klimax::tdg

#endif  // KLIMAX__TDG__TIMED_HPP_

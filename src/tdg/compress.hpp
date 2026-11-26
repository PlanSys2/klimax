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

#include <functional>
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

/**
 * @brief Every durative action as two instantaneous ones, its start (with its name) and its
 * end (hddl::end_of() its name), as Kairos' engine for tasks that need concurrency does on the
 * ground task (temporal/snap.hpp), here before grounding:
 * - start: the conditions at start, those over all that its effects at start do not give, and
 *   that it is not running; its effects at start, and it runs;
 * - end: that it runs, the conditions at end and those over all; its effects at end, and it
 *   no longer runs.
 *
 * Between the two anything may happen: that the conditions over all hold meanwhile, and that
 * the times fit, is for whoever times the plan to say (Kairos does: schedule.hpp).
 *
 * The reason, and the task as it was, for what cannot be written so: timed initial literals,
 * ?duration or continuous change in conditions and effects, or a condition at end over a
 * variable that is not a parameter.
 */
std::string start_and_end(kairos::lifted::Task & task);

/// The numeric fluent that says when the next step of a plan may start (see clock()).
inline constexpr const char * kClock = "klimax-clock";
/// Between a step and the next, as PDDL 2.1 asks of what happens at different times.
inline constexpr double kEpsilon = 0.001;

/**
 * @brief For a task with timed initial literals, before compress(): a clock that every step
 * of a plan moves on by what it lasts (and kEpsilon), so that with one action after another
 * its value is when the next may start. `step` says whether an action is a step of the plan
 * (not the test of a method's precondition).
 *
 * The reason, and the task as it was, if a duration cannot be read.
 */
std::string clock(
  kairos::lifted::Task & task, const std::function<bool(const std::string &)> & step);

}  // namespace klimax::tdg

#endif  // KLIMAX__TDG__COMPRESS_HPP_

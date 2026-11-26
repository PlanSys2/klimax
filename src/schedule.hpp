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


#ifndef KLIMAX__SCHEDULE_HPP_
#define KLIMAX__SCHEDULE_HPP_

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "hddl/model.hpp"
#include "verify/verifier.hpp"

namespace klimax
{

/**
 * @brief When each action of a plan found with one durative action after another can start, as
 * early as allowed by what the actions read and write and by `precedences` (pairs of positions
 * in the plan: the second starts when the first has ended, or later; the orderings of the
 * hierarchy).
 *
 * Kairos does it (temporal::schedule): the actions of the plan are grounded and compressed as
 * Kairos plans with them, and the plan replayed there. Empty if Kairos cannot replay or
 * schedule it; the plan is then left with one action after another.
 */
std::vector<verifier::Timed> schedule(
  const hddl::Model & model, const std::vector<verifier::Order::Action> & steps,
  const std::vector<std::pair<uint32_t, uint32_t>> & precedences);

/// @brief Something that takes place in a plan found with the start and the end of each
/// durative action as two steps: the start of a step of the plan (or the whole of it, if it
/// has no duration), or its end.
struct Happening
{
  uint32_t step = 0;  // of the plan
  bool end = false;
};

/**
 * @brief The same for a plan found with the starts and the ends of its durative actions in the
 * order `happenings`: Kairos gives them times that keep that order, with every action lasting
 * what it must and what must hold while it runs holding (temporal::TemporalFilter). Empty if
 * no times do.
 */
std::vector<verifier::Timed> schedule(
  const hddl::Model & model, const std::vector<verifier::Order::Action> & steps,
  const std::vector<Happening> & happenings);

}  // namespace klimax

#endif  // KLIMAX__SCHEDULE_HPP_

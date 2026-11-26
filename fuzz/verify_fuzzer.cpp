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

#include <cstddef>
#include <cstdint>

#include "klimax/check.hpp"

#include "parts.hpp"

// A domain, a problem and a plan: verified without a crash; a valid plan has no reason against
// it, an invalid one has one, and its decomposition is that of its steps
extern "C" int LLVMFuzzerTestOneInput(const uint8_t * data, size_t size)
{
  const auto in = klimax::fuzz::parts(data, size, 3);
  const auto result = klimax::verify(in[0], in[1], in[2], "domain.hddl", "problem.hddl", true);
  if (result.valid != result.reason.empty()) {
    klimax::fuzz::fail("verify", result.valid ? "valid with a reason" : "invalid without one");
  }
  if (!result.valid && !result.steps_of_plan.empty()) {
    klimax::fuzz::fail("verify", "steps of an invalid plan");
  }
  for (const auto & step : result.steps_of_plan) {
    for (const auto & [first, second] : step.order) {
      if (first >= step.subtasks.size() || second >= step.subtasks.size()) {
        klimax::fuzz::fail("verify", "an order outside the subtasks");
      }
    }
  }
  return 0;
}

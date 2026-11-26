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
#include "klimax/solve.hpp"

#include "parts.hpp"

// A domain and a problem, planned for briefly. Every plan goes through the verifier, which
// shares nothing with the search: one it rejects is an internal error, as any other bug. Input
// errors are the same that check() finds, and a plan found verifies again on its own
extern "C" int LLVMFuzzerTestOneInput(const uint8_t * data, size_t size)
{
  const auto in = klimax::fuzz::parts(data, size, 2);
  klimax::SolveOptions options;
  options.seconds = 0.2;
  const auto result = klimax::solve(in[0], in[1], options);
  if (result.status == klimax::Status::InternalError) {
    klimax::fuzz::fail("solve", result.message.c_str());
  }
  const bool invalid = result.status == klimax::Status::InvalidInput;
  if (invalid == klimax::check(in[0], in[1]).ok) {
    klimax::fuzz::fail("solve", invalid ? "invalid input that check() reads" :
      "planned for input that check() rejects");
  }
  if (result.status == klimax::Status::Solved && !klimax::verify(in[0], in[1], result.plan).valid) {
    klimax::fuzz::fail("solve", "a plan that does not verify");
  }
  return 0;
}

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

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "klimax/check.hpp"

#include "parts.hpp"

namespace
{

bool has_error(const std::vector<klimax::Diagnostic> & diagnostics)
{
  return std::any_of(diagnostics.begin(), diagnostics.end(),
           [](const klimax::Diagnostic & d) {return d.severity == klimax::Severity::Error;});
}

}  // namespace

// A domain and a problem: read and analyzed, without a crash; not ok exactly when there is an
// error. Then grounded, briefly
extern "C" int LLVMFuzzerTestOneInput(const uint8_t * data, size_t size)
{
  const auto in = klimax::fuzz::parts(data, size, 2);
  const auto checked = klimax::check(in[0], in[1]);
  if (checked.ok == has_error(checked.diagnostics)) {
    klimax::fuzz::fail("check", checked.ok ? "ok with an error" : "not ok without an error");
  }
  if (checked.ok) {
    // What reads, grounds or says why not (a timeout is neither: not ok, without an error)
    const auto grounded = klimax::ground(in[0], in[1], 0.2);
    if (grounded.ok && has_error(grounded.diagnostics)) {
      klimax::fuzz::fail("ground", "ok with an error");
    }
  }
  return 0;
}

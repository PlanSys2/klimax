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
#include <string_view>

#include "hddl/parser.hpp"
#include "kairos/diagnostics.hpp"

#include "parts.hpp"

// Any text, as a domain and as a problem: no crash, and a failure says why
extern "C" int LLVMFuzzerTestOneInput(const uint8_t * data, size_t size)
{
  const std::string_view text(reinterpret_cast<const char *>(data), size);
  const auto domain = klimax::hddl::parse_domain(text);
  if (!domain.ok() && !kairos::has_errors(domain.diagnostics())) {
    klimax::fuzz::fail("hddl parser", "domain fails without an error");
  }
  const auto problem = klimax::hddl::parse_problem(text);
  if (!problem.ok() && !kairos::has_errors(problem.diagnostics())) {
    klimax::fuzz::fail("hddl parser", "problem fails without an error");
  }
  return 0;
}

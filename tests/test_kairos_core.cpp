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

#include <iostream>
#include <string_view>

#include "kairos/version.hpp"
#include "klimax/klimax.hpp"
#include "pddl/parser.hpp"

namespace
{

int failures = 0;

void check(bool condition, std::string_view what)
{
  if (!condition) {
    std::cerr << "FAILED: " << what << '\n';
    ++failures;
  }
}

}  // namespace

int main()
{
  check(klimax::version() == KLIMAX_VERSION_STRING, "library and header versions agree");
  check(klimax::kairos_version() == kairos::version(), "same Kairos inside and outside");

  // Kairos' parser, an internal module
  const auto good = kairos::pddl::parse_domain(
    "(define (domain d) (:predicates (p)) (:action a :effect (p)))");
  check(good.ok(), "a valid domain parses");
  const auto bad = kairos::pddl::parse_domain("(define (domain d) (:predicates (p))");
  check(!bad.ok(), "an unbalanced domain is rejected");
  const auto empty = kairos::pddl::parse_domain("");
  check(!empty.ok(), "an empty text is rejected");

  if (failures == 0) {
    std::cout << "ok\n";
  }
  return failures == 0 ? 0 : 1;
}

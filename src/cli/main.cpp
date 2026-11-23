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

#include "klimax/klimax.hpp"

namespace
{

void print_usage(std::ostream & out)
{
  out << "Usage: klimax [--version] [--help]\n"
      << "Klimax, an HTN planner for HDDL 2.1. Planning is not implemented yet.\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  for (int i = 1; i < argc; ++i) {
    const std::string_view arg = argv[i];
    if (arg == "--version") {
      std::cout << "klimax " << klimax::version()
                << " (kairos " << klimax::kairos_version() << ")\n";
      return 0;
    }
    if (arg == "--help" || arg == "-h") {
      print_usage(std::cout);
      return 0;
    }
    std::cerr << "klimax: unknown argument '" << arg << "'\n";
    print_usage(std::cerr);
    return 2;
  }
  print_usage(std::cerr);
  return 2;
}

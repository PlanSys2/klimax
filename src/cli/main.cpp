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

#include <fstream>
#include <iostream>
#include <iterator>
#include <optional>
#include <string>
#include <string_view>

#include "klimax/klimax.hpp"

namespace
{

void print_usage(std::ostream & out)
{
  out << "Usage: klimax --check DOMAIN PROBLEM\n"
      << "       klimax --version | --help\n"
      << "Klimax, an HTN planner for HDDL 2.1. Planning is not implemented yet.\n"
      << "  --check   read the domain and the problem and report every error\n";
}

std::optional<std::string> read_file(const char * path)
{
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    std::cerr << "klimax: cannot read '" << path << "'\n";
    return std::nullopt;
  }
  return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

// Exit codes: 0 valid input, 1 errors in the input, 2 bad usage
int check(const char * domain_path, const char * problem_path)
{
  auto domain = read_file(domain_path);
  auto problem = read_file(problem_path);
  if (!domain || !problem) {
    return 2;
  }
  klimax::CheckResult r = klimax::check(*domain, *problem, domain_path, problem_path);
  for (const klimax::Diagnostic & d : r.diagnostics) {
    std::cerr << d.rendered << "\n";
  }
  if (!r.ok) {
    return 1;
  }
  const klimax::Summary & s = r.summary;
  std::cout << "types " << s.types << ", predicates " << s.predicates << ", actions "
            << s.actions << ", tasks " << s.tasks << ", methods " << s.methods << ", objects "
            << s.objects << ", initial tasks " << s.initial_tasks << "\n";
  return 0;
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
    if (arg == "--check") {
      if (i + 2 >= argc || i + 3 != argc) {
        std::cerr << "klimax: --check takes a domain and a problem\n";
        return 2;
      }
      return check(argv[i + 1], argv[i + 2]);
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

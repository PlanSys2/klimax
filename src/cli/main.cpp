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
  out << "Usage: klimax DOMAIN PROBLEM [--timeout SECONDS] [--anytime] [--no-verify]\n"
      << "       klimax --check DOMAIN PROBLEM\n"
      << "       klimax --verify DOMAIN PROBLEM PLAN\n"
      << "       klimax --ground DOMAIN PROBLEM [SECONDS]\n"
      << "       klimax --version | --help\n"
      << "Klimax, an HTN planner for HDDL 2.1. It plans HDDL 1.0 with numeric fluents for now;\n"
      << "the plan and its decomposition are written in the format of the IPC HTN tracks.\n"
      << "  --anytime go on after the first plan, looking for better ones (cheaper by the\n"
      << "            metric of the problem, shorter if it has none), until none is left or\n"
      << "            the time runs out; the best found is written\n"
      << "  --no-verify  do not check the plan with the verifier before writing it\n"
      << "  --check   read the domain and the problem and report every error\n"
      << "  --verify  check a plan in the format of the IPC HTN tracks\n"
      << "  --ground  ground the problem and print its size\n";
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
            << s.objects << ", initial tasks " << s.initial_tasks << ", "
            << (s.total_order ? "total order" : "partial order") << "\n";
  return 0;
}

// Exit codes: 0 grounded, 1 errors in the input or out of time, 2 bad usage
int ground(
  const char * domain_path, const char * problem_path, double seconds, const char * plan_path)
{
  auto domain = read_file(domain_path);
  auto problem = read_file(problem_path);
  std::optional<std::string> plan = plan_path != nullptr ? read_file(plan_path) : std::string();
  if (!domain || !problem || !plan) {
    return 2;
  }
  klimax::GroundResult r = klimax::ground(
    *domain, *problem, seconds, domain_path, problem_path, *plan);
  for (const klimax::Diagnostic & d : r.diagnostics) {
    if (d.severity == klimax::Severity::Error) {
      std::cerr << d.rendered << "\n";
    }
  }
  if (!r.ok) {
    return 1;
  }
  std::cout << "facts " << r.facts << ", actions " << r.actions << ", abstract tasks "
            << r.abstract_tasks << ", methods " << r.methods << ", "
            << (r.unsolvable ? "unsolvable, " : "") << r.milliseconds << " ms\n";
  if (!r.missing.empty()) {
    std::cout << "not in the ground problem: " << r.missing << "\n";
    return 1;
  }
  return 0;
}

// Exit codes: 0 solved, 1 errors in the input, 2 bad usage, 10 unsolvable, 11 out of time,
// 12 not supported
int solve(
  const char * domain_path, const char * problem_path, double seconds, bool anytime, bool verify)
{
  auto domain = read_file(domain_path);
  auto problem = read_file(problem_path);
  if (!domain || !problem) {
    return 2;
  }
  klimax::SolveOptions options;
  options.seconds = seconds;
  options.anytime = anytime;
  options.verify = verify;
  if (anytime) {
    options.on_plan = [](const std::string &, size_t steps) {
        std::cerr << "; plan of " << steps << " actions\n";
      };
  }
  klimax::SolveResult r = klimax::solve(*domain, *problem, options, domain_path, problem_path);
  for (const klimax::Diagnostic & d : r.diagnostics) {
    if (d.severity == klimax::Severity::Error) {
      std::cerr << d.rendered << "\n";
    }
  }
  std::cerr << "; " << klimax::to_string(r.status) << (r.message.empty() ? "" : ": ") << r.message
            << "; ground " << r.ground_ms << " ms, search " << r.search_ms << " ms, "
            << r.expansions << " expansions";
  if (r.cost) {
    std::cerr << "; metric " << *r.cost;
  }
  std::cerr << (r.optimal ? "; no better plan exists" : "") << "\n";
  switch (r.status) {
    case klimax::Status::Solved:
      std::cout << r.plan;
      return 0;
    case klimax::Status::Unsolvable: return 10;
    case klimax::Status::Timeout: return 11;
    case klimax::Status::Unsupported: return 12;
    default: return 1;
  }
}

// Exit codes: 0 valid plan, 1 invalid plan or errors in the input, 2 bad usage
int verify(const char * domain_path, const char * problem_path, const char * plan_path)
{
  auto domain = read_file(domain_path);
  auto problem = read_file(problem_path);
  auto plan = read_file(plan_path);
  if (!domain || !problem || !plan) {
    return 2;
  }
  klimax::VerifyResult r = klimax::verify(*domain, *problem, *plan, domain_path, problem_path);
  for (const klimax::Diagnostic & d : r.diagnostics) {
    if (d.severity == klimax::Severity::Error) {
      std::cerr << d.rendered << "\n";
    }
  }
  if (!r.valid) {
    std::cout << "Plan verification result: false\n" << r.reason << "\n";
    return 1;
  }
  std::cout << "Plan verification result: true\n";
  if (r.metric) {
    std::cout << "Plan metric: " << *r.metric << "\n";
  }
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  // klimax DOMAIN PROBLEM [--timeout SECONDS] [--anytime]
  if (argc >= 3 && argv[1][0] != '-') {
    double seconds = 0.0;
    bool anytime = false;
    bool verify = true;
    for (int i = 3; i < argc; ++i) {
      const std::string_view arg = argv[i];
      if (arg == "--timeout" && i + 1 < argc) {
        seconds = std::stod(argv[++i]);
      } else if (arg == "--anytime") {
        anytime = true;
      } else if (arg == "--no-verify") {
        verify = false;
      } else {
        print_usage(std::cerr);
        return 2;
      }
    }
    return solve(argv[1], argv[2], seconds, anytime, verify);
  }
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
    if (arg == "--ground") {
      // A plan after the time limit: checks that the ground problem contains it (for testing)
      if (i + 3 > argc || i + 5 < argc) {
        std::cerr << "klimax: --ground takes a domain, a problem and optionally a time limit\n";
        return 2;
      }
      return ground(argv[i + 1], argv[i + 2], i + 4 <= argc ? std::stod(argv[i + 3]) : 0.0,
               i + 5 == argc ? argv[i + 4] : nullptr);
    }
    if (arg == "--verify") {
      if (i + 4 != argc) {
        std::cerr << "klimax: --verify takes a domain, a problem and a plan\n";
        return 2;
      }
      return verify(argv[i + 1], argv[i + 2], argv[i + 3]);
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

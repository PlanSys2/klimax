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

#include <cstdio>
#include <fstream>
#include <iostream>
#include <iterator>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include <unistd.h>

#include "klimax/klimax.hpp"

namespace
{

void print_usage(std::ostream & out)
{
  out << "Usage: klimax DOMAIN PROBLEM [--timeout SECONDS] [--anytime] [--no-verify] [--tree]\n"
      << "       klimax --tree DOMAIN PROBLEM PLAN\n"
      << "       klimax --check DOMAIN PROBLEM\n"
      << "       klimax --verify DOMAIN PROBLEM PLAN\n"
      << "       klimax --ground DOMAIN PROBLEM [SECONDS]\n"
      << "       klimax --version | --help\n"
      << "Klimax, an HTN planner for HDDL 2.1. It plans HDDL 1.0 with numeric fluents and,\n"
      << "of time, durative actions one after another; the plan and its decomposition are\n"
      << "written in the format of the IPC HTN tracks, with times if actions are durative.\n"
      << "  --anytime go on after the first plan, looking for better ones (cheaper by the\n"
      << "            metric of the problem, shorter if it has none), until none is left or\n"
      << "            the time runs out; the best found is written\n"
      << "  --no-verify  do not check the plan with the verifier before writing it\n"
      << "  --tree    write the plan as a tree, each task with its method and what it became\n"
      << "            below it, and the actions numbered in the order of execution; with a\n"
      << "            PLAN, that plan (which is verified first)\n"
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

// A plan as a tree: every task with the method that decomposes it and its subtasks below, in
// the order the method writes them; the actions with their place in the plan and, if the plan
// has times, when they start and end. In colour on a terminal (not with NO_COLOR set).
void print_tree(
  const std::vector<klimax::PlanStep> & steps, const std::vector<std::string> & sequence,
  const std::vector<std::string> & roots, const std::string & plan)
{
  const bool colour = isatty(STDOUT_FILENO) != 0 && std::getenv("NO_COLOR") == nullptr;
  auto paint = [&](const char * code, const std::string & text) {
      return colour ? std::string("\x1b[") + code + "m" + text + "\x1b[0m" : text;
    };
  std::unordered_map<std::string, const klimax::PlanStep *> by_id;
  for (const klimax::PlanStep & s : steps) {
    by_id.emplace(s.id, &s);
  }
  std::unordered_map<std::string, size_t> place;
  for (size_t i = 0; i < sequence.size(); ++i) {
    place.emplace(sequence[i], i + 1);
  }
  // The times, from the lines "id time: (action args) [duration]" of the plan
  std::unordered_map<std::string, std::pair<double, double>> times;
  for (size_t at = 0; at < plan.size(); ) {
    const size_t end = std::min(plan.find('\n', at), plan.size());
    const std::string line = plan.substr(at, end - at);
    at = end + 1;
    char id[64];
    double time = 0.0;
    const size_t open = line.rfind('[');
    if (open != std::string::npos && std::sscanf(line.c_str(), "%63s %lf:", id, &time) == 2) {
      times[id] = {time, std::strtod(line.c_str() + open + 1, nullptr)};
    }
  }
  const size_t width = std::to_string(sequence.size()).size();
  auto label = [&](const klimax::PlanStep & s) {
      std::string name = s.task;
      for (const std::string & a : s.args) {
        name += " " + a;
      }
      if (!s.primitive) {
        // Nothing below: the method had nothing to do
        return paint("1", name) + paint("2", "  ← " + s.method +
                 (s.subtasks.empty() ? "  (nothing to do)" : ""));
      }
      std::string number = std::to_string(place[s.id]);
      number.insert(0, width - number.size(), ' ');
      std::string out = paint("36", number) + "  " + paint("32", name);
      if (const auto t = times.find(s.id); t != times.end()) {
        char when[96];
        std::snprintf(when, sizeof(when), "  [%.3f → %.3f]", t->second.first,
          t->second.first + t->second.second);
        out += paint("2", when);
      }
      return out;
    };
  struct Frame
  {
    const klimax::PlanStep * step;
    std::string prefix;
    bool last;
    bool root;
  };
  std::vector<Frame> stack;
  for (size_t r = roots.size(); r-- > 0; ) {
    if (const auto found = by_id.find(roots[r]); found != by_id.end()) {
      stack.push_back(Frame{found->second, "", true, true});
    }
  }
  bool first = true;
  while (!stack.empty()) {
    const Frame f = stack.back();
    stack.pop_back();
    if (f.root && !first) {
      std::cout << "\n";
    }
    first = false;
    std::cout << f.prefix << (f.root ? "" : f.last ? "└── " : "├── ") << label(*f.step) << "\n";
    const std::string below = f.prefix + (f.root ? "" : f.last ? "    " : "│   ");
    for (size_t c = f.step->subtasks.size(); c-- > 0; ) {
      if (const auto found = by_id.find(f.step->subtasks[c]); found != by_id.end()) {
        stack.push_back(Frame{found->second, below, c + 1 == f.step->subtasks.size(), false});
      }
    }
  }
}

// Exit codes: 0 solved, 1 errors in the input, 2 bad usage, 10 unsolvable, 11 out of time,
// 12 not supported
int solve(
  const char * domain_path, const char * problem_path, double seconds, bool anytime, bool verify,
  bool tree)
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
  options.decomposition = tree;
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
  if (r.makespan) {
    std::cerr << "; makespan " << *r.makespan;
  }
  std::cerr << (r.optimal ? "; no better plan exists" : "") << "\n";
  switch (r.status) {
    case klimax::Status::Solved:
      if (tree) {
        print_tree(r.steps_of_plan, r.sequence, r.roots, r.plan);
      } else {
        std::cout << r.plan;
      }
      return 0;
    case klimax::Status::Unsolvable: return 10;
    case klimax::Status::Timeout: return 11;
    case klimax::Status::Unsupported: return 12;
    default: return 1;
  }
}

// Exit codes: 0 valid plan, 1 invalid plan or errors in the input, 2 bad usage
int verify(
  const char * domain_path, const char * problem_path, const char * plan_path, bool tree = false)
{
  auto domain = read_file(domain_path);
  auto problem = read_file(problem_path);
  auto plan = read_file(plan_path);
  if (!domain || !problem || !plan) {
    return 2;
  }
  klimax::VerifyResult r = klimax::verify(
    *domain, *problem, *plan, domain_path, problem_path, tree);
  for (const klimax::Diagnostic & d : r.diagnostics) {
    if (d.severity == klimax::Severity::Error) {
      std::cerr << d.rendered << "\n";
    }
  }
  if (!r.valid) {
    std::cout << "Plan verification result: false\n" << r.reason << "\n";
    return 1;
  }
  if (tree) {
    print_tree(r.steps_of_plan, r.sequence, r.roots, *plan);
    return 0;
  }
  std::cout << "Plan verification result: true\n";
  if (r.metric) {
    std::cout << "Plan metric: " << *r.metric << "\n";
  }
  if (r.makespan) {
    std::cout << "Plan makespan: " << *r.makespan << "\n";
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
    bool tree = false;
    for (int i = 3; i < argc; ++i) {
      const std::string_view arg = argv[i];
      if (arg == "--timeout" && i + 1 < argc) {
        seconds = std::stod(argv[++i]);
      } else if (arg == "--anytime") {
        anytime = true;
      } else if (arg == "--no-verify") {
        verify = false;
      } else if (arg == "--tree") {
        tree = true;
      } else {
        print_usage(std::cerr);
        return 2;
      }
    }
    return solve(argv[1], argv[2], seconds, anytime, verify, tree);
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
    if (arg == "--tree") {
      if (i + 4 != argc) {
        std::cerr << "klimax: --tree takes a domain, a problem and a plan\n";
        return 2;
      }
      return verify(argv[i + 1], argv[i + 2], argv[i + 3], true);
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

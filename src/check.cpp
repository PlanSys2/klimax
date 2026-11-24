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

#include "klimax/check.hpp"
#include "klimax/solve.hpp"

#include <algorithm>
#include <chrono>
#include <map>
#include <new>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "hddl/model.hpp"
#include "hddl/parser.hpp"
#include "kairos/diagnostics.hpp"
#include "pddl/lexer.hpp"
#include "search/total_order.hpp"
#include "tdg/grounder.hpp"
#include "verify/verifier.hpp"

namespace klimax
{

namespace
{

void append(
  std::vector<Diagnostic> & out, const std::vector<kairos::Diagnostic> & in,
  std::string_view source)
{
  for (const kairos::Diagnostic & d : in) {
    Diagnostic r;
    r.severity = d.severity == kairos::Severity::Error ? Severity::Error :
      d.severity == kairos::Severity::Warning ? Severity::Warning : Severity::Note;
    r.code = d.code;
    r.message = d.message;
    if (d.location) {
      r.file = d.location->file;
      r.line = d.location->line;
      r.column = d.location->column;
    }
    r.rendered = kairos::to_string(d, source);
    out.push_back(std::move(r));
  }
}

// Names as first written in the domain or the problem. HDDL does not distinguish case and
// Klimax works in lower case, but the plan is for tools that may: the verifier of the IPC, in
// its strict mode, wants the spelling of the input.
using Spelling = std::unordered_map<std::string, std::string>;

Spelling spelling_of(std::string_view domain, std::string_view problem)
{
  Spelling spelling;
  for (std::string_view text : {domain, problem}) {
    for (const kairos::pddl::Token & token : kairos::pddl::tokenize(text)) {
      if (token.kind == kairos::pddl::TokenKind::Name) {
        std::string original(token.text);
        std::string lower = kairos::SymbolTable::to_lower(original);
        if (lower != original) {
          spelling.emplace(std::move(lower), std::move(original));
        }
      }
    }
  }
  return spelling;
}

std::string with_original_names(const std::string & plan, const Spelling & spelling)
{
  if (spelling.empty()) {
    return plan;
  }
  std::string out;
  out.reserve(plan.size());
  size_t at = 0;
  while (at < plan.size()) {
    size_t end = at;
    while (end < plan.size() && plan[end] != ' ' && plan[end] != '\n') {
      ++end;
    }
    auto found = spelling.find(plan.substr(at, end - at));
    out += found == spelling.end() ? plan.substr(at, end - at) : found->second;
    if (end < plan.size()) {
      out += plan[end];
    }
    at = end + 1;
  }
  return out;
}

// The first line of `plan` whose action or decomposition is not in the ground problem
std::string first_missing(
  const hddl::Model & model, const tdg::Problem & ground, std::string_view plan)
{
  std::map<std::string, uint32_t> tasks;  // "name arg1 arg2" of each ground task
  for (uint32_t t = 0; t < ground.tasks.size(); ++t) {
    std::string key = model.tasks[ground.tasks[t].task].name;
    for (uint32_t o : ground.tasks[t].args) {
      key += " " + model.objects.objects.items[kairos::sem::ObjectId(o)].name;
    }
    tasks.emplace(std::move(key), t);
  }
  std::istringstream in{std::string(plan)};
  std::string line;
  bool inside = false;
  while (std::getline(in, line)) {
    for (char & c : line) {
      c = c == '(' || c == ')' || c == '\r' || c == '\t' ? ' ' :
        c >= 'A' && c <= 'Z' ? static_cast<char>(c - 'A' + 'a') : c;
    }
    std::istringstream words(line);
    std::vector<std::string> w;
    for (std::string word; words >> word; ) {
      w.push_back(word);
    }
    if (w.empty()) {
      continue;
    }
    if (w[0] == "==>" || w[0] == "<==") {
      inside = w[0] == "==>";
      continue;
    }
    if (!inside || w[0] == "root" || w.size() < 2) {
      continue;
    }
    std::string key = w[1];
    size_t i = 2;
    for (; i < w.size() && w[i] != "->"; ++i) {
      key += " " + w[i];
    }
    auto found = tasks.find(key);
    if (found == tasks.end()) {
      return "task (" + key + ")";
    }
    if (i + 1 < w.size()) {
      const auto & methods = ground.tasks[found->second].methods;
      const bool has = std::any_of(methods.begin(), methods.end(), [&](uint32_t m) {
            return model.methods[ground.methods[m].method].name == w[i + 1];
          });
      if (!has) {
        return "method " + w[i + 1] + " of (" + key + ")";
      }
    }
  }
  return "";
}

}  // namespace

CheckResult check(
  std::string_view domain, std::string_view problem, std::string_view domain_file,
  std::string_view problem_file)
{
  CheckResult result;
  auto d = hddl::parse_domain(domain, domain_file);
  append(result.diagnostics, d.diagnostics(), domain);
  auto p = hddl::parse_problem(problem, problem_file);
  append(result.diagnostics, p.diagnostics(), problem);
  if (!d.ok() || !p.ok()) {
    return result;
  }
  // Diagnostics of the model come from either file: rendered without the source line
  auto model = hddl::analyze(std::move(d).value(), std::move(p).value());
  append(result.diagnostics, model.diagnostics(), "");
  if (!model.ok()) {
    return result;
  }
  Summary & s = result.summary;
  s.types = model->domain.types.size();
  s.predicates = model->domain.predicates.items.size();
  s.actions = model->user_actions + model->pddl.durative_actions.size();
  s.tasks = model->tasks.size() - s.actions;
  s.methods = model->methods.size();
  s.objects = model->objects.objects.items.size();
  s.initial_tasks = model->initial.steps.size();
  s.has_goal = model->has_goal;
  s.total_order = model->total_order;
  result.ok = true;
  return result;
}

GroundResult ground(
  std::string_view domain, std::string_view problem, double seconds,
  std::string_view domain_file, std::string_view problem_file, std::string_view plan)
{
  GroundResult result;
  auto d = hddl::parse_domain(domain, domain_file);
  append(result.diagnostics, d.diagnostics(), domain);
  auto p = hddl::parse_problem(problem, problem_file);
  append(result.diagnostics, p.diagnostics(), problem);
  if (!d.ok() || !p.ok()) {
    return result;
  }
  auto model = hddl::analyze(std::move(d).value(), std::move(p).value());
  append(result.diagnostics, model.diagnostics(), "");
  if (!model.ok()) {
    return result;
  }
  kairos::BudgetConfig config;
  if (seconds > 0.0) {
    config.deadline = kairos::BudgetConfig::Clock::now() +
      std::chrono::duration_cast<kairos::BudgetConfig::Clock::duration>(
      std::chrono::duration<double>(seconds));
  }
  kairos::Budget budget(config);
  auto ground = tdg::ground(model.value(), &budget);
  append(result.diagnostics, ground.diagnostics(), "");
  if (!ground.ok()) {
    return result;
  }
  result.ok = true;
  result.unsolvable = ground->unsolvable;
  result.facts = ground->pddl.atoms.size();
  result.actions = ground->primitive_tasks;
  result.abstract_tasks = ground->tasks.size() - ground->primitive_tasks;
  result.methods = ground->methods.size();
  result.milliseconds = ground->stats.milliseconds;
  if (!plan.empty()) {
    result.missing = first_missing(ground->model, ground.value(), plan);
  }
  return result;
}

SolveResult solve(
  std::string_view domain, std::string_view problem, const SolveOptions & options,
  std::string_view domain_file, std::string_view problem_file)
{
  SolveResult result;
  const auto start = std::chrono::steady_clock::now();
  auto d = hddl::parse_domain(domain, domain_file);
  append(result.diagnostics, d.diagnostics(), domain);
  auto p = hddl::parse_problem(problem, problem_file);
  append(result.diagnostics, p.diagnostics(), problem);
  if (!d.ok() || !p.ok()) {
    return result;
  }
  auto model = hddl::analyze(std::move(d).value(), std::move(p).value());
  append(result.diagnostics, model.diagnostics(), "");
  if (!model.ok()) {
    return result;
  }
  if (!model->has_htn) {
    result.status = Status::Unsupported;
    result.message = "a problem without :htn is a PDDL problem: use Kairos";
    return result;
  }
  kairos::BudgetConfig config;
  if (options.seconds > 0.0) {
    config.deadline = start + std::chrono::duration_cast<kairos::BudgetConfig::Clock::duration>(
      std::chrono::duration<double>(options.seconds));
  }
  kairos::Budget budget(config);
  auto ground = tdg::ground(model.value(), &budget);
  auto since = [&]() {
      return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    };
  result.ground_ms = since();
  if (!ground.ok()) {
    const bool budget_out = !ground.diagnostics().empty() &&
      ground.diagnostics()[0].code == "ground.budget";
    if (!budget_out) {
      append(result.diagnostics, ground.diagnostics(), "");
    }
    result.status = budget_out ? Status::Timeout : Status::Unsupported;
    result.message = budget_out ? "" : "the problem cannot be grounded";
    return result;
  }
  search::Result found;
  try {
    search::Options how;
    how.anytime = options.anytime;
    if (options.on_plan) {
      how.on_plan = [&](const std::string & plan, size_t steps) {
          options.on_plan(with_original_names(plan, spelling_of(domain, problem)), steps);
        };
    }
    found = search::total_order(ground.value(), &budget, how);
  } catch (const std::bad_alloc &) {
    found.outcome = search::Outcome::OutOfBudget;
    found.message = "out of memory";
  }
  result.search_ms = since() - result.ground_ms;
  result.expansions = found.expansions;
  result.steps = found.steps;
  result.plans = found.plans;
  result.optimal = found.optimal;
  result.message = std::move(found.message);
  switch (found.outcome) {
    case search::Outcome::Solved:
      result.status = Status::Solved;
      {
        const Spelling spelling = spelling_of(domain, problem);
        result.plan = with_original_names(found.plan, spelling);
        if (options.decomposition) {
          verifier::Tree tree;
          const verifier::Outcome checked = verifier::verify(model.value(), found.plan, &tree);
          if (!checked.valid) {
            result.status = Status::InternalError;
            result.message = "the plan found is not valid: " + checked.reason;
            result.plan.clear();
            break;
          }
          auto name = [&](std::string & word) {
              if (const auto it = spelling.find(word); it != spelling.end()) {
                word = it->second;
              }
            };
          for (verifier::Step & step : tree.steps) {
            PlanStep out;
            name(step.task);
            name(step.method);
            std::for_each(step.args.begin(), step.args.end(), name);
            std::for_each(step.params.begin(), step.params.end(), name);
            std::for_each(step.labels.begin(), step.labels.end(), name);
            out.id = std::move(step.id);
            out.task = std::move(step.task);
            out.args = std::move(step.args);
            out.primitive = step.primitive;
            out.method = std::move(step.method);
            out.params = std::move(step.params);
            out.subtasks = std::move(step.subtasks);
            out.labels = std::move(step.labels);
            result.steps_of_plan.push_back(std::move(out));
          }
          std::for_each(tree.root_labels.begin(), tree.root_labels.end(), name);
          result.sequence = std::move(tree.sequence);
          result.roots = std::move(tree.roots);
          result.root_labels = std::move(tree.root_labels);
        }
      }
      break;
    case search::Outcome::Unsolvable:
      result.status = Status::Unsolvable;
      break;
    case search::Outcome::OutOfBudget:
      result.status = Status::Timeout;
      break;
    case search::Outcome::Unsupported:
      result.status = Status::Unsupported;
      break;
  }
  return result;
}

std::string_view to_string(Status status)
{
  switch (status) {
    case Status::Solved: return "solved";
    case Status::Unsolvable: return "unsolvable";
    case Status::Timeout: return "timeout";
    case Status::Unsupported: return "unsupported";
    case Status::InternalError: return "internal error";
    default: return "invalid input";
  }
}

VerifyResult verify(
  std::string_view domain, std::string_view problem, std::string_view plan,
  std::string_view domain_file, std::string_view problem_file)
{
  VerifyResult result;
  auto d = hddl::parse_domain(domain, domain_file);
  append(result.diagnostics, d.diagnostics(), domain);
  auto p = hddl::parse_problem(problem, problem_file);
  append(result.diagnostics, p.diagnostics(), problem);
  if (!d.ok() || !p.ok()) {
    result.reason = "the domain or the problem has errors";
    return result;
  }
  auto model = hddl::analyze(std::move(d).value(), std::move(p).value());
  append(result.diagnostics, model.diagnostics(), "");
  if (!model.ok()) {
    result.reason = "the domain or the problem has errors";
    return result;
  }
  verifier::Outcome outcome = verifier::verify(model.value(), plan);
  result.valid = outcome.valid;
  result.reason = std::move(outcome.reason);
  return result;
}

}  // namespace klimax

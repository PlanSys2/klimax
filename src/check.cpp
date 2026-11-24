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

#include <algorithm>
#include <chrono>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "hddl/model.hpp"
#include "hddl/parser.hpp"
#include "kairos/diagnostics.hpp"
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

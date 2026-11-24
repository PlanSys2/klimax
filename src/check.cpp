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

#include <string>
#include <utility>
#include <vector>

#include "hddl/model.hpp"
#include "hddl/parser.hpp"
#include "kairos/diagnostics.hpp"

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
  s.has_goal = model->problem.goal.valid();
  s.total_order = model->total_order;
  result.ok = true;
  return result;
}

}  // namespace klimax

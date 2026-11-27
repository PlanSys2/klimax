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


#include "schedule.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>

#include "ground/grounder.hpp"
#include "kairos/plan.hpp"
#include "normalize/normalizer.hpp"
#include "sas/translate.hpp"
#include "state/registry.hpp"
#include "state/successors.hpp"
#include "temporal/compress.hpp"
#include "temporal/snap.hpp"

namespace klimax
{

namespace
{

namespace lifted = kairos::lifted;
namespace sas = kairos::sas;
namespace sem = kairos::sem;

std::string lowered(std::string s)
{
  for (char & c : s) {
    if (c >= 'A' && c <= 'Z') {
      c = static_cast<char>(c - 'A' + 'a');
    }
  }
  return s;
}

// "(name object ...)", as Kairos names the instances of its actions
std::string instance(const hddl::Model & m, const verifier::Order::Action & step)
{
  std::string out = "(" + lowered(m.tasks[step.task].name);
  for (uint32_t o : step.args) {
    out += " " + lowered(m.objects.objects.items[sem::ObjectId(o)].name);
  }
  return out + ")";
}

// KLIMAX_TRACE_SCHEDULE: why the plan is left as it was
std::vector<verifier::Timed> gave_up(const std::string & why)
{
  if (std::getenv("KLIMAX_TRACE_SCHEDULE") != nullptr) {
    std::fprintf(stderr, "schedule: %s\n", why.c_str());
  }
  return {};
}

// Kairos' task with only the instances of the actions that the plan uses
std::optional<sas::Task> task_of(
  const hddl::Model & model, const std::vector<verifier::Order::Action> & steps)
{
  lifted::Task task = lifted::normalize(model.pddl, model.domain, model.problem, model.objects);
  // Only the instances that the plan uses are grounded: each action needs a fact that only
  // they have
  std::map<std::string, std::set<std::vector<uint32_t>>> used;
  for (const verifier::Order::Action & s : steps) {
    used[lowered(model.tasks[s.task].name)].insert(s.args);
  }
  auto restrict = [&](const std::string & name, const std::vector<lifted::Variable> & variables,
      uint32_t parameters, lifted::Condition & condition) {
      std::vector<sem::TypeSpec> types;
      lifted::Literal literal;
      for (uint32_t v = 0; v < parameters; ++v) {
        types.push_back(variables[v].types);
        literal.atom.args.push_back(lifted::Term::variable(v));
      }
      literal.atom.predicate = task.add_predicate("klimax-needed " + name, false, types);
      condition.literals.push_back(literal);
      for (const std::vector<uint32_t> & row : used[lowered(name)]) {
        std::vector<sem::ObjectId> objects;
        for (uint32_t o : row) {
          objects.emplace_back(o);
        }
        task.init_facts.emplace_back(literal.atom.predicate, std::move(objects));
      }
    };
  for (size_t i = 0; i < task.actions.size(); ++i) {
    lifted::Action & a = task.actions[i];
    restrict(a.name, a.variables, a.parameters, a.precondition);
  }
  for (size_t i = 0; i < task.durative_actions.size(); ++i) {
    lifted::DurativeAction & a = task.durative_actions[i];
    restrict(a.name, a.variables, a.parameters, a.at_start);
  }
  kairos::ground::GroundOptions options;
  options.drop_useless_actions = false;
  auto ground = kairos::ground::ground(task, options);
  if (!ground.ok()) {
    gave_up("the actions of the plan cannot be grounded: " +
      (ground.diagnostics().empty() ? std::string() : ground.diagnostics()[0].message));
    return std::nullopt;
  }
  auto translated = sas::translate(task, std::move(ground).value());
  if (!translated.ok()) {
    gave_up("no SAS task");
    return std::nullopt;
  }
  return std::move(translated).value();
}

// The times of the steps from Kairos' plan, which gives them in the order of time: those of
// the same action, in the order of the plan
std::vector<verifier::Timed> times_of(
  const hddl::Model & model, const std::vector<verifier::Order::Action> & steps,
  const kairos::Plan & plan)
{
  std::unordered_map<std::string, std::vector<const kairos::PlanStep *>> timed;
  for (const kairos::PlanStep & p : plan.steps) {
    timed[p.action()].push_back(&p);
  }
  std::unordered_map<std::string, size_t> taken;
  std::vector<verifier::Timed> times;
  for (const verifier::Order::Action & s : steps) {
    const std::string name = instance(model, s);
    const auto found = timed.find(name);
    size_t & next = taken[name];
    if (found == timed.end() || next >= found->second.size()) {
      return gave_up("Kairos' plan has no step " + name);
    }
    const kairos::PlanStep & p = *found->second[next++];
    times.push_back(verifier::Timed{p.time, p.duration});
  }
  return times;
}

}  // namespace

std::vector<verifier::Timed> schedule(
  const hddl::Model & model, const std::vector<verifier::Order::Action> & steps,
  const std::vector<std::pair<uint32_t, uint32_t>> & precedences)
{
  const std::optional<sas::Task> grounded = task_of(model, steps);
  if (!grounded) {
    return {};
  }
  const sas::Task & original = *grounded;
  const kairos::temporal::Compressed compressed = kairos::temporal::compress(original);
  kairos::state::StateRegistry registry(compressed.task, nullptr, false);
  kairos::state::StateSpace space(compressed.task, registry);
  // An action may have several instances with the same name (the cases of a disjunction):
  // the one that applies where the plan gets to
  std::unordered_map<std::string, std::vector<uint32_t>> by_name;
  for (uint32_t a = 0; a < compressed.task.actions.size(); ++a) {
    by_name[sas::instance_name(compressed.task, compressed.task.actions[a])].push_back(a);
  }
  std::vector<uint32_t> actions;
  kairos::state::StateId state = registry.initial();
  for (const verifier::Order::Action & s : steps) {
    const auto found = by_name.find(instance(model, s));
    std::optional<kairos::state::StateId> next;
    if (found != by_name.end()) {
      for (uint32_t a : found->second) {
        if ((next = space.successor(state, a))) {
          actions.push_back(a);
          break;
        }
      }
    }
    if (!next) {
      return gave_up("step " + instance(model, s) + (found == by_name.end() ?
               " is not an action of the task" : " cannot be replayed"));
    }
    state = *next;
  }
  const std::optional<kairos::Plan> plan = kairos::temporal::schedule(
    original, compressed, space, actions, 0.001, {}, precedences);
  if (!plan || plan->steps.size() != steps.size()) {
    return gave_up("Kairos gives no schedule");
  }
  return times_of(model, steps, *plan);
}

std::vector<verifier::Timed> schedule(
  const hddl::Model & model, const std::vector<verifier::Order::Action> & steps,
  const std::vector<Happening> & happenings, const verifier::Order * order)
{
  const std::optional<sas::Task> grounded = task_of(model, steps);
  if (!grounded) {
    return {};
  }
  const sas::Task & original = *grounded;
  const kairos::temporal::SnapTask snap = kairos::temporal::snap(original);
  if (!snap.unsupported.empty()) {
    return gave_up("Kairos does not time this task: " + snap.unsupported);
  }
  kairos::state::StateRegistry registry(snap.task, nullptr, false);
  kairos::state::StateSpace space(snap.task, registry);
  // The actions of the snap task by the name of the action they are of: the instantaneous
  // ones, and the start of each durative one (its end comes next)
  std::unordered_map<std::string, std::vector<uint32_t>> by_name;
  for (uint32_t a = 0; a < original.actions.size(); ++a) {
    by_name[sas::instance_name(original, original.actions[a])].push_back(a);
  }
  for (uint32_t d = 0; d < original.durative_actions.size(); ++d) {
    by_name[sas::instance_name(original, original.durative_actions[d])].push_back(
      static_cast<uint32_t>(snap.instant + 2 * d));
  }
  std::vector<uint32_t> actions;
  std::vector<uint32_t> started(steps.size(), 0);  // the action each step started with
  kairos::state::StateId state = registry.initial();
  for (const Happening & h : happenings) {
    std::optional<kairos::state::StateId> next;
    if (h.end) {
      next = space.successor(state, started[h.step] + 1);
      if (next) {
        actions.push_back(started[h.step] + 1);
      }
    } else if (const auto found = by_name.find(instance(model, steps[h.step]));
      found != by_name.end())
    {
      for (uint32_t a : found->second) {
        if ((next = space.successor(state, a))) {
          actions.push_back(a);
          started[h.step] = a;
          break;
        }
      }
    }
    if (!next) {
      return gave_up(std::string(h.end ? "the end of " : "the start of ") +
               instance(model, steps[h.step]) + " cannot be replayed");
    }
    state = *next;
  }
  // What the hierarchy says of the times, as bounds between happenings: where each step
  // starts and ends among them (both at once for an instantaneous one)
  using Bound = kairos::temporal::TemporalFilter::Bound;
  std::vector<Bound> bounds;
  if (order != nullptr && order->actions.size() == steps.size()) {
    std::vector<size_t> start(steps.size(), 0), end(steps.size(), 0);
    for (size_t k = 0; k < happenings.size(); ++k) {
      (happenings[k].end ? end : start)[happenings[k].step] = k;
      if (!model.tasks[steps[happenings[k].step].task].durative) {
        end[happenings[k].step] = k;
      }
    }
    for (const auto & [first, second] : order->before) {
      bounds.push_back(Bound{end[first], start[second], 0.0});
    }
    constexpr double kAfter = 0.001;  // strictly later: by the epsilon of the plan
    for (const verifier::Order::Time & t : order->times) {
      const size_t a = (t.a.end ? end : start)[t.a.action];
      const size_t b = (t.b.end ? end : start)[t.b.action];
      bounds.push_back(t.op == kairos::ast::CompareOp::Eq ? Bound{a, b, 0.0, 0.0} :
        Bound{a, b, t.op == kairos::ast::CompareOp::Lt ? kAfter : 0.0});
    }
  }
  kairos::temporal::TemporalFilter filter(original, snap, space, 0.001);
  const std::optional<kairos::Plan> plan = filter.plan(actions, bounds);
  if (!plan || plan->steps.size() != steps.size()) {
    return gave_up("Kairos finds no times for the starts and the ends in this order");
  }
  return times_of(model, steps, *plan);
}

}  // namespace klimax

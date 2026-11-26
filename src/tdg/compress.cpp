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


#include "tdg/compress.hpp"

#include "hddl/split.hpp"

#include <algorithm>
#include <set>
#include <utility>
#include <vector>

namespace klimax::tdg
{
namespace
{

namespace lifted = kairos::lifted;

bool uses_duration(const lifted::Expr & e)
{
  return e.op == lifted::ExprOp::Duration || e.op == lifted::ExprOp::ContinuousT ||
         std::any_of(e.children.begin(), e.children.end(), uses_duration);
}

void functions(const lifted::Expr & e, std::set<uint32_t> & out)
{
  if (e.op == lifted::ExprOp::Fluent) {
    out.insert(e.function.value());
  }
  for (const lifted::Expr & c : e.children) {
    functions(c, out);
  }
}

// What a condition reads: whether ?duration, and which functions
bool reads(const lifted::Condition & c, std::set<uint32_t> & out)
{
  bool duration = false;
  for (const lifted::Comparison & k : c.comparisons) {
    if (k.objects) {
      for (const lifted::Term * t : {&k.lhs_term, &k.rhs_term}) {
        if (t->kind == lifted::Term::Kind::Function) {
          out.insert(t->index);
        }
      }
      continue;
    }
    duration = duration || uses_duration(k.lhs) || uses_duration(k.rhs);
    functions(k.lhs, out);
    functions(k.rhs, out);
  }
  for (const lifted::Quantified & q : c.foralls) {
    duration = reads(q.body, out) || duration;
  }
  return duration;
}

std::string one(const lifted::DurativeAction & d, lifted::Action & a)
{
  a.name = d.name;
  a.variables = d.variables;
  a.parameters = d.parameters;
  a.hidden = d.hidden;
  const std::string who = "durative action " + d.name + ": ";
  // What the effects at start leave: atoms and the functions they change
  std::vector<const lifted::Effect *> start;
  std::set<uint32_t> changed_at_start;
  for (const lifted::Effect & e : d.effects) {
    if (e.continuous) {
      return who + "continuous effects";
    }
    std::set<uint32_t> read;
    if (reads(e.condition, read) || reads(e.start_condition, read) || uses_duration(e.value)) {
      return who + "?duration in an effect";
    }
    if (e.timing != lifted::Timing::End) {
      start.push_back(&e);
      if (e.kind == lifted::Effect::Kind::Numeric || e.kind == lifted::Effect::Kind::ObjectAssign) {
        changed_at_start.insert(e.function.value());
      }
    }
  }
  auto given = [&](const lifted::Literal & l) {
      return std::any_of(start.begin(), start.end(), [&](const lifted::Effect * e) {
                 return e->variables.empty() && e->condition.empty() &&
                        e->kind == (l.positive ? lifted::Effect::Kind::Add :
                        lifted::Effect::Kind::Delete) && e->atom == l.atom;
               });
    };
  std::set<uint32_t> read;
  if (reads(d.at_start, read)) {
    return who + "?duration in a condition";
  }
  a.precondition = d.at_start;
  for (const lifted::Condition * later : {&d.over_all, &d.at_end}) {
    std::set<uint32_t> after;
    if (reads(*later, after)) {
      return who + "?duration in a condition";
    }
    if (std::any_of(after.begin(), after.end(),
      [&](uint32_t f) {return changed_at_start.count(f) != 0;}))
    {
      return who + "a condition reads a fluent that the action changed at start";
    }
    for (const lifted::Literal & l : later->literals) {
      if (!given(l)) {
        a.precondition.literals.push_back(l);
      }
    }
    a.precondition.comparisons.insert(a.precondition.comparisons.end(),
      later->comparisons.begin(), later->comparisons.end());
    a.precondition.foralls.insert(a.precondition.foralls.end(),
      later->foralls.begin(), later->foralls.end());
  }
  // The effects at end come after those at start
  for (const lifted::Effect & e : d.effects) {
    if (e.timing != lifted::Timing::End) {
      continue;
    }
    if (!e.condition.empty() || !e.start_condition.empty()) {
      return who + "a conditional effect at end";
    }
    std::set<uint32_t> value;
    functions(e.value, value);
    if (e.kind == lifted::Effect::Kind::Numeric || e.kind == lifted::Effect::Kind::ObjectAssign) {
      value.insert(e.function.value());
    }
    if (std::any_of(value.begin(), value.end(),
      [&](uint32_t f) {return changed_at_start.count(f) != 0;}))
    {
      return who + "a fluent changed at start and again at end";
    }
  }
  for (const lifted::Effect & e : d.effects) {
    // An atom added at start and deleted at end is deleted
    const bool undone = e.timing != lifted::Timing::End && e.kind == lifted::Effect::Kind::Add &&
      std::any_of(d.effects.begin(), d.effects.end(), [&](const lifted::Effect & later) {
          return later.timing == lifted::Timing::End &&
                 later.kind == lifted::Effect::Kind::Delete && later.atom == e.atom &&
                 later.variables == e.variables;
        });
    if (!undone) {
      a.effects.push_back(e);
      a.effects.back().timing = lifted::Timing::Instant;
    }
  }
  return "";
}

}  // namespace

std::string start_and_end(kairos::lifted::Task & task)
{
  if (!task.timed_events.empty()) {
    return "timed initial literals in a problem whose actions must overlap";
  }
  std::vector<lifted::Action> made;
  for (const lifted::DurativeAction & d : task.durative_actions) {
    const std::string who = "durative action " + d.name + ": ";
    std::set<uint32_t> read;
    if (reads(d.at_start, read) || reads(d.over_all, read) || reads(d.at_end, read)) {
      return who + "?duration in a condition";
    }
    lifted::Action start;
    lifted::Action end;
    start.name = d.name;
    end.name = hddl::end_of(d.name);
    start.variables = d.variables;
    start.parameters = d.parameters;
    start.hidden = d.hidden;
    // The end has the parameters only: nothing else says which start it ends
    end.variables.assign(d.variables.begin(), d.variables.begin() + d.parameters);
    end.parameters = d.parameters;
    auto only_parameters = [&](const lifted::Condition & c) {
        auto in = [&](const lifted::Term & t) {
            return t.kind != lifted::Term::Kind::Variable || t.index < d.parameters;
          };
        for (const lifted::Literal & l : c.literals) {
          if (!std::all_of(l.atom.args.begin(), l.atom.args.end(), in)) {
            return false;
          }
        }
        return c.foralls.empty() && std::all_of(c.comparisons.begin(), c.comparisons.end(),
                 [&](const lifted::Comparison & k) {
                   return !k.objects && k.lhs.children.empty() && k.rhs.children.empty() &&
                          std::all_of(k.lhs.args.begin(), k.lhs.args.end(), in) &&
                          std::all_of(k.rhs.args.begin(), k.rhs.args.end(), in);
                 });
      };
    if (!only_parameters(d.at_end) || !only_parameters(d.over_all)) {
      return who + "a condition at end or over all that is not over its parameters alone";
    }
    std::vector<kairos::sem::TypeSpec> types;
    lifted::Literal running;
    for (uint32_t v = 0; v < d.parameters; ++v) {
      types.push_back(d.variables[v].types);
      running.atom.args.push_back(lifted::Term::variable(v));
    }
    running.atom.predicate = task.add_predicate("klimax-running " + d.name, false, types);
    start.precondition = d.at_start;
    end.precondition = d.at_end;
    for (const lifted::Literal & l : d.over_all.literals) {
      const bool given = std::any_of(d.effects.begin(), d.effects.end(),
          [&](const lifted::Effect & e) {
            return e.timing != lifted::Timing::End && e.variables.empty() &&
                   e.condition.empty() && e.atom == l.atom &&
                   e.kind == (l.positive ? lifted::Effect::Kind::Add :
                   lifted::Effect::Kind::Delete);
          });
      if (!given) {
        start.precondition.literals.push_back(l);
      }
      end.precondition.literals.push_back(l);
    }
    for (const lifted::Comparison & k : d.over_all.comparisons) {
      end.precondition.comparisons.push_back(k);
    }
    lifted::Literal idle = running;
    idle.positive = false;
    start.precondition.literals.push_back(idle);
    end.precondition.literals.push_back(running);
    for (const lifted::Effect & e : d.effects) {
      std::set<uint32_t> in_effect;
      if (e.continuous || reads(e.condition, in_effect) || reads(e.start_condition, in_effect) ||
        uses_duration(e.value))
      {
        return who + "?duration or continuous change in an effect";
      }
      const bool at_end = e.timing == lifted::Timing::End;
      if (at_end && (!e.start_condition.empty() || !only_parameters(e.condition))) {
        return who + "a conditional effect at end";
      }
      lifted::Effect instant = e;
      instant.timing = lifted::Timing::Instant;
      (at_end ? end : start).effects.push_back(std::move(instant));
    }
    lifted::Effect runs;
    runs.kind = lifted::Effect::Kind::Add;
    runs.atom = running.atom;
    start.effects.push_back(runs);
    runs.kind = lifted::Effect::Kind::Delete;
    end.effects.push_back(runs);
    made.push_back(std::move(start));
    made.push_back(std::move(end));
  }
  for (lifted::Action & a : made) {
    task.actions.push_back(std::move(a));
  }
  task.durative_actions.clear();
  return "";
}

std::string clock(
  kairos::lifted::Task & task, const std::function<bool(const std::string &)> & step)
{
  const kairos::sem::FunctionId id(static_cast<uint32_t>(task.function_names.size()));
  auto moves = [&](lifted::Expr by, lifted::Timing timing) {
      lifted::Effect e;
      e.kind = lifted::Effect::Kind::Numeric;
      e.timing = timing;
      e.function = id;
      e.op = kairos::ast::AssignOp::Increase;
      lifted::Expr pause;
      pause.value = kEpsilon;
      e.value.op = lifted::ExprOp::Add;
      e.value.children.push_back(std::move(by));
      e.value.children.push_back(std::move(pause));
      return e;
    };
  std::vector<lifted::Effect> durative;
  for (const lifted::DurativeAction & d : task.durative_actions) {
    // What it lasts: what its duration is equal to, or else the least it allows
    const lifted::DurationConstraint * chosen = nullptr;
    for (const lifted::DurationConstraint & c : d.duration) {
      if (c.op == lifted::CompareOp::Eq || (chosen == nullptr && c.op == lifted::CompareOp::Ge)) {
        chosen = &c;
      }
    }
    if (chosen == nullptr || uses_duration(chosen->value)) {
      return "durative action " + d.name + ": a duration with no value or lower bound";
    }
    durative.push_back(moves(chosen->value, lifted::Timing::Start));
  }
  task.function_names.push_back(kClock);
  task.numeric.push_back(true);
  task.function_params.emplace_back();
  lifted::InitFluent start;
  start.function = id;
  task.init_fluents.push_back(start);
  for (size_t i = 0; i < task.durative_actions.size(); ++i) {
    task.durative_actions[i].effects.push_back(std::move(durative[i]));
  }
  for (lifted::Action & a : task.actions) {
    if (step(a.name)) {
      a.effects.push_back(moves(lifted::Expr{}, lifted::Timing::Instant));
    }
  }
  return "";
}

std::string compress(kairos::lifted::Task & task)
{
  std::vector<lifted::Action> made;
  for (const lifted::DurativeAction & d : task.durative_actions) {
    made.emplace_back();
    if (std::string why = one(d, made.back()); !why.empty()) {
      return why;
    }
  }
  for (lifted::Action & a : made) {
    task.actions.push_back(std::move(a));
  }
  task.durative_actions.clear();
  return "";
}

}  // namespace klimax::tdg

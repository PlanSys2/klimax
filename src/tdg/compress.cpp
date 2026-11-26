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

std::string compress(kairos::lifted::Task & task)
{
  if (!task.timed_events.empty()) {
    return "timed initial literals";
  }
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

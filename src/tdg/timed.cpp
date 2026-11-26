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


#include "tdg/timed.hpp"

#include <algorithm>
#include <limits>
#include <map>
#include <utility>
#include <vector>

#include "tdg/compress.hpp"

namespace klimax::tdg
{
namespace
{

namespace ground = kairos::ground;
using Interval = std::pair<double, double>;  // holds from, until
constexpr double kNever = std::numeric_limits<double>::infinity();

ground::Expr constant(double value)
{
  ground::Expr e;
  e.value = value;
  return e;
}

ground::Expr sum(ground::Expr a, ground::Expr b)
{
  if (a.op == ground::ExprOp::Constant && b.op == ground::ExprOp::Constant) {
    return constant(a.value + b.value);
  }
  ground::Expr e;
  e.op = ground::ExprOp::Add;
  e.children.push_back(std::move(a));
  e.children.push_back(std::move(b));
  return e;
}

std::vector<Interval> both(const std::vector<Interval> & a, const std::vector<Interval> & b)
{
  std::vector<Interval> out;
  for (const Interval & x : a) {
    for (const Interval & y : b) {
      const Interval i{std::max(x.first, y.first), std::min(x.second, y.second)};
      if (i.first < i.second) {
        out.push_back(i);
      }
    }
  }
  return out;
}

std::vector<Interval> rest(const std::vector<Interval> & a)
{
  std::vector<Interval> out;
  double from = 0.0;
  for (const Interval & x : a) {
    if (from < x.first) {
      out.emplace_back(from, x.first);
    }
    from = x.second;
  }
  if (from < kNever) {
    out.emplace_back(from, kNever);
  }
  return out;
}

bool mentions(const ground::Condition & c, const std::map<ground::AtomId, std::vector<Interval>> & timed)
{
  auto in = [&](const std::vector<ground::AtomId> & atoms) {
      return std::any_of(atoms.begin(), atoms.end(),
               [&](ground::AtomId a) {return timed.count(a) != 0;});
    };
  return in(c.positive) || in(c.negative);
}

}  // namespace

std::string timed_literals(kairos::ground::Task & task)
{
  if (task.timed_events.empty()) {
    return "";
  }
  uint32_t clock = 0;
  while (clock < task.numeric_fluents.size() &&
    task.function_names[task.numeric_fluents[clock].function.index()] != kClock)
  {
    ++clock;
  }
  if (clock == task.numeric_fluents.size()) {
    return "timed initial literals";  // nothing moves the clock
  }
  // When each atom of a timed literal holds
  std::map<ground::AtomId, std::vector<std::pair<double, bool>>> changes;
  for (const ground::TimedEvent & e : task.timed_events) {
    if (e.kind != ground::TimedEvent::Kind::Add && e.kind != ground::TimedEvent::Kind::Delete) {
      return "timed values of fluents";
    }
    changes[e.index].emplace_back(e.time, e.kind == ground::TimedEvent::Kind::Add);
  }
  std::map<ground::AtomId, std::vector<Interval>> timed;
  for (auto & [atom, events] : changes) {
    std::stable_sort(events.begin(), events.end(),
      [](const auto & a, const auto & b) {return a.first < b.first;});
    bool holds = std::find(task.init_atoms.begin(), task.init_atoms.end(), atom) !=
      task.init_atoms.end();
    double since = 0.0;
    std::vector<Interval> & when = timed[atom];
    for (const auto & [time, add] : events) {
      if (holds && !add) {
        when.emplace_back(since, time);
      } else if (!holds && add) {
        since = time;
      }
      holds = add;
    }
    if (holds) {
      when.emplace_back(since, kNever);
    }
  }
  if (mentions(task.goal, timed) || mentions(task.always, timed)) {
    return "a timed initial literal read by the goal";
  }
  std::vector<ground::Action> actions;
  for (const ground::Action & a : task.actions) {
    const ground::Effect * moves = nullptr;
    for (const ground::Effect & e : a.effects) {
      if (mentions(e.condition, timed)) {
        return "a timed initial literal read by the condition of an effect";
      }
      if ((e.kind == ground::Effect::Kind::Add || e.kind == ground::Effect::Kind::Delete) &&
        timed.count(e.atom) != 0)
      {
        return "an atom changed by timed initial literals and by actions";
      }
      if (e.kind == ground::Effect::Kind::Numeric && e.fluent == clock) {
        moves = &e;
      }
    }
    for (const auto * list : {&a.changes.add, &a.changes.del}) {
      for (ground::AtomId atom : *list) {
        if (timed.count(atom) != 0) {
          return "an atom changed by timed initial literals and by actions";
        }
      }
    }
    if (!mentions(a.precondition, timed)) {
      actions.push_back(a);
      continue;
    }
    // The intervals in which all it reads of them holds
    std::vector<Interval> windows{Interval{0.0, kNever}};
    ground::Action plain = a;
    plain.precondition.positive.clear();
    plain.precondition.negative.clear();
    for (ground::AtomId atom : a.precondition.positive) {
      if (timed.count(atom) != 0) {
        windows = both(windows, timed.at(atom));
      } else {
        plain.precondition.positive.push_back(atom);
      }
    }
    for (ground::AtomId atom : a.precondition.negative) {
      if (timed.count(atom) != 0) {
        windows = both(windows, rest(timed.at(atom)));
      } else {
        plain.precondition.negative.push_back(atom);
      }
    }
    ground::Expr now;
    now.op = ground::ExprOp::Fluent;
    now.fluent = clock;
    // What the step lasts (nothing: the test of a method's precondition, at the clock)
    const ground::Expr lasts = moves == nullptr ? constant(0.0) :
      sum(moves->value, constant(-kEpsilon));
    for (const Interval & w : windows) {
      // A literal at time t is there for what starts at t + epsilon, and until t - epsilon
      const double opens = w.first == 0.0 ? 0.0 : w.first + kEpsilon;
      auto ends_in_time = [&](ground::Action & made, ground::Expr start) {
          if (w.second < kNever) {
            made.precondition.numeric.push_back(ground::NumericCondition{
                ground::CompareOp::Le, sum(std::move(start), lasts),
                constant(w.second - kEpsilon)});
          }
        };
      ground::Action inside = plain;
      if (opens > 0.0) {
        inside.precondition.numeric.push_back(
          ground::NumericCondition{ground::CompareOp::Ge, now, constant(opens)});
      }
      ends_in_time(inside, now);
      actions.push_back(std::move(inside));
      if (opens > 0.0 && moves != nullptr) {
        // Before it opens: the step waits, and starts then
        ground::Action waits = plain;
        waits.precondition.numeric.push_back(
          ground::NumericCondition{ground::CompareOp::Lt, now, constant(opens)});
        ends_in_time(waits, constant(opens));
        for (ground::Effect & e : waits.effects) {
          if (e.kind == ground::Effect::Kind::Numeric && e.fluent == clock) {
            e.op = kairos::ast::AssignOp::Assign;
            e.value = sum(constant(opens), e.value);
          }
        }
        actions.push_back(std::move(waits));
      }
    }
  }
  task.actions = std::move(actions);
  task.timed_events.clear();
  return "";
}

}  // namespace klimax::tdg

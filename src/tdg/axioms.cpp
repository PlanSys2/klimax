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

#include "tdg/axioms.hpp"

#include <algorithm>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace klimax::tdg
{

namespace
{

namespace ground = kairos::ground;
using Cases = std::vector<ground::Condition>;  // a disjunction of conjunctions

struct Too
{
  std::string why;
};

ground::CompareOp negated(ground::CompareOp op)
{
  switch (op) {
    case ground::CompareOp::Lt: return ground::CompareOp::Ge;
    case ground::CompareOp::Le: return ground::CompareOp::Gt;
    case ground::CompareOp::Gt: return ground::CompareOp::Le;
    case ground::CompareOp::Ge: return ground::CompareOp::Lt;
    case ground::CompareOp::Eq: return ground::CompareOp::Ne;
    case ground::CompareOp::Ne: return ground::CompareOp::Eq;
  }
  return op;
}

// Both conjunctions at once; none if an atom would be true and false
std::optional<ground::Condition> both(const ground::Condition & a, const ground::Condition & b)
{
  ground::Condition c = a;
  auto add = [](std::vector<ground::AtomId> & to, const std::vector<ground::AtomId> & from) {
      for (ground::AtomId x : from) {
        if (std::find(to.begin(), to.end(), x) == to.end()) {
          to.push_back(x);
        }
      }
    };
  add(c.positive, b.positive);
  add(c.negative, b.negative);
  for (ground::AtomId x : c.positive) {
    if (std::find(c.negative.begin(), c.negative.end(), x) != c.negative.end()) {
      return std::nullopt;
    }
  }
  c.numeric.insert(c.numeric.end(), b.numeric.begin(), b.numeric.end());
  c.objects.insert(c.objects.end(), b.objects.begin(), b.objects.end());
  return c;
}

class Inliner
{
public:
  Inliner(const ground::Task & task, size_t max_copies)
  : task_(task), max_(max_copies)
  {
    for (const ground::Axiom & a : task.axioms) {
      rules_[a.head].push_back(&a.body);
    }
  }

  // The cases of a condition, with no derived atom in them
  Cases cases(const ground::Condition & c)
  {
    ground::Condition plain = c;
    plain.positive.clear();
    plain.negative.clear();
    Cases out{plain};
    for (ground::AtomId a : c.positive) {
      if (task_.is_derived(a)) {
        out = product(out, holds(a));
      } else {
        out = product(out, Cases{atom(a, true)});
      }
    }
    for (ground::AtomId a : c.negative) {
      if (task_.is_derived(a)) {
        out = product(out, fails(a));
      } else {
        out = product(out, Cases{atom(a, false)});
      }
    }
    return out;
  }

private:
  static ground::Condition atom(ground::AtomId a, bool positive)
  {
    ground::Condition c;
    (positive ? c.positive : c.negative).push_back(a);
    return c;
  }

  Cases product(const Cases & a, const Cases & b)
  {
    Cases out;
    for (const auto & x : a) {
      for (const auto & y : b) {
        if (auto c = both(x, y)) {
          out.push_back(std::move(*c));
          if (out.size() > max_) {
            throw Too{"more than " + std::to_string(max_) + " cases of a condition"};
          }
        }
      }
    }
    return out;
  }

  // When a derived atom holds: one of its rules does
  Cases holds(ground::AtomId a)
  {
    if (auto known = holds_.find(a); known != holds_.end()) {
      return known->second;
    }
    enter(a);
    Cases out;
    for (const ground::Condition * body : rules_[a]) {
      for (auto & c : cases(*body)) {
        out.push_back(std::move(c));
        if (out.size() > max_) {
          throw Too{"more than " + std::to_string(max_) + " cases of a derived atom"};
        }
      }
    }
    leave(a);
    return holds_[a] = out;
  }

  // When it does not: each case of each rule fails, in one of its parts
  Cases fails(ground::AtomId a)
  {
    if (auto known = fails_.find(a); known != fails_.end()) {
      return known->second;
    }
    Cases out{ground::Condition{}};
    for (const ground::Condition & c : holds(a)) {
      Cases one;
      for (ground::AtomId x : c.positive) {
        one.push_back(atom(x, false));
      }
      for (ground::AtomId x : c.negative) {
        one.push_back(atom(x, true));
      }
      for (const auto & n : c.numeric) {
        ground::Condition k;
        k.numeric.push_back(ground::NumericCondition{negated(n.op), n.lhs, n.rhs});
        one.push_back(std::move(k));
      }
      for (const auto & o : c.objects) {
        ground::Condition k;
        k.objects.push_back(ground::ObjectCondition{!o.equal, o.lhs, o.rhs});
        one.push_back(std::move(k));
      }
      if (one.empty()) {
        return fails_[a] = Cases{};  // a rule that always holds: never false
      }
      out = product(out, one);
    }
    return fails_[a] = out;
  }

  void enter(ground::AtomId a)
  {
    if (std::find(path_.begin(), path_.end(), a) != path_.end()) {
      throw Too{"derived atoms that read each other in a cycle"};
    }
    path_.push_back(a);
  }
  void leave(ground::AtomId) {path_.pop_back();}

  const ground::Task & task_;
  size_t max_;
  std::map<ground::AtomId, std::vector<const ground::Condition *>> rules_;
  std::map<ground::AtomId, Cases> holds_;
  std::map<ground::AtomId, Cases> fails_;
  std::vector<ground::AtomId> path_;
};

bool reads_derived(const ground::Task & task, const ground::Condition & c)
{
  auto derived = [&](ground::AtomId a) {return task.is_derived(a);};
  return std::any_of(c.positive.begin(), c.positive.end(), derived) ||
         std::any_of(c.negative.begin(), c.negative.end(), derived);
}

}  // namespace

std::string inline_axioms(kairos::ground::Task & task, size_t max_copies)
{
  if (task.axioms.empty()) {
    return "";
  }
  try {
    Inliner in(task, max_copies);
    for (const auto & list : {&task.actions, &task.processes, &task.events}) {
      for (const ground::Action & a : *list) {
        for (const ground::Effect & e : a.effects) {
          if (reads_derived(task, e.condition) || reads_derived(task, e.start_condition)) {
            return "derived atoms in conditional effects";
          }
        }
      }
    }
    for (const ground::DurativeAction & a : task.durative_actions) {
      for (const ground::Effect & e : a.effects) {
        if (reads_derived(task, e.condition) || reads_derived(task, e.start_condition)) {
          return "derived atoms in conditional effects";
        }
      }
    }
    std::vector<ground::Action> actions;
    for (const ground::Action & a : task.actions) {
      for (auto & c : in.cases(a.precondition)) {
        actions.push_back(a);
        actions.back().precondition = std::move(c);
      }
    }
    std::vector<ground::DurativeAction> durative;
    for (const ground::DurativeAction & a : task.durative_actions) {
      const Cases start = in.cases(a.at_start), all = in.cases(a.over_all),
        end = in.cases(a.at_end);
      for (const auto & s : start) {
        for (const auto & o : all) {
          for (const auto & e : end) {
            durative.push_back(a);
            durative.back().at_start = s;
            durative.back().over_all = o;
            durative.back().at_end = e;
            if (durative.size() > task.durative_actions.size() * max_copies) {
              return "more than " + std::to_string(max_copies) + " copies of a durative action";
            }
          }
        }
      }
    }
    Cases goal = in.cases(task.goal);
    if (goal.size() > 1) {
      return "a goal with several cases";
    }
    if (!task.processes.empty() || !task.events.empty()) {
      return "derived atoms with processes or events";
    }
    task.actions = std::move(actions);
    task.durative_actions = std::move(durative);
    if (goal.empty()) {
      task.goal_unreachable = true;
    } else {
      task.goal = std::move(goal.front());
    }
    task.axioms.clear();
    return "";
  } catch (const Too & t) {
    return t.why;
  }
}

}  // namespace klimax::tdg

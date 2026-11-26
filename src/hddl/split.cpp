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

#include "hddl/split.hpp"

#include <string>
#include <utility>
#include <vector>

namespace klimax::hddl
{

namespace
{

constexpr uint32_t kGone = 0xffffffffu;

// How many steps mention each parameter, and whether anything else does
struct Usage
{
  std::vector<uint32_t> steps;  // number of steps that mention it
  std::vector<bool> elsewhere;  // the task, the precondition or a constraint mentions it
};

Usage usage(const MethodInfo & m)
{
  Usage u;
  u.steps.assign(m.params.size(), 0);
  u.elsewhere.assign(m.params.size(), false);
  auto mark = [&](const Arg & a) {
      if (a.variable) {
        u.elsewhere[a.index] = true;
      }
    };
  for (const Arg & a : m.task_args) {
    mark(a);
  }
  for (uint32_t p : m.precondition_params) {
    u.elsewhere[p] = true;
  }
  for (const VarConstraint & c : m.network.constraints) {
    mark(c.lhs);
    mark(c.rhs);
  }
  for (const SortConstraint & c : m.network.sorts) {
    u.elsewhere[c.param] = true;
  }
  for (const Step & s : m.network.steps) {
    std::vector<bool> seen(m.params.size(), false);
    for (const Arg & a : s.args) {
      if (a.variable && !seen[a.index]) {
        seen[a.index] = true;
        ++u.steps[a.index];
      }
    }
  }
  return u;
}

// Splits the own parameters of the subtasks of `method` (a copy: `model` grows meanwhile)
size_t split_one(Model & model, MethodInfo & method)
{
  if (method.network.steps.size() < 2) {
    return 0;  // a single subtask: nothing to multiply with
  }
  const Usage u = usage(method);
  std::vector<bool> own(u.steps.size(), false);  // used by one step and nothing else
  bool any = false;
  for (size_t p = 0; p < own.size(); ++p) {
    own[p] = u.steps[p] == 1 && !u.elsewhere[p];
    any = any || own[p];
  }
  if (!any) {
    return 0;
  }
  size_t moved = 0;
  for (size_t s = 0; s < method.network.steps.size(); ++s) {
    const Step step = method.network.steps[s];
    bool has_own = false;
    for (const Arg & a : step.args) {
      has_own = has_own || (a.variable && own[a.index]);
    }
    if (!has_own) {
      continue;
    }
    const TaskInfo target = model.tasks[step.task];
    // The new task keeps the arguments that are not the step's own variables
    TaskInfo task;
    task.name = "split:" + method.name + "#" + std::to_string(s);
    task.artificial = true;
    task.loc = step.loc;
    MethodInfo chooser;
    chooser.name = task.name;
    chooser.artificial = true;
    chooser.loc = step.loc;
    Step outer = step;
    outer.args.clear();
    Step inner;
    inner.task = step.task;
    inner.loc = step.loc;
    std::vector<uint32_t> chosen(method.params.size(), kGone);  // own variable -> chooser's
    for (size_t i = 0; i < step.args.size(); ++i) {
      const Arg & a = step.args[i];
      if (a.variable && own[a.index]) {
        if (chosen[a.index] == kGone) {
          chosen[a.index] = static_cast<uint32_t>(chooser.params.size());
          chooser.params.push_back(method.params[a.index]);
        }
        inner.args.push_back(Arg{true, chosen[a.index]});
        continue;
      }
      const uint32_t param = static_cast<uint32_t>(chooser.params.size());
      chooser.params.push_back(Param{"?a" + std::to_string(i), target.params[i]});
      chooser.task_args.push_back(Arg{true, param});
      inner.args.push_back(Arg{true, param});
      task.params.push_back(target.params[i]);
      outer.args.push_back(a);
    }
    chooser.task = model.tasks.push_back(std::move(task));
    chooser.network.steps.push_back(std::move(inner));
    chooser.network.total_order = true;
    outer.task = chooser.task;
    method.network.steps[s] = std::move(outer);
    model.methods.push_back(std::move(chooser));
    ++moved;
  }
  // Drop the parameters that moved and renumber the rest
  std::vector<uint32_t> renumber(method.params.size(), kGone);
  std::vector<Param> kept;
  for (size_t p = 0; p < method.params.size(); ++p) {
    if (!own[p]) {
      renumber[p] = static_cast<uint32_t>(kept.size());
      kept.push_back(std::move(method.params[p]));
    }
  }
  method.params = std::move(kept);
  auto fix = [&](Arg & a) {
      if (a.variable) {
        a.index = renumber[a.index];
      }
    };
  for (Arg & a : method.task_args) {
    fix(a);
  }
  for (uint32_t & p : method.precondition_params) {
    p = renumber[p];
  }
  for (Step & s : method.network.steps) {
    for (Arg & a : s.args) {
      fix(a);
    }
  }
  for (VarConstraint & c : method.network.constraints) {
    fix(c.lhs);
    fix(c.rhs);
  }
  for (SortConstraint & c : method.network.sorts) {
    c.param = renumber[c.param];
  }
  return moved;
}

}  // namespace

size_t test_before(Model & model)
{
  size_t wrapped = 0;
  const size_t original = model.methods.size();
  for (size_t index = 0; index < original; ++index) {
    for (size_t k = 0; k < model.methods[index].network.states.size(); ++k) {
      const StateConstraint c = model.methods[index].network.states[k];
      const bool before = c.kind == ConstraintKind::Before || c.kind == ConstraintKind::Between;
      if (!before || c.action < 0) {
        continue;
      }
      const uint32_t target = c.kind == ConstraintKind::Before ? c.a : c.b;
      const MethodInfo method = model.methods[index];
      const Step step = method.network.steps[target];
      // A task with every parameter of the method: its condition and its subtask use them
      TaskInfo task;
      task.name = "before:" + method.name + "#" + std::to_string(k);
      task.artificial = true;
      task.loc = step.loc;
      MethodInfo test;
      test.name = task.name;
      test.artificial = true;
      test.loc = step.loc;
      test.params = method.params;
      Step outer;
      outer.id = step.id;
      outer.loc = step.loc;
      for (uint32_t p = 0; p < method.params.size(); ++p) {
        task.params.push_back(method.params[p].types);
        test.task_args.push_back(Arg{true, p});
        outer.args.push_back(Arg{true, p});
      }
      test.precondition_action = c.action;
      test.precondition_params = c.action_params;
      test.network.steps.push_back(step);
      test.network.total_order = true;
      test.task = model.tasks.push_back(std::move(task));
      outer.task = test.task;
      model.methods[index].network.steps[target] = std::move(outer);
      model.methods.push_back(std::move(test));
      ++wrapped;
    }
  }
  model.methods_of.assign(model.tasks.size(), {});
  for (size_t i = 0; i < model.methods.size(); ++i) {
    model.methods_of[model.methods[i].task.index()].push_back(static_cast<uint32_t>(i));
  }
  return wrapped;
}

size_t split_parameters(Model & model)
{
  size_t moved = 0;
  const size_t original = model.methods.size();
  for (size_t index = 0; index < original; ++index) {
    MethodInfo method = model.methods[index];
    if (size_t n = split_one(model, method)) {
      moved += n;
      model.methods[index] = std::move(method);
    }
  }
  // The initial task network, as a method of nothing
  MethodInfo initial;
  initial.name = "initial";
  initial.params = model.initial_params;
  initial.network = model.initial;
  if (size_t n = split_one(model, initial)) {
    moved += n;
    model.initial_params = std::move(initial.params);
    model.initial = std::move(initial.network);
  }
  model.methods_of.assign(model.tasks.size(), {});
  for (size_t i = 0; i < model.methods.size(); ++i) {
    model.methods_of[model.methods[i].task.index()].push_back(static_cast<uint32_t>(i));
  }
  return moved;
}

}  // namespace klimax::hddl

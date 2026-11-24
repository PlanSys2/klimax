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

#include "tdg/grounder.hpp"

#include <algorithm>
#include <chrono>
#include <map>
#include <memory>
#include <span>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ground/datalog.hpp"
#include "ground/grounder.hpp"
#include "normalize/normalizer.hpp"

namespace klimax::tdg
{

namespace
{

namespace sem = kairos::sem;
namespace datalog = kairos::datalog;
using hddl::Arg;
using hddl::Model;
using hddl::TaskId;

using Row = std::vector<uint32_t>;

struct RowHash
{
  size_t operator()(const Row & r) const noexcept
  {
    size_t h = 1469598103934665603ull;
    for (uint32_t v : r) {
      h = (h ^ v) * 1099511628211ull;
    }
    return h;
  }
};

struct OutOfBudget
{
};

class Grounder
{
public:
  Grounder(const Model & model, kairos::Budget * budget)
  : m_(model), budget_(budget) {}

  kairos::Result<Problem> run()
  {
    const auto start = std::chrono::steady_clock::now();
    auto lifted = kairos::lifted::normalize(m_.pddl, m_.domain, m_.problem, m_.objects);
    kairos::ground::GroundOptions options;
    options.drop_useless_actions = false;  // the method preconditions are such actions
    auto grounded = kairos::ground::ground(lifted, options, budget_);
    if (!grounded.ok()) {
      return kairos::Result<Problem>::failure(grounded.diagnostics());
    }
    p_.pddl = std::move(grounded).value();
    p_.stats.kairos_actions = p_.pddl.actions.size();
    try {
      program();
      if (!prog_.solve(budget_)) {
        throw OutOfBudget{};
      }
      collect();
    } catch (const OutOfBudget &) {
      return kairos::Result<Problem>::failure(
        {kairos::Diagnostic::error("ground.budget", "the task is too large to ground in the "
          "time or memory given")});
    }
    p_.stats.milliseconds = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
    return kairos::Result<Problem>::success(std::move(p_));
  }

private:
  void tick()
  {
    if (budget_ != nullptr && budget_->exhausted()) {
      throw OutOfBudget{};
    }
  }

  const sem::ObjectInfo & object(uint32_t o) const
  {
    return m_.objects.objects.items[sem::ObjectId(o)];
  }

  bool fits(uint32_t o, const sem::TypeSpec & types) const
  {
    return types.empty() || m_.domain.types.fits(object(o).types, types);
  }

  // The unary predicate of the objects of a type, for parameters nothing else binds
  uint32_t objects_of(const sem::TypeSpec & types)
  {
    std::vector<uint32_t> key;
    for (sem::TypeId t : types) {
      key.push_back(t.value());
    }
    auto [it, fresh] = type_predicate_.emplace(key, 0);
    if (fresh) {
      it->second = prog_.add_predicate(1, "objects");
      for (uint32_t o = 0; o < m_.objects.objects.items.size(); ++o) {
        if (fits(o, types)) {
          prog_.add_fact(it->second, std::span<const uint32_t>(&o, 1));
        }
      }
      prog_.mark_static(it->second);
    }
    return it->second;
  }

  static datalog::Atom atom(uint32_t predicate, const std::vector<Arg> & args)
  {
    datalog::Atom a;
    a.predicate = predicate;
    for (const Arg & arg : args) {
      a.args.push_back(arg.variable ? datalog::Term::var(arg.index) :
        datalog::Term::constant(arg.index));
    }
    return a;
  }

  // Derivable, bottom-up: a method instance when its precondition is reachable and all its
  // subtasks are derivable; an abstract task when some method of it is; an action when Kairos
  // reached it
  void program()
  {
    // Facts: the instances of each action schema of Kairos' task
    std::unordered_map<std::string, uint32_t> schema_by_name;
    for (uint32_t s = 0; s < p_.pddl.action_names.size(); ++s) {
      schema_by_name.emplace(p_.pddl.action_names[s], s);
    }
    instances_.resize(p_.pddl.action_names.size());
    for (uint32_t a = 0; a < p_.pddl.actions.size(); ++a) {
      Row row;
      for (sem::ObjectId o : p_.pddl.actions[a].args) {
        row.push_back(o.value());
      }
      instances_[p_.pddl.actions[a].schema][row].push_back(a);
    }
    auto relation = [&](kairos::Symbol name, size_t arity, uint32_t & schema) {
        const uint32_t predicate = prog_.add_predicate(static_cast<uint32_t>(arity));
        auto it = schema_by_name.find(std::string(m_.pddl.symbols.name(name)));
        schema = it == schema_by_name.end() ? kNone : it->second;
        if (schema != kNone) {
          for (const auto & [row, actions] : instances_[schema]) {
            prog_.add_fact(predicate, row);
          }
        }
        prog_.mark_static(predicate);
        return predicate;
      };
    task_predicate_.resize(m_.tasks.size());
    task_schema_.assign(m_.tasks.size(), kNone);
    for (uint32_t t = 0; t < m_.tasks.size(); ++t) {
      const hddl::TaskInfo & info = m_.tasks[TaskId(t)];
      if (info.primitive && !info.durative) {
        task_predicate_[t] = relation(
          m_.pddl.actions[info.action].name, info.params.size(), task_schema_[t]);
      }
    }
    precondition_predicate_.assign(m_.methods.size(), kNone);
    precondition_schema_.assign(m_.methods.size(), kNone);
    for (uint32_t i = 0; i < m_.methods.size(); ++i) {
      const hddl::MethodInfo & method = m_.methods[i];
      if (method.precondition_action >= 0) {
        precondition_predicate_[i] = relation(
          m_.pddl.actions[static_cast<size_t>(method.precondition_action)].name,
          method.precondition_params.size(), precondition_schema_[i]);
      }
    }
    for (uint32_t t = 0; t < m_.tasks.size(); ++t) {
      const hddl::TaskInfo & info = m_.tasks[TaskId(t)];
      if (!info.primitive || info.durative) {
        task_predicate_[t] = prog_.add_predicate(static_cast<uint32_t>(info.params.size()));
      }
    }
    // Rules. Only what the initial network can ask for is derived: need(task, bound arguments)
    // goes down from it, and a method instance is derived only for a needed task.
    for (const hddl::MethodInfo & method : m_.methods) {
      method_predicate_.push_back(
        prog_.add_predicate(static_cast<uint32_t>(method.params.size())));
    }
    initial_predicate_ = prog_.add_predicate(static_cast<uint32_t>(m_.initial_params.size()));
    instance_rule(initial_predicate_, m_.initial_params, m_.initial, nullptr, kNone, {});
    needs_below(m_.initial_params, m_.initial, nullptr, kNone, {});
    while (!pending_.empty()) {
      tick();
      const auto [task, mask] = pending_.back();
      pending_.pop_back();
      const uint32_t need = need_predicate_.at({task, mask});
      for (uint32_t i : m_.methods_of[task]) {
        const hddl::MethodInfo & method = m_.methods[i];
        datalog::Atom needed;
        needed.predicate = need;
        for (size_t c = 0; c < method.task_args.size(); ++c) {
          if ((mask >> c) & 1u) {
            const Arg & a = method.task_args[c];
            needed.args.push_back(a.variable ? datalog::Term::var(a.index) :
              datalog::Term::constant(a.index));
          }
        }
        instance_rule(method_predicate_[i], method.params, method.network, &needed,
          precondition_predicate_[i], method.precondition_params);
        needs_below(method.params, method.network, &needed, precondition_predicate_[i],
          method.precondition_params);
      }
    }
    for (uint32_t i = 0; i < m_.methods.size(); ++i) {
      const hddl::MethodInfo & method = m_.methods[i];
      datalog::Rule up;
      up.head = atom(task_predicate_[method.task.index()], method.task_args);
      up.variables = static_cast<uint32_t>(method.params.size());
      datalog::Atom all;
      all.predicate = method_predicate_[i];
      for (uint32_t p = 0; p < method.params.size(); ++p) {
        all.args.push_back(datalog::Term::var(p));
      }
      up.body.push_back(std::move(all));
      prog_.add_rule(std::move(up));
    }
  }

  bool is_abstract(TaskId id) const
  {
    return !m_.tasks[id].primitive || m_.tasks[id].durative;
  }

  // need(subtask, its arguments that are known) :- need(task), precondition, actions.
  // The arguments known are constants and the variables that the task asked for, the
  // precondition and the primitive subtasks bind.
  void needs_below(
    const std::vector<hddl::Param> & params, const hddl::Network & net,
    const datalog::Atom * needed, uint32_t precondition,
    const std::vector<uint32_t> & precondition_params)
  {
    std::vector<datalog::Atom> body;
    std::vector<bool> bound(params.size(), false);
    if (needed != nullptr) {
      body.push_back(*needed);
    }
    if (precondition != kNone) {
      datalog::Atom a;
      a.predicate = precondition;
      for (uint32_t p : precondition_params) {
        a.args.push_back(datalog::Term::var(p));
      }
      body.push_back(std::move(a));
    }
    for (const hddl::Step & step : net.steps) {
      if (!is_abstract(step.task)) {
        body.push_back(atom(task_predicate_[step.task.index()], step.args));
      }
    }
    for (const datalog::Atom & a : body) {
      for (const datalog::Term & t : a.args) {
        if (t.is_variable) {
          bound[t.value] = true;
        }
      }
    }
    for (const hddl::Step & step : net.steps) {
      if (!is_abstract(step.task)) {
        continue;
      }
      uint64_t mask = 0;
      datalog::Rule rule;
      rule.variables = static_cast<uint32_t>(params.size());
      for (size_t c = 0; c < step.args.size() && c < 64; ++c) {
        const Arg & a = step.args[c];
        if (!a.variable || bound[a.index]) {
          mask |= uint64_t{1} << c;
          rule.head.args.push_back(a.variable ? datalog::Term::var(a.index) :
            datalog::Term::constant(a.index));
        }
      }
      auto [it, fresh] = need_predicate_.emplace(
        std::make_pair(step.task.value(), mask), 0);
      if (fresh) {
        it->second = prog_.add_predicate(static_cast<uint32_t>(rule.head.args.size()));
        pending_.emplace_back(step.task.value(), mask);
      }
      rule.head.predicate = it->second;
      if (body.empty()) {
        // Asked for unconditionally: its known arguments are constants
        Row constants;
        for (const datalog::Term & t : rule.head.args) {
          constants.push_back(t.value);
        }
        prog_.add_fact(it->second, constants);
        continue;
      }
      rule.body = body;
      prog_.add_rule(std::move(rule));
    }
  }

  // instance(params) :- need, precondition, subtasks, types and constraints
  void instance_rule(
    uint32_t predicate, const std::vector<hddl::Param> & params, const hddl::Network & net,
    const datalog::Atom * needed, uint32_t precondition,
    const std::vector<uint32_t> & precondition_params)
  {
    const uint32_t n = static_cast<uint32_t>(params.size());
    datalog::Rule rule;
    rule.variables = n;
    rule.head.predicate = predicate;
    std::vector<bool> bound(n, false);
    for (uint32_t p = 0; p < n; ++p) {
      rule.head.args.push_back(datalog::Term::var(p));
    }
    if (needed != nullptr) {
      rule.body.push_back(*needed);
      for (const datalog::Term & t : needed->args) {
        if (t.is_variable) {
          bound[t.value] = true;
        }
      }
    }
    if (precondition != kNone) {
      datalog::Atom a;
      a.predicate = precondition;
      for (uint32_t p : precondition_params) {
        a.args.push_back(datalog::Term::var(p));
        bound[p] = true;
      }
      rule.body.push_back(std::move(a));
    }
    for (const hddl::Step & step : net.steps) {
      rule.body.push_back(atom(task_predicate_[step.task.index()], step.args));
      for (const Arg & arg : step.args) {
        if (arg.variable) {
          bound[arg.index] = true;
        }
      }
    }
    for (uint32_t p = 0; p < n; ++p) {
      if (!bound[p]) {
        // Nothing gives it a value: every object of its type
        datalog::Atom a;
        a.predicate = objects_of(params[p].types);
        a.args.push_back(datalog::Term::var(p));
        rule.body.push_back(std::move(a));
        continue;
      }
      // A subtask may allow more than the parameter's own type does
      auto allowed = std::make_shared<std::vector<bool>>();
      bool all = true;
      for (uint32_t o = 0; o < m_.objects.objects.items.size(); ++o) {
        allowed->push_back(fits(o, params[p].types));
        all = all && allowed->back();
      }
      if (!all) {
        rule.checks.push_back(datalog::Check{{p}, [allowed, p](const uint32_t * b) {
              return (*allowed)[b[p]];
            }});
      }
    }
    for (const hddl::VarConstraint & c : net.constraints) {
      const Arg l = c.lhs;
      const Arg r = c.rhs;
      const bool equal = c.equal;
      if (!l.variable && !r.variable) {
        if ((l.index == r.index) != equal) {
          return;  // never holds: no rule
        }
        continue;
      }
      std::vector<uint32_t> variables;
      for (const Arg & a : {l, r}) {
        if (a.variable) {
          variables.push_back(a.index);
        }
      }
      rule.checks.push_back(datalog::Check{variables, [l, r, equal](const uint32_t * b) {
            const uint32_t lv = l.variable ? b[l.index] : l.index;
            const uint32_t rv = r.variable ? b[r.index] : r.index;
            return (lv == rv) == equal;
          }});
    }
    for (const hddl::SortConstraint & c : net.sorts) {
      auto allowed = std::make_shared<std::vector<bool>>();
      for (uint32_t o = 0; o < m_.objects.objects.items.size(); ++o) {
        allowed->push_back(fits(o, c.types) != c.negated);
      }
      const uint32_t p = c.param;
      rule.checks.push_back(datalog::Check{{p}, [allowed, p](const uint32_t * b) {
            return (*allowed)[b[p]];
          }});
    }
    if (rule.body.empty()) {
      // No parameters, no precondition, no subtasks: it simply is
      prog_.add_fact(predicate, Row{});
      return;
    }
    prog_.add_rule(std::move(rule));
  }

  uint32_t task(TaskId id, const Row & args)
  {
    Row key{id.value()};
    key.insert(key.end(), args.begin(), args.end());
    auto [it, fresh] = task_index_.emplace(std::move(key), static_cast<uint32_t>(p_.tasks.size()));
    if (fresh) {
      Task t;
      t.task = id;
      t.args = args;
      const uint32_t schema = task_schema_[id.index()];
      const hddl::TaskInfo & info = m_.tasks[id];
      if (info.primitive && !info.durative) {
        if (schema != kNone) {
          t.actions = instances_[schema].at(args);
        }
        ++p_.primitive_tasks;
      } else {
        queue_.push_back(it->second);
      }
      p_.tasks.push_back(std::move(t));
    }
    return it->second;
  }

  uint32_t method(
    uint32_t index, uint32_t parent, const hddl::Network & net, std::span<const uint32_t> binding)
  {
    Method out;
    out.method = index;
    out.task = parent;
    out.params.assign(binding.begin(), binding.end());
    if (index != kNone && precondition_predicate_[index] != kNone) {
      out.has_precondition = true;
      Row row;
      for (uint32_t p : m_.methods[index].precondition_params) {
        row.push_back(binding[p]);
      }
      out.precondition = instances_[precondition_schema_[index]].at(row);
    }
    for (const hddl::Step & step : net.steps) {
      Row args;
      for (const Arg & a : step.args) {
        args.push_back(a.variable ? binding[a.index] : a.index);
      }
      out.subtasks.push_back(task(step.task, args));
    }
    p_.methods.push_back(std::move(out));
    return static_cast<uint32_t>(p_.methods.size() - 1);
  }

  // From the initial network down, through the derivable instances only
  void collect()
  {
    // The derivable instances of each method, by the ground task they decompose
    std::unordered_map<Row, std::vector<std::pair<uint32_t, uint32_t>>, RowHash> by_task;
    for (uint32_t i = 0; i < m_.methods.size(); ++i) {
      const hddl::MethodInfo & info = m_.methods[i];
      p_.stats.methods_built += prog_.facts(method_predicate_[i]).size();
      for (uint32_t fact : prog_.facts(method_predicate_[i])) {
        tick();
        std::span<const uint32_t> binding = prog_.args(fact);
        Row key{info.task.value()};
        for (const Arg & a : info.task_args) {
          key.push_back(a.variable ? binding[a.index] : a.index);
        }
        by_task[std::move(key)].emplace_back(i, fact);
      }
    }
    p_.stats.tasks_built = by_task.size();
    for (uint32_t fact : prog_.facts(initial_predicate_)) {
      p_.initial.push_back(method(kNone, kNone, m_.initial, prog_.args(fact)));
    }
    p_.unsolvable = p_.initial.empty();
    while (!queue_.empty()) {
      tick();
      const uint32_t at = queue_.back();
      queue_.pop_back();
      Row key{p_.tasks[at].task.value()};
      key.insert(key.end(), p_.tasks[at].args.begin(), p_.tasks[at].args.end());
      auto found = by_task.find(key);
      if (found == by_task.end()) {
        continue;  // cannot happen: a subtask of a derivable instance is derivable
      }
      for (auto [index, fact] : found->second) {
        const uint32_t id = method(index, at, m_.methods[index].network, prog_.args(fact));
        p_.tasks[at].methods.push_back(id);
      }
    }
  }

  const Model & m_;
  kairos::Budget * budget_;
  Problem p_;
  datalog::Program prog_;
  std::vector<std::unordered_map<Row, std::vector<uint32_t>, RowHash>> instances_;  // by schema
  std::vector<uint32_t> task_predicate_;          // by TaskId
  std::vector<uint32_t> task_schema_;             // by TaskId: kNone if abstract or unreached
  std::vector<uint32_t> precondition_predicate_;  // by method: kNone if it has no precondition
  std::vector<uint32_t> precondition_schema_;
  std::vector<uint32_t> method_predicate_;
  uint32_t initial_predicate_ = 0;
  std::map<std::pair<uint32_t, uint64_t>, uint32_t> need_predicate_;  // by task and known args
  std::vector<std::pair<uint32_t, uint64_t>> pending_;
  std::map<std::vector<uint32_t>, uint32_t> type_predicate_;
  std::unordered_map<Row, uint32_t, RowHash> task_index_;
  std::vector<uint32_t> queue_;                   // abstract ground tasks to decompose
};

}  // namespace

kairos::Result<Problem> ground(const hddl::Model & model, kairos::Budget * budget)
{
  return Grounder(model, budget).run();
}

}  // namespace klimax::tdg

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
#include <cstdio>
#include <cstdlib>
#include <map>
#include <memory>
#include <span>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "ground/datalog.hpp"
#include "ground/grounder.hpp"
#include "hddl/split.hpp"
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
  : owned_(model), m_(owned_), budget_(budget)
  {
    split_ = hddl::split_parameters(owned_);
  }

  kairos::Result<Problem> run()
  {
    const auto start = std::chrono::steady_clock::now();
    auto since = [&]() {
        return std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - start).count();
      };
    lifted_ = kairos::lifted::normalize(m_.pddl, m_.domain, m_.problem, m_.objects);
    try {
      program();
      const bool solved = prog_.solve(budget_);
      p_.stats.datalog_ms = since();
      if (trace_) {
        std::fprintf(stderr, "datalog: %zu facts, %zu rules, %zu probes, %.0f ms%s\n",
          prog_.stats().facts, prog_.stats().rules, prog_.stats().join_probes,
          p_.stats.datalog_ms, solved ? "" : " (out of budget)");
        std::vector<std::pair<size_t, uint32_t>> sizes;
        for (uint32_t q = 0; q < prog_.predicates(); ++q) {
          sizes.emplace_back(prog_.facts(q).size(), q);
        }
        std::sort(sizes.rbegin(), sizes.rend());
        for (size_t i = 0; i < sizes.size() && i < 6 && sizes[i].first > 20000; ++i) {
          std::fprintf(stderr, "  %s/%u: %zu facts\n", prog_.name(sizes[i].second).c_str(),
            prog_.arity(sizes[i].second), sizes[i].first);
        }
      }
      if (const char * wanted = std::getenv("KLIMAX_TRACE_PREDICATE")) {
        // The facts of the predicates whose name contains the text given
        for (uint32_t q = 0; q < prog_.predicates(); ++q) {
          if (prog_.name(q).find(wanted) == std::string::npos) {
            continue;
          }
          std::fprintf(stderr, "%s/%u: %zu facts\n", prog_.name(q).c_str(), prog_.arity(q),
            prog_.facts(q).size());
          for (size_t f = 0; f < prog_.facts(q).size() && f < 60; ++f) {
            std::string text = "   ";
            for (uint32_t o : prog_.args(prog_.facts(q)[f])) {
              text += " " + object(o).name;
            }
            std::fprintf(stderr, "%s\n", text.c_str());
          }
        }
      }
      if (trace_) {
        // Asked for but never derived: where the hierarchy stops
        for (const auto & [key, need] : need_predicate_) {
          const uint32_t derived = task_predicate_[key.first];
          if (!prog_.facts(need).empty() && prog_.facts(derived).empty()) {
            std::fprintf(stderr, "  never derived: %s (asked %zu times)\n",
              prog_.name(derived).c_str(), prog_.facts(need).size());
            for (size_t f = 0; f < prog_.facts(need).size() && f < 6; ++f) {
              std::string text = "    with";
              for (uint32_t o : prog_.args(prog_.facts(need)[f])) {
                text += " " + object(o).name;
              }
              std::fprintf(stderr, "%s\n", text.c_str());
            }
          }
        }
        for (uint32_t i = 0; i < m_.methods.size(); ++i) {
          if (precondition_predicate_[i] != kNone &&
            prog_.facts(precondition_predicate_[i]).empty())
          {
            std::fprintf(stderr, "  precondition never holds: %s\n", m_.methods[i].name.c_str());
          }
        }
      }
      if (!solved) {
        throw OutOfBudget{};
      }
      // Kairos grounds only the action instances that what the initial network reaches uses
      collect();
      guard();
      kairos::ground::GroundOptions options;
      options.drop_useless_actions = false;  // the method preconditions are such actions
      auto grounded = kairos::ground::ground(lifted_, options, budget_);
      if (!grounded.ok()) {
        return kairos::Result<Problem>::failure(grounded.diagnostics());
      }
      p_.pddl = std::move(grounded).value();
      p_.stats.kairos_actions = p_.pddl.actions.size();
      p_.stats.kairos_ms = since() - p_.stats.datalog_ms;
      if (trace_) {
        std::fprintf(stderr, "kairos: %zu actions, %zu atoms, %.0f ms\n", p_.pddl.actions.size(),
          p_.pddl.atoms.size(), p_.stats.kairos_ms);
      }
      instances();
      attach();
      prune();
      if (trace_) {
        std::fprintf(stderr, "collect: %zu tasks, %zu methods, %.0f ms\n", p_.tasks.size(),
          p_.methods.size(), since() - p_.stats.kairos_ms - p_.stats.datalog_ms);
      }
    } catch (const OutOfBudget &) {
      return kairos::Result<Problem>::failure(
        {kairos::Diagnostic::error("ground.budget", "the task is too large to ground in the "
          "time or memory given")});
    }
    p_.stats.milliseconds = since();
    p_.stats.split_subtasks = split_;
    p_.model = std::move(owned_);
    return kairos::Result<Problem>::success(std::move(p_));
  }

private:
  using Lifted = kairos::lifted::Action;

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

  bool is_abstract(TaskId id) const
  {
    return !m_.tasks[id].primitive || m_.tasks[id].durative;
  }

  // The unary predicate of the objects of a type, for variables nothing else binds
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

  // A test that a variable has one of `types`; nothing if every object does
  void type_check(datalog::Rule & rule, uint32_t variable, const sem::TypeSpec & types)
  {
    auto allowed = std::make_shared<std::vector<bool>>();
    bool all = true;
    for (uint32_t o = 0; o < m_.objects.objects.items.size(); ++o) {
      allowed->push_back(fits(o, types));
      all = all && allowed->back();
    }
    if (!all) {
      rule.checks.push_back(datalog::Check{{variable}, [allowed, variable](const uint32_t * b) {
            return (*allowed)[b[variable]];
          }});
    }
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

  // An atom of the lifted task; false if it has a function term (then it is left out, which
  // only makes the result larger)
  bool atom(const kairos::lifted::Atom & in, datalog::Atom & out) const
  {
    out.predicate = fact_[in.predicate.index()];
    out.args.clear();
    for (const kairos::lifted::Term & t : in.args) {
      if (t.kind == kairos::lifted::Term::Kind::Function) {
        return false;
      }
      out.args.push_back(t.kind == kairos::lifted::Term::Kind::Variable ?
        datalog::Term::var(t.index) : datalog::Term::constant(t.index));
    }
    return true;
  }

  // Adds a rule as a chain of joins that starts at its first condition (what is asked for, when
  // there is one) and takes next the condition that shares most variables with what is
  // already joined. Left to itself the engine may start by crossing two unrelated tables.
  void add_ordered(datalog::Rule rule)
  {
    if (rule.body.size() <= 2) {
      prog_.add_rule(std::move(rule));
      return;
    }
    const uint32_t n = rule.variables;
    auto variables = [n](const datalog::Atom & a) {
        std::vector<bool> v(n, false);
        for (const datalog::Term & t : a.args) {
          if (t.is_variable) {
            v[t.value] = true;
          }
        }
        return v;
      };
    std::vector<datalog::Atom> rest(rule.body.begin() + 1, rule.body.end());
    std::vector<datalog::Check> checks = std::move(rule.checks);
    datalog::Atom current = rule.body[0];
    std::vector<bool> have = variables(current);
    while (!rest.empty()) {
      size_t best = 0;
      int best_shared = -1;
      for (size_t i = 0; i < rest.size(); ++i) {
        int shared = 0;
        for (const datalog::Term & t : rest[i].args) {
          shared += t.is_variable && have[t.value] ? 1 : 0;
        }
        if (shared > best_shared) {
          best_shared = shared;
          best = i;
        }
      }
      datalog::Atom next = std::move(rest[best]);
      rest.erase(rest.begin() + static_cast<std::ptrdiff_t>(best));
      for (const datalog::Term & t : next.args) {
        if (t.is_variable) {
          have[t.value] = true;
        }
      }
      datalog::Rule step;
      step.variables = n;
      step.body.push_back(std::move(current));
      step.body.push_back(std::move(next));
      // The checks whose variables are all joined by now
      for (auto it = checks.begin(); it != checks.end(); ) {
        if (std::all_of(it->variables.begin(), it->variables.end(),
          [&](uint32_t v) {return have[v];}))
        {
          step.checks.push_back(std::move(*it));
          it = checks.erase(it);
        } else {
          ++it;
        }
      }
      if (rest.empty()) {
        step.head = rule.head;
        prog_.add_rule(std::move(step));
        return;
      }
      // What the rest of the chain still needs
      std::vector<bool> later = variables(rule.head);
      for (const datalog::Atom & a : rest) {
        for (const datalog::Term & t : a.args) {
          if (t.is_variable) {
            later[t.value] = true;
          }
        }
      }
      for (const datalog::Check & c : checks) {
        for (uint32_t v : c.variables) {
          later[v] = true;
        }
      }
      datalog::Atom partial;
      for (uint32_t v = 0; v < n; ++v) {
        if (have[v] && later[v]) {
          partial.args.push_back(datalog::Term::var(v));
        }
      }
      partial.predicate = prog_.add_predicate(
        static_cast<uint32_t>(partial.args.size()), "part of " + prog_.name(rule.head.predicate));
      step.head = partial;
      prog_.add_rule(std::move(step));
      current = std::move(partial);
      have = variables(current);
    }
  }

  static void mark(const datalog::Atom & a, std::vector<bool> & bound)
  {
    for (const datalog::Term & t : a.args) {
      if (t.is_variable && t.value < bound.size()) {
        bound[t.value] = true;
      }
    }
  }

  // One Datalog program for everything, so that each part limits the others:
  // - need(task, known arguments): what the initial network can ask for, top-down;
  // - act(action instance): asked for and with its positive preconditions reachable;
  // - the facts those actions add (relaxed reachability, only of what is asked for);
  // - method instances and tasks that can be decomposed into such actions, bottom-up.
  void program()
  {
    // Facts of the planning problem
    const size_t predicates = lifted_.predicate_names.size();
    std::vector<bool> changes(predicates, false);
    for (const Lifted & a : lifted_.actions) {
      for (const kairos::lifted::Effect & e : a.effects) {
        if (e.kind == kairos::lifted::Effect::Kind::Add) {
          changes[e.atom.predicate.index()] = true;
        }
      }
    }
    for (const kairos::lifted::Axiom & x : lifted_.axioms) {
      changes[x.head.index()] = true;
    }
    for (size_t q = 0; q < predicates; ++q) {
      fact_.push_back(prog_.add_predicate(
          static_cast<uint32_t>(lifted_.predicate_params[q].size()), lifted_.predicate_names[q]));
    }
    for (const auto & [predicate, args] : lifted_.init_facts) {
      Row row;
      for (sem::ObjectId o : args) {
        row.push_back(o.value());
      }
      prog_.add_fact(fact_[predicate.index()], row);
    }
    for (size_t q = 0; q < predicates; ++q) {
      if (!changes[q]) {
        prog_.mark_static(fact_[q]);
      }
    }
    // Predicates of the hierarchy
    for (uint32_t k = 0; k < lifted_.actions.size(); ++k) {
      const Lifted & a = lifted_.actions[k];
      copies_[a.name].push_back(k);
      act_.push_back(prog_.add_predicate(a.parameters + a.hidden, "act " + a.name));
    }
    for (uint32_t t = 0; t < m_.tasks.size(); ++t) {
      const hddl::TaskInfo & info = m_.tasks[TaskId(t)];
      task_predicate_.push_back(prog_.add_predicate(
          static_cast<uint32_t>(info.params.size()), "task " + info.name));
    }
    for (const hddl::MethodInfo & method : m_.methods) {
      method_predicate_.push_back(prog_.add_predicate(
          static_cast<uint32_t>(method.params.size()), "method " + method.name));
      precondition_predicate_.push_back(method.precondition_action < 0 ? kNone :
        prog_.add_predicate(static_cast<uint32_t>(method.precondition_params.size()),
        "precondition " + method.name));
    }
    initial_predicate_ = prog_.add_predicate(
      static_cast<uint32_t>(m_.initial_params.size()), "initial");

    // Rules that do not depend on what is asked for
    for (uint32_t k = 0; k < lifted_.actions.size(); ++k) {
      effects(k);
    }
    for (const kairos::lifted::Axiom & x : lifted_.axioms) {
      axiom(x);
    }
    for (uint32_t t = 0; t < m_.tasks.size(); ++t) {
      const hddl::TaskInfo & info = m_.tasks[TaskId(t)];
      if (info.primitive && !info.durative) {
        projections(action_name(info.action), task_predicate_[t]);
      }
    }
    for (uint32_t i = 0; i < m_.methods.size(); ++i) {
      const hddl::MethodInfo & method = m_.methods[i];
      if (method.precondition_action >= 0) {
        projections(action_name(static_cast<uint32_t>(method.precondition_action)),
          precondition_predicate_[i]);
      }
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

    // From the initial network down
    instance_rule(initial_predicate_, m_.initial_params, m_.initial, nullptr, kNone, {});
    needs_below(m_.initial_params, m_.initial, nullptr, kNone, {});
    while (!pending_.empty()) {
      tick();
      const auto [task, mask] = pending_.back();
      pending_.pop_back();
      const uint32_t need = need_predicate_.at({task, mask});
      const hddl::TaskInfo & info = m_.tasks[TaskId(task)];
      if (info.primitive) {
        if (!info.durative) {
          // The parameters of an action are its first variables
          datalog::Atom needed;
          needed.predicate = need;
          for (uint32_t c = 0; c < info.params.size(); ++c) {
            if ((mask >> c) & 1u) {
              needed.args.push_back(datalog::Term::var(c));
            }
          }
          for (uint32_t k : copies_[action_name(info.action)]) {
            applicable(k, &needed, 0);
          }
        }
        continue;
      }
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
        if (method.precondition_action >= 0) {
          // The same need, in the variables of the precondition's action: its parameters are
          // precondition_params, in that order; the others are of no use to it
          datalog::Atom mine = needed;
          uint32_t extra = 0;
          for (datalog::Term & t : mine.args) {
            if (!t.is_variable) {
              continue;
            }
            auto at = std::find(method.precondition_params.begin(),
                method.precondition_params.end(), t.value);
            t.value = at != method.precondition_params.end() ?
              static_cast<uint32_t>(at - method.precondition_params.begin()) :
              kAnonymous + extra++;
          }
          for (uint32_t k :
            copies_[action_name(static_cast<uint32_t>(method.precondition_action))])
          {
            applicable(k, &mine, extra);
          }
        }
      }
    }
  }

  std::string action_name(uint32_t action) const
  {
    return std::string(m_.pddl.symbols.name(m_.pddl.actions[action].name));
  }

  // act(parameters, hidden) :- need, positive preconditions. Variables numbered kAnonymous
  // and up in `needed` (there are `extra` of them) are renumbered after the action's own.
  void applicable(uint32_t k, const datalog::Atom * needed, uint32_t extra)
  {
    const Lifted & a = lifted_.actions[k];
    const uint32_t own = static_cast<uint32_t>(a.variables.size());
    const uint32_t head = a.parameters + a.hidden;
    datalog::Rule rule;
    rule.variables = own + extra;
    rule.head.predicate = act_[k];
    for (uint32_t v = 0; v < head; ++v) {
      rule.head.args.push_back(datalog::Term::var(v));
    }
    std::vector<bool> bound(own, false);
    if (needed != nullptr) {
      datalog::Atom n = *needed;
      for (datalog::Term & t : n.args) {
        if (t.is_variable && t.value >= kAnonymous) {
          t.value = own + (t.value - kAnonymous);
        }
      }
      mark(n, bound);
      rule.body.push_back(std::move(n));
    }
    for (const kairos::lifted::Literal & l : a.precondition.literals) {
      datalog::Atom b;
      if (l.positive && atom(l.atom, b)) {
        mark(b, bound);
        rule.body.push_back(std::move(b));
      }
    }
    for (uint32_t v = 0; v < head; ++v) {
      if (!bound[v]) {
        datalog::Atom b;
        b.predicate = objects_of(a.variables[v].types);
        b.args.push_back(datalog::Term::var(v));
        rule.body.push_back(std::move(b));
        bound[v] = true;
      } else {
        type_check(rule, v, a.variables[v].types);
      }
    }
    using kairos::lifted::CompareOp;
    using Kind = kairos::lifted::Term::Kind;
    for (const kairos::lifted::Comparison & c : a.precondition.comparisons) {
      if (!c.objects || (c.op != CompareOp::Eq && c.op != CompareOp::Ne) ||
        c.lhs_term.kind == Kind::Function || c.rhs_term.kind == Kind::Function)
      {
        continue;
      }
      const kairos::lifted::Term l = c.lhs_term;
      const kairos::lifted::Term r = c.rhs_term;
      std::vector<uint32_t> variables;
      bool usable = true;
      for (const kairos::lifted::Term & t : {l, r}) {
        if (t.kind == Kind::Variable) {
          usable = usable && t.index < own && bound[t.index];
          variables.push_back(t.index);
        }
      }
      if (!usable) {
        continue;
      }
      const bool equal = c.op == CompareOp::Eq;
      rule.checks.push_back(datalog::Check{variables, [l, r, equal](const uint32_t * b) {
            const uint32_t lv = l.kind == Kind::Variable ? b[l.index] : l.index;
            const uint32_t rv = r.kind == Kind::Variable ? b[r.index] : r.index;
            return (lv == rv) == equal;
          }});
    }
    if (rule.body.empty()) {
      prog_.add_fact(act_[k], Row{});
      return;
    }
    add_ordered(std::move(rule));
  }

  // predicate(parameters) :- act(parameters, hidden), for every copy of the action
  void projections(const std::string & name, uint32_t predicate)
  {
    for (uint32_t k : copies_[name]) {
      const Lifted & a = lifted_.actions[k];
      datalog::Rule rule;
      rule.variables = a.parameters + a.hidden;
      rule.head.predicate = predicate;
      datalog::Atom body;
      body.predicate = act_[k];
      for (uint32_t v = 0; v < a.parameters + a.hidden; ++v) {
        body.args.push_back(datalog::Term::var(v));
        if (v < a.parameters) {
          rule.head.args.push_back(datalog::Term::var(v));
        }
      }
      rule.body.push_back(std::move(body));
      prog_.add_rule(std::move(rule));
    }
  }

  // fact :- act, for every atom the action may add (conditions of the effect are ignored)
  void effects(uint32_t k)
  {
    const Lifted & a = lifted_.actions[k];
    const uint32_t head = a.parameters + a.hidden;
    for (const kairos::lifted::Effect & e : a.effects) {
      datalog::Rule rule;
      if (e.kind != kairos::lifted::Effect::Kind::Add || !atom(e.atom, rule.head)) {
        continue;
      }
      rule.variables = static_cast<uint32_t>(a.variables.size());
      datalog::Atom body;
      body.predicate = act_[k];
      for (uint32_t v = 0; v < head; ++v) {
        body.args.push_back(datalog::Term::var(v));
      }
      rule.body.push_back(std::move(body));
      std::vector<bool> bound(a.variables.size(), false);
      std::fill(bound.begin(), bound.begin() + head, true);
      for (const datalog::Term & t : rule.head.args) {
        if (t.is_variable && !bound[t.value]) {  // a variable of a forall
          bound[t.value] = true;
          datalog::Atom any;
          any.predicate = objects_of(a.variables[t.value].types);
          any.args.push_back(t);
          rule.body.push_back(std::move(any));
        }
      }
      prog_.add_rule(std::move(rule));
    }
  }

  void axiom(const kairos::lifted::Axiom & x)
  {
    datalog::Rule rule;
    rule.variables = static_cast<uint32_t>(x.variables.size());
    rule.head.predicate = fact_[x.head.index()];
    std::vector<bool> bound(x.variables.size(), false);
    for (uint32_t v = 0; v < x.head_arity; ++v) {
      rule.head.args.push_back(datalog::Term::var(v));
    }
    for (const kairos::lifted::Literal & l : x.body.literals) {
      datalog::Atom b;
      if (l.positive && atom(l.atom, b)) {
        mark(b, bound);
        rule.body.push_back(std::move(b));
      }
    }
    for (uint32_t v = 0; v < x.head_arity; ++v) {
      if (!bound[v]) {
        datalog::Atom any;
        any.predicate = objects_of(x.variables[v].types);
        any.args.push_back(datalog::Term::var(v));
        rule.body.push_back(std::move(any));
      }
    }
    if (rule.body.empty()) {
      prog_.add_fact(rule.head.predicate, Row{});
      return;
    }
    prog_.add_rule(std::move(rule));
  }

  // need(subtask, its arguments that are known) :- need(task), precondition, earlier actions.
  // Known are the constants and the variables bound by what the task asked for, by the
  // precondition and by the primitive subtasks written before that share a variable with it
  // (only earlier ones: two actions cannot each wait for the other).
  void needs_below(
    const std::vector<hddl::Param> & params, const hddl::Network & net,
    const datalog::Atom * needed, uint32_t precondition,
    const std::vector<uint32_t> & precondition_params)
  {
    std::vector<datalog::Atom> always;
    if (needed != nullptr) {
      always.push_back(*needed);
    }
    if (precondition != kNone) {
      datalog::Atom a;
      a.predicate = precondition;
      for (uint32_t p : precondition_params) {
        a.args.push_back(datalog::Term::var(p));
      }
      always.push_back(std::move(a));
    }
    for (size_t s = 0; s < net.steps.size(); ++s) {
      const hddl::Step & step = net.steps[s];
      std::vector<bool> mine(params.size(), false);
      for (const Arg & a : step.args) {
        if (a.variable) {
          mine[a.index] = true;
        }
      }
      std::vector<datalog::Atom> body = always;
      for (size_t o = 0; o < s; ++o) {
        const hddl::Step & other = net.steps[o];
        if (!is_abstract(other.task) && std::any_of(other.args.begin(), other.args.end(),
          [&](const Arg & a) {return a.variable && mine[a.index];}))
        {
          body.push_back(atom(task_predicate_[other.task.index()], other.args));
        }
      }
      std::vector<bool> bound(params.size(), false);
      for (const datalog::Atom & a : body) {
        mark(a, bound);
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
        it->second = prog_.add_predicate(static_cast<uint32_t>(rule.head.args.size()),
            "need " + m_.tasks[step.task].name);
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
      rule.body = std::move(body);
      add_ordered(std::move(rule));
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
      mark(*needed, bound);
      rule.body.push_back(*needed);
    }
    if (precondition != kNone) {
      datalog::Atom a;
      a.predicate = precondition;
      for (uint32_t p : precondition_params) {
        a.args.push_back(datalog::Term::var(p));
      }
      mark(a, bound);
      rule.body.push_back(std::move(a));
    }
    for (const hddl::Step & step : net.steps) {
      rule.body.push_back(atom(task_predicate_[step.task.index()], step.args));
      mark(rule.body.back(), bound);
    }
    for (uint32_t p = 0; p < n; ++p) {
      if (!bound[p]) {
        // Nothing gives it a value: every object of its type
        datalog::Atom a;
        a.predicate = objects_of(params[p].types);
        a.args.push_back(datalog::Term::var(p));
        rule.body.push_back(std::move(a));
      } else {
        type_check(rule, p, params[p].types);  // a subtask may allow more than the parameter
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
    add_ordered(std::move(rule));
  }

  // Gives every action of the lifted task a static precondition that lists the instances used
  // by what was collected, so that Kairos grounds those and no others
  void guard()
  {
    std::map<std::string, std::unordered_set<Row, RowHash>> used;
    for (const Task & t : p_.tasks) {
      const hddl::TaskInfo & info = m_.tasks[t.task];
      if (info.primitive && !info.durative) {
        used[action_name(info.action)].insert(t.args);
      }
    }
    for (const Method & method : p_.methods) {
      if (method.has_precondition) {
        const hddl::MethodInfo & info = m_.methods[method.method];
        Row row;
        for (uint32_t p : info.precondition_params) {
          row.push_back(method.params[p]);
        }
        used[action_name(static_cast<uint32_t>(info.precondition_action))].insert(std::move(row));
      }
    }
    for (const auto & [name, copies] : copies_) {
      const Lifted & first = lifted_.actions[copies[0]];
      std::vector<sem::TypeSpec> types;
      for (uint32_t v = 0; v < first.parameters; ++v) {
        types.push_back(first.variables[v].types);
      }
      const sem::PredicateId needed = lifted_.add_predicate("klimax-needed " + name, false, types);
      for (uint32_t k : copies) {
        kairos::lifted::Literal literal;
        literal.atom.predicate = needed;
        for (uint32_t v = 0; v < first.parameters; ++v) {
          literal.atom.args.push_back(kairos::lifted::Term::variable(v));
        }
        lifted_.actions[k].precondition.literals.push_back(std::move(literal));
      }
      for (const Row & row : used[name]) {
        tick();
        std::vector<sem::ObjectId> objects;
        for (uint32_t o : row) {
          objects.emplace_back(o);
        }
        lifted_.init_facts.emplace_back(needed, std::move(objects));
      }
    }
  }

  // Gives each primitive task and each method precondition its instances in Kairos' task
  void attach()
  {
    for (Task & t : p_.tasks) {
      const uint32_t schema = task_schema_[t.task.index()];
      if (schema != kNone) {
        auto found = instances_[schema].find(t.args);
        if (found != instances_[schema].end()) {
          t.actions = found->second;
        }
      }
    }
    for (Method & method : p_.methods) {
      if (!method.has_precondition) {
        continue;
      }
      const uint32_t schema = precondition_schema_[method.method];
      if (schema == kNone) {
        continue;
      }
      Row row;
      for (uint32_t p : m_.methods[method.method].precondition_params) {
        row.push_back(method.params[p]);
      }
      auto found = instances_[schema].find(row);
      if (found != instances_[schema].end()) {
        method.precondition = found->second;
      }
    }
  }

  // Kairos' instances of each action schema, by arguments
  void instances()
  {
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
    auto schema = [&](uint32_t action) {
        auto it = schema_by_name.find(action_name(action));
        return it == schema_by_name.end() ? kNone : it->second;
      };
    for (uint32_t t = 0; t < m_.tasks.size(); ++t) {
      const hddl::TaskInfo & info = m_.tasks[TaskId(t)];
      task_schema_.push_back(info.primitive && !info.durative ? schema(info.action) : kNone);
    }
    for (const hddl::MethodInfo & method : m_.methods) {
      precondition_schema_.push_back(method.precondition_action < 0 ? kNone :
        schema(static_cast<uint32_t>(method.precondition_action)));
    }
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
      if (is_abstract(id)) {
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
    out.has_precondition = index != kNone && precondition_predicate_[index] != kNone;
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

  // Keeps what can be decomposed into actions and is reachable from the initial network
  void prune()
  {
    const size_t nt = p_.tasks.size();
    const size_t nm = p_.methods.size();
    // Bottom-up: a method is usable when all its subtasks are; an abstract task, when some
    // method is; an action, when Kairos reached it
    std::vector<bool> task_ok(nt, false);
    std::vector<bool> method_ok(nm, false);
    std::vector<uint32_t> missing(nm, 0);
    std::vector<std::vector<uint32_t>> used_in(nt);
    std::vector<uint32_t> ready;
    for (uint32_t m = 0; m < nm; ++m) {
      for (uint32_t t : p_.methods[m].subtasks) {
        used_in[t].push_back(m);
        ++missing[m];
      }
      if (p_.methods[m].has_precondition && p_.methods[m].precondition.empty()) {
        ++missing[m];  // Kairos dropped its precondition: never usable
      }
      if (missing[m] == 0) {
        method_ok[m] = true;
      }
    }
    for (uint32_t t = 0; t < nt; ++t) {
      const bool primitive = !is_abstract(p_.tasks[t].task);
      if (primitive ? !p_.tasks[t].actions.empty() :
        std::any_of(p_.tasks[t].methods.begin(), p_.tasks[t].methods.end(),
        [&](uint32_t m) {return method_ok[m];}))
      {
        task_ok[t] = true;
        ready.push_back(t);
      }
    }
    while (!ready.empty()) {
      const uint32_t t = ready.back();
      ready.pop_back();
      for (uint32_t m : used_in[t]) {
        if (--missing[m] != 0) {
          continue;
        }
        method_ok[m] = true;
        const uint32_t parent = p_.methods[m].task;
        if (parent != kNone && !task_ok[parent]) {
          task_ok[parent] = true;
          ready.push_back(parent);
        }
      }
    }
    // Top-down through usable methods
    std::vector<bool> task_live(nt, false);
    std::vector<bool> method_live(nm, false);
    std::vector<uint32_t> stack;
    for (uint32_t m : p_.initial) {
      if (method_ok[m]) {
        method_live[m] = true;
        stack.push_back(m);
      }
    }
    p_.unsolvable = stack.empty();
    while (!stack.empty()) {
      const uint32_t m = stack.back();
      stack.pop_back();
      for (uint32_t t : p_.methods[m].subtasks) {
        if (task_live[t]) {
          continue;
        }
        task_live[t] = true;
        for (uint32_t child : p_.tasks[t].methods) {
          if (method_ok[child] && !method_live[child]) {
            method_live[child] = true;
            stack.push_back(child);
          }
        }
      }
    }
    // Renumber
    std::vector<uint32_t> new_task(nt, kNone);
    std::vector<uint32_t> new_method(nm, kNone);
    std::vector<Task> tasks;
    std::vector<Method> methods;
    for (uint32_t t = 0; t < nt; ++t) {
      if (task_live[t]) {
        new_task[t] = static_cast<uint32_t>(tasks.size());
        tasks.push_back(std::move(p_.tasks[t]));
        p_.primitive_tasks += tasks.back().actions.empty() ? 0u : 1u;
      }
    }
    for (uint32_t m = 0; m < nm; ++m) {
      if (method_live[m]) {
        new_method[m] = static_cast<uint32_t>(methods.size());
        methods.push_back(std::move(p_.methods[m]));
      }
    }
    auto renumber = [](std::vector<uint32_t> & ids, const std::vector<uint32_t> & map) {
        std::vector<uint32_t> out;
        for (uint32_t id : ids) {
          if (map[id] != kNone) {
            out.push_back(map[id]);
          }
        }
        ids = std::move(out);
      };
    for (Task & t : tasks) {
      renumber(t.methods, new_method);
    }
    for (Method & m : methods) {
      renumber(m.subtasks, new_task);
      if (m.task != kNone) {
        m.task = new_task[m.task];
      }
    }
    renumber(p_.initial, new_method);
    p_.tasks = std::move(tasks);
    p_.methods = std::move(methods);
  }

  Model owned_;       // the model given, with parameters split
  const Model & m_;
  size_t split_ = 0;
  kairos::Budget * budget_;
  bool trace_ = std::getenv("KLIMAX_TRACE_GROUND") != nullptr;  // sizes and times to stderr
  Problem p_;
  kairos::lifted::Task lifted_;
  datalog::Program prog_;
  static constexpr uint32_t kAnonymous = 1u << 30;  // variables of a need that an action ignores
  std::vector<uint32_t> fact_;                     // by predicate of the lifted task
  std::vector<uint32_t> act_;                      // by action of the lifted task
  std::map<std::string, std::vector<uint32_t>> copies_;  // lifted actions by name
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

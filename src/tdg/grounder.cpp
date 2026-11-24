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
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ground/grounder.hpp"
#include "normalize/normalizer.hpp"

namespace klimax::tdg
{

namespace
{

namespace sem = kairos::sem;
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

// The instances of one action schema, with indexes by the columns that are bound
struct Relation
{
  std::vector<Row> rows;                       // arguments of each instance
  std::vector<std::vector<uint32_t>> actions;  // its instances in pddl.actions
  std::unordered_map<Row, uint32_t, RowHash> by_row;
  std::unordered_map<uint64_t, std::unordered_map<Row, std::vector<uint32_t>, RowHash>> index;

  void add(const Row & row, uint32_t action)
  {
    auto [it, fresh] = by_row.emplace(row, static_cast<uint32_t>(rows.size()));
    if (fresh) {
      rows.push_back(row);
      actions.emplace_back();
    }
    actions[it->second].push_back(action);
  }

  // The rows whose columns in `mask` have the values `key`
  const std::vector<uint32_t> & lookup(uint64_t mask, const Row & key)
  {
    static const std::vector<uint32_t> none;
    auto built = index.find(mask);
    if (built == index.end()) {
      auto & table = index[mask];
      for (uint32_t r = 0; r < rows.size(); ++r) {
        Row k;
        for (size_t c = 0; c < rows[r].size(); ++c) {
          if ((mask >> c) & 1u) {
            k.push_back(rows[r][c]);
          }
        }
        table[k].push_back(r);
      }
      built = index.find(mask);
    }
    auto found = built->second.find(key);
    return found == built->second.end() ? none : found->second;
  }
};

// A relation used in a method: column i holds argument pattern[i]
struct Generator
{
  Relation * relation = nullptr;
  std::vector<Arg> pattern;
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
      relations();
      build();
    } catch (const OutOfBudget &) {
      return kairos::Result<Problem>::failure(
        {kairos::Diagnostic::error("ground.budget", "the task is too large to ground in the "
          "time or memory given")});
    }
    prune();
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

  // One relation per action schema of Kairos' task
  void relations()
  {
    std::unordered_map<std::string, size_t> by_name;
    for (size_t s = 0; s < p_.pddl.action_names.size(); ++s) {
      by_name.emplace(p_.pddl.action_names[s], s);
    }
    schema_.assign(p_.pddl.action_names.size(), Relation{});
    for (uint32_t a = 0; a < p_.pddl.actions.size(); ++a) {
      const auto & action = p_.pddl.actions[a];
      Row row;
      for (sem::ObjectId o : action.args) {
        row.push_back(o.value());
      }
      schema_[action.schema].add(row, a);
    }
    auto relation = [&](kairos::Symbol name) -> Relation * {
        auto it = by_name.find(std::string(m_.pddl.symbols.name(name)));
        return it == by_name.end() ? &empty_ : &schema_[it->second];
      };
    for (size_t t = 0; t < m_.tasks.size(); ++t) {
      const hddl::TaskInfo & info = m_.tasks[TaskId(static_cast<uint32_t>(t))];
      of_task_.push_back(info.primitive && !info.durative ?
        relation(m_.pddl.actions[info.action].name) : nullptr);
    }
    for (const hddl::MethodInfo & method : m_.methods) {
      of_precondition_.push_back(method.precondition_action < 0 ? nullptr :
        relation(m_.pddl.actions[static_cast<size_t>(method.precondition_action)].name));
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
      if (Relation * r = of_task_[id.index()]) {
        auto row = r->by_row.find(args);
        if (row != r->by_row.end()) {
          t.actions = r->actions[row->second];
        }
      } else {
        queue_.push_back(it->second);
      }
      p_.tasks.push_back(std::move(t));
    }
    return it->second;
  }

  void build()
  {
    // The initial task network is a method of a task that is not in the model
    instantiate(kNone, kNone, m_.initial_params, m_.initial,
      std::vector<uint32_t>(m_.initial_params.size(), kNone));
    while (!queue_.empty()) {
      tick();
      const uint32_t at = queue_.back();
      queue_.pop_back();
      const TaskId id = p_.tasks[at].task;
      for (uint32_t index : m_.methods_of[id.index()]) {
        const hddl::MethodInfo & method = m_.methods[index];
        std::vector<uint32_t> binding(method.params.size(), kNone);
        if (bind(method.task_args, p_.tasks[at].args, method.params, binding)) {
          instantiate(index, at, method.params, method.network, std::move(binding));
        }
      }
    }
    p_.stats.tasks_built = p_.tasks.size();
    p_.stats.methods_built = p_.methods.size();
  }

  // Gives `pattern` the values of `row`; false if it contradicts `binding` or a type
  bool bind(
    const std::vector<Arg> & pattern, const Row & row, const std::vector<hddl::Param> & params,
    std::vector<uint32_t> & binding) const
  {
    for (size_t i = 0; i < pattern.size(); ++i) {
      if (!pattern[i].variable) {
        if (pattern[i].index != row[i]) {
          return false;
        }
        continue;
      }
      uint32_t & slot = binding[pattern[i].index];
      if (slot == kNone) {
        const sem::TypeSpec & types = params[pattern[i].index].types;
        if (!types.empty() && !m_.domain.types.fits(object(row[i]).types, types)) {
          return false;
        }
        slot = row[i];
      } else if (slot != row[i]) {
        return false;
      }
    }
    return true;
  }

  // Every instance of a method (or of the initial network) that extends `binding`
  void instantiate(
    uint32_t method, uint32_t parent, const std::vector<hddl::Param> & params,
    const hddl::Network & net, std::vector<uint32_t> binding)
  {
    std::vector<Generator> generators;
    if (method != kNone && of_precondition_[method] != nullptr) {
      Generator g;
      g.relation = of_precondition_[method];
      for (uint32_t p : m_.methods[method].precondition_params) {
        g.pattern.push_back(Arg{true, p});
      }
      generators.push_back(std::move(g));
    }
    for (const hddl::Step & step : net.steps) {
      if (Relation * r = of_task_[step.task.index()]) {
        generators.push_back(Generator{r, step.args});
      }
    }
    std::vector<bool> used(generators.size(), false);
    join(method, parent, params, net, generators, used, generators.size(), binding);
  }

  bool constraints_hold(const hddl::Network & net, const std::vector<uint32_t> & binding) const
  {
    auto value = [&](const Arg & a) {return a.variable ? binding[a.index] : a.index;};
    for (const hddl::VarConstraint & c : net.constraints) {
      const uint32_t l = value(c.lhs);
      const uint32_t r = value(c.rhs);
      if (l != kNone && r != kNone && (l == r) != c.equal) {
        return false;
      }
    }
    for (const hddl::SortConstraint & c : net.sorts) {
      if (binding[c.param] != kNone &&
        m_.domain.types.fits(object(binding[c.param]).types, c.types) == c.negated)
      {
        return false;
      }
    }
    return true;
  }

  void join(
    uint32_t method, uint32_t parent, const std::vector<hddl::Param> & params,
    const hddl::Network & net, std::vector<Generator> & generators, std::vector<bool> & used,
    size_t left, std::vector<uint32_t> & binding)
  {
    if (!constraints_hold(net, binding)) {
      return;
    }
    if (left == 0) {
      free_parameters(method, parent, params, net, binding, 0);
      return;
    }
    // The generator with fewest candidate rows for what is already bound
    size_t best = generators.size();
    const std::vector<uint32_t> * best_rows = nullptr;
    for (size_t g = 0; g < generators.size(); ++g) {
      if (used[g]) {
        continue;
      }
      uint64_t mask = 0;
      Row key;
      const Generator & gen = generators[g];
      for (size_t c = 0; c < gen.pattern.size() && c < 64; ++c) {
        const uint32_t v = gen.pattern[c].variable ? binding[gen.pattern[c].index] :
          gen.pattern[c].index;
        if (v != kNone) {
          mask |= uint64_t{1} << c;
          key.push_back(v);
        }
      }
      const std::vector<uint32_t> & rows = gen.relation->lookup(mask, key);
      if (best_rows == nullptr || rows.size() < best_rows->size()) {
        best = g;
        best_rows = &rows;
      }
    }
    used[best] = true;
    const Generator & gen = generators[best];
    const std::vector<uint32_t> rows = *best_rows;  // a copy: lookups below may rehash
    for (uint32_t r : rows) {
      tick();
      std::vector<uint32_t> saved = binding;
      if (bind(gen.pattern, gen.relation->rows[r], params, binding)) {
        join(method, parent, params, net, generators, used, left - 1, binding);
      }
      binding = std::move(saved);
    }
    used[best] = false;
  }

  // The parameters no relation binds take every object of their type
  void free_parameters(
    uint32_t method, uint32_t parent, const std::vector<hddl::Param> & params,
    const hddl::Network & net, std::vector<uint32_t> & binding, size_t from)
  {
    size_t p = from;
    while (p < binding.size() && binding[p] != kNone) {
      ++p;
    }
    if (p == binding.size()) {
      emit(method, parent, net, binding);
      return;
    }
    for (uint32_t o = 0; o < m_.objects.objects.items.size(); ++o) {
      if (!params[p].types.empty() && !m_.domain.types.fits(object(o).types, params[p].types)) {
        continue;
      }
      tick();
      binding[p] = o;
      if (constraints_hold(net, binding)) {
        free_parameters(method, parent, params, net, binding, p + 1);
      }
    }
    binding[p] = kNone;
  }

  void emit(
    uint32_t method, uint32_t parent, const hddl::Network & net,
    const std::vector<uint32_t> & binding)
  {
    Method out;
    out.method = method;
    out.task = parent;
    out.params = binding;
    if (method != kNone && of_precondition_[method] != nullptr) {
      out.has_precondition = true;
      Row row;
      for (uint32_t p : m_.methods[method].precondition_params) {
        row.push_back(binding[p]);
      }
      Relation & r = *of_precondition_[method];
      out.precondition = r.actions[r.by_row.at(row)];
    }
    for (const hddl::Step & step : net.steps) {
      Row args;
      for (const Arg & a : step.args) {
        args.push_back(a.variable ? binding[a.index] : a.index);
      }
      out.subtasks.push_back(task(step.task, args));
    }
    const uint32_t id = static_cast<uint32_t>(p_.methods.size());
    if (parent == kNone) {
      p_.initial.push_back(id);
    } else {
      p_.tasks[parent].methods.push_back(id);
    }
    p_.methods.push_back(std::move(out));
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
      if (missing[m] == 0) {
        method_ok[m] = true;
      }
    }
    for (uint32_t t = 0; t < nt; ++t) {
      const bool primitive = of_task_[p_.tasks[t].task.index()] != nullptr;
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

  const Model & m_;
  kairos::Budget * budget_;
  Problem p_;
  Relation empty_;
  std::vector<Relation> schema_;             // by action schema of Kairos' task
  std::vector<Relation *> of_task_;          // by TaskId: null for abstract tasks
  std::vector<Relation *> of_precondition_;  // by method: null if it has no precondition
  std::unordered_map<Row, uint32_t, RowHash> task_index_;
  std::vector<uint32_t> queue_;              // abstract ground tasks to decompose
};

}  // namespace

kairos::Result<Problem> ground(const hddl::Model & model, kairos::Budget * budget)
{
  return Grounder(model, budget).run();
}

}  // namespace klimax::tdg

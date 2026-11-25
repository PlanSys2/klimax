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


#include "search/sat.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "sat/solver.hpp"

namespace klimax::search
{

namespace
{

namespace ground = kairos::ground;
using sat::Lit;
using sat::lit;
using tdg::kNone;

// A place in a layer: the tasks it may hold and the variables of the state before it
struct Position
{
  std::vector<uint32_t> tasks;        // ground tasks, in increasing order
  std::vector<uint32_t> task_var;
  std::vector<uint32_t> methods;      // of its compound tasks, in increasing order
  std::vector<uint32_t> method_var;
  uint32_t some = kNone;              // variable: it holds a task
  uint32_t compound = kNone;          // variable: it holds a compound task
  std::vector<uint32_t> state;        // the variable of each atom
  uint32_t child = 0;                 // its first position in the next layer
  uint32_t width = 1;
  bool settled = false;               // only actions: the same position in every layer after
};

}  // namespace

class SatSearch::Impl
{
public:
  explicit Impl(const tdg::Problem & problem)
  : p_(problem), m_(problem.model), g_(problem.pddl),
    trace_(std::getenv("KLIMAX_TRACE_SEARCH") != nullptr) {}

  Result run(kairos::Budget * budget)
  {
    if (!finished_) {
      budget_ = budget;
      last_ = Result();
      finished_ = search(last_);
      last_.expansions = s_.conflicts();
    }
    return last_;
  }

private:
  // False when the budget ran out: the next call goes on from there
  bool search(Result & result)
  {
    if (layers_.empty()) {
      for (const ground::Action & a : g_.actions) {
        if (!a.effects.empty() || !plain(a.precondition)) {
          result.message = "the search by satisfiability takes no conditional effects or numbers";
          return true;
        }
      }
      if (!plain(g_.goal)) {
        result.message = "the search by satisfiability takes no numbers in the goal";
        return true;
      }
      words_ = (g_.atoms.size() + 63) / 64;
      if (p_.tasks.size() * words_ > kWords) {
        result.outcome = Outcome::OutOfBudget;
        result.message = "too large for the search by satisfiability";
        return true;
      }
      orders();
      classes();
      changes();
      first_layer();
    }
    while (true) {
      if (all_ == kNone) {
        if (making_ == 0 && layers_.size() > kLayers) {
          result.outcome = Outcome::OutOfBudget;
          result.message = "the search by satisfiability reached its last layer";
          return true;
        }
        const Made made = expand();
        if (made == Made::Part) {
          result.outcome = Outcome::OutOfBudget;
          return false;
        }
        if (made == Made::Full) {
          result.outcome = Outcome::OutOfBudget;
          result.message = "the search by satisfiability ran out of its memory";
          return true;
        }
        // Every position of the layer holds an action or nothing
        all_ = s_.new_var();
        for (const Position & q : layers_.back()) {
          if (q.compound != kNone) {
            s_.add(lit(all_, true), lit(q.compound, true));
          }
        }
      }
      const sat::Answer answer = s_.solve({lit(all_)}, budget_);
      if (trace_) {
        size_t tasks = 0, methods = 0;
        for (const Position & q : layers_.back()) {
          tasks += q.tasks.size();
          methods += q.methods.size();
        }
        std::fprintf(stderr, "sat: layer %zu, %zu positions, %zu tasks, %zu methods, "
          "%zu variables, %zu clauses, %.0f ms: %s\n", layers_.size() - 1, layers_.back().size(),
          tasks, methods, s_.variables(), s_.clauses(),
          std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start_).count(),
          answer == sat::Answer::Sat ? "plan" : answer == sat::Answer::Unsat ? "none" : "stopped");
      }
      if (answer == sat::Answer::Sat) {
        result.outcome = Outcome::Solved;
        result.plan = plan(result.steps);
        result.plans = 1;
        result.message = "satisfiability, " + std::to_string(layers_.size() - 1) + " layers";
        return true;
      }
      if (answer == sat::Answer::Unknown) {
        result.outcome = Outcome::OutOfBudget;
        return false;
      }
      s_.add(lit(all_, true));
      all_ = kNone;
    }
  }

  static constexpr size_t kWords = size_t{1} << 27;       // of the sets of atoms, 1 GB
  static constexpr size_t kStateVars = size_t{1} << 27;   // entries of the states of the positions
  static constexpr size_t kVariables = 20000000;
  static constexpr size_t kClauses = 50000000;
  static constexpr uint32_t kLayers = 64;

  static bool plain(const ground::Condition & c) {return c.numeric.empty() && c.objects.empty();}

  bool primitive(uint32_t task) const {return m_.tasks[p_.tasks[task].task].primitive;}

  // The steps of each network in the order to execute them
  void orders()
  {
    auto linear = [](const hddl::Network & net) {
        const size_t n = net.steps.size();
        std::vector<uint32_t> pending(n, 0);
        for (auto [a, b] : net.order) {
          (void)a;
          ++pending[b];
        }
        std::vector<uint32_t> out;
        std::vector<bool> done(n, false);
        while (out.size() < n) {
          for (uint32_t i = 0; i < n; ++i) {
            if (!done[i] && pending[i] == 0) {
              done[i] = true;
              out.push_back(i);
              for (auto [a, b] : net.order) {
                if (a == i) {
                  --pending[b];
                }
              }
              break;
            }
          }
        }
        return out;
      };
    for (const hddl::MethodInfo & method : m_.methods) {
      order_.push_back(linear(method.network));
    }
    initial_order_ = linear(m_.initial);
  }

  // The k-th subtask of a ground method, in the order of execution
  uint32_t subtask(const tdg::Method & method, size_t k) const
  {
    return method.subtasks[(method.method == kNone ? initial_order_ : order_[method.method])[k]];
  }

  static uint64_t mix(uint64_t x)
  {
    x ^= x >> 33;
    x *= 0xff51afd7ed558ccdull;
    x ^= x >> 33;
    x *= 0xc4ceb9fe1a85ec53ull;
    return x ^ (x >> 33);
  }

  // What a ground method asks for, with each subtask taken as its class
  uint64_t signature(const tdg::Method & method) const
  {
    uint64_t h = mix(method.has_precondition ? 2 : 1);
    for (uint32_t a : method.precondition) {
      h = mix(h ^ (a + 1));
    }
    h = mix(h + 0x9e3779b97f4a7c15ull);
    for (size_t k = 0; k < method.subtasks.size(); ++k) {
      h = mix(h ^ (class_[subtask(method, k)] + 1));
    }
    return h;
  }

  // Tasks that can be decomposed in the same ways are one for the formula: a task whose methods
  // do not use one of its arguments, in all the values of that argument. Classes are refined
  // until two tasks of a class have methods with the same preconditions and subtasks of the
  // same classes; each class is encoded as its first task, with one method of each kind
  void classes()
  {
    const uint32_t n = static_cast<uint32_t>(p_.tasks.size());
    class_.resize(n);
    for (uint32_t t = 0; t < n; ++t) {
      class_[t] = primitive(t) ? t : n + p_.tasks[t].task.value();
    }
    std::vector<uint64_t> kinds;
    for (size_t before = 0, round = 0; round < 64; ++round) {
      std::unordered_map<uint64_t, uint32_t> ids;
      std::vector<uint32_t> next(n);
      for (uint32_t t = 0; t < n; ++t) {
        if (primitive(t)) {
          next[t] = t;
          continue;
        }
        kinds.clear();
        for (uint32_t m : p_.tasks[t].methods) {
          kinds.push_back(signature(p_.methods[m]));
        }
        std::sort(kinds.begin(), kinds.end());
        kinds.erase(std::unique(kinds.begin(), kinds.end()), kinds.end());
        uint64_t h = mix(class_[t] + 1);
        for (uint64_t k : kinds) {
          h = mix(h ^ k);
        }
        next[t] = ids.emplace(h, t).first->second;  // the first task with these methods
      }
      class_ = std::move(next);
      if (ids.size() == before) {
        break;
      }
      before = ids.size();
    }
    kinds_.resize(n);
    for (uint32_t t = 0; t < n; ++t) {
      if (primitive(t) || class_[t] != t) {
        continue;
      }
      std::unordered_set<uint64_t> seen;
      for (uint32_t m : p_.tasks[t].methods) {
        if (seen.insert(signature(p_.methods[m])).second) {
          kinds_[t].push_back(m);
        }
      }
    }
  }

  // The class of the k-th subtask of a ground method
  uint32_t part(const tdg::Method & method, size_t k) const {return class_[subtask(method, k)];}

  uint64_t * changed(uint32_t task) {return &changed_[task * words_];}
  const uint64_t * changed(uint32_t task) const {return &changed_[task * words_];}

  // The atoms each task may change, through any of its decompositions
  void changes()
  {
    changed_.assign(p_.tasks.size() * words_, 0);
    for (uint32_t t = 0; t < p_.tasks.size(); ++t) {
      for (uint32_t a : p_.tasks[t].actions) {
        for (ground::AtomId x : g_.actions[a].changes.add) {
          changed(t)[x / 64] |= uint64_t{1} << (x % 64);
        }
        for (ground::AtomId x : g_.actions[a].changes.del) {
          changed(t)[x / 64] |= uint64_t{1} << (x % 64);
        }
      }
    }
    for (bool more = true; more; ) {
      more = false;
      for (const tdg::Method & method : p_.methods) {
        if (method.task == kNone) {
          continue;
        }
        uint64_t * to = changed(method.task);
        for (uint32_t sub : method.subtasks) {
          const uint64_t * from = changed(sub);
          for (size_t w = 0; w < words_; ++w) {
            if ((from[w] & ~to[w]) != 0) {
              to[w] |= from[w];
              more = true;
            }
          }
        }
      }
    }
  }

  void first_layer()
  {
    std::vector<Position> layer(2);
    Position & root = layer[0];
    root.methods = p_.initial;
    std::sort(root.methods.begin(), root.methods.end());
    for (size_t i = 0; i < root.methods.size(); ++i) {
      root.method_var.push_back(s_.new_var());
    }
    root.some = s_.new_var();
    root.compound = s_.new_var();
    s_.add(lit(root.some));
    s_.add(lit(root.compound));
    std::vector<Lit> any;
    for (uint32_t v : root.method_var) {
      any.push_back(lit(v));
    }
    s_.add(any);
    std::vector<bool> initially(g_.atoms.size(), false);
    for (ground::AtomId a : g_.init_atoms) {
      initially[a] = true;
    }
    for (size_t a = 0; a < g_.atoms.size(); ++a) {
      root.state.push_back(s_.new_var());
      s_.add(lit(root.state.back(), !initially[a]));
      layer[1].state.push_back(s_.new_var());
    }
    for (ground::AtomId a : g_.goal.positive) {
      s_.add(lit(layer[1].state[a]));
    }
    for (ground::AtomId a : g_.goal.negative) {
      s_.add(lit(layer[1].state[a], true));
    }
    layer[1].settled = true;
    for (size_t i = 0; i < root.methods.size(); ++i) {
      condition(root, lit(root.method_var[i], true), p_.methods[root.methods[i]]);
    }
    state_vars_ = 2 * g_.atoms.size();
    layers_.push_back(std::move(layer));
  }

  void at_most_one(const std::vector<uint32_t> & vars)
  {
    if (vars.size() <= 4) {
      for (size_t i = 0; i < vars.size(); ++i) {
        for (size_t j = i + 1; j < vars.size(); ++j) {
          s_.add(lit(vars[i], true), lit(vars[j], true));
        }
      }
      return;
    }
    // One more variable per element: some element up to this one is true
    uint32_t before = kNone;
    for (size_t i = 0; i < vars.size(); ++i) {
      const uint32_t upto = i + 1 < vars.size() ? s_.new_var() : kNone;
      if (upto != kNone) {
        s_.add(lit(vars[i], true), lit(upto));
      }
      if (before != kNone) {
        s_.add(lit(vars[i], true), lit(before, true));
        if (upto != kNone) {
          s_.add(lit(before, true), lit(upto));
        }
      }
      before = upto;
    }
  }

  // `unless` or the precondition of the method holds in the state of the position
  void condition(const Position & q, Lit unless, const tdg::Method & method)
  {
    if (!method.has_precondition) {
      return;
    }
    requires_one(q, unless, method.precondition);
  }

  // `unless` or the precondition of one of the action instances holds
  void requires_one(const Position & q, Lit unless, const std::vector<uint32_t> & instances)
  {
    std::vector<Lit> one{unless};
    for (uint32_t a : instances) {
      Lit not_this = unless;
      if (instances.size() > 1) {
        const uint32_t chosen = s_.new_var();
        one.push_back(lit(chosen));
        not_this = lit(chosen, true);
      }
      for (ground::AtomId x : g_.actions[a].precondition.positive) {
        s_.add(not_this, lit(q.state[x]));
      }
      for (ground::AtomId x : g_.actions[a].precondition.negative) {
        s_.add(not_this, lit(q.state[x], true));
      }
    }
    if (instances.size() != 1) {
      s_.add(one);  // with no instance, never
    }
  }

  enum class Made : uint8_t {Whole, Part, Full};

  // The next layer from the last one: its positions first, then its clauses. Part when the
  // budget ran out (the next call goes on from there); Full if it does not fit
  Made expand()
  {
    auto out = [&]() {return budget_ != nullptr && budget_->exhausted();};
    if (making_ == 0) {
      making_ = 1;
      at_ = 0;
      next_.clear();
    }
    std::vector<Position> & next = next_;
    const size_t from = layers_.size() - (making_ == 1 ? 1 : 2);
    for (size_t & i = at_; making_ == 1 && i < layers_[from].size(); ++i) {
      if (out()) {
        return Made::Part;
      }
      Position & p = layers_[from][i];
      p.child = static_cast<uint32_t>(next.size());
      if (p.methods.empty()) {
        // Only actions, or the end: the same position, with the same variables
        p.width = 1;
        Position same = p;
        same.settled = true;
        state_vars_ += same.state.size();
        next.push_back(std::move(same));
        continue;
      }
      size_t width = 1;
      for (uint32_t m : p.methods) {
        width = std::max(width, p_.methods[m].subtasks.size());
      }
      p.width = static_cast<uint32_t>(width);
      for (size_t k = 0; k < width; ++k) {
        Position q;
        for (uint32_t m : p.methods) {
          if (k < p_.methods[m].subtasks.size()) {
            q.tasks.push_back(part(p_.methods[m], k));
          }
        }
        if (k == 0) {
          for (uint32_t t : p.tasks) {
            if (primitive(t)) {
              q.tasks.push_back(t);
            }
          }
        }
        std::sort(q.tasks.begin(), q.tasks.end());
        q.tasks.erase(std::unique(q.tasks.begin(), q.tasks.end()), q.tasks.end());
        for (uint32_t t : q.tasks) {
          q.task_var.push_back(s_.new_var());
          if (!primitive(t)) {
            q.methods.insert(q.methods.end(), kinds_[t].begin(), kinds_[t].end());
          }
        }
        std::sort(q.methods.begin(), q.methods.end());
        for (size_t j = 0; j < q.methods.size(); ++j) {
          q.method_var.push_back(s_.new_var());
        }
        q.some = s_.new_var();
        q.compound = q.methods.empty() ? kNone : s_.new_var();
        if (k == 0) {
          q.state = p.state;
        } else {
          // What the position before may change has a new variable; the rest is the same one
          const Position & before = next.back();
          q.state = before.state;
          std::vector<uint64_t> may(words_, 0);
          for (uint32_t t : before.tasks) {
            const uint64_t * c = changed(t);
            for (size_t w = 0; w < words_; ++w) {
              may[w] |= c[w];
            }
          }
          for (size_t a = 0; a < q.state.size(); ++a) {
            if (((may[a / 64] >> (a % 64)) & 1u) != 0) {
              q.state[a] = s_.new_var();
            }
          }
        }
        state_vars_ += q.state.size();
        next.push_back(std::move(q));
      }
      if (state_vars_ > kStateVars || s_.variables() > kVariables || s_.clauses() > kClauses) {
        return Made::Full;
      }
    }
    if (making_ == 1) {
      layers_.push_back(std::move(next));
      next.clear();
      making_ = 2;
      at_ = 0;
    }
    const std::vector<Position> & parents = layers_[layers_.size() - 2];
    const std::vector<Position> & layer = layers_.back();
    for (size_t & i = at_; i < parents.size(); ++i) {
      if (out()) {
        return Made::Part;
      }
      const Position & p = parents[i];
      if (p.methods.empty()) {
        continue;
      }
      for (uint32_t k = 0; k < p.width; ++k) {
        inside(layer[p.child + k], layer[p.child + k + 1]);
      }
      between(p, layer);
      if (s_.clauses() > kClauses) {
        return Made::Full;
      }
    }
    making_ = 0;
    return Made::Whole;
  }

  static uint32_t var_of(const std::vector<uint32_t> & keys, const std::vector<uint32_t> & vars,
    uint32_t key)
  {
    const auto it = std::lower_bound(keys.begin(), keys.end(), key);
    return it != keys.end() && *it == key ? vars[static_cast<size_t>(it - keys.begin())] : kNone;
  }

  // What holds within a position: one task at most, with a method if it is compound, its
  // preconditions in the state before and its effects in the state after
  void inside(const Position & q, const Position & after)
  {
    std::vector<uint32_t> compound;
    std::vector<std::pair<uint32_t, uint32_t>> adds;     // atom, the variable of the task
    std::vector<std::pair<uint32_t, uint32_t>> deletes;
    for (size_t j = 0; j < q.tasks.size(); ++j) {
      const uint32_t t = q.tasks[j];
      const uint32_t x = q.task_var[j];
      s_.add(lit(x, true), lit(q.some));
      if (!primitive(t)) {
        compound.push_back(x);
        s_.add(lit(x, true), lit(q.compound));
        std::vector<Lit> one{lit(x, true)};
        for (uint32_t m : kinds_[t]) {
          const uint32_t y = var_of(q.methods, q.method_var, m);
          one.push_back(lit(y));
          s_.add(lit(y, true), lit(x));
        }
        s_.add(one);
        continue;
      }
      const std::vector<uint32_t> & instances = p_.tasks[t].actions;
      requires_one(q, lit(x, true), instances);
      if (instances.empty()) {
        continue;
      }
      // The instances of an action differ in the precondition only
      const ground::AtomChanges & changes = g_.actions[instances[0]].changes;
      for (ground::AtomId a : changes.add) {
        s_.add(lit(x, true), lit(after.state[a]));
        adds.emplace_back(a, x);
      }
      for (ground::AtomId a : changes.del) {
        if (std::find(changes.add.begin(), changes.add.end(), a) == changes.add.end()) {
          s_.add(lit(x, true), lit(after.state[a], true));
          deletes.emplace_back(a, x);
        }
      }
    }
    if (q.compound != kNone) {
      std::vector<Lit> one{lit(q.compound, true)};
      for (uint32_t x : compound) {
        one.push_back(lit(x));
      }
      s_.add(one);
    }
    // Two methods at once would need the same subtasks in the same places: either is a plan
    at_most_one(q.task_var);
    for (size_t j = 0; j < q.methods.size(); ++j) {
      condition(q, lit(q.method_var[j], true), p_.methods[q.methods[j]]);
    }
    // An atom changes only if an action here changes it, or under a compound task
    std::sort(adds.begin(), adds.end());
    std::sort(deletes.begin(), deletes.end());
    std::vector<uint64_t> under(words_, 0);  // what the compound tasks here may change
    for (uint32_t t : q.tasks) {
      if (!primitive(t)) {
        const uint64_t * c = changed(t);
        for (size_t w = 0; w < words_; ++w) {
          under[w] |= c[w];
        }
      }
    }
    size_t next_add = 0;
    size_t next_delete = 0;
    for (uint32_t a = 0; a < q.state.size(); ++a) {
      std::vector<Lit> appears{lit(q.state[a]), lit(after.state[a], true)};
      std::vector<Lit> goes{lit(q.state[a], true), lit(after.state[a])};
      for (; next_add < adds.size() && adds[next_add].first == a; ++next_add) {
        appears.push_back(lit(adds[next_add].second));
      }
      for (; next_delete < deletes.size() && deletes[next_delete].first == a; ++next_delete) {
        goes.push_back(lit(deletes[next_delete].second));
      }
      if (q.state[a] == after.state[a]) {
        continue;
      }
      if (((under[a / 64] >> (a % 64)) & 1u) != 0) {
        appears.push_back(lit(q.compound));
        goes.push_back(lit(q.compound));
      }
      s_.add(std::move(appears));
      s_.add(std::move(goes));
    }
  }

  // What holds between a position and those it becomes in the next layer
  void between(const Position & p, const std::vector<Position> & layer)
  {
    std::vector<std::vector<std::vector<Lit>>> support(p.width);  // by position, by task
    for (uint32_t k = 0; k < p.width; ++k) {
      support[k].resize(layer[p.child + k].tasks.size());
    }
    auto place = [&](uint32_t k, uint32_t task) {
        const Position & q = layer[p.child + k];
        const auto it = std::lower_bound(q.tasks.begin(), q.tasks.end(), task);
        return static_cast<size_t>(it - q.tasks.begin());
      };
    for (size_t j = 0; j < p.tasks.size(); ++j) {
      if (!primitive(p.tasks[j])) {
        continue;
      }
      // An action stays in the first position, and the others hold nothing
      const Lit unless = lit(p.task_var[j], true);
      const size_t at = place(0, p.tasks[j]);
      s_.add(unless, lit(layer[p.child].task_var[at]));
      support[0][at].push_back(lit(p.task_var[j]));
      for (uint32_t k = 1; k < p.width; ++k) {
        s_.add(unless, lit(layer[p.child + k].some, true));
      }
    }
    for (size_t j = 0; j < p.methods.size(); ++j) {
      const tdg::Method & method = p_.methods[p.methods[j]];
      const Lit unless = lit(p.method_var[j], true);
      for (uint32_t k = 0; k < p.width; ++k) {
        if (k < method.subtasks.size()) {
          const size_t at = place(k, part(method, k));
          s_.add(unless, lit(layer[p.child + k].task_var[at]));
          support[k][at].push_back(lit(p.method_var[j]));
        } else {
          s_.add(unless, lit(layer[p.child + k].some, true));
        }
      }
    }
    // A task is there only for what put it there
    for (uint32_t k = 0; k < p.width; ++k) {
      const Position & q = layer[p.child + k];
      for (size_t j = 0; j < q.tasks.size(); ++j) {
        std::vector<Lit> one = std::move(support[k][j]);
        one.push_back(lit(q.task_var[j], true));
        s_.add(std::move(one));
      }
    }
  }

  std::string name(uint32_t task) const
  {
    std::string out = m_.tasks[p_.tasks[task].task].name;
    for (uint32_t o : p_.tasks[task].args) {
      out += " " + m_.objects.objects.items[kairos::sem::ObjectId(o)].name;
    }
    return out;
  }

  // The plan of the model, in the format of the IPC: the task of each position and, under a
  // compound one, those of its positions in the next layer. A task added by parameter
  // splitting stands for its subtasks
  std::string plan(size_t & steps) const
  {
    std::string actions = "==>\n";
    std::string tree;
    uint32_t ids = 0;
    steps = 0;
    // Returns the identifiers that stand for the position: its own, or those of its subtasks.
    // `want` is the task that the method above asks for there, of the class the position holds
    auto walk = [&](auto && self, size_t layer, size_t i, uint32_t want) -> std::vector<uint32_t> {
        const Position & q = layers_[layer][i];
        if (want != kNone && primitive(want)) {
          const uint32_t id = ++ids;
          actions += std::to_string(id) + " " + name(want) + "\n";
          ++steps;
          return {id};
        }
        uint32_t kind = kNone;
        for (size_t j = 0; j < q.methods.size() && kind == kNone; ++j) {
          if (s_.value(q.method_var[j]) &&
            (want == kNone || p_.methods[q.methods[j]].task == class_[want]))
          {
            kind = q.methods[j];
          }
        }
        // The method of the task wanted that is of the kind chosen for its class
        uint32_t method = kind;
        if (want != kNone) {
          const uint64_t wanted = signature(p_.methods[kind]);
          for (uint32_t m : p_.tasks[want].methods) {
            if (signature(p_.methods[m]) == wanted) {
              method = m;
              break;
            }
          }
        }
        const tdg::Method & ground = p_.methods[method];
        const bool shown = want != kNone && !m_.tasks[p_.tasks[want].task].artificial;
        const uint32_t id = shown ? ++ids : 0;
        std::vector<uint32_t> children;
        for (size_t k = 0; k < ground.subtasks.size(); ++k) {
          const std::vector<uint32_t> below =
            self(self, layer + 1, q.child + k, subtask(ground, k));
          children.insert(children.end(), below.begin(), below.end());
        }
        if (!shown) {
          return children;
        }
        std::string line = std::to_string(id) + " " + name(want) + " -> " +
          m_.methods[ground.method].name;
        for (uint32_t c : children) {
          line += " " + std::to_string(c);
        }
        tree += line + "\n";
        return {id};
      };
    std::string roots = "root";
    for (uint32_t c : walk(walk, 0, 0, kNone)) {
      roots += " " + std::to_string(c);
    }
    return actions + roots + "\n" + tree + "<==\n";
  }

  const tdg::Problem & p_;
  const hddl::Model & m_;
  const ground::Task & g_;
  kairos::Budget * budget_ = nullptr;
  bool trace_;
  bool finished_ = false;
  Result last_;
  int making_ = 0;                           // of the next layer: 1 its positions, 2 its clauses
  size_t at_ = 0;                            // the position to go on from
  std::vector<Position> next_;               // the layer being made
  uint32_t all_ = kNone;                     // the variable of the layer to solve, if it is made
  sat::Solver s_;
  std::vector<std::vector<uint32_t>> order_;
  std::vector<uint32_t> initial_order_;
  size_t words_ = 0;
  std::vector<uint64_t> changed_;            // by task, a set of atoms
  std::vector<uint32_t> class_;              // by task, the first task of its class
  std::vector<std::vector<uint32_t>> kinds_;  // by first task of a class, its different methods
  std::vector<std::vector<Position>> layers_;
  size_t state_vars_ = 0;
  std::chrono::steady_clock::time_point start_ = std::chrono::steady_clock::now();
};

SatSearch::SatSearch(const tdg::Problem & problem)
: impl_(std::make_unique<Impl>(problem)) {}

SatSearch::~SatSearch() = default;

Result SatSearch::run(kairos::Budget * budget) {return impl_->run(budget);}

Result sat(const tdg::Problem & problem, kairos::Budget * budget)
{
  return SatSearch(problem).run(budget);
}

}  // namespace klimax::search

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

#include "search/total_order.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <iterator>
#include <memory>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace klimax::search
{

namespace
{

using tdg::kNone;
namespace ground = kairos::ground;

// A task waiting in the stack: a persistent list, shared between alternatives
struct Cell
{
  uint32_t task = 0;   // ground task
  uint32_t node = 0;   // its node in the decomposition tree
  uint32_t next = kNone;
  uint64_t hash = 0;   // of the whole list from here
  uint64_t rest = 0;   // fewest actions the whole list from here can take
};

struct Node
{
  uint32_t task = 0;
  uint32_t parent = kNone;
  uint32_t method = kNone;  // abstract: the ground method applied
};

// A decomposition that may be undone: where to come back to and what to try next
struct Choice
{
  uint32_t top = kNone;     // the stack before decomposing
  std::vector<uint32_t> candidates;  // ground methods to try, best first
  uint32_t next_method = 0;
  size_t cells = 0;
  size_t nodes = 0;
  size_t trail = 0;
  size_t executed = 0;
  uint64_t state_hash = 0;
};

uint64_t mix(uint64_t x)
{
  x ^= x >> 33;
  x *= 0xff51afd7ed558ccdull;
  x ^= x >> 33;
  x *= 0xc4ceb9fe1a85ec53ull;
  x ^= x >> 33;
  return x;
}

// The problem without deletes and with any action allowed anywhere: a task is done by an action
// of it, or by a method once its precondition and all its subtasks are. Additive costs from a
// state (each action counts 1), computed by Dijkstra's algorithm over propositions (atoms and
// "task done"). Infinite means impossible even so, hence impossible.
class Relaxed
{
public:
  static constexpr uint32_t kUnreached = 0x3fffffffu;

  explicit Relaxed(const tdg::Problem & p)
  : atoms_(static_cast<uint32_t>(p.pddl.atoms.size()))
  {
    const ground::Task & g = p.pddl;
    auto begin = [&](uint32_t base) {
        base_.push_back(base);
        pre_start_.push_back(static_cast<uint32_t>(pre_.size()));
        eff_start_.push_back(static_cast<uint32_t>(eff_.size()));
      };
    for (uint32_t t = 0; t < p.tasks.size(); ++t) {
      for (uint32_t a : p.tasks[t].actions) {
        const ground::Action & action = g.actions[a];
        begin(1);
        pre_.insert(pre_.end(), action.precondition.positive.begin(),
          action.precondition.positive.end());
        eff_.push_back(atoms_ + t);
        eff_.insert(eff_.end(), action.changes.add.begin(), action.changes.add.end());
        for (const ground::Effect & e : action.effects) {
          if (e.kind == ground::Effect::Kind::Add) {
            eff_.push_back(e.atom);  // whatever its condition
          }
        }
      }
    }
    for (uint32_t m = 0; m < p.methods.size(); ++m) {
      const tdg::Method & method = p.methods[m];
      method_first_.push_back(static_cast<uint32_t>(base_.size()));
      // One operator per instance of the precondition (one if it has none)
      const size_t copies = method.has_precondition ? method.precondition.size() : 1;
      for (size_t c = 0; c < copies; ++c) {
        begin(0);
        if (method.has_precondition) {
          const ground::Condition & pre = g.actions[method.precondition[c]].precondition;
          pre_.insert(pre_.end(), pre.positive.begin(), pre.positive.end());
        }
        for (uint32_t t : method.subtasks) {
          pre_.push_back(atoms_ + t);
        }
        if (method.task != kNone) {
          eff_.push_back(atoms_ + method.task);
        }
      }
    }
    method_first_.push_back(static_cast<uint32_t>(base_.size()));
    pre_start_.push_back(static_cast<uint32_t>(pre_.size()));
    eff_start_.push_back(static_cast<uint32_t>(eff_.size()));
    // Which operators wait for each proposition (once per time they need it)
    const uint32_t props = atoms_ + static_cast<uint32_t>(p.tasks.size());
    user_start_.assign(props + 1, 0);
    for (uint32_t x : pre_) {
      ++user_start_[x + 1];
    }
    for (uint32_t x = 0; x < props; ++x) {
      user_start_[x + 1] += user_start_[x];
    }
    users_.resize(pre_.size());
    std::vector<uint32_t> at(user_start_.begin(), user_start_.end() - 1);
    for (uint32_t op = 0; op < base_.size(); ++op) {
      for (uint32_t i = pre_start_[op]; i < pre_start_[op + 1]; ++i) {
        users_[at[pre_[i]]++] = op;
      }
    }
    cost_.resize(props);
    value_.resize(base_.size());
    waiting_.resize(base_.size());
  }

  void evaluate(const std::vector<bool> & state)
  {
    std::fill(cost_.begin(), cost_.end(), kUnreached);
    heap_.clear();
    for (uint32_t op = 0; op < base_.size(); ++op) {
      waiting_[op] = pre_start_[op + 1] - pre_start_[op];
      value_[op] = base_[op];
    }
    for (uint32_t a = 0; a < atoms_; ++a) {
      if (state[a]) {
        cost_[a] = 0;
        heap_.emplace_back(0, a);
      }
    }
    for (uint32_t op = 0; op < base_.size(); ++op) {
      if (waiting_[op] == 0) {
        fire(op);
      }
    }
    std::make_heap(heap_.begin(), heap_.end(), std::greater<>());
    while (!heap_.empty()) {
      std::pop_heap(heap_.begin(), heap_.end(), std::greater<>());
      const auto [cost, x] = heap_.back();
      heap_.pop_back();
      if (cost != cost_[x]) {
        continue;  // found cheaper since
      }
      for (uint32_t i = user_start_[x]; i < user_start_[x + 1]; ++i) {
        const uint32_t op = users_[i];
        value_[op] = std::min(kUnreached, value_[op] + cost);
        if (--waiting_[op] == 0) {
          fire(op);
        }
      }
    }
  }

  // Cost of completing a method from the state evaluated; kUnreached if impossible
  uint32_t method(uint32_t m) const
  {
    uint32_t best = kUnreached;
    for (uint32_t op = method_first_[m]; op < method_first_[m + 1]; ++op) {
      if (waiting_[op] == 0) {
        best = std::min(best, value_[op]);
      }
    }
    return best;
  }

  // Cost of getting a task done from the state evaluated
  uint32_t task(uint32_t t) const {return cost_[atoms_ + t];}

  // Cost of making an atom true from the state evaluated
  uint32_t atom(uint32_t a) const {return cost_[a];}

  size_t operators() const {return base_.size();}

private:
  void fire(uint32_t op)
  {
    const uint32_t cost = value_[op];
    for (uint32_t i = eff_start_[op]; i < eff_start_[op + 1]; ++i) {
      const uint32_t x = eff_[i];
      if (cost < cost_[x]) {
        cost_[x] = cost;
        heap_.emplace_back(cost, x);
        std::push_heap(heap_.begin(), heap_.end(), std::greater<>());
      }
    }
  }

  uint32_t atoms_;
  std::vector<uint32_t> base_;       // by operator: 1 for an action, 0 for a method
  std::vector<uint32_t> pre_start_;
  std::vector<uint32_t> pre_;
  std::vector<uint32_t> eff_start_;
  std::vector<uint32_t> eff_;
  std::vector<uint32_t> method_first_;  // operators of each ground method
  std::vector<uint32_t> user_start_;
  std::vector<uint32_t> users_;
  std::vector<uint32_t> cost_;       // by proposition
  std::vector<uint32_t> value_;      // by operator: its cost so far
  std::vector<uint32_t> waiting_;    // by operator: preconditions not reached yet
  std::vector<std::pair<uint32_t, uint32_t>> heap_;
};

class Search
{
public:
  Search(const tdg::Problem & problem, kairos::Budget * budget)
  : p_(problem), m_(problem.model), g_(problem.pddl), budget_(budget) {}

  Result run()
  {
    Result result;
    if (!m_.total_order) {
      result.message = "a task network is partially ordered";
      return result;
    }
    if (!g_.axioms.empty() || !g_.numeric_fluents.empty() || !g_.object_fluents.empty() ||
      !g_.durative_actions.empty())
    {
      result.message = "derived predicates, numbers or durative actions";
      return result;
    }
    const auto t0 = std::chrono::steady_clock::now();
    auto lap = [&](const char * what) {
        if (std::getenv("KLIMAX_TRACE_SEARCH") != nullptr) {
          std::fprintf(stderr, "%s: %.0f ms\n", what, std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - t0).count());
        }
      };
    orders();
    lap("orders");
    costs();
    lap("costs");
    infer();
    lap("infer");
    if (std::getenv("KLIMAX_NO_HEURISTIC") == nullptr) {
      relaxed_ = std::make_unique<Relaxed>(p_);
      if (std::getenv("KLIMAX_TRACE_SEARCH") != nullptr) {
        std::fprintf(stderr, "relaxed problem: %zu operators\n", relaxed_->operators());
      }
      if (relaxed_->operators() > kHeuristicLimit) {
        relaxed_.reset();  // too large to evaluate at every decomposition
      }
    }
    lap("relaxed");
    std::mt19937_64 random(20261124);
    for (size_t a = 0; a < g_.atoms.size(); ++a) {
      zobrist_.push_back(random());
    }
    // Plans of at most `bound` actions first, then twice as long, and so on: a search without
    // a bound finds plans with long detours. A branch is cut when the actions done plus the
    // fewest the pending tasks can take exceed the bound.
    uint64_t least = kInfinite;
    for (uint32_t m : p_.initial) {
      least = std::min(least, method_cost_[m]);
    }
    uint64_t guided_bound = 0;
    if (relaxed_ != nullptr) {
      // The relaxed cost from the initial state says about how long the plan is: starting
      // well below it only means searching every short non-plan first
      std::vector<bool> initial(g_.atoms.size(), false);
      for (ground::AtomId a : g_.init_atoms) {
        initial[a] = true;
      }
      relaxed_->evaluate(initial);
      uint32_t estimate = Relaxed::kUnreached;
      for (uint32_t m : p_.initial) {
        estimate = std::min(estimate, relaxed_->method(m));
      }
      if (estimate == Relaxed::kUnreached) {
        result.outcome = Outcome::Unsolvable;  // impossible even without deletes
        return result;
      }
      guided_bound = std::max<uint64_t>(std::max<uint64_t>(least, 1), uint64_t{estimate} * 2);
    }
    // Two ways of searching, each better on some domains: guided by the relaxed costs, and with
    // the methods in their fixed order. They take turns, each turn with twice the work of
    // the one before, so that the total is at most a few times what the better one needs.
    const bool trace = std::getenv("KLIMAX_TRACE_SEARCH") != nullptr;
    for (uint64_t turn = 4000; ; turn *= 2) {
      if (relaxed_ != nullptr && !best_first_over_) {
        limit_ = work_ + turn;
        const Outcome outcome = best_first(result);
        result.expansions = expansions_;
        if (trace) {
          std::fprintf(stderr, "best first: %s, %llu expansions so far, %zu nodes\n",
            outcome == Outcome::Solved ? "solved" : outcome == Outcome::Unsolvable ? "nothing" :
            "stopped", static_cast<unsigned long long>(expansions_), bf_nodes_.size());
        }
        if (outcome == Outcome::Solved || outcome == Outcome::Unsolvable) {
          result.outcome = outcome;
          result.steps = bf_steps_;
          return result;
        }
        if (budget_ != nullptr && budget_->exhausted()) {
          result.outcome = Outcome::OutOfBudget;
          return result;
        }
      }
      for (const bool guided : {true, false}) {
        if (guided && relaxed_ == nullptr) {
          continue;
        }
        guided_ = guided;
        limit_ = relaxed_ == nullptr ? ~uint64_t{0} : work_ + turn;
        for (uint64_t bound = guided ? guided_bound : std::max<uint64_t>(least, 1); ;
          bound *= 2)
        {
          const Outcome outcome = iteration(bound, result);
          result.expansions = expansions_;
          if (trace) {
            std::fprintf(stderr, "%s, bound %llu: %s, %llu expansions so far%s\n",
              guided ? "guided" : "fixed order", static_cast<unsigned long long>(bound),
              outcome == Outcome::Solved ? "solved" : outcome == Outcome::Unsolvable ?
              "nothing" : "stopped", static_cast<unsigned long long>(expansions_),
              cut_ ? ", cut" : "");
          }
          if (outcome == Outcome::Solved || (outcome == Outcome::Unsolvable && !cut_)) {
            result.outcome = outcome;  // without cuts, the whole space was searched
            return result;
          }
          if (outcome == Outcome::OutOfBudget) {
            if (budget_ != nullptr && budget_->exhausted()) {
              result.outcome = outcome;
              return result;
            }
            break;  // out of its turn: the other way now
          }
        }
      }
    }
  }

private:
  static constexpr uint64_t kInfinite = ~uint64_t{0} / 4;
  static constexpr size_t kLookAheadLimit = 20000;   // operators of the relaxed problem
  static constexpr size_t kHeuristicLimit = 400000;

  Outcome iteration(uint64_t bound, Result & result)
  {
    bound_ = bound;
    cut_ = false;
    state_.assign(g_.atoms.size(), false);
    state_hash_ = 0;
    for (ground::AtomId a : g_.init_atoms) {
      state_[a] = true;
      state_hash_ ^= zobrist_[a];
    }
    trail_.clear();
    cells_.clear();
    executed_.clear();
    choices_.clear();
    choice_task_.clear();
    seen_.clear();
    evaluated_ = false;
    top_ = kNone;
    // The ways of instantiating the initial network are the methods of a task above all
    nodes_.assign(1, Node{kNone, kNone, kNone});
    bool found = descend(kNone, 0, kNone, 0);
    while (found || backtrack()) {
      found = false;
      if ((budget_ != nullptr && budget_->exhausted()) || work_ >= limit_) {
        return Outcome::OutOfBudget;
      }
      if (top_ == kNone) {
        if (holds(g_.goal)) {
          result.plan = plan();
          result.steps = executed_.size();
          return Outcome::Solved;
        }
        continue;  // every task done but the goal does not hold
      }
      ++expansions_;
      ++work_;
      const Cell cell = cells_[top_];
      const tdg::Task & task = p_.tasks[cell.task];
      if (!task.actions.empty()) {
        // An action: applicable through any of its instances (they differ in the precondition)
        const ground::Action * action = nullptr;
        for (uint32_t a : task.actions) {
          if (holds(g_.actions[a].precondition)) {
            action = &g_.actions[a];
            break;
          }
        }
        if (action == nullptr) {
          continue;
        }
        apply(*action);
        executed_.push_back(cell.node);
        top_ = cell.next;
        found = true;
        continue;
      }
      found = descend(cell.task, cell.node, cell.next, 0);
    }
    return Outcome::Unsolvable;
  }

  // The fewest actions each task and method can be decomposed into, whatever the state; and
  // the methods of each task from the one that needs fewest
  void costs()
  {
    task_cost_.assign(p_.tasks.size(), kInfinite);
    method_cost_.assign(p_.methods.size(), kInfinite);
    for (size_t t = 0; t < p_.tasks.size(); ++t) {
      if (!p_.tasks[t].actions.empty()) {
        task_cost_[t] = 1;
      }
    }
    for (bool changed = true; changed; ) {
      changed = false;
      for (size_t m = 0; m < p_.methods.size(); ++m) {
        uint64_t sum = 0;
        for (uint32_t t : p_.methods[m].subtasks) {
          sum = std::min(kInfinite, sum + task_cost_[t]);
        }
        if (sum < method_cost_[m]) {
          method_cost_[m] = sum;
          const uint32_t parent = p_.methods[m].task;
          if (parent != kNone && sum < task_cost_[parent]) {
            task_cost_[parent] = sum;
            changed = true;
          }
        }
      }
    }
    auto sorted = [&](std::vector<uint32_t> methods) {
        std::stable_sort(methods.begin(), methods.end(), [&](uint32_t a, uint32_t b) {
            return method_cost_[a] < method_cost_[b];
          });
        return methods;
      };
    for (const tdg::Task & t : p_.tasks) {
      methods_.push_back(sorted(t.methods));
    }
    initial_ = sorted(p_.initial);
  }

  // What must hold when a task starts, whatever its decomposition, as literals (atom * 2, plus
  // 1 if it must be false). For an action, its precondition. For a method, its precondition,
  // what its first subtask needs, and what a later subtask needs that no earlier one can bring
  // about. For an abstract task, what all its methods need. A method whose needs do not hold
  // cannot be completed, so it is not tried.
  void infer()
  {
    const size_t nt = p_.tasks.size();
    const size_t nm = p_.methods.size();
    if (nm > 400000) {
      return;  // too large for the tables: searched without them
    }
    auto literals = [&](const std::vector<uint32_t> & actions) {
        // Common to every instance (they differ when a disjunction was split)
        std::vector<uint32_t> common;
        bool first = true;
        for (uint32_t a : actions) {
          std::vector<uint32_t> mine;
          for (ground::AtomId x : g_.actions[a].precondition.positive) {
            mine.push_back(x * 2);
          }
          for (ground::AtomId x : g_.actions[a].precondition.negative) {
            mine.push_back(x * 2 + 1);
          }
          std::sort(mine.begin(), mine.end());
          mine.erase(std::unique(mine.begin(), mine.end()), mine.end());
          if (first) {
            common = std::move(mine);
            first = false;
          } else {
            std::vector<uint32_t> both;
            std::set_intersection(common.begin(), common.end(), mine.begin(), mine.end(),
              std::back_inserter(both));
            common = std::move(both);
          }
        }
        return common;
      };
    // What each task may make true or false, through any decomposition (least fixpoint)
    std::vector<std::vector<uint32_t>> may(nt);  // atom * 2 if it may add it, + 1 if delete
    for (size_t t = 0; t < nt; ++t) {
      for (uint32_t a : p_.tasks[t].actions) {
        const ground::Action & action = g_.actions[a];
        for (ground::AtomId x : action.changes.add) {
          may[t].push_back(x * 2);
        }
        for (ground::AtomId x : action.changes.del) {
          may[t].push_back(x * 2 + 1);
        }
        for (const ground::Effect & e : action.effects) {
          if (e.kind == ground::Effect::Kind::Add) {
            may[t].push_back(e.atom * 2);
          } else if (e.kind == ground::Effect::Kind::Delete) {
            may[t].push_back(e.atom * 2 + 1);
          }
        }
      }
      std::sort(may[t].begin(), may[t].end());
      may[t].erase(std::unique(may[t].begin(), may[t].end()), may[t].end());
    }
    // Where each task is used: the methods that have it as a subtask
    std::vector<std::vector<uint32_t>> used_in(nt);
    for (uint32_t m = 0; m < nm; ++m) {
      for (uint32_t t : p_.methods[m].subtasks) {
        if (used_in[t].empty() || used_in[t].back() != m) {
          used_in[t].push_back(m);
        }
      }
    }
    size_t total = 0;
    uint64_t ticks = 0;
    const auto began = std::chrono::steady_clock::now();
    auto out_of_budget = [&]() {
        return (++ticks & 255u) == 0 && budget_ != nullptr && budget_->exhausted();
      };
    // Tasks that can reach each other through decompositions may change the same atoms: the
    // strongly connected components of "has as subtask", from the leaves up, each one taking
    // what its own actions and the components below it change. Every edge is used once.
    std::vector<std::vector<uint32_t>> children(nt);
    for (uint32_t t = 0; t < nt; ++t) {
      for (uint32_t m : p_.tasks[t].methods) {
        for (uint32_t c : p_.methods[m].subtasks) {
          if (c != t) {
            children[t].push_back(c);
          }
        }
      }
      std::sort(children[t].begin(), children[t].end());
      children[t].erase(std::unique(children[t].begin(), children[t].end()), children[t].end());
    }
    std::vector<uint32_t> component(nt, kNone);
    std::vector<std::vector<uint32_t>> component_may;
    {
      std::vector<uint32_t> index(nt, kNone);
      std::vector<uint32_t> low(nt, 0);
      std::vector<bool> on_stack(nt, false);
      std::vector<uint32_t> stack;
      std::vector<std::pair<uint32_t, uint32_t>> calls;  // task and next child to look at
      std::vector<uint8_t> marked(g_.atoms.size() * 2, 0);
      uint32_t counter = 0;
      for (uint32_t root = 0; root < nt; ++root) {
        if (index[root] != kNone) {
          continue;
        }
        calls.emplace_back(root, 0);
        while (!calls.empty()) {
          auto & [t, next] = calls.back();
          if (next == 0) {
            index[t] = low[t] = counter++;
            stack.push_back(t);
            on_stack[t] = true;
          }
          if (next < children[t].size()) {
            const uint32_t c = children[t][next++];
            if (index[c] == kNone) {
              calls.emplace_back(c, 0);
            } else if (on_stack[c]) {
              low[t] = std::min(low[t], index[c]);
            }
            continue;
          }
          const uint32_t done = t;
          calls.pop_back();
          if (!calls.empty()) {
            low[calls.back().first] = std::min(low[calls.back().first], low[done]);
          }
          if (low[done] != index[done]) {
            continue;
          }
          // `done` closes a component: everything below it is already computed
          const uint32_t id = static_cast<uint32_t>(component_may.size());
          std::vector<uint32_t> members;
          while (true) {
            const uint32_t member = stack.back();
            stack.pop_back();
            on_stack[member] = false;
            component[member] = id;
            members.push_back(member);
            if (member == done) {
              break;
            }
          }
          std::vector<uint32_t> all;
          auto take = [&](const std::vector<uint32_t> & more) {
              for (uint32_t e : more) {
                if (!marked[e]) {
                  marked[e] = 1;
                  all.push_back(e);
                }
              }
            };
          for (uint32_t member : members) {
            take(may[member]);
            for (uint32_t c : children[member]) {
              if (component[c] != id) {
                take(component_may[component[c]]);
              }
            }
          }
          for (uint32_t e : all) {
            marked[e] = 0;
          }
          std::sort(all.begin(), all.end());
          total += all.size();
          component_may.push_back(std::move(all));
          if (total > 50000000 || out_of_budget()) {
            return;
          }
        }
      }
    }
    for (uint32_t t = 0; t < nt; ++t) {
      may[t].clear();
      may[t].shrink_to_fit();
    }
    const uint64_t unions = component_may.size();
    if (std::getenv("KLIMAX_TRACE_SEARCH") != nullptr) {
      std::fprintf(stderr, "infer: may-sets %zu entries, %llu unions, %.0f ms\n", total,
        static_cast<unsigned long long>(unions), std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - began).count());
    }
    // What each task and method needs (unknown means "anything"; then it only shrinks)
    std::vector<std::vector<uint32_t>> needs(nt);
    std::vector<bool> known(nt, false);
    std::vector<std::vector<uint32_t>> method_needs(nm);
    std::vector<bool> method_known(nm, false);
    std::vector<std::vector<uint32_t>> own(nm);  // of the method's own precondition
    for (uint32_t m = 0; m < nm; ++m) {
      if (p_.methods[m].has_precondition) {
        own[m] = literals(p_.methods[m].precondition);
      }
    }
    for (size_t t = 0; t < nt; ++t) {
      if (!p_.tasks[t].actions.empty()) {
        needs[t] = literals(p_.tasks[t].actions);
        known[t] = true;
      }
    }
    std::vector<uint8_t> brought(g_.atoms.size() * 2, 0);
    uint64_t evaluations = 0;
    std::vector<uint32_t> work;
    std::vector<bool> queued(nm, true);
    for (uint32_t m = static_cast<uint32_t>(nm); m-- > 0; ) {
      work.push_back(m);
    }
    while (!work.empty()) {
      if (out_of_budget()) {
        return;
      }
      const uint32_t m = work.back();
      work.pop_back();
      queued[m] = false;
      ++evaluations;
      const tdg::Method & method = p_.methods[m];
      const std::vector<uint32_t> & order = method.method == kNone ? initial_order_ :
        order_[method.method];
      // Only once all its subtasks are known: from then on what it needs can only shrink
      if (std::any_of(method.subtasks.begin(), method.subtasks.end(),
        [&](uint32_t t) {return !known[t];}))
      {
        continue;  // comes back when the subtask is known
      }
      std::vector<uint32_t> mine = own[m];
      std::vector<uint32_t> touched;
      for (uint32_t step : order) {
        const uint32_t t = method.subtasks[step];
        for (uint32_t l : needs[t]) {
          if (!brought[l]) {
            mine.push_back(l);
          }
        }
        // After it, what it may add can be true and what it may delete can be false
        for (uint32_t e : component_may[component[t]]) {
          if (!brought[e]) {
            brought[e] = 1;
            touched.push_back(e);
          }
        }
      }
      for (uint32_t e : touched) {
        brought[e] = 0;
      }
      std::sort(mine.begin(), mine.end());
      mine.erase(std::unique(mine.begin(), mine.end()), mine.end());
      if (method_known[m] && mine == method_needs[m]) {
        continue;
      }
      method_needs[m] = std::move(mine);
      method_known[m] = true;
      const uint32_t parent = method.task;
      if (parent == kNone) {
        continue;
      }
      // The task needs what all its methods need: one more method can only take away
      std::vector<uint32_t> common;
      if (!known[parent]) {
        common = method_needs[m];
      } else {
        std::set_intersection(needs[parent].begin(), needs[parent].end(),
          method_needs[m].begin(), method_needs[m].end(), std::back_inserter(common));
      }
      if (!known[parent] || common.size() != needs[parent].size()) {
        needs[parent] = std::move(common);
        known[parent] = true;
        for (uint32_t user : used_in[parent]) {
          if (!queued[user]) {
            queued[user] = true;
            work.push_back(user);
          }
        }
      }
    }
    if (std::getenv("KLIMAX_TRACE_SEARCH") != nullptr) {
      size_t entries = 0;
      for (const auto & v : method_needs) {
        entries += v.size();
      }
      std::fprintf(stderr, "infer: needs %zu entries, %llu method evaluations\n", entries,
        static_cast<unsigned long long>(evaluations));
    }
    method_needs_ = std::move(method_needs);
  }

  bool needs_hold(uint32_t method) const
  {
    if (method_needs_.empty()) {
      return true;
    }
    for (uint32_t l : method_needs_[method]) {
      if (state_[l / 2] == ((l & 1u) != 0)) {
        return false;
      }
    }
    return true;
  }

  // The steps of each network in the order to execute them
  void orders()
  {
    auto linear = [](const hddl::Network & net) {
        const size_t n = net.steps.size();
        std::vector<uint32_t> pending(n, 0);
        for (auto [a, b] : net.order) {
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

  bool holds(const ground::Condition & c) const
  {
    for (ground::AtomId a : c.positive) {
      if (!state_[a]) {
        return false;
      }
    }
    for (ground::AtomId a : c.negative) {
      if (state_[a]) {
        return false;
      }
    }
    return true;
  }

  void set(ground::AtomId a, bool value)
  {
    if (state_[a] != value) {
      state_[a] = value;
      state_hash_ ^= zobrist_[a];
      trail_.push_back(a);
    }
  }

  void apply(const ground::Action & action)
  {
    // Conditional effects read the state before the action
    std::vector<std::pair<ground::AtomId, bool>> later;
    for (const ground::Effect & e : action.effects) {
      if ((e.kind == ground::Effect::Kind::Add || e.kind == ground::Effect::Kind::Delete) &&
        holds(e.condition))
      {
        later.emplace_back(e.atom, e.kind == ground::Effect::Kind::Add);
      }
    }
    for (ground::AtomId a : action.changes.del) {
      set(a, false);
    }
    for (const auto & [a, value] : later) {
      if (!value) {
        set(a, false);
      }
    }
    for (ground::AtomId a : action.changes.add) {
      set(a, true);
    }
    for (const auto & [a, value] : later) {
      if (value) {
        set(a, true);
      }
    }
  }

  // What the goal of the problem still costs from the state evaluated; kInfinite if some of it
  // cannot be made true any more. A plan must also end in the goal: a state that destroys part
  // of it is further from the end.
  uint64_t goal_cost() const
  {
    uint64_t sum = 0;
    for (ground::AtomId a : g_.goal.positive) {
      const uint32_t cost = relaxed_->atom(a);
      if (cost == Relaxed::kUnreached) {
        return kInfinite;
      }
      sum += cost;
    }
    return sum;
  }

  // --- Best first ----------------------------------------------------------------------------
  // Greedy best-first search over (state, stack): the node with the lowest relaxed cost of what
  // is left is expanded next, wherever it is. Expanding a node runs the actions at the top of
  // its stack and then gives one successor per method of the task that follows. It keeps what
  // it has between turns.

  struct BfNode
  {
    uint32_t parent = kNone;
    uint32_t top = kNone;         // its stack, in bf_cells_
    uint32_t did_cell = kNone;    // the task it decomposed to get here (kNone: the initial one)
    uint32_t did_method = kNone;
    uint32_t first_child = 0;     // the cells that decomposition made
    uint32_t child_count = 0;
    uint32_t exec_start = 0;      // the actions run when it was expanded, in bf_exec_
    uint32_t exec_count = 0;
    uint32_t state = 0;           // stored state: its parent's until expanded, then its own
  };

  void bf_store()
  {
    const size_t words = (state_.size() + 63) / 64;
    const size_t at = bf_states_.size();
    bf_states_.resize(at + words, 0);
    for (size_t a = 0; a < state_.size(); ++a) {
      if (state_[a]) {
        bf_states_[at + a / 64] |= uint64_t{1} << (a % 64);
      }
    }
    bf_hashes_.push_back(state_hash_);
  }

  void bf_load(uint32_t index)
  {
    const size_t words = (state_.size() + 63) / 64;
    const size_t at = size_t{index} * words;
    for (size_t a = 0; a < state_.size(); ++a) {
      state_[a] = (bf_states_[at + a / 64] >> (a % 64)) & 1u;
    }
    state_hash_ = bf_hashes_[index];
  }

  // The relaxed costs from the current state, unless they are the ones already computed
  void bf_evaluate()
  {
    if (!evaluated_ || evaluated_hash_ != state_hash_) {
      relaxed_->evaluate(state_);
      work_ += relaxed_->operators() / 32 + 1;
      evaluated_ = true;
      evaluated_hash_ = state_hash_;
      ++evaluations_;
    }
  }

  // One successor of `node` per method of `task` that can be applied in the current state.
  // Each is valued after the actions it starts with (on a small enough problem): the state they
  // leave, and from it the relaxed cost of its other subtasks, of the stack and of the goal
  // (when look-ahead is on; otherwise from the current state).
  void bf_successors(uint32_t node, uint32_t task, uint32_t cell, uint32_t below)
  {
    // Measured: valuing each successor from its parent's state, and only looking further when
    // it is expanded, solves more than valuing it after its first actions
    const bool look_ahead = relaxed_->operators() <= kLookAheadLimit &&
      std::getenv("KLIMAX_BF_LOOK_AHEAD") != nullptr;
    const std::vector<uint32_t> & methods = task == kNone ? initial_ : methods_[task];
    for (uint32_t m : methods) {
      const tdg::Method & method = p_.methods[m];
      if (method.has_precondition && std::none_of(method.precondition.begin(),
        method.precondition.end(), [&](uint32_t a) {return holds(g_.actions[a].precondition);}))
      {
        continue;
      }
      if (!needs_hold(m)) {
        continue;
      }
      const std::vector<uint32_t> & order = method.method == kNone ? initial_order_ :
        order_[method.method];
      const size_t mark = trail_.size();
      const uint64_t hash = state_hash_;
      size_t done = 0;
      bool possible = true;
      if (look_ahead) {
        for (; done < order.size(); ++done) {
          const tdg::Task & first = p_.tasks[method.subtasks[order[done]]];
          if (first.actions.empty()) {
            break;
          }
          const ground::Action * action = nullptr;
          for (uint32_t a : first.actions) {
            if (holds(g_.actions[a].precondition)) {
              action = &g_.actions[a];
              break;
            }
          }
          if (action == nullptr) {
            possible = false;
            break;
          }
          apply(*action);
        }
      }
      uint64_t h = 0;
      if (possible) {
        bf_evaluate();
        for (size_t k = done; k < order.size() && possible; ++k) {
          const uint32_t c = relaxed_->task(method.subtasks[order[k]]);
          possible = c != Relaxed::kUnreached;
          h += c;
        }
        uint32_t c = below;
        for (int depth = 0; c != kNone && depth < 64 && possible; ++depth) {
          const uint32_t cost = relaxed_->task(bf_cells_[c].task);
          possible = cost != Relaxed::kUnreached;
          h += cost;
          c = bf_cells_[c].next;
        }
        const uint64_t goal = goal_cost();
        possible = possible && goal != kInfinite;
        h += goal == kInfinite ? 0 : goal;
      }
      while (trail_.size() > mark) {
        const ground::AtomId a = trail_.back();
        trail_.pop_back();
        state_[a] = !state_[a];
      }
      state_hash_ = hash;
      if (!possible) {
        continue;
      }
      BfNode child;
      child.parent = node;
      child.did_cell = cell;
      child.did_method = m;
      child.first_child = static_cast<uint32_t>(bf_cells_.size());
      child.child_count = static_cast<uint32_t>(order.size());
      child.state = bf_nodes_[node].state;
      bf_cells_.resize(bf_cells_.size() + order.size());
      uint32_t top = below;
      for (size_t k = order.size(); k-- > 0; ) {
        Cell & made = bf_cells_[child.first_child + k];
        made.task = method.subtasks[order[k]];
        made.next = top;
        made.hash = mix((top == kNone ? 0x9e3779b97f4a7c15ull : bf_cells_[top].hash) ^
            (uint64_t{made.task} + 0x632be59bd9b4e019ull));
        top = child.first_child + static_cast<uint32_t>(k);
      }
      child.top = top;
      bf_nodes_.push_back(child);
      // Lowest cost first
      h = std::min<uint64_t>(h, 0xffffffffull);
      const uint64_t age = bf_oldest_first_ ? bf_nodes_.size() : 0xffffffffull - bf_nodes_.size();
      bf_open_.emplace_back((h << 32) | age, static_cast<uint32_t>(bf_nodes_.size() - 1));
      std::push_heap(bf_open_.begin(), bf_open_.end(), std::greater<>());
    }
  }

  Outcome best_first(Result & result)
  {
    if (bf_nodes_.empty()) {
      state_.assign(g_.atoms.size(), false);
      state_hash_ = 0;
      for (ground::AtomId a : g_.init_atoms) {
        state_[a] = true;
        state_hash_ ^= zobrist_[a];
      }
      bf_store();
      bf_nodes_.push_back(BfNode{});
      evaluated_ = false;
      bf_successors(0, kNone, kNone, kNone);
    }
    while (!bf_open_.empty()) {
      if ((budget_ != nullptr && budget_->exhausted()) || work_ >= limit_) {
        return Outcome::OutOfBudget;
      }
      if (bf_states_.size() * sizeof(uint64_t) > (size_t{3} << 30)) {
        best_first_over_ = true;  // out of room: the other searches go on
        return Outcome::OutOfBudget;
      }
      std::pop_heap(bf_open_.begin(), bf_open_.end(), std::greater<>());
      const uint32_t n = bf_open_.back().second;
      if (std::getenv("KLIMAX_TRACE_BEST_FIRST") != nullptr && (++bf_pops_ & 1023u) == 0) {
        size_t depth = 0;
        for (uint32_t k = n; k != kNone; k = bf_nodes_[k].parent) {
          depth += bf_nodes_[k].exec_count;
        }
        std::fprintf(stderr, "pop %llu: h %llu, %zu actions so far, open %zu\n",
          static_cast<unsigned long long>(bf_pops_),
          static_cast<unsigned long long>(bf_open_.back().first >> 32), depth, bf_open_.size());
        if (bf_pops_ == 4096) {
          // The actions that led here, to see what the search is doing
          std::vector<uint32_t> path;
          for (uint32_t k = n; k != kNone; k = bf_nodes_[k].parent) {
            path.push_back(k);
          }
          size_t shown = 0;
          for (auto it = path.rbegin(); it != path.rend() && shown < 400; ++it) {
            const BfNode & node = bf_nodes_[*it];
            if (node.did_method != kNone && node.did_cell != kNone) {
              std::fprintf(stderr, "    [%s -> %s]\n", name(bf_cells_[node.did_cell].task).c_str(),
                m_.methods[p_.methods[node.did_method].method].name.c_str());
            }
            for (uint32_t k = 0; k < node.exec_count; ++k, ++shown) {
              std::fprintf(stderr, "  %s\n",
                name(bf_cells_[bf_exec_[node.exec_start + k]].task).c_str());
            }
          }
        }
      }
      bf_open_.pop_back();
      ++expansions_;
      ++work_;
      bf_load(bf_nodes_[n].state);
      evaluated_ = false;
      // The actions at the top of the stack: no choice there
      uint32_t top = bf_nodes_[n].top;
      const uint32_t exec_start = static_cast<uint32_t>(bf_exec_.size());
      bool dead = false;
      while (top != kNone && !p_.tasks[bf_cells_[top].task].actions.empty()) {
        const ground::Action * action = nullptr;
        for (uint32_t a : p_.tasks[bf_cells_[top].task].actions) {
          if (holds(g_.actions[a].precondition)) {
            action = &g_.actions[a];
            break;
          }
        }
        if (action == nullptr) {
          dead = true;
          break;
        }
        apply(*action);
        bf_exec_.push_back(top);
        top = bf_cells_[top].next;
        ++work_;
      }
      trail_.clear();
      if (dead) {
        bf_exec_.resize(exec_start);
        continue;
      }
      bf_nodes_[n].exec_start = exec_start;
      bf_nodes_[n].exec_count = static_cast<uint32_t>(bf_exec_.size()) - exec_start;
      if (top == kNone) {
        if (holds(g_.goal)) {
          result.plan = bf_plan(n);
          return Outcome::Solved;
        }
        continue;
      }
      if (!bf_seen_.insert(mix(state_hash_ ^ bf_cells_[top].hash)).second) {
        continue;
      }
      bf_nodes_[n].state = static_cast<uint32_t>(bf_hashes_.size());
      bf_store();
      bf_successors(n, bf_cells_[top].task, top, bf_cells_[top].next);
    }
    return Outcome::Unsolvable;
  }

  // The plan of the path that ends in `goal`, without the tasks added by parameter splitting
  std::string bf_plan(uint32_t goal) const
  {
    std::vector<uint32_t> path;
    for (uint32_t n = goal; n != kNone; n = bf_nodes_[n].parent) {
      path.push_back(n);
    }
    std::reverse(path.begin(), path.end());
    auto text = [&](uint32_t cell) {return std::to_string(cell + 1);};
    auto artificial_cell = [&](uint32_t cell) {
        return m_.tasks[p_.tasks[bf_cells_[cell].task].task].artificial;
      };
    std::unordered_map<uint32_t, uint32_t> only_child;  // of the cells added by splitting
    for (uint32_t n : path) {
      const BfNode & node = bf_nodes_[n];
      if (node.did_cell != kNone && artificial_cell(node.did_cell)) {
        only_child[node.did_cell] = node.first_child;
      }
    }
    auto real = [&](uint32_t cell) {
        while (artificial_cell(cell)) {
          cell = only_child.at(cell);
        }
        return cell;
      };
    std::string actions = "==>\n";
    std::string roots = "root";
    std::string tree;
    size_t steps = 0;
    for (uint32_t n : path) {
      const BfNode & node = bf_nodes_[n];
      if (node.did_method != kNone) {
        std::string children;
        for (uint32_t k = 0; k < node.child_count; ++k) {
          children += " " + text(real(node.first_child + k));
        }
        if (node.did_cell == kNone) {
          roots += children;
        } else if (!artificial_cell(node.did_cell)) {
          tree += text(node.did_cell) + " " + name(bf_cells_[node.did_cell].task) + " -> " +
            m_.methods[p_.methods[node.did_method].method].name + children + "\n";
        }
      }
      for (uint32_t k = 0; k < node.exec_count; ++k) {
        const uint32_t cell = bf_exec_[node.exec_start + k];
        actions += text(cell) + " " + name(bf_cells_[cell].task) + "\n";
        ++steps;
      }
    }
    bf_steps_ = steps;
    return actions + roots + "\n" + tree + "<==\n";
  }

  // The methods of a task that can be applied now, most promising first: by their cost in the
  // relaxed problem from the current state, leaving out those impossible even there
  std::vector<uint32_t> candidates(uint32_t task)
  {
    const std::vector<uint32_t> & methods = task == kNone ? initial_ : methods_[task];
    std::vector<uint32_t> out;
    for (uint32_t m : methods) {
      const tdg::Method & method = p_.methods[m];
      if (method.has_precondition && std::none_of(method.precondition.begin(),
        method.precondition.end(), [&](uint32_t a) {return holds(g_.actions[a].precondition);}))
      {
        continue;
      }
      if (!needs_hold(m)) {
        ++pruned_;
        continue;  // some subtask could never be executed
      }
      out.push_back(m);
    }
    if (out.size() < 2 || relaxed_ == nullptr || !guided_) {
      return out;
    }
    // Each method is judged after the actions it starts with: the state they leave, and from
    // it the relaxed cost of the rest of its subtasks and of what waits in the stack. A method
    // whose first actions cannot run, or that leaves something impossible, is dropped.
    const uint32_t below = task == kNone ? kNone : cells_[top_].next;
    // One evaluation costs time in proportion to the size of the problem: on a large one,
    // every method is judged from the current state alone
    const bool look_ahead = out.size() <= 24 && relaxed_->operators() <= kLookAheadLimit;
    std::vector<std::pair<uint64_t, uint32_t>> ranked;
    for (uint32_t m : out) {
      const tdg::Method & method = p_.methods[m];
      const std::vector<uint32_t> & order = method.method == kNone ? initial_order_ :
        order_[method.method];
      const size_t mark = trail_.size();
      const uint64_t hash = state_hash_;
      size_t done = 0;
      bool possible = true;
      if (look_ahead) {
        for (; done < order.size(); ++done) {
          const tdg::Task & first = p_.tasks[method.subtasks[order[done]]];
          if (first.actions.empty()) {
            break;
          }
          const ground::Action * action = nullptr;
          for (uint32_t a : first.actions) {
            if (holds(g_.actions[a].precondition)) {
              action = &g_.actions[a];
              break;
            }
          }
          if (action == nullptr) {
            possible = false;
            break;
          }
          apply(*action);
        }
      }
      uint64_t cost = done;
      if (possible) {
        if (!evaluated_ || evaluated_hash_ != state_hash_) {
          relaxed_->evaluate(state_);
          work_ += relaxed_->operators() / 32 + 1;  // about what it costs, in expansions
          evaluated_ = true;
          evaluated_hash_ = state_hash_;
          ++evaluations_;
        }
        for (size_t k = done; k < order.size() && possible; ++k) {
          const uint32_t c = relaxed_->task(method.subtasks[order[k]]);
          possible = c != Relaxed::kUnreached;
          cost += c;
        }
        uint32_t cell = below;
        for (int depth = 0; cell != kNone && depth < 64 && possible; ++depth) {
          const uint32_t c = relaxed_->task(cells_[cell].task);
          possible = c != Relaxed::kUnreached;
          cost += c;
          cell = cells_[cell].next;
        }
        const uint64_t goal = goal_cost();
        possible = possible && goal != kInfinite;
        cost += goal == kInfinite ? 0 : goal;
      }
      while (trail_.size() > mark) {
        const ground::AtomId a = trail_.back();
        trail_.pop_back();
        state_[a] = !state_[a];
      }
      state_hash_ = hash;
      if (possible) {
        ranked.emplace_back(cost, m);
      } else {
        ++pruned_;
      }
    }
    std::stable_sort(ranked.begin(), ranked.end(), [](const auto & a, const auto & b) {
        return a.first < b.first;
      });
    out.clear();
    for (const auto & [cost, m] : ranked) {
      out.push_back(m);
    }
    return out;
  }

  // Replaces a task by the subtasks of the first of its methods that leads somewhere not seen
  // before. `task` is kNone for the initial network.
  bool descend(uint32_t task, uint32_t node, uint32_t below, uint32_t from)
  {
    (void)from;
    return try_from(task, node, below, candidates(task), 0);
  }

  bool try_from(
    uint32_t task, uint32_t node, uint32_t below, std::vector<uint32_t> methods, uint32_t from)
  {
    for (uint32_t i = from; i < methods.size(); ++i) {
      const tdg::Method & method = p_.methods[methods[i]];
      Choice choice;
      choice.top = top_;
      choice.next_method = i + 1;
      choice.cells = cells_.size();
      choice.nodes = nodes_.size();
      choice.trail = trail_.size();
      choice.executed = executed_.size();
      choice.state_hash = state_hash_;
      const std::vector<uint32_t> & order = method.method == kNone ? initial_order_ :
        order_[method.method];
      // Children in the order of execution; pushed last to first
      const uint32_t first_node = static_cast<uint32_t>(nodes_.size());
      for (uint32_t step : order) {
        nodes_.push_back(Node{method.subtasks[step], node, kNone});
      }
      uint32_t top = below;
      for (size_t k = order.size(); k-- > 0; ) {
        Cell cell;
        cell.task = method.subtasks[order[k]];
        cell.node = first_node + static_cast<uint32_t>(k);
        cell.next = top;
        cell.hash = mix((top == kNone ? 0x9e3779b97f4a7c15ull : cells_[top].hash) ^
            (uint64_t{cell.task} + 0x632be59bd9b4e019ull));
        cell.rest = std::min(kInfinite, task_cost_[cell.task] +
            (top == kNone ? 0 : cells_[top].rest));
        cells_.push_back(cell);
        top = static_cast<uint32_t>(cells_.size() - 1);
      }
      const uint64_t done = executed_.size();
      const bool too_long = done + (top == kNone ? 0 : cells_[top].rest) > bound_;
      cut_ = cut_ || too_long;
      // Seen with as few actions done: nothing new from here
      const uint64_t key = mix(state_hash_ ^ (top == kNone ? 0x51ull : cells_[top].hash));
      bool repeated = false;
      if (!too_long) {
        auto [it, fresh] = seen_.emplace(key, done);
        repeated = !fresh && it->second <= done;
        if (!fresh && it->second > done) {
          it->second = done;
        }
      }
      if (too_long || repeated) {
        cells_.resize(choice.cells);
        nodes_.resize(choice.nodes);
        continue;
      }
      nodes_[node].method = methods[i];
      choice.candidates = std::move(methods);
      choices_.push_back(std::move(choice));
      choice_task_.push_back({task, node});
      top_ = top;
      return true;
    }
    return false;
  }

  // Undoes the last decomposition and tries its next method; false when none is left anywhere
  bool backtrack()
  {
    while (!choices_.empty()) {
      Choice choice = std::move(choices_.back());
      const auto [task, node] = choice_task_.back();
      choices_.pop_back();
      choice_task_.pop_back();
      while (trail_.size() > choice.trail) {
        const ground::AtomId a = trail_.back();
        trail_.pop_back();
        state_[a] = !state_[a];
      }
      state_hash_ = choice.state_hash;
      cells_.resize(choice.cells);
      nodes_.resize(choice.nodes);
      executed_.resize(choice.executed);
      top_ = choice.top;
      const uint32_t below = task == kNone ? kNone : cells_[top_].next;
      if (try_from(task, node, below, std::move(choice.candidates), choice.next_method)) {
        return true;
      }
    }
    return false;
  }

  std::string name(uint32_t task) const
  {
    std::string out = m_.tasks[p_.tasks[task].task].name;
    for (uint32_t o : p_.tasks[task].args) {
      out += " " + m_.objects.objects.items[kairos::sem::ObjectId(o)].name;
    }
    return out;
  }

  bool artificial(uint32_t node) const
  {
    return m_.tasks[p_.tasks[nodes_[node].task].task].artificial;
  }

  // The plan and its decomposition, without the tasks added by parameter splitting
  std::string plan() const
  {
    std::vector<std::vector<uint32_t>> children(nodes_.size());
    for (uint32_t n = 1; n < nodes_.size(); ++n) {
      children[nodes_[n].parent].push_back(n);
    }
    // A task added by splitting stands for its only subtask
    auto real = [&](uint32_t n) {
        while (artificial(n)) {
          n = children[n].at(0);
        }
        return n;
      };
    std::string out = "==>\n";
    for (uint32_t n : executed_) {
      out += std::to_string(n) + " " + name(nodes_[n].task) + "\n";
    }
    out += "root";
    for (uint32_t c : children[0]) {
      out += " " + std::to_string(real(c));
    }
    out += "\n";
    for (uint32_t n = 1; n < nodes_.size(); ++n) {
      if (nodes_[n].method == kNone || artificial(n)) {
        continue;
      }
      out += std::to_string(n) + " " + name(nodes_[n].task) + " -> " +
        m_.methods[p_.methods[nodes_[n].method].method].name;
      for (uint32_t c : children[n]) {
        out += " " + std::to_string(real(c));
      }
      out += "\n";
    }
    return out + "<==\n";
  }

  const tdg::Problem & p_;
  const hddl::Model & m_;
  const ground::Task & g_;
  kairos::Budget * budget_;
  std::vector<std::vector<uint32_t>> order_;  // by method of the model
  std::vector<uint32_t> initial_order_;
  std::vector<bool> state_;
  std::vector<uint64_t> zobrist_;
  uint64_t state_hash_ = 0;
  std::vector<ground::AtomId> trail_;   // atoms flipped, to undo
  std::vector<Cell> cells_;
  std::vector<Node> nodes_;             // 0 is the root above the initial tasks
  std::vector<uint32_t> executed_;      // nodes of the actions, in plan order
  uint32_t top_ = kNone;
  std::vector<Choice> choices_;
  std::vector<std::pair<uint32_t, uint32_t>> choice_task_;  // task and node of each choice
  std::unordered_map<uint64_t, uint64_t> seen_;  // state and stack -> fewest actions done
  std::vector<uint64_t> task_cost_;
  std::vector<uint64_t> method_cost_;
  std::vector<std::vector<uint32_t>> methods_;  // of each task, cheapest first
  std::vector<uint32_t> initial_;
  std::vector<std::vector<uint32_t>> method_needs_;  // by ground method; empty: not inferred
  uint64_t pruned_ = 0;
  std::unique_ptr<Relaxed> relaxed_;  // null: methods in their fixed order
  bool evaluated_ = false;            // relaxed_ holds the costs from evaluated_hash_
  uint64_t evaluated_hash_ = 0;
  uint64_t evaluations_ = 0;
  std::vector<Cell> bf_cells_;
  std::vector<BfNode> bf_nodes_;
  std::vector<uint32_t> bf_exec_;
  std::vector<uint64_t> bf_states_;
  std::vector<uint64_t> bf_hashes_;
  std::vector<std::pair<uint64_t, uint32_t>> bf_open_;
  std::unordered_set<uint64_t> bf_seen_;
  bool best_first_over_ = false;
  uint64_t bf_pops_ = 0;
  // Among equal costs the oldest first: the alternatives of a decomposition often tie, and
  // going down the newest commits to one of them (measured: more problems solved this way)
  bool bf_oldest_first_ = std::getenv("KLIMAX_BF_NEWEST") == nullptr;
  mutable size_t bf_steps_ = 0;
  bool guided_ = true;               // this turn ranks the methods with relaxed_
  uint64_t work_ = 0;                // expansions, plus the cost of the evaluations
  uint64_t limit_ = ~uint64_t{0};    // work at which this turn ends
  uint64_t bound_ = 0;
  bool cut_ = false;  // some branch was cut by the bound in this iteration
  uint64_t expansions_ = 0;
};

}  // namespace

Result total_order(const tdg::Problem & problem, kairos::Budget * budget)
{
  if (problem.unsolvable) {
    Result result;
    result.outcome = Outcome::Unsolvable;
    return result;
  }
  return Search(problem, budget).run();
}

}  // namespace klimax::search

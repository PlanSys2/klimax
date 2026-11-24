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
#include <bit>
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
  uint64_t reach = 0;  // which atoms of the goal (the first 64) the list from here may make true
  uint32_t mask = kNone;  // the atoms the list from here may read (Search::mask_for)
};

struct Node
{
  uint32_t task = 0;
  uint32_t parent = kNone;
  uint32_t method = kNone;  // abstract: the ground method applied
  uint32_t start = 0;       // abstract: actions done when it was decomposed
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
  static constexpr uint32_t kFar = 100000;  // ordered: not reached, which proves nothing

  // With `ordered`, an action makes its effects true only as a step of a method, after the
  // steps before it (and with the method's precondition): an action alone only gets its own
  // task done. Otherwise any action may run anywhere, which lets a late step of a method (a
  // bookkeeping action with no precondition, say) pay for a task that its early steps should.
  // The ordered costs are better estimates but no proof: a state they call impossible may not be.
  Relaxed(const tdg::Problem & p, bool ordered, const std::vector<std::vector<uint32_t>> & orders,
    const std::vector<uint32_t> & initial_order)
  : atoms_(static_cast<uint32_t>(p.pddl.atoms.size())), ordered_(ordered)
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
        if (!ordered_) {
          adds(action, eff_);
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
    if (ordered_) {
      // The effects of each primitive step, once the method's precondition and the steps up
      // to it are done
      for (uint32_t m = 0; m < p.methods.size(); ++m) {
        const tdg::Method & method = p.methods[m];
        const std::vector<uint32_t> & order = method.method == kNone ? initial_order :
          orders[method.method];
        const size_t copies = method.has_precondition ? method.precondition.size() : 1;
        for (size_t k = 0; k < order.size(); ++k) {
          const tdg::Task & step = p.tasks[method.subtasks[order[k]]];
          if (step.actions.empty()) {
            continue;
          }
          std::vector<uint32_t> made;
          for (uint32_t a : step.actions) {
            adds(g.actions[a], made);
          }
          if (made.empty()) {
            continue;
          }
          for (size_t c = 0; c < copies; ++c) {
            begin(0);
            if (method.has_precondition) {
              const ground::Condition & pre = g.actions[method.precondition[c]].precondition;
              pre_.insert(pre_.end(), pre.positive.begin(), pre.positive.end());
            }
            for (size_t j = 0; j <= k; ++j) {
              pre_.push_back(atoms_ + method.subtasks[order[j]]);
            }
            eff_.insert(eff_.end(), made.begin(), made.end());
          }
        }
      }
    }
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
    support_.resize(props);
    in_plan_.assign(props, 0);
    op_in_plan_.assign(base_.size(), 0);
    value_.resize(base_.size());
    waiting_.resize(base_.size());
  }

  void evaluate(const std::vector<bool> & state)
  {
    std::fill(cost_.begin(), cost_.end(), kUnreached);
    plan_undo(0);
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
        // A cost too large to hold is still a cost: only what is never reached is kUnreached
        value_[op] = std::min(kUnreached - 1, value_[op] + cost);
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
    return far(best);
  }

  // Cost of getting a task done from the state evaluated
  uint32_t task(uint32_t t) const {return far(cost_[atoms_ + t]);}

  // Cost of making an atom true from the state evaluated
  uint32_t atom(uint32_t a) const {return far(cost_[a]);}

  size_t operators() const {return base_.size();}
  bool ordered() const {return ordered_;}

  // A relaxed plan from the state evaluated, built up task by task and atom by atom: each call
  // adds what its argument needs through the cheapest way found, and returns how many actions
  // that added to the plan. An action that several of them need is counted once, which the sum
  // of their costs does not do. `plan_size()` and `plan_undo()` take the plan back to an
  // earlier point.
  uint32_t plan_task(uint32_t t) {return plan_add(atoms_ + t);}
  uint32_t plan_atom(uint32_t a) {return plan_add(a);}
  size_t plan_size() const {return plan_.size();}

  void plan_undo(size_t size)
  {
    while (plan_.size() > size) {
      const uint32_t entry = plan_.back();
      plan_.pop_back();
      if ((entry & 1u) != 0) {
        op_in_plan_[entry / 2] = 0;
      } else {
        in_plan_[entry / 2] = 0;
      }
    }
  }

private:
  uint32_t far(uint32_t cost) const
  {
    return ordered_ ? std::min(cost, kFar) : cost;
  }

  // The atoms an action may make true, whatever the conditions of its effects
  static void adds(const ground::Action & action, std::vector<uint32_t> & out)
  {
    out.insert(out.end(), action.changes.add.begin(), action.changes.add.end());
    for (const ground::Effect & e : action.effects) {
      if (e.kind == ground::Effect::Kind::Add) {
        out.push_back(e.atom);
      }
    }
  }

  uint32_t plan_add(uint32_t goal)
  {
    uint32_t actions = 0;
    todo_.assign(1, goal);
    while (!todo_.empty()) {
      const uint32_t x = todo_.back();
      todo_.pop_back();
      if (cost_[x] == 0 || cost_[x] == kUnreached || in_plan_[x]) {
        continue;  // true already (or impossible: the caller has seen that)
      }
      in_plan_[x] = 1;
      plan_.push_back(x * 2);
      const uint32_t op = support_[x];
      if (op_in_plan_[op]) {
        continue;
      }
      op_in_plan_[op] = 1;
      plan_.push_back(op * 2 + 1);
      actions += base_[op];
      todo_.insert(todo_.end(), pre_.begin() + pre_start_[op], pre_.begin() + pre_start_[op + 1]);
    }
    return actions;
  }

  void fire(uint32_t op)
  {
    const uint32_t cost = value_[op];
    for (uint32_t i = eff_start_[op]; i < eff_start_[op + 1]; ++i) {
      const uint32_t x = eff_[i];
      if (cost < cost_[x]) {
        cost_[x] = cost;
        support_[x] = op;
        heap_.emplace_back(cost, x);
        std::push_heap(heap_.begin(), heap_.end(), std::greater<>());
      }
    }
  }

  uint32_t atoms_;
  bool ordered_ = false;
  std::vector<uint32_t> base_;       // by operator: 1 for an action, 0 for a method
  std::vector<uint32_t> pre_start_;
  std::vector<uint32_t> pre_;
  std::vector<uint32_t> eff_start_;
  std::vector<uint32_t> eff_;
  std::vector<uint32_t> method_first_;  // operators of each ground method
  std::vector<uint32_t> user_start_;
  std::vector<uint32_t> users_;
  std::vector<uint32_t> cost_;       // by proposition
  std::vector<uint32_t> support_;    // by proposition: the operator that gave it its cost
  std::vector<uint8_t> in_plan_;     // by proposition and by operator: in the relaxed plan
  std::vector<uint8_t> op_in_plan_;
  std::vector<uint32_t> plan_;       // what was added, to undo: proposition * 2, operator * 2 + 1
  std::vector<uint32_t> todo_;
  std::vector<uint32_t> value_;      // by operator: its cost so far
  std::vector<uint32_t> waiting_;    // by operator: preconditions not reached yet
  std::vector<std::pair<uint32_t, uint32_t>> heap_;
};

class Search
{
public:
  Search(const tdg::Problem & problem, kairos::Budget * budget)
  : p_(problem), m_(problem.model), g_(problem.pddl), budget_(budget) {}

  // Shorter plans than the one in `result`, until there is none or the budget runs out
  void improve(Result & result, const Options & options)
  {
    result.plans = 1;
    if (options.on_plan) {
      options.on_plan(result.plan, result.steps);
    }
    relaxed_ = free_.get();
    guided_ = false;
    limit_ = ~uint64_t{0};
    while (result.steps > 0) {
      Result shorter;
      const Outcome outcome = iteration(result.steps - 1, shorter);
      result.expansions = expansions_;
      if (outcome != Outcome::Solved) {
        // The whole space under the bound was searched: in one order of each network only,
        // if the problem is partially ordered
        result.optimal = outcome == Outcome::Unsolvable && !linearized_;
        if (outcome == Outcome::Unsolvable && linearized_) {
          improve_interleaving(result, options);  // none shorter in this order: in any?
        }
        return;
      }
      result.plan = std::move(shorter.plan);
      result.steps = shorter.steps;
      result.message = "depth first, fixed order, under the length of the plan before";
      ++result.plans;
      if (options.on_plan) {
        options.on_plan(result.plan, result.steps);
      }
    }
    result.optimal = true;
  }

  // The same over partially ordered networks, interleaving: best first under the bound, from
  // scratch each time. Exhausting it is a proof.
  void improve_interleaving(Result & result, const Options & options)
  {
    if (free_ == nullptr) {
      return;
    }
    limit_ = ~uint64_t{0};
    while (result.steps > 0) {
      po_nodes_.clear();
      po_items_.clear();
      po_edges_.clear();
      po_open_.clear();
      po_seen_.clear();
      po_fewest_.clear();
      po_ids_ = 0;
      po_over_ = false;
      po_bound_ = result.steps - 1;
      Result shorter;
      const Outcome outcome = partial_order(shorter);
      result.expansions = expansions_;
      if (outcome != Outcome::Solved) {
        result.optimal = outcome == Outcome::Unsolvable;
        return;
      }
      result.plan = std::move(shorter.plan);
      result.steps = bf_steps_;
      result.message = "best first, partial order, under the length of the plan before";
      ++result.plans;
      if (options.on_plan) {
        options.on_plan(result.plan, result.steps);
      }
    }
    result.optimal = true;
  }

  bool linearized() const {return linearized_;}

  Result run()
  {
    Result result;
    // A partially ordered problem is searched with each task network in one of the orders it
    // allows (as written, where the constraints leave it open). A plan found so is a plan of
    // the problem; not finding one proves nothing, since another order might do.
    linearized_ = !m_.total_order;
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
      free_ = std::make_unique<Relaxed>(p_, false, order_, initial_order_);
      if (free_->operators() > kHeuristicLimit) {
        free_.reset();  // too large to evaluate at every decomposition
      } else if (std::getenv("KLIMAX_FREE_ACTIONS") == nullptr) {
        ordered_ = std::make_unique<Relaxed>(p_, true, order_, initial_order_);
      }
      if (std::getenv("KLIMAX_TRACE_SEARCH") != nullptr && free_ != nullptr) {
        std::fprintf(stderr, "relaxed problem: %zu operators, ordered %zu\n", free_->operators(),
          ordered_ != nullptr ? ordered_->operators() : 0);
      }
      relaxed_ = free_.get();
      bfs_[0].relaxed = free_.get();
      bfs_[1].relaxed = ordered_.get();
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
      if (estimate >= Relaxed::kFar) {
        guided_bound = 1000000;  // no estimate to go by
      }
    }
    // Two ways of searching, each better on some domains: guided by the relaxed costs, and with
    // the methods in their fixed order. They take turns, each turn with twice the work of
    // the one before, so that the total is at most a few times what the better one needs.
    const bool trace = std::getenv("KLIMAX_TRACE_SEARCH") != nullptr;
    // Share of each way in a turn: best first with the free and with the ordered relaxation,
    // depth first guided and in fixed order (KLIMAX_TURNS=a,b,c,d to try others)
    // Measured at 1 s on the total-order benchmark of IPC 2023: equal shares solve 420, twice
    // for the first one 413 (more at 100 ms, fewer in Logistics)
    uint64_t share[4] = {1, 1, 1, 1};
    if (const char * given = std::getenv("KLIMAX_TURNS")) {
      std::sscanf(given, "%lu,%lu,%lu,%lu", &share[0], &share[1], &share[2], &share[3]);
    }
    for (uint64_t turn = 4000; ; turn *= 2) {
      if (linearized_ && (po_over_ || free_ == nullptr) && dfs_over_[1] &&
        (dfs_over_[0] || relaxed_ == nullptr) && (bfs_[0].over || bfs_[0].relaxed == nullptr) &&
        (bfs_[1].over || bfs_[1].relaxed == nullptr))
      {
        result.outcome = Outcome::Unsolvable;  // not a proof: see total_order()
        return result;
      }
      if (linearized_ && free_ != nullptr && !po_over_ && !po_exhausted_) {
        limit_ = work_ + turn;
        const Outcome outcome = partial_order(result);
        evaluated_ = false;
        result.expansions = expansions_;
        if (trace) {
          std::fprintf(stderr, "partial order: %s, %llu expansions so far, %zu nodes\n",
            outcome == Outcome::Solved ? "solved" : outcome == Outcome::Unsolvable ? "nothing" :
            "stopped", static_cast<unsigned long long>(expansions_), po_nodes_.size());
        }
        if (outcome == Outcome::Solved) {
          result.outcome = outcome;
          result.steps = bf_steps_;
          result.message = "best first, partial order";
          return result;
        }
        if (outcome == Outcome::Unsolvable) {
          result.outcome = outcome;  // every interleaving was searched
          proved_ = true;
          return result;
        }
        if (budget_ != nullptr && budget_->exhausted()) {
          result.outcome = Outcome::OutOfBudget;
          return result;
        }
      }
      for (Bf & search : bfs_) {
        if (search.relaxed == nullptr || search.over) {
          continue;
        }
        bf_ = &search;
        relaxed_ = search.relaxed;
        evaluated_ = false;
        limit_ = work_ + turn * share[search.relaxed->ordered() ? 1 : 0];
        if (limit_ == work_) {
          continue;
        }
        const Outcome outcome = best_first(result);
        relaxed_ = free_.get();
        evaluated_ = false;
        result.expansions = expansions_;
        if (trace) {
          std::fprintf(stderr, "best first (%s): %s, %llu expansions so far, %zu nodes\n",
            search.relaxed->ordered() ? "ordered" : "free",
            outcome == Outcome::Solved ? "solved" : outcome == Outcome::Unsolvable ? "nothing" :
            "stopped", static_cast<unsigned long long>(expansions_), search.nodes.size());
        }
        if (outcome == Outcome::Unsolvable && linearized_) {
          search.over = true;  // nothing in this order: the other ways go on
          continue;
        }
        // The ordered costs put far what they cannot reach instead of dropping it, so an empty
        // list is a proof with either
        if (outcome == Outcome::Solved || outcome == Outcome::Unsolvable) {
          result.outcome = outcome;
          result.steps = bf_steps_;
          result.message = search.relaxed->ordered() ? "best first, ordered relaxation" :
            "best first, free relaxation";
          return result;
        }
        if (budget_ != nullptr && budget_->exhausted()) {
          result.outcome = Outcome::OutOfBudget;
          return result;
        }
      }
      for (const bool guided : {true, false}) {
        if ((guided && relaxed_ == nullptr) || dfs_over_[guided ? 0 : 1]) {
          continue;
        }
        guided_ = guided;
        limit_ = relaxed_ == nullptr ? ~uint64_t{0} : work_ + turn * share[guided ? 2 : 3];
        if (limit_ == work_) {
          continue;
        }
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
          if (outcome == Outcome::Unsolvable && !cut_ && linearized_) {
            dfs_over_[guided ? 0 : 1] = true;  // nothing in this order: the other ways go on
            break;
          }
          if (outcome == Outcome::Solved || (outcome == Outcome::Unsolvable && !cut_)) {
            result.outcome = outcome;  // without cuts, the whole space was searched
            result.message = guided ? "depth first, guided" : "depth first, fixed order";
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
  static constexpr size_t kSearchMemory = size_t{3} << 29;  // 1.5 GB for each best-first search

  Outcome iteration(uint64_t bound, Result & result)
  {
    bound_ = bound;
    cut_ = false;
    initial_state();
    trail_.clear();
    cells_.clear();
    executed_.clear();
    choices_.clear();
    choice_task_.clear();
    seen_.clear();
    evaluated_ = false;
    top_ = kNone;
    // The ways of instantiating the initial network are the methods of a task above all
    nodes_.assign(1, Node{kNone, kNone, kNone, 0});
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
    // The atoms each component may read: in a precondition of an action or of a method, or in
    // the condition of an effect, through any decomposition
    std::vector<std::vector<uint32_t>> component_reads;
    std::vector<uint8_t> read(g_.atoms.size(), 0);
    size_t total_reads = 0;
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
          std::vector<uint32_t> reads;
          auto reads_atom = [&](ground::AtomId x) {
              if (!read[x]) {
                read[x] = 1;
                reads.push_back(x);
              }
            };
          auto reads_condition = [&](const ground::Condition & c) {
              std::for_each(c.positive.begin(), c.positive.end(), reads_atom);
              std::for_each(c.negative.begin(), c.negative.end(), reads_atom);
            };
          for (uint32_t member : members) {
            for (uint32_t a : p_.tasks[member].actions) {
              reads_condition(g_.actions[a].precondition);
              for (const ground::Effect & e : g_.actions[a].effects) {
                reads_condition(e.condition);
              }
            }
            for (uint32_t m : p_.tasks[member].methods) {
              for (uint32_t a : p_.methods[m].precondition) {
                reads_condition(g_.actions[a].precondition);
              }
            }
            for (uint32_t c : children[member]) {
              if (component[c] != id) {
                std::for_each(component_reads[component[c]].begin(),
                  component_reads[component[c]].end(), reads_atom);
              }
            }
          }
          for (uint32_t x : reads) {
            read[x] = 0;
          }
          total_reads += reads.size();
          component_reads.push_back(std::move(reads));
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

    // What is sure after a task, whatever its decomposition: for an action, what it adds and
    // deletes; for a method, what a step makes sure and no later step may undo; for an abstract
    // task, what all its methods make sure. As literals: atom * 2 sure true, + 1 sure false.
    std::vector<std::vector<uint32_t>> sure(nt);
    std::vector<bool> sure_known(nt, false);
    std::vector<std::vector<uint32_t>> method_sure(nm);
    std::vector<bool> method_sure_known(nm, false);
    for (size_t t = 0; t < nt && stack_walk_ > 0; ++t) {
      if (p_.tasks[t].actions.empty()) {
        continue;
      }
      // Common to every instance of the action
      bool first = true;
      for (uint32_t a : p_.tasks[t].actions) {
        std::vector<uint32_t> mine;
        for (ground::AtomId x : g_.actions[a].changes.add) {
          mine.push_back(x * 2);
        }
        for (ground::AtomId x : g_.actions[a].changes.del) {
          mine.push_back(x * 2 + 1);
        }
        // What a conditional effect may also change is not sure either way
        for (const ground::Effect & e : g_.actions[a].effects) {
          if (e.kind == ground::Effect::Kind::Add || e.kind == ground::Effect::Kind::Delete) {
            mine.erase(std::remove_if(mine.begin(), mine.end(), [&](uint32_t l) {
                  return l / 2 == e.atom;
                }), mine.end());
          }
        }
        std::sort(mine.begin(), mine.end());
        if (first) {
          sure[t] = std::move(mine);
          first = false;
        } else {
          std::vector<uint32_t> both;
          std::set_intersection(sure[t].begin(), sure[t].end(), mine.begin(), mine.end(),
            std::back_inserter(both));
          sure[t] = std::move(both);
        }
      }
      sure_known[t] = true;
    }
    if (stack_walk_ > 0) {
      std::vector<uint8_t> held(g_.atoms.size() * 2, 0);
      std::vector<uint32_t> pending;
      std::vector<bool> waiting(nm, true);
      for (uint32_t m = static_cast<uint32_t>(nm); m-- > 0; ) {
        pending.push_back(m);
      }
      while (!pending.empty()) {
        if (out_of_budget()) {
          return;
        }
        const uint32_t m = pending.back();
        pending.pop_back();
        waiting[m] = false;
        const tdg::Method & method = p_.methods[m];
        if (std::any_of(method.subtasks.begin(), method.subtasks.end(),
          [&](uint32_t t) {return !sure_known[t];}))
        {
          continue;
        }
        const std::vector<uint32_t> & order = method.method == kNone ? initial_order_ :
          order_[method.method];
        std::vector<uint32_t> now;  // literals sure so far, in `held`
        for (uint32_t step : order) {
          const uint32_t t = method.subtasks[step];
          // It may add an atom: no longer sure false; may delete it: no longer sure true
          for (uint32_t e : component_may[component[t]]) {
            held[e ^ 1u] = 0;
          }
          for (uint32_t l : sure[t]) {
            if (!held[l]) {
              held[l] = 1;
              now.push_back(l);
            }
          }
        }
        std::vector<uint32_t> mine;
        for (uint32_t l : now) {
          if (held[l]) {
            mine.push_back(l);
          }
          held[l] = 0;
        }
        std::sort(mine.begin(), mine.end());
        mine.erase(std::unique(mine.begin(), mine.end()), mine.end());
        if (method_sure_known[m] && mine == method_sure[m]) {
          continue;
        }
        method_sure[m] = std::move(mine);
        method_sure_known[m] = true;
        const uint32_t parent = method.task;
        if (parent == kNone) {
          continue;
        }
        std::vector<uint32_t> common;
        if (!sure_known[parent]) {
          common = method_sure[m];
        } else {
          std::set_intersection(sure[parent].begin(), sure[parent].end(),
            method_sure[m].begin(), method_sure[m].end(), std::back_inserter(common));
        }
        if (!sure_known[parent] || common.size() != sure[parent].size()) {
          sure[parent] = std::move(common);
          sure_known[parent] = true;
          for (uint32_t user : used_in[parent]) {
            if (!waiting[user]) {
              waiting[user] = true;
              pending.push_back(user);
            }
          }
        }
      }
    }
    // Which atoms of the goal each task may make true (the first 64 of them)
    task_reach_.assign(nt, 0);
    for (size_t i = 0; i < g_.goal.positive.size() && i < 64; ++i) {
      const uint32_t wanted = g_.goal.positive[i] * 2;
      for (size_t t = 0; t < nt; ++t) {
        const std::vector<uint32_t> & changes = component_may[component[t]];
        if (std::binary_search(changes.begin(), changes.end(), wanted)) {
          task_reach_[t] |= uint64_t{1} << i;
        }
      }
    }
    task_needs_ = std::move(needs);
    task_sure_ = std::move(sure);
    task_may_ = std::move(component_may);
    task_component_ = std::move(component);
    // The atoms of the goal matter to every stack: mask 0
    words_ = (g_.atoms.size() + 63) / 64;
    if (total_reads <= 50000000 && g_.atoms.size() <= (size_t{1} << 17) &&
      std::getenv("KLIMAX_NO_RELEVANCE") == nullptr)
    {
      component_reads_ = std::move(component_reads);
      masks_.assign(words_, 0);
      auto mark = [&](ground::AtomId a) {masks_[a / 64] |= uint64_t{1} << (a % 64);};
      std::for_each(g_.goal.positive.begin(), g_.goal.positive.end(), mark);
      std::for_each(g_.goal.negative.begin(), g_.goal.negative.end(), mark);
      mask_all_.assign(1, false);
      relevance_ = true;
    }
    known_.assign(g_.atoms.size(), 0);
  }

  // Looks down the stack from `top` with what is known of the state: before each task, what it
  // needs must not be surely false; after it, what it may change is unknown and what it makes
  // sure is known. False when some task further down can no longer be executed, whatever
  // happens in between. Works on the stack of cells given.
  bool stack_feasible(const std::vector<Cell> & cells, uint32_t top)
  {
    if (task_needs_.empty() || std::getenv("KLIMAX_NO_STACK_CHECK") != nullptr) {
      return true;
    }
    // What is missing of the goal must be something a task still in the stack may make true:
    // once the last one that could is done, it is too late
    const uint64_t reach = top == kNone ? 0 : cells[top].reach;
    for (size_t i = 0; i < g_.goal.positive.size() && i < 64; ++i) {
      if (!state_[g_.goal.positive[i]] && ((reach >> i) & 1u) == 0) {
        return false;
      }
    }
    if (stack_walk_ == 0) {
      return true;
    }
    // known_[atom]: 0 as in the state, 1 unknown, 2 surely true, 3 surely false
    touched_.clear();
    bool feasible = true;
    size_t work = 0;
    for (uint32_t c = top; c != kNone && feasible && work < stack_walk_; c = cells[c].next) {
      const uint32_t t = cells[c].task;
      for (uint32_t l : task_needs_[t]) {
        const uint32_t a = l / 2;
        const bool wanted = (l & 1u) == 0;
        const uint8_t k = known_[a];
        const bool surely_other = k == 0 ? state_[a] != wanted : k == (wanted ? 3 : 2);
        if (surely_other) {
          feasible = false;
          break;
        }
      }
      const std::vector<uint32_t> & may = task_may_[task_component_[t]];
      work += task_needs_[t].size() + may.size() + task_sure_[t].size();
      for (uint32_t e : may) {
        const uint32_t a = e / 2;
        const bool adds = (e & 1u) == 0;
        const uint8_t k = known_[a];
        const bool value = k == 0 ? state_[a] : k == 2;
        if (k != 1 && value != adds) {
          if (k == 0) {
            touched_.push_back(a);
          }
          known_[a] = 1;
        }
      }
      for (uint32_t l : task_sure_[t]) {
        const uint32_t a = l / 2;
        if (known_[a] == 0) {
          touched_.push_back(a);
        }
        known_[a] = (l & 1u) == 0 ? 2 : 3;
      }
    }
    for (uint32_t a : touched_) {
      known_[a] = 0;
    }
    return feasible;
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

  // The mask of a stack with `task` on top of one with mask `below`: the atoms some task in it
  // may read, and those of the goal. kNone: no mask, every atom counts.
  uint32_t mask_for(uint32_t task, uint32_t below)
  {
    if (!relevance_ || below == kNone) {
      return kNone;
    }
    const uint64_t key = (uint64_t{task_component_[task]} << 32) | below;
    if (const auto it = mask_of_.find(key); it != mask_of_.end()) {
      return it->second;
    }
    scratch_.assign(masks_.begin() + static_cast<std::ptrdiff_t>(below * words_),
      masks_.begin() + static_cast<std::ptrdiff_t>((below + 1) * words_));
    bool grew = false;
    for (uint32_t a : component_reads_[task_component_[task]]) {
      const uint64_t bit = uint64_t{1} << (a % 64);
      grew = grew || (scratch_[a / 64] & bit) == 0;
      scratch_[a / 64] |= bit;
    }
    uint32_t id = below;
    if (grew) {
      uint64_t content = 0;
      size_t count = 0;
      for (uint64_t w : scratch_) {
        content = mix(content ^ w);
        count += static_cast<size_t>(std::popcount(w));
      }
      // The same set reached another way is the same mask
      const auto [first, last] = mask_by_content_.equal_range(content);
      id = kNone;
      for (auto it = first; it != last; ++it) {
        if (std::equal(scratch_.begin(), scratch_.end(),
          masks_.begin() + static_cast<std::ptrdiff_t>(it->second * words_)))
        {
          id = it->second;
        }
      }
      if (id == kNone && masks_.size() * sizeof(uint64_t) < kMaskMemory) {
        id = static_cast<uint32_t>(mask_all_.size());
        masks_.insert(masks_.end(), scratch_.begin(), scratch_.end());
        mask_all_.push_back(count == g_.atoms.size());
        mask_by_content_.emplace(content, id);
      }
    }
    if (mask_of_.size() < 4000000) {
      mask_of_.emplace(key, id);
    }
    return id;
  }

  // Of the state as far as `mask` goes: two states that differ only in atoms that nothing left
  // to do reads, and the goal does not mention, have the same future
  uint64_t relevant_hash(uint32_t mask) const
  {
    if (mask == kNone || mask_all_[mask]) {
      return state_hash_;
    }
    const uint64_t * m = &masks_[size_t{mask} * words_];
    uint64_t h = 0x243f6a8885a308d3ull;
    for (size_t i = 0; i < words_; ++i) {
      h = (h ^ (bits_[i] & m[i])) * 0x9e3779b97f4a7c15ull;
      h ^= h >> 29;
    }
    return h;
  }

  void flip(ground::AtomId a)
  {
    state_[a] = !state_[a];
    bits_[a / 64] ^= uint64_t{1} << (a % 64);
  }

  void initial_state()
  {
    state_.assign(g_.atoms.size(), false);
    bits_.assign((g_.atoms.size() + 63) / 64, 0);
    state_hash_ = 0;
    for (ground::AtomId a : g_.init_atoms) {
      state_[a] = true;
      bits_[a / 64] |= uint64_t{1} << (a % 64);
      state_hash_ ^= zobrist_[a];
    }
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
      bits_[a / 64] ^= uint64_t{1} << (a % 64);
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
    uint32_t top = kNone;         // its stack, in bf_->cells
    uint32_t did_cell = kNone;    // the task it decomposed to get here (kNone: the initial one)
    uint32_t did_method = kNone;
    uint32_t first_child = 0;     // the cells that decomposition made
    uint32_t child_count = 0;
    uint32_t exec_start = 0;      // what was done when it was expanded, in bf_->exec
    uint32_t exec_count = 0;
    uint32_t actions = 0;         // of them, actions
    uint32_t state = 0;           // stored state: its parent's until expanded, then its own
  };

  // What a node did when expanded: an action (method kNone) or a decomposition with no
  // alternative, made on the spot
  struct BfEvent
  {
    uint32_t cell = 0;
    uint32_t method = kNone;
    uint32_t first_child = 0;
    uint32_t child_count = 0;
  };

  // The cells of the subtasks of `method` on top of `below`; returns the new top
  uint32_t bf_push(const tdg::Method & method, uint32_t below, uint32_t & first, uint32_t & count)
  {
    const std::vector<uint32_t> & order = method.method == kNone ? initial_order_ :
      order_[method.method];
    first = static_cast<uint32_t>(bf_->cells.size());
    count = static_cast<uint32_t>(order.size());
    bf_->cells.resize(bf_->cells.size() + order.size());
    uint32_t top = below;
    for (size_t k = order.size(); k-- > 0; ) {
      Cell & made = bf_->cells[first + k];
      made.task = method.subtasks[order[k]];
      made.next = top;
      made.hash = mix((top == kNone ? 0x9e3779b97f4a7c15ull : bf_->cells[top].hash) ^
          (uint64_t{made.task} + 0x632be59bd9b4e019ull));
      made.reach = (task_reach_.empty() ? ~uint64_t{0} : task_reach_[made.task]) |
        (top == kNone ? 0 : bf_->cells[top].reach);
      made.mask = mask_for(made.task, top == kNone ? 0 : bf_->cells[top].mask);
      top = first + static_cast<uint32_t>(k);
    }
    return top;
  }

  // The methods of a task whose precondition and inferred needs hold in the current state
  std::vector<uint32_t> bf_applicable(uint32_t task) const
  {
    std::vector<uint32_t> out;
    for (uint32_t m : task == kNone ? initial_ : methods_[task]) {
      const tdg::Method & method = p_.methods[m];
      if (method.has_precondition && std::none_of(method.precondition.begin(),
        method.precondition.end(), [&](uint32_t a) {return holds(g_.actions[a].precondition);}))
      {
        continue;
      }
      if (needs_hold(m)) {
        out.push_back(m);
      }
    }
    return out;
  }

  void bf_store()
  {
    const size_t words = (state_.size() + 63) / 64;
    const size_t at = bf_->states.size();
    bf_->states.resize(at + words, 0);
    for (size_t a = 0; a < state_.size(); ++a) {
      if (state_[a]) {
        bf_->states[at + a / 64] |= uint64_t{1} << (a % 64);
      }
    }
    bf_->hashes.push_back(state_hash_);
  }

  void bf_load(uint32_t index)
  {
    const size_t words = (state_.size() + 63) / 64;
    const size_t at = size_t{index} * words;
    for (size_t a = 0; a < state_.size(); ++a) {
      state_[a] = (bf_->states[at + a / 64] >> (a % 64)) & 1u;
    }
    std::copy_n(bf_->states.begin() + static_cast<std::ptrdiff_t>(at), words, bits_.begin());
    state_hash_ = bf_->hashes[index];
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
  void bf_successors(
    uint32_t node, const std::vector<uint32_t> & methods, uint32_t cell, uint32_t below)
  {
    // Measured: valuing each successor from its parent's state, and only looking further when
    // it is expanded, solves more than valuing it after its first actions
    const bool look_ahead = relaxed_->operators() <= kLookAheadLimit &&
      std::getenv("KLIMAX_BF_LOOK_AHEAD") != nullptr;
    for (uint32_t m : methods) {
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
      uint64_t h = 0;
      if (possible && plan_costs_[relaxed_->ordered() ? 1 : 0]) {
        // The actions of one relaxed plan for everything left: the subtasks, the stack and
        // the goal. What several of them need is counted once.
        bf_evaluate();
        const size_t start = relaxed_->plan_size();
        auto task = [&](uint32_t t) {
            const uint32_t c = relaxed_->task(t);
            possible = possible && c != Relaxed::kUnreached;
            h += c >= Relaxed::kFar ? c : relaxed_->plan_task(t);
          };
        for (size_t k = done; k < order.size() && possible; ++k) {
          task(method.subtasks[order[k]]);
        }
        uint32_t c = below;
        for (int depth = 0; c != kNone && depth < 64 && possible; ++depth) {
          task(bf_->cells[c].task);
          c = bf_->cells[c].next;
        }
        for (size_t i = 0; i < g_.goal.positive.size() && possible; ++i) {
          const uint32_t cost = relaxed_->atom(g_.goal.positive[i]);
          possible = cost != Relaxed::kUnreached;
          h += cost >= Relaxed::kFar ? cost : relaxed_->plan_atom(g_.goal.positive[i]);
        }
        relaxed_->plan_undo(start);
      } else if (possible) {
        bf_evaluate();
        for (size_t k = done; k < order.size() && possible; ++k) {
          const uint32_t c = relaxed_->task(method.subtasks[order[k]]);
          possible = c != Relaxed::kUnreached;
          h += c;
        }
        uint32_t c = below;
        for (int depth = 0; c != kNone && depth < 64 && possible; ++depth) {
          const uint32_t cost = relaxed_->task(bf_->cells[c].task);
          possible = cost != Relaxed::kUnreached;
          h += cost;
          c = bf_->cells[c].next;
        }
        const uint64_t goal = goal_cost();
        possible = possible && goal != kInfinite;
        h += goal == kInfinite ? 0 : goal;
      }
      while (trail_.size() > mark) {
        const ground::AtomId a = trail_.back();
        trail_.pop_back();
        flip(a);
      }
      state_hash_ = hash;
      if (!possible) {
        continue;
      }
      BfNode child;
      child.parent = node;
      child.did_cell = cell;
      child.did_method = m;
      child.state = bf_->nodes[node].state;
      child.top = bf_push(method, below, child.first_child, child.child_count);
      if (!stack_feasible(bf_->cells, child.top)) {
        bf_->cells.resize(child.first_child);
        continue;  // something further down the stack can no longer be executed
      }
      bf_->nodes.push_back(child);
      // Lowest cost first
      h = std::min<uint64_t>(h, 0xffffffffull);
      const uint64_t age = bf_oldest_first_ ? bf_->nodes.size() : 0xffffffffull - bf_->nodes.size();
      bf_->open.emplace_back((h << 32) | age, static_cast<uint32_t>(bf_->nodes.size() - 1));
      std::push_heap(bf_->open.begin(), bf_->open.end(), std::greater<>());
    }
  }

  Outcome best_first(Result & result)
  {
    if (bf_->nodes.empty()) {
      initial_state();
      bf_store();
      bf_->nodes.push_back(BfNode{});
      evaluated_ = false;
      bf_successors(0, bf_applicable(kNone), kNone, kNone);
    }
    while (!bf_->open.empty()) {
      if ((budget_ != nullptr && budget_->exhausted()) || work_ >= limit_) {
        return Outcome::OutOfBudget;
      }
      if (bf_->states.size() * sizeof(uint64_t) + bf_->cells.size() * sizeof(Cell) +
        bf_->nodes.size() * sizeof(BfNode) + bf_->exec.size() * sizeof(BfEvent) +
        bf_->open.size() * 16 + bf_->seen.size() * 32 > kSearchMemory)
      {
        bf_->over = true;  // out of room: the other searches go on
        return Outcome::OutOfBudget;
      }
      std::pop_heap(bf_->open.begin(), bf_->open.end(), std::greater<>());
      const uint32_t n = bf_->open.back().second;
      if (std::getenv("KLIMAX_TRACE_BEST_FIRST") != nullptr && (++bf_->pops & 1023u) == 0) {
        size_t depth = 0;
        for (uint32_t k = n; k != kNone; k = bf_->nodes[k].parent) {
          depth += bf_->nodes[k].actions;
        }
        std::fprintf(stderr, "pop %llu: h %llu, %zu actions so far, open %zu\n",
          static_cast<unsigned long long>(bf_->pops),
          static_cast<unsigned long long>(bf_->open.back().first >> 32), depth, bf_->open.size());
      }
      bf_->open.pop_back();
      ++expansions_;
      ++work_;
      bf_load(bf_->nodes[n].state);
      evaluated_ = false;
      // Everything without a choice is done here: the actions at the top of the stack, and the
      // decompositions for which a single method can be applied. Only a real choice makes
      // successors, so that chains of bookkeeping cost neither nodes nor evaluations.
      uint32_t top = bf_->nodes[n].top;
      const uint32_t exec_start = static_cast<uint32_t>(bf_->exec.size());
      uint32_t actions = 0;
      bool dead = false;
      std::vector<uint32_t> methods;
      for (int forced = 0; ; ) {
        while (top != kNone && !p_.tasks[bf_->cells[top].task].actions.empty()) {
          const ground::Action * action = nullptr;
          for (uint32_t a : p_.tasks[bf_->cells[top].task].actions) {
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
          bf_->exec.push_back(BfEvent{top, kNone, 0, 0});
          top = bf_->cells[top].next;
          ++actions;
          ++work_;
        }
        if (dead || top == kNone) {
          break;
        }
        if (!bf_->seen.insert(mix(relevant_hash(bf_->cells[top].mask) ^
          bf_->cells[top].hash)).second)
        {
          dead = true;
          break;
        }
        methods = bf_applicable(bf_->cells[top].task);
        if (methods.size() != 1 || ++forced > 256) {
          dead = methods.empty();
          break;
        }
        BfEvent event;
        event.cell = top;
        event.method = methods[0];
        top = bf_push(p_.methods[methods[0]], bf_->cells[top].next, event.first_child,
            event.child_count);
        bf_->exec.push_back(event);
        ++work_;
      }
      trail_.clear();
      if (dead) {
        bf_->exec.resize(exec_start);
        continue;
      }
      bf_->nodes[n].exec_start = exec_start;
      bf_->nodes[n].exec_count = static_cast<uint32_t>(bf_->exec.size()) - exec_start;
      bf_->nodes[n].actions = actions;
      if (top == kNone) {
        if (holds(g_.goal)) {
          result.plan = bf_plan(n);
          return Outcome::Solved;
        }
        continue;
      }
      bf_->nodes[n].state = static_cast<uint32_t>(bf_->hashes.size());
      bf_store();
      bf_successors(n, methods, top, bf_->cells[top].next);
    }
    return Outcome::Unsolvable;
  }

  // The plan of the path that ends in `goal`, without the tasks added by parameter splitting
  std::string bf_plan(uint32_t goal) const
  {
    std::vector<uint32_t> path;
    for (uint32_t n = goal; n != kNone; n = bf_->nodes[n].parent) {
      path.push_back(n);
    }
    std::reverse(path.begin(), path.end());
    auto text = [&](uint32_t cell) {return std::to_string(cell + 1);};
    auto artificial_cell = [&](uint32_t cell) {
        return m_.tasks[p_.tasks[bf_->cells[cell].task].task].artificial;
      };
    // Every decomposition on the path, in order: the one that made each node and those it did
    // on the spot
    std::vector<BfEvent> events;
    for (uint32_t n : path) {
      const BfNode & node = bf_->nodes[n];
      if (node.did_method != kNone) {
        events.push_back(BfEvent{node.did_cell, node.did_method, node.first_child,
            node.child_count});
      }
      for (uint32_t k = 0; k < node.exec_count; ++k) {
        events.push_back(bf_->exec[node.exec_start + k]);
      }
    }
    std::unordered_map<uint32_t, uint32_t> only_child;  // of the cells added by splitting
    for (const BfEvent & e : events) {
      if (e.method != kNone && e.cell != kNone && artificial_cell(e.cell)) {
        only_child[e.cell] = e.first_child;
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
    for (const BfEvent & e : events) {
      if (e.method == kNone) {
        actions += text(e.cell) + " " + name(bf_->cells[e.cell].task) + "\n";
        ++steps;
        continue;
      }
      std::string children;
      for (uint32_t k = 0; k < e.child_count; ++k) {
        children += " " + text(real(e.first_child + k));
      }
      if (e.cell == kNone) {
        roots += children;
      } else if (!artificial_cell(e.cell)) {
        tree += text(e.cell) + " " + name(bf_->cells[e.cell].task) + " -> " +
          m_.methods[p_.methods[e.method].method].name + children + "\n";
      }
    }
    bf_steps_ = steps;
    return actions + roots + "\n" + tree + "<==\n";
  }

  // --- Partial order ---------------------------------------------------------------------------
  // Greedy best-first search over (state, task network), for problems that need the subtasks
  // of different tasks interleaved. A node holds the tasks still to do and the orderings
  // between them; any task with nothing before it may go next: an action is executed, an
  // abstract task is replaced by the network of one of its methods (what came after the task
  // comes after all of it). Valued with the free relaxation, which ignores order as the
  // problem does. Keeps what it has between turns.

  struct PoItem
  {
    uint32_t task = 0;
    uint32_t id = 0;  // of this occurrence, in the plan
  };

  struct PoNode
  {
    uint32_t parent = kNone;
    uint32_t state = 0;        // stored state
    uint32_t items = 0;        // its network: po_items_[items .. items + item_count)
    uint32_t item_count = 0;
    uint32_t edges = 0;        // po_edges_: pairs (a, b) of positions, a before b
    uint32_t edge_count = 0;
    // What made it: an action (method kNone) or a decomposition
    uint32_t did = kNone;      // id of the occurrence
    uint32_t did_task = 0;
    uint32_t method = kNone;
    uint32_t first_child = 0;  // ids of the subtasks, by step of the method
    uint32_t child_count = 0;
    bool root = false;
    uint32_t actions = 0;      // done to get here
  };

  uint64_t po_key(const PoNode & node) const
  {
    // The same whatever the order the items are stored in
    uint64_t sum = 0;
    for (uint32_t i = 0; i < node.item_count; ++i) {
      sum += mix(uint64_t{po_items_[node.items + i].task} + 0x9e3779b97f4a7c15ull);
    }
    for (uint32_t e = 0; e < node.edge_count; ++e) {
      const auto [a, b] = po_edges_[node.edges + e];
      sum += mix((uint64_t{po_items_[node.items + a].task} << 32) ^
          po_items_[node.items + b].task ^ 0x51ed270b5bd1e995ull);
    }
    return mix(state_hash_ ^ sum);
  }

  void po_push(PoNode node, uint64_t h)
  {
    po_nodes_.push_back(node);
    po_open_.emplace_back((std::min<uint64_t>(h, 0xffffffffull) << 32) | po_nodes_.size(),
      static_cast<uint32_t>(po_nodes_.size() - 1));
    std::push_heap(po_open_.begin(), po_open_.end(), std::greater<>());
  }

  // The network of `from` without the item at `position`, with the steps of `method` in its
  // place (none for an action)
  void po_replace(const PoNode & from, uint32_t position, const tdg::Method * method, PoNode & to)
  {
    const uint32_t steps = method == nullptr ? 0 : static_cast<uint32_t>(method->subtasks.size());
    to.items = static_cast<uint32_t>(po_items_.size());
    to.edges = static_cast<uint32_t>(po_edges_.size());
    // Old positions after `position` move one down; the steps go at the end
    auto moved = [&](uint32_t old) {return old < position ? old : old - 1;};
    for (uint32_t i = 0; i < from.item_count; ++i) {
      if (i != position) {
        po_items_.push_back(po_items_[from.items + i]);
      }
    }
    const uint32_t first = from.item_count - 1;
    to.first_child = po_ids_;
    to.child_count = steps;
    for (uint32_t k = 0; k < steps; ++k) {
      po_items_.push_back(PoItem{method->subtasks[k], po_ids_++});
    }
    for (uint32_t e = 0; e < from.edge_count; ++e) {
      const auto [a, b] = po_edges_[from.edges + e];
      if (a == position) {
        // What came after the task comes after every one of its steps
        for (uint32_t k = 0; k < steps; ++k) {
          po_edges_.emplace_back(first + k, moved(b));
        }
      } else if (b != position) {  // the item had nothing before it
        po_edges_.emplace_back(moved(a), moved(b));
      }
    }
    if (method != nullptr) {
      const hddl::Network & net = method->method == kNone ? m_.initial :
        m_.methods[method->method].network;
      for (auto [a, b] : net.order) {
        po_edges_.emplace_back(first + a, first + b);
      }
    }
    to.item_count = static_cast<uint32_t>(po_items_.size()) - to.items;
    to.edge_count = static_cast<uint32_t>(po_edges_.size()) - to.edges;
  }

  Outcome partial_order(Result & result)
  {
    relaxed_ = free_.get();
    if (po_nodes_.empty()) {
      initial_state();
      po_states_.clear();
      po_hashes_.clear();
      po_store();
      // One node per way of instantiating the initial network
      PoNode empty;
      for (uint32_t m : initial_) {
        PoNode node;
        node.root = true;
        node.method = m;
        // An item to replace: the task above all
        empty.items = static_cast<uint32_t>(po_items_.size());
        empty.item_count = 1;
        empty.edges = static_cast<uint32_t>(po_edges_.size());
        empty.edge_count = 0;
        po_items_.push_back(PoItem{0, kNone});
        po_replace(empty, 0, &p_.methods[m], node);
        po_push(node, 0);
      }
    }
    while (!po_open_.empty()) {
      if ((budget_ != nullptr && budget_->exhausted()) || work_ >= limit_) {
        return Outcome::OutOfBudget;
      }
      // Each search keeps what it has between turns: none may take more than its part of the
      // memory, or the others would have none
      if ((po_states_.size() + po_items_.size() + po_edges_.size()) * sizeof(uint64_t) +
        po_nodes_.size() * sizeof(PoNode) + po_open_.size() * 16 +
        (po_seen_.size() + po_fewest_.size()) * 32 >
        kSearchMemory)
      {
        po_over_ = true;
        return Outcome::OutOfBudget;
      }
      std::pop_heap(po_open_.begin(), po_open_.end(), std::greater<>());
      const uint32_t n = po_open_.back().second;
      po_open_.pop_back();
      ++expansions_;
      ++work_;
      const PoNode node = po_nodes_[n];
      po_load(node.state);
      if (node.item_count == 0) {
        if (holds(g_.goal)) {
          result.plan = po_plan(n);
          return Outcome::Solved;
        }
        continue;
      }
      if (po_bound_ == kInfinite) {
        if (!po_seen_.insert(po_key(node)).second) {
          continue;
        }
      } else {
        // Under a bound on the length, the same again only counts with fewer actions done
        const auto [seen, fresh] = po_fewest_.emplace(po_key(node), node.actions);
        if (!fresh) {
          if (seen->second <= node.actions) {
            continue;
          }
          seen->second = node.actions;
        }
        uint64_t least = node.actions;
        for (uint32_t i = 0; i < node.item_count; ++i) {
          least += task_cost_[po_items_[node.items + i].task];
        }
        if (least > po_bound_) {
          continue;  // no plan this short through here
        }
      }
      relaxed_->evaluate(state_);
      work_ += relaxed_->operators() / 32 + 1;
      uint64_t base = goal_cost();
      bool possible = base != kInfinite;
      for (uint32_t i = 0; i < node.item_count && possible; ++i) {
        const uint32_t c = relaxed_->task(po_items_[node.items + i].task);
        possible = c != Relaxed::kUnreached;
        base += c;
      }
      if (!possible) {
        continue;
      }
      // The items with nothing before them
      std::vector<bool> waits(node.item_count, false);
      for (uint32_t e = 0; e < node.edge_count; ++e) {
        waits[po_edges_[node.edges + e].second] = true;
      }
      for (uint32_t i = 0; i < node.item_count; ++i) {
        if (waits[i]) {
          continue;
        }
        const PoItem item = po_items_[node.items + i];
        const tdg::Task & task = p_.tasks[item.task];
        const uint64_t without = base - relaxed_->task(item.task);
        if (!task.actions.empty()) {
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
          PoNode child;
          child.parent = n;
          child.did = item.id;
          child.did_task = item.task;
          child.actions = node.actions + 1;
          po_replace(node, i, nullptr, child);
          const size_t mark = trail_.size();
          const uint64_t hash = state_hash_;
          apply(*action);
          child.state = static_cast<uint32_t>(po_hashes_.size());
          po_store();
          while (trail_.size() > mark) {
            const ground::AtomId a = trail_.back();
            trail_.pop_back();
            flip(a);
          }
          state_hash_ = hash;
          po_push(child, without);
          continue;
        }
        for (uint32_t m : methods_[item.task]) {
          const tdg::Method & method = p_.methods[m];
          if (method.has_precondition && std::none_of(method.precondition.begin(),
            method.precondition.end(),
            [&](uint32_t a) {return holds(g_.actions[a].precondition);}))
          {
            continue;
          }
          const uint32_t cost = relaxed_->method(m);
          if (cost == Relaxed::kUnreached) {
            continue;
          }
          PoNode child;
          child.parent = n;
          child.state = node.state;
          child.did = item.id;
          child.did_task = item.task;
          child.method = m;
          child.actions = node.actions;
          po_replace(node, i, &method, child);
          po_push(child, without + cost);
        }
      }
    }
    return Outcome::Unsolvable;
  }

  void po_store()
  {
    const size_t words = (state_.size() + 63) / 64;
    const size_t at = po_states_.size();
    po_states_.resize(at + words, 0);
    for (size_t a = 0; a < state_.size(); ++a) {
      if (state_[a]) {
        po_states_[at + a / 64] |= uint64_t{1} << (a % 64);
      }
    }
    po_hashes_.push_back(state_hash_);
  }

  void po_load(uint32_t index)
  {
    const size_t words = (state_.size() + 63) / 64;
    const size_t at = size_t{index} * words;
    for (size_t a = 0; a < state_.size(); ++a) {
      state_[a] = (po_states_[at + a / 64] >> (a % 64)) & 1u;
    }
    std::copy_n(po_states_.begin() + static_cast<std::ptrdiff_t>(at), words, bits_.begin());
    state_hash_ = po_hashes_[index];
  }

  std::string po_plan(uint32_t goal) const
  {
    std::vector<uint32_t> path;
    for (uint32_t n = goal; n != kNone; n = po_nodes_[n].parent) {
      path.push_back(n);
    }
    std::reverse(path.begin(), path.end());
    auto text = [&](uint32_t id) {return std::to_string(id + 1);};
    // The task of each occurrence, and the only subtask of those added by splitting
    std::unordered_map<uint32_t, uint32_t> task_of;
    std::unordered_map<uint32_t, uint32_t> only_child;
    for (uint32_t n : path) {
      const PoNode & node = po_nodes_[n];
      if (node.method == kNone) {
        continue;
      }
      const tdg::Method & method = p_.methods[node.method];
      for (uint32_t k = 0; k < node.child_count; ++k) {
        task_of[node.first_child + k] = method.subtasks[k];
      }
      if (!node.root && m_.tasks[p_.tasks[node.did_task].task].artificial) {
        only_child[node.did] = node.first_child;
      }
    }
    auto real = [&](uint32_t id) {
        while (m_.tasks[p_.tasks[task_of.at(id)].task].artificial) {
          id = only_child.at(id);
        }
        return id;
      };
    std::string actions = "==>\n";
    std::string roots = "root";
    std::string tree;
    size_t steps = 0;
    for (uint32_t n : path) {
      const PoNode & node = po_nodes_[n];
      if (node.method == kNone) {
        actions += text(node.did) + " " + name(node.did_task) + "\n";
        ++steps;
        continue;
      }
      const tdg::Method & method = p_.methods[node.method];
      // The subtasks in an order the method allows, as the format asks
      const std::vector<uint32_t> & order = method.method == kNone ? initial_order_ :
        order_[method.method];
      std::string children;
      for (uint32_t step : order) {
        children += " " + text(real(node.first_child + step));
      }
      if (node.root) {
        roots += children;
      } else if (!m_.tasks[p_.tasks[node.did_task].task].artificial) {
        tree += text(node.did) + " " + name(node.did_task) + " -> " +
          m_.methods[method.method].name + children + "\n";
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
        flip(a);
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
    // A task inside itself with nothing done in between: in a plan with as few steps as can
    // be, each of them covers fewer actions than the one around it (otherwise the inner one
    // would do in its place), so there are at most as many as actions are left under the
    // bound. Without this, a method that calls its own task before any action never ends.
    const uint32_t done_now = static_cast<uint32_t>(executed_.size());
    if (task != kNone) {
      uint64_t nested = 0;
      for (uint32_t a = nodes_[node].parent; a != kNone && a != 0 && nodes_[a].start == done_now;
        a = nodes_[a].parent)
      {
        nested += nodes_[a].task == task;
      }
      if (nested > bound_ - std::min<uint64_t>(bound_, done_now) + 1) {
        cut_ = true;
        return false;
      }
    }
    nodes_[node].start = done_now;
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
        nodes_.push_back(Node{method.subtasks[step], node, kNone, 0});
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
        cell.reach = (task_reach_.empty() ? ~uint64_t{0} : task_reach_[cell.task]) |
          (top == kNone ? 0 : cells_[top].reach);
        cell.mask = mask_for(cell.task, top == kNone ? 0 : cells_[top].mask);
        cells_.push_back(cell);
        top = static_cast<uint32_t>(cells_.size() - 1);
      }
      if (!stack_feasible(cells_, top)) {
        ++pruned_;
        cells_.resize(choice.cells);
        nodes_.resize(choice.nodes);
        continue;  // something further down the stack can no longer be executed
      }
      const uint64_t done = executed_.size();
      const bool too_long = done + (top == kNone ? 0 : cells_[top].rest) > bound_;
      cut_ = cut_ || too_long;
      // Seen with as few actions done: nothing new from here
      const uint64_t key = top == kNone ? mix(relevant_hash(relevance_ ? 0 : kNone) ^ 0x51ull) :
        mix(relevant_hash(cells_[top].mask) ^ cells_[top].hash);
      bool repeated = false;
      if (!too_long && seen_.size() >= 40000000) {
        // Out of room to remember more: search on without (slower, still correct)
        auto it = seen_.find(key);
        repeated = it != seen_.end() && it->second <= done;
      } else if (!too_long) {
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
        flip(a);
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
  std::vector<std::vector<uint32_t>> task_needs_;    // by ground task
  std::vector<std::vector<uint32_t>> task_sure_;     // literals sure after the task
  std::vector<std::vector<uint32_t>> task_may_;      // by component: what it may change
  std::vector<uint32_t> task_component_;
  std::vector<uint64_t> task_reach_;                 // goal atoms each task may make true
  // Relevance: which atoms a stack may still read (see mask_for)
  static constexpr size_t kMaskMemory = size_t{1} << 28;
  bool relevance_ = false;
  // Best first values what is left by the size of a relaxed plan, not by the sum of the costs
  // with the ordered relaxation; the free one keeps the sum. Measured at 1 s on the total-order
  // benchmark of IPC 2023: 440 solved so, 429 with plans in both, 423 with them only in the free
  // one, 421 with sums in both (KLIMAX_PLAN_COSTS=ab: a for the free one, b for the ordered)
  bool plan_costs_[2] = {
    std::getenv("KLIMAX_PLAN_COSTS") != nullptr && std::getenv("KLIMAX_PLAN_COSTS")[0] == '1',
    std::getenv("KLIMAX_PLAN_COSTS") == nullptr || std::getenv("KLIMAX_PLAN_COSTS")[1] == '1'};
  size_t words_ = 0;                                 // of a state as bits
  std::vector<uint64_t> bits_;                       // the state, as state_
  std::vector<std::vector<uint32_t>> component_reads_;
  std::vector<uint64_t> masks_;                      // words_ each
  std::vector<bool> mask_all_;                       // the mask has every atom
  std::unordered_map<uint64_t, uint32_t> mask_of_;   // (component, mask below) -> mask
  std::unordered_multimap<uint64_t, uint32_t> mask_by_content_;
  std::vector<uint64_t> scratch_;
  // How far stack_feasible looks down the stack with what is sure after each task, in entries
  // of the tables it reads (KLIMAX_STACK_WALK). Off: measured on the total-order benchmark of
  // IPC 2023 at 1 s, it costs time and prunes nothing the other checks do not (436 solved
  // without it, 415 looking 20000 entries ahead)
  size_t stack_walk_ = std::getenv("KLIMAX_STACK_WALK") != nullptr ?
    std::strtoul(std::getenv("KLIMAX_STACK_WALK"), nullptr, 10) : 0;
  std::vector<uint8_t> known_;                       // scratch of stack_feasible
  std::vector<uint32_t> touched_;
  uint64_t pruned_ = 0;
  bool linearized_ = false;  // the problem is partially ordered: one order of each network

public:
  bool proved_ = false;      // Unsolvable came from the search that tries every interleaving

private:
  std::unique_ptr<Relaxed> free_;     // any action anywhere: what it calls impossible, is
  std::unique_ptr<Relaxed> ordered_;  // effects only in the order of the methods: sharper
  Relaxed * relaxed_ = nullptr;       // the one in use; null: methods in their fixed order
  bool evaluated_ = false;            // relaxed_ holds the costs from evaluated_hash_
  uint64_t evaluated_hash_ = 0;
  uint64_t evaluations_ = 0;
  // One best-first search: what it has so far, and the costs that guide it
  struct Bf
  {
    Relaxed * relaxed = nullptr;
    std::vector<Cell> cells;
    std::vector<BfNode> nodes;
    std::vector<BfEvent> exec;
    std::vector<uint64_t> states;
    std::vector<uint64_t> hashes;
    std::vector<std::pair<uint64_t, uint32_t>> open;
    std::unordered_set<uint64_t> seen;
    bool over = false;  // out of room
    uint64_t pops = 0;
  };
  std::vector<PoNode> po_nodes_;
  std::vector<PoItem> po_items_;
  std::vector<std::pair<uint32_t, uint32_t>> po_edges_;
  std::vector<uint64_t> po_states_;
  std::vector<uint64_t> po_hashes_;
  std::vector<std::pair<uint64_t, uint32_t>> po_open_;
  std::unordered_set<uint64_t> po_seen_;
  std::unordered_map<uint64_t, uint32_t> po_fewest_;  // under a bound: fewest actions done
  uint64_t po_bound_ = kInfinite;                     // on the length of the plan
  uint32_t po_ids_ = 0;
  bool po_over_ = false;
  bool po_exhausted_ = false;  // its whole space was searched: no plan at all
  bool dfs_over_[2] = {false, false};  // guided, fixed order: nothing in the order tried
  Bf bfs_[2];           // guided by the free relaxation and by the ordered one
  Bf * bf_ = &bfs_[0];  // the one taking its turn
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

Result total_order(
  const tdg::Problem & problem, kairos::Budget * budget, const Options & options)
{
  if (problem.unsolvable) {
    Result result;
    result.outcome = Outcome::Unsolvable;  // whatever the order: nothing can be decomposed
    return result;
  }
  Search search(problem, budget);
  Result result = search.run();
  if (result.outcome == Outcome::Solved) {
    if (options.anytime) {
      search.improve(result, options);
    } else {
      result.plans = 1;
      if (options.on_plan) {
        options.on_plan(result.plan, result.steps);
      }
    }
  }
  if (result.outcome == Outcome::Unsolvable && !problem.model.total_order && !search.proved_) {
    result.outcome = Outcome::Unsupported;
    result.message = "no plan with the subtasks of each task network in one order, and the "
      "search that interleaves them ran out of memory";
  }
  return result;
}

}  // namespace klimax::search

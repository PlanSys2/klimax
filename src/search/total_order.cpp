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
#include <random>
#include <string>
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
    orders();
    state_.assign(g_.atoms.size(), false);
    std::mt19937_64 random(20261124);
    for (size_t a = 0; a < g_.atoms.size(); ++a) {
      zobrist_.push_back(random());
    }
    for (ground::AtomId a : g_.init_atoms) {
      state_[a] = true;
      state_hash_ ^= zobrist_[a];
    }
    // The ways of instantiating the initial network are the methods of a task above all
    nodes_.push_back(Node{kNone, kNone, kNone});
    bool found = descend(kNone, 0, kNone, 0);
    while (found || backtrack()) {
      found = false;
      if (budget_ != nullptr && budget_->exhausted()) {
        result.outcome = Outcome::OutOfBudget;
        result.expansions = expansions_;
        return result;
      }
      if (top_ == kNone) {
        if (holds(g_.goal)) {
          result.outcome = Outcome::Solved;
          result.plan = plan();
          result.steps = executed_.size();
          result.expansions = expansions_;
          return result;
        }
        continue;  // every task done but the goal does not hold
      }
      ++expansions_;
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
    result.outcome = Outcome::Unsolvable;
    result.expansions = expansions_;
    return result;
  }

private:
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

  // Replaces a task by the subtasks of its first applicable method from `from` on whose result
  // was not seen before. `task` is kNone for the initial network.
  bool descend(uint32_t task, uint32_t node, uint32_t below, uint32_t from)
  {
    const std::vector<uint32_t> & methods = task == kNone ? p_.initial : p_.tasks[task].methods;
    for (uint32_t i = from; i < methods.size(); ++i) {
      const tdg::Method & method = p_.methods[methods[i]];
      if (method.has_precondition && std::none_of(method.precondition.begin(),
        method.precondition.end(), [&](uint32_t a) {return holds(g_.actions[a].precondition);}))
      {
        continue;
      }
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
        cells_.push_back(cell);
        top = static_cast<uint32_t>(cells_.size() - 1);
      }
      const uint64_t key = mix(state_hash_ ^ (top == kNone ? 0x51ull : cells_[top].hash));
      if (!seen_.insert(key).second) {
        cells_.resize(choice.cells);
        nodes_.resize(choice.nodes);
        continue;
      }
      nodes_[node].method = methods[i];
      choices_.push_back(choice);
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
      const Choice choice = choices_.back();
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
      if (descend(task, node, below, choice.next_method)) {
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
  std::unordered_set<uint64_t> seen_;
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

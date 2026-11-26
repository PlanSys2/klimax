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

#include "verify/verifier.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "validate/validator.hpp"
#include "validate/world.hpp"
#include "verify/interpreter.hpp"

namespace klimax::verifier
{

namespace
{

namespace val = kairos::val;

// --- The plan as written ----------------------------------------------------------------------

struct PlanLine
{
  std::string id;
  std::string name;
  std::vector<std::string> args;
  bool abstract = false;
  std::string method;
  std::vector<std::string> children;
  std::optional<double> time;      // "id time: (action args) [duration]"
  std::optional<double> duration;
};

struct PlanText
{
  std::vector<PlanLine> lines;  // primitive steps in plan order, then decompositions
  std::vector<std::string> roots;
};

std::string lowered(std::string s)
{
  for (char & c : s) {
    if (c >= 'A' && c <= 'Z') {
      c = static_cast<char>(c - 'A' + 'a');
    }
  }
  return s;
}

PlanText read_plan(std::string_view text)
{
  PlanText plan;
  std::istringstream in{std::string(text)};
  std::string line;
  bool inside = false;
  bool seen_root = false;
  while (std::getline(in, line)) {
    for (char & c : line) {
      if (c == '(' || c == ')' || c == '\r' || c == '\t') {
        c = ' ';
      }
    }
    std::istringstream words(lowered(line));
    std::vector<std::string> w;
    for (std::string word; words >> word; ) {
      w.push_back(word);
    }
    if (w.empty() || w[0][0] == ';') {
      continue;
    }
    if (w[0] == "==>") {
      inside = true;
      continue;
    }
    if (w[0] == "<==") {
      break;
    }
    if (!inside) {
      continue;
    }
    if (w[0] == "root") {
      if (seen_root) {
        invalid("the plan has two 'root' lines");
      }
      seen_root = true;
      plan.roots.assign(w.begin() + 1, w.end());
      continue;
    }
    PlanLine entry;
    // A step with its time: "id time: name args [duration]"
    if (w.size() >= 2 && w[1].back() == ':') {
      auto number = [&](std::string digits) {
          try {
            size_t used = 0;
            const double value = std::stod(digits, &used);
            if (used == digits.size() && value >= 0.0) {
              return value;
            }
          } catch (const std::exception &) {
          }
          invalid("plan line '" + line + "': '" + digits + "' is not a time");
        };
      entry.time = number(w[1].substr(0, w[1].size() - 1));
      w.erase(w.begin() + 1);
      if (w.back().size() > 2 && w.back().front() == '[' && w.back().back() == ']') {
        entry.duration = number(w.back().substr(1, w.back().size() - 2));
        w.pop_back();
      }
    }
    if (w.size() < 2) {
      invalid("plan line '" + line + "' has no task");
    }
    entry.id = w[0];
    entry.name = w[1];
    auto arrow = std::find(w.begin() + 2, w.end(), "->");
    entry.args.assign(w.begin() + 2, arrow);
    if (arrow != w.end()) {
      if (arrow + 1 == w.end()) {
        invalid("decomposition of " + entry.id + " names no method");
      }
      entry.abstract = true;
      entry.method = *(arrow + 1);
      entry.children.assign(arrow + 2, w.end());
    } else if (seen_root) {
      invalid("plan line '" + line + "' after 'root' is not a decomposition");
    }
    plan.lines.push_back(std::move(entry));
  }
  if (!inside) {
    invalid("no plan found: the '==>' line is missing");
  }
  if (!seen_root) {
    invalid("the plan has no 'root' line");
  }
  return plan;
}

// --- The decomposition tree -------------------------------------------------------------------

struct Node
{
  std::string id;
  TaskId task;
  std::vector<uint32_t> args;
  bool primitive = false;
  uint32_t position = 0;       // primitive: index in the sequence
  uint32_t method = kNone;     // abstract
  std::vector<uint32_t> children;  // abstract: as written, then in the order of the method
  uint32_t parent = kNone;
  bool is_root = false;
  uint32_t first_leaf = kNone;  // smallest and largest position below; kNone if no leaf
  uint32_t last_leaf = kNone;
  std::vector<std::vector<uint32_t>> bindings;  // abstract: values of the method's parameters
  size_t held = 0;  // the one of them its precondition held with
  // Too many to list: the precondition chooses, from `partial` (kNone where free)
  bool open = false;
  std::vector<uint32_t> partial;
  // Events of the node in the precedence graph
  uint32_t start = 0;
  uint32_t test = kNone;
  uint32_t end = 0;
  // Times (HDDL 2.1). A task starts when the first of its actions does and ends when the last
  // one does; with none, when its method was applied
  double from = 0.0;
  double to = 0.0;
  double tested = 0.0;          // abstract: when its method was applied
  // abstract: of the functions in its duration constraints, read when its method was applied
  std::vector<std::pair<ast::ExprId, double>> values;
  bool began = false;           // some action below it has started
};

struct Event
{
  uint32_t pending = 0;  // predecessors not yet placed
  std::vector<uint32_t> next;
  uint32_t node = kNone;  // tests only
  bool placed = false;
};

class Verifier
{
public:
  Verifier(const Model & model, std::string_view text)
  : m_(model), plan_(read_plan(text)),
    world_(model.pddl, model.domain, model.problem, model.objects, exact())
  {
  }

  void run()
  {
    if (!m_.has_htn) {
      invalid("unsupported: a problem without :htn is a PDDL problem");
    }
    if (!m_.pddl.derived.empty() || m_.problem.constraints.valid() || m_.pddl.constraints.valid()) {
      invalid("unsupported: derived predicates or trajectory constraints");
    }
    for (size_t i = 0; i < m_.tasks.size(); ++i) {
      task_index_.emplace(m_.tasks[TaskId(static_cast<uint32_t>(i))].name, i);
    }
    for (size_t i = 0; i < m_.methods.size(); ++i) {
      method_index_.emplace(m_.methods[i].name, i);
    }
    build_nodes();
    link_tree();
    leaf_ranges();
    match_roots();
    for (uint32_t n = 0; n < nodes_.size(); ++n) {
      if (!nodes_[n].primitive) {
        match_method(n);
      }
    }
    build_events();
    execute();
  }

  // The plan as its decomposition, once run() has accepted it
  std::optional<double> metric() const {return metric_;}

  /// What the times of the plan break of the constraints on times and durations, if that is
  /// all that is wrong with it
  const std::string & times_broken() const {return times_broken_;}

  void order(Order & out) const
  {
    for (uint32_t index : sequence_) {
      out.actions.push_back(Order::Action{nodes_[index].task, nodes_[index].args});
    }
    // The actions below each node, by position
    std::vector<std::vector<uint32_t>> leaves(nodes_.size());
    for (size_t k = order_.size(); k-- > 0; ) {
      const Node & n = nodes_[order_[k]];
      if (n.primitive) {
        leaves[order_[k]].push_back(n.position);
      }
      if (n.parent != kNone) {
        auto & up = leaves[n.parent];
        up.insert(up.end(), leaves[order_[k]].begin(), leaves[order_[k]].end());
      }
    }
    out.complete = true;
    auto network = [&](const hddl::Network & net, const std::vector<uint32_t> & steps) {
        for (auto [a, b] : net.order) {
          for (uint32_t first : leaves[steps[a]]) {
            for (uint32_t second : leaves[steps[b]]) {
              if (out.before.size() >= out.limit) {
                out.complete = false;
                return;
              }
              out.before.emplace_back(first, second);
            }
          }
        }
      };
    network(m_.initial, roots_);
    for (const Node & n : nodes_) {
      if (!n.primitive && out.complete) {
        network(m_.methods[n.method].network, n.children);
      }
    }
    if (!out.complete) {
      out.before.clear();
    }
  }

  /// The time and the duration of every action, if some action is durative
  std::vector<Timed> times() const {return temporal_ ? times_ : std::vector<Timed>();}

  Tree tree() const
  {
    Tree out;
    auto labels = [](const hddl::Network & net) {
        std::vector<std::string> ids;
        for (const hddl::Step & step : net.steps) {
          ids.push_back(step.id);
        }
        return ids;
      };
    for (const Node & n : nodes_) {
      Step step;
      step.id = n.id;
      step.task = m_.tasks[n.task].name;
      for (uint32_t a : n.args) {
        step.args.push_back(object(a).name);
      }
      step.primitive = n.primitive;
      if (!n.primitive) {
        const hddl::MethodInfo & method = m_.methods[n.method];
        step.method = method.name;
        for (uint32_t value : n.bindings.at(n.held)) {
          step.params.push_back(object(value).name);
        }
        for (uint32_t c : n.children) {
          step.subtasks.push_back(nodes_[c].id);
        }
        step.labels = labels(method.network);
      }
      out.steps.push_back(std::move(step));
    }
    for (uint32_t n : sequence_) {
      out.sequence.push_back(nodes_[n].id);
    }
    for (uint32_t n : roots_) {
      out.roots.push_back(nodes_[n].id);
    }
    out.root_labels = labels(m_.initial);
    return out;
  }

private:
  const sem::ObjectInfo & object(uint32_t o) const
  {
    return m_.objects.objects.items[sem::ObjectId(o)];
  }

  std::string show(const Node & n) const
  {
    std::string s = n.id + " (" + m_.tasks[n.task].name;
    for (uint32_t a : n.args) {
      s += " " + object(a).name;
    }
    return s + ")";
  }

  void build_nodes()
  {
    uint32_t position = 0;
    for (const PlanLine & line : plan_.lines) {
      Node n;
      n.id = line.id;
      auto task = task_index_.find(line.name);
      if (task == task_index_.end()) {
        invalid("step " + line.id + ": unknown task or action " + line.name);
      }
      n.task = TaskId(static_cast<uint32_t>(task->second));
      const hddl::TaskInfo & info = m_.tasks[n.task];
      if (info.primitive == line.abstract) {
        invalid("step " + line.id + ": " + line.name +
          (info.primitive ? " is an action and cannot be decomposed" :
          " is an abstract task and has no decomposition"));
      }
      if (line.args.size() != info.params.size()) {
        invalid("step " + line.id + ": " + line.name + " takes " +
          std::to_string(info.params.size()) + " arguments");
      }
      for (size_t i = 0; i < line.args.size(); ++i) {
        auto o = m_.objects.objects.find(line.args[i]);
        if (!o) {
          invalid("step " + line.id + ": unknown object " + line.args[i]);
        }
        if (!info.params[i].empty() &&
          !m_.domain.types.fits(m_.objects.objects.items[*o].types, info.params[i]))
        {
          invalid("step " + line.id + ": " + line.args[i] + " has the wrong type for argument " +
            std::to_string(i + 1) + " of " + line.name);
        }
        n.args.push_back(o->value());
      }
      n.primitive = info.primitive;
      if (line.time && !n.primitive) {
        invalid("step " + line.id + ": only actions have a time");
      }
      if (n.primitive) {
        timed_ = timed_ || line.time.has_value();
        if (timed_ != line.time.has_value()) {
          invalid("step " + line.id + ": either every action has a time or none");
        }
        times_.push_back(Timed{line.time.value_or(0.0), line.duration.value_or(0.0)});
        durations_given_.push_back(line.duration.has_value());
        n.position = position++;
        sequence_.push_back(static_cast<uint32_t>(nodes_.size()));
      } else {
        auto method = method_index_.find(line.method);
        if (method == method_index_.end()) {
          invalid("step " + line.id + ": unknown method " + line.method);
        }
        n.method = static_cast<uint32_t>(method->second);
        if (m_.methods[n.method].task != n.task) {
          invalid("step " + line.id + ": method " + line.method + " does not decompose " +
            line.name);
        }
      }
      if (!node_index_.emplace(line.id, static_cast<uint32_t>(nodes_.size())).second) {
        invalid("step id " + line.id + " is used twice");
      }
      nodes_.push_back(std::move(n));
    }
  }

  uint32_t node_of(const std::string & id, const std::string & where) const
  {
    auto it = node_index_.find(id);
    if (it == node_index_.end()) {
      invalid(where + " refers to step " + id + ", which is not in the plan");
    }
    return it->second;
  }

  // Parents and children; every step must hang from a root exactly once
  void link_tree()
  {
    for (const std::string & id : plan_.roots) {
      uint32_t n = node_of(id, "'root'");
      if (nodes_[n].is_root) {
        invalid("step " + id + " is twice in 'root'");
      }
      nodes_[n].is_root = true;
      roots_.push_back(n);
    }
    size_t line = 0;
    for (uint32_t n = 0; n < nodes_.size(); ++n, ++line) {
      for (const std::string & id : plan_.lines[line].children) {
        uint32_t c = node_of(id, "step " + nodes_[n].id);
        if (nodes_[c].parent != kNone || nodes_[c].is_root) {
          invalid("step " + id + (nodes_[c].is_root ? " is in 'root' and is also a subtask" :
            " is a subtask of more than one task"));
        }
        nodes_[c].parent = n;
        nodes_[n].children.push_back(c);
      }
    }
    // Reachable from the roots: this also excludes cycles
    std::vector<uint32_t> stack = roots_;
    size_t reached = 0;
    while (!stack.empty()) {
      uint32_t n = stack.back();
      stack.pop_back();
      ++reached;
      order_.push_back(n);
      stack.insert(stack.end(), nodes_[n].children.begin(), nodes_[n].children.end());
    }
    if (reached != nodes_.size()) {
      for (const Node & n : nodes_) {
        if (n.parent == kNone && !n.is_root) {
          invalid("step " + n.id + " is not part of the decomposition of the initial tasks");
        }
      }
      invalid("the decomposition contains a cycle");
    }
  }

  void leaf_ranges()
  {
    for (auto it = order_.rbegin(); it != order_.rend(); ++it) {  // children before parents
      Node & n = nodes_[*it];
      if (n.primitive) {
        n.first_leaf = n.last_leaf = n.position;
        continue;
      }
      for (uint32_t c : n.children) {
        if (nodes_[c].first_leaf != kNone) {
          n.first_leaf = n.first_leaf == kNone ? nodes_[c].first_leaf :
            std::min(n.first_leaf, nodes_[c].first_leaf);
          n.last_leaf = n.last_leaf == kNone ? nodes_[c].last_leaf :
            std::max(n.last_leaf, nodes_[c].last_leaf);
        }
      }
    }
  }

  // Assigns each child one step of `net` with the same task and arguments that agree with
  // `binding`. As in the IPC format, the children are written in an order the network allows:
  // each one takes a step whose predecessors are already taken. `children` comes back in the
  // order of the steps.
  bool match(
    const hddl::Network & net, const std::vector<hddl::Param> & params,
    std::vector<uint32_t> & children, std::vector<uint32_t> & binding)
  {
    std::vector<uint32_t> chosen(net.steps.size(), kNone);
    uint64_t budget = 1000000;
    if (!match_from(net, params, children, binding, chosen, 0, budget)) {
      return false;
    }
    children = chosen;
    return true;
  }

  bool match_from(
    const hddl::Network & net, const std::vector<hddl::Param> & params,
    const std::vector<uint32_t> & children, std::vector<uint32_t> & binding,
    std::vector<uint32_t> & chosen, size_t at, uint64_t & budget)
  {
    if (at == children.size()) {
      for (auto [a, b] : net.order) {
        const Node & na = nodes_[chosen[a]];
        const Node & nb = nodes_[chosen[b]];
        if (na.last_leaf != kNone && nb.first_leaf != kNone && na.last_leaf > nb.first_leaf) {
          return false;
        }
      }
      // Another assignment of the children may give values that respect the constraints
      return !complete(params, net, binding).empty();
    }
    const Node & child = nodes_[children[at]];
    for (size_t step = 0; step < net.steps.size(); ++step) {
      const hddl::Step & s = net.steps[step];
      if (chosen[step] != kNone || child.task != s.task) {
        continue;
      }
      const bool blocked = std::any_of(net.order.begin(), net.order.end(), [&](const auto & o) {
            return o.second == step && chosen[o.first] == kNone;
          });
      if (blocked) {
        continue;
      }
      if (budget-- == 0) {
        invalid("too many ways of matching the subtasks of a method");
      }
      std::vector<uint32_t> trial = binding;
      if (!unify(s.args, child.args, trial) || !well_typed(params, trial)) {
        continue;
      }
      chosen[step] = children[at];
      std::vector<uint32_t> saved = binding;
      binding = trial;
      if (match_from(net, params, children, binding, chosen, at + 1, budget)) {
        return true;
      }
      binding = saved;
      chosen[step] = kNone;
    }
    return false;
  }

  bool well_typed(const std::vector<hddl::Param> & params, const std::vector<uint32_t> & binding)
  {
    for (size_t p = 0; p < params.size(); ++p) {
      if (binding[p] != kNone && !params[p].types.empty() &&
        !m_.domain.types.fits(object(binding[p]).types, params[p].types))
      {
        return false;
      }
    }
    return true;
  }

  static bool unify(
    const std::vector<Arg> & pattern, const std::vector<uint32_t> & values,
    std::vector<uint32_t> & binding)
  {
    for (size_t i = 0; i < pattern.size(); ++i) {
      if (!pattern[i].variable) {
        if (pattern[i].index != values[i]) {
          return false;
        }
      } else if (binding[pattern[i].index] == kNone) {
        binding[pattern[i].index] = values[i];
      } else if (binding[pattern[i].index] != values[i]) {
        return false;
      }
    }
    return true;
  }

  bool consistent(const hddl::Network & net, const std::vector<uint32_t> & b) const
  {
    auto value = [&](const Arg & a) {return a.variable ? b[a.index] : a.index;};
    for (const hddl::VarConstraint & c : net.constraints) {
      if ((value(c.lhs) == value(c.rhs)) != c.equal) {
        return false;
      }
    }
    for (const hddl::SortConstraint & c : net.sorts) {
      if (m_.domain.types.fits(object(b[c.param]).types, c.types) == c.negated) {
        return false;
      }
    }
    return true;
  }

  // The ways of completing `binding` that respect types and variable constraints: all, or one
  // if no `open` is given. Too many to list: `open` is set, and nothing is returned
  std::vector<std::vector<uint32_t>> complete(
    const std::vector<hddl::Param> & params, const hddl::Network & net,
    std::vector<uint32_t> binding, bool * open = nullptr)
  {
    std::vector<std::vector<uint32_t>> result;
    std::vector<uint32_t> free;
    for (uint32_t p = 0; p < params.size(); ++p) {
      if (binding[p] == kNone) {
        free.push_back(p);
      } else if (!params[p].types.empty() &&
        !m_.domain.types.fits(object(binding[p]).types, params[p].types))
      {
        return result;
      }
    }
    std::vector<std::vector<uint32_t>> domains;
    for (uint32_t p : free) {
      std::vector<uint32_t> values;
      for (uint32_t o = 0; o < m_.objects.objects.items.size(); ++o) {
        if (params[p].types.empty() || m_.domain.types.fits(object(o).types, params[p].types)) {
          values.push_back(o);
        }
      }
      domains.push_back(std::move(values));
    }
    std::vector<size_t> at(free.size(), 0);
    while (true) {
      bool empty = false;
      for (size_t i = 0; i < free.size(); ++i) {
        if (domains[i].empty()) {
          empty = true;
          break;
        }
        binding[free[i]] = domains[i][at[i]];
      }
      if (empty) {
        break;
      }
      if (consistent(net, binding)) {
        result.push_back(binding);
        if (open == nullptr) {
          return result;
        }
        if (result.size() > kListed) {
          *open = true;
          return {};
        }
      }
      size_t i = 0;
      for (; i < free.size(); ++i) {
        if (++at[i] < domains[i].size()) {
          break;
        }
        at[i] = 0;
      }
      if (i == free.size()) {
        break;
      }
    }
    return result;
  }

  void match_roots()
  {
    std::vector<uint32_t> binding(m_.initial_params.size(), kNone);
    if (roots_.size() != m_.initial.steps.size() || !match(m_.initial, m_.initial_params, roots_, binding)) {
      invalid("the 'root' steps are not the tasks of the initial task network");
    }
    if (complete(m_.initial_params, m_.initial, binding).empty()) {
      invalid("the 'root' steps break the constraints of the initial task network");
    }
  }

  void match_method(uint32_t index)
  {
    Node & n = nodes_[index];
    const hddl::MethodInfo & method = m_.methods[n.method];
    std::vector<uint32_t> binding(method.params.size(), kNone);
    if (!unify(method.task_args, n.args, binding)) {
      invalid("step " + show(n) + ": method " + method.name + " does not apply to these "
        "arguments");
    }
    if (n.children.size() != method.network.steps.size() ||
      !match(method.network, method.params, n.children, binding))
    {
      invalid("step " + show(n) + ": its subtasks are not those of method " + method.name +
        ", or break its ordering, types or constraints");
    }
    n.bindings = complete(method.params, method.network, binding, &n.open);
    if (n.open) {
      n.partial = std::move(binding);
    } else if (n.bindings.empty()) {
      invalid("step " + show(n) + ": the parameters of method " + method.name +
        " have the wrong types or break its constraints");
    }
  }

  uint32_t event()
  {
    events_.emplace_back();
    return static_cast<uint32_t>(events_.size() - 1);
  }

  void edge(uint32_t from, uint32_t to)
  {
    events_[from].next.push_back(to);
    ++events_[to].pending;
  }

  // start(n) <= precondition test <= every subtask <= end(n); "a before b" is end(a) <= start(b)
  void build_events()
  {
    for (Node & n : nodes_) {
      if (n.primitive && m_.tasks[n.task].durative) {
        // It starts and it ends: other steps may come in between
        n.start = event();
        n.end = event();
        edge(n.start, n.end);
      } else if (n.primitive) {
        n.start = n.end = event();
      } else {
        n.start = event();
        n.test = event();
        n.end = event();
      }
    }
    auto order = [&](const hddl::Network & net, const std::vector<uint32_t> & steps) {
        for (auto [a, b] : net.order) {
          edge(nodes_[steps[a]].end, nodes_[steps[b]].start);
        }
      };
    order(m_.initial, roots_);
    for (uint32_t i = 0; i < nodes_.size(); ++i) {
      Node & n = nodes_[i];
      if (n.primitive) {
        continue;
      }
      events_[n.test].node = i;
      edge(n.start, n.test);
      edge(n.test, n.end);
      for (uint32_t c : n.children) {
        edge(n.test, nodes_[c].start);
        edge(nodes_[c].end, n.end);
      }
      order(m_.methods[n.method].network, n.children);
    }
  }

  bool precondition_holds(Node & n)
  {
    const hddl::MethodInfo & method = m_.methods[n.method];
    if (n.open) {
      return choose(n, method);
    }
    if (method.precondition_action < 0) {
      return true;
    }
    const ast::Action & a = m_.pddl.actions[static_cast<size_t>(method.precondition_action)];
    for (const std::vector<uint32_t> & binding : n.bindings) {
      val::Bindings env;
      for (size_t p = 0; p < a.params.size(); ++p) {
        env.emplace_back(a.params[p].name, binding[method.precondition_params[p]]);
      }
      if (holds(world_.domain_eval, a.precondition, env)) {
        n.held = static_cast<size_t>(&binding - n.bindings.data());
        return true;
      }
    }
    return false;
  }

  // Values for the free parameters of a method with which its precondition holds now: those
  // that the precondition mentions as Kairos' evaluator joins them with the state (the atoms
  // true in it, the objects it compares with), and by type the rest
  bool choose(Node & n, const hddl::MethodInfo & method)
  {
    const ast::Action * a = method.precondition_action < 0 ? nullptr :
      &m_.pddl.actions[static_cast<size_t>(method.precondition_action)];
    size_t work = 0;
    std::vector<uint32_t> binding = n.partial;
    bool found = false;
    if (a == nullptr || !a->precondition.valid()) {
      found = by_type(method, binding, work);
    } else {
      val::Bindings env;
      val::Vars vars;
      std::vector<uint32_t> free;  // the method parameter of each of `vars`
      for (size_t p = 0; p < a->params.size(); ++p) {
        const uint32_t param = method.precondition_params[p];
        if (binding[param] != kNone) {
          env.emplace_back(a->params[p].name, binding[param]);
        } else {
          vars.emplace_back(a->params[p].name, method.params[param].types);
          free.push_back(param);
        }
      }
      const size_t fixed = env.size();
      try {
        world_.domain_eval.for_each_joined_binding(
          vars, a->precondition, val::Evaluator::Ctx{state_}, env, [&]() {
            if (++work > kChoices) {
              invalid("too many values for the free parameters of method " + method.name);
            }
            std::vector<uint32_t> next = binding;
            for (size_t v = 0; v < vars.size(); ++v) {
              next[free[v]] = env[fixed + v].second;
            }
            val::Bindings all = env;
            if (holds(world_.domain_eval, a->precondition, all) && by_type(method, next, work)) {
              binding = std::move(next);
              found = true;
            }
            return !found;
          });
      } catch (const val::EvalError &) {
        found = false;
      } catch (const val::Unsupported & u) {
        invalid("unsupported: " + u.message);
      }
    }
    if (!found) {
      return false;
    }
    n.bindings.assign(1, std::move(binding));
    n.held = 0;
    n.open = false;
    return true;
  }

  // The parameters still free (the precondition does not mention them): any objects of their
  // types that the constraints of the method allow
  bool by_type(const hddl::MethodInfo & method, std::vector<uint32_t> & binding, size_t & work)
  {
    for (uint32_t p = 0; p < method.params.size(); ++p) {
      if (binding[p] != kNone) {
        continue;
      }
      const auto & types = method.params[p].types;
      for (uint32_t o = 0; o < m_.objects.objects.items.size(); ++o) {
        if (!types.empty() && !m_.domain.types.fits(object(o).types, types)) {
          continue;
        }
        if (++work > kChoices) {
          invalid("too many values for the free parameters of method " + method.name);
        }
        std::vector<uint32_t> next = binding;
        next[p] = o;
        if (by_type(method, next, work)) {
          binding = std::move(next);
          return true;
        }
      }
      return false;
    }
    return consistent(method.network, binding);
  }

  // Kairos' validator evaluates the syntax tree on a state. A condition that reads a fluent
  // with no value does not hold
  bool holds(val::Evaluator & evaluator, ast::FormulaId formula, val::Bindings & env)
  {
    try {
      return evaluator.holds(formula, val::Evaluator::Ctx{state_}, env);
    } catch (const val::EvalError &) {
      return false;
    } catch (const val::Unsupported & u) {
      invalid("unsupported: " + u.message);
    }
  }

  // A durative action under way: its parameters, and the state it started in if its effects at
  // end depend on it
  struct Running
  {
    val::Bindings env;
    double duration = 0.0;
    std::optional<val::State> before;
  };

  void check(
    const ast::DurativeAction & a, const Node & n, Running & running, val::Phase phase,
    const char * which)
  {
    const val::Evaluator::Ctx c{
      state_, running.before ? &*running.before : nullptr, phase, running.duration};
    bool ok = false;
    try {
      ok = !a.condition.valid() || world_.domain_eval.holds(a.condition, c, running.env);
    } catch (const val::EvalError &) {
      ok = false;
    } catch (const val::Unsupported & u) {
      invalid("unsupported: " + u.message);
    }
    if (!ok) {
      invalid("step " + show(n) + " is not applicable: its condition " + which +
        " does not hold");
    }
  }

  // The start of a durative action: it lasts what its duration says, its conditions at start
  // hold, and its effects at start take place
  Running begin(const ast::DurativeAction & a, const Node & n, Timed & when)
  {
    Running running;
    for (size_t p = 0; p < a.params.size(); ++p) {
      running.env.emplace_back(a.params[p].name, n.args[p]);
    }
    if (!durations_given_[n.position]) {
      when.duration = duration(a, running.env, n);
    }
    running.duration = when.duration;
    using Ctx = val::Evaluator::Ctx;
    Ctx lasting{state_, nullptr, val::Phase::Instant, when.duration};
    lasting.duration_check = true;
    try {
      if (a.duration.valid() && !world_.domain_eval.holds(a.duration, lasting, running.env)) {
        invalid("step " + show(n) + " does not last what its duration says");
      }
    } catch (const val::EvalError & e) {
      invalid("step " + show(n) + ": its duration cannot be computed: " + e.message);
    } catch (const val::Unsupported & u) {
      invalid("unsupported: " + u.message);
    }
    check(a, n, running, val::Phase::Start, "at start");
    if (a.effect.valid() && world_.domain_eval.has_when(a.effect)) {
      running.before = state_;
    }
    if (a.effect.valid()) {
      apply(a.effect, Ctx{state_, nullptr, val::Phase::Start, when.duration}, running.env, n);
    }
    return running;
  }

  // Its end: the conditions at end, then the effects at end
  void finish(const ast::DurativeAction & a, const Node & n, Running & running)
  {
    check(a, n, running, val::Phase::End, "at end");
    if (a.effect.valid()) {
      apply(a.effect, val::Evaluator::Ctx{state_, running.before ? &*running.before : nullptr,
          val::Phase::End, running.duration}, running.env, n);
    }
  }

  // What a durative action lasts when the plan does not say: what its duration is equal to,
  // or else the least it allows, or else the most
  double duration(const ast::DurativeAction & a, val::Bindings & env, const Node & n)
  {
    std::optional<double> equal, least, most;
    std::vector<ast::FormulaId> pending;
    if (a.duration.valid()) {
      pending.push_back(a.duration);
    }
    const ast::Nodes & nodes = m_.pddl.nodes;
    try {
      while (!pending.empty()) {
        const ast::Formula & f = nodes.formulas[pending.back()];
        pending.pop_back();
        if (f.kind != ast::FormulaKind::Compare) {
          pending.insert(pending.end(), f.children.begin(), f.children.end());
          continue;
        }
        const bool left = nodes.exprs[f.lhs].kind == ast::ExprKind::Duration;
        if (!left && nodes.exprs[f.rhs].kind != ast::ExprKind::Duration) {
          continue;
        }
        const double value = world_.domain_eval.number(
          left ? f.rhs : f.lhs, val::Evaluator::Ctx{state_}, env);
        // ?duration on the right turns the comparison round
        const bool lower = (f.compare == ast::CompareOp::Ge || f.compare == ast::CompareOp::Gt) ==
          left;
        if (f.compare == ast::CompareOp::Eq) {
          equal = value;
        } else if (lower) {
          least = std::max(least.value_or(value), value);
        } else {
          most = std::min(most.value_or(value), value);
        }
      }
    } catch (const val::EvalError & e) {
      invalid("step " + show(n) + ": its duration cannot be computed: " + e.message);
    } catch (const val::Unsupported & u) {
      invalid("unsupported: " + u.message);
    }
    return equal ? *equal : least ? *least : most.value_or(0.0);
  }

  // The actions with their times, as Kairos' validator checks a temporal plan: the rules of
  // PDDL 2.1 on what happens at the same time, which following the plan step by step does
  // not look at
  void validate_times()
  {
    kairos::Plan plan;
    plan.temporal = true;
    for (uint32_t index : sequence_) {
      const Node & n = nodes_[index];
      kairos::PlanStep step;
      step.time = times_[n.position].time;
      step.duration = times_[n.position].duration;
      step.name = lowered(m_.tasks[n.task].name);
      for (uint32_t o : n.args) {
        step.args.push_back(lowered(object(o).name));
      }
      plan.steps.push_back(std::move(step));
    }
    const kairos::ValidationReport report = val::validate(
      m_.pddl, m_.domain, m_.problem, m_.objects, plan, kairos::ValidateOptions{});
    if (report.status == kairos::ValidationStatus::Unsupported) {
      invalid("unsupported: " + (report.diagnostics.empty() ? std::string("this plan") :
        report.diagnostics[0].message));
    }
    if (!report.valid()) {
      invalid("as a temporal plan: " + (report.diagnostics.empty() ? std::string("not valid") :
        report.diagnostics[0].message));
    }
  }

  // Comparisons as written: the searches compare so
  static const kairos::ValidateOptions & exact()
  {
    static const kairos::ValidateOptions options = [] {
        kairos::ValidateOptions o;
        o.tolerance = 0.0;
        return o;
      }();
    return options;
  }

  // The effects of a step, all read in the state before it
  void apply(
    ast::EffectId effect, const val::Evaluator::Ctx & context, val::Bindings & env,
    const Node & n)
  {
    val::Changes changes;
    try {
      world_.domain_eval.effects(
        effect, context, env, context.phase == val::Phase::Instant, changes);
    } catch (const val::EvalError &) {
      invalid("step " + show(n) + " is not applicable: it reads a number with no value");
    } catch (const val::Unsupported & u) {
      invalid("unsupported: " + u.message);
    }
    std::vector<double> numbers;
    for (const val::Changes::Num & c : changes.nums) {
      const auto old = state_.numbers.find(c.key);
      double value = c.value;
      if (c.op != ast::AssignOp::Assign) {
        if (old == state_.numbers.end() || (c.op == ast::AssignOp::ScaleDown && c.value == 0.0)) {
          invalid("step " + show(n) + " is not applicable: it reads a number with no value");
        }
        switch (c.op) {
          case ast::AssignOp::Increase: value = old->second + c.value; break;
          case ast::AssignOp::Decrease: value = old->second - c.value; break;
          case ast::AssignOp::ScaleUp: value = old->second * c.value; break;
          default: value = old->second / c.value; break;
        }
      }
      numbers.push_back(value);
    }
    for (const val::Key & k : changes.dels) {
      state_.atoms.erase(k);
    }
    for (const val::Key & k : changes.adds) {
      state_.atoms.insert(k);
    }
    for (size_t i = 0; i < numbers.size(); ++i) {
      state_.numbers[changes.nums[i].key] = numbers[i];
    }
    for (const val::Changes::Obj & c : changes.objs) {
      if (c.value) {
        state_.objects[c.key] = *c.value;
      } else {
        state_.objects.erase(c.key);
      }
    }
    state_.changed();
  }

  // Places `e` and whatever becomes placeable; tests wait until their precondition holds
  void place(uint32_t e)
  {
    std::vector<uint32_t> stack{e};
    while (!stack.empty()) {
      uint32_t at = stack.back();
      stack.pop_back();
      events_[at].placed = true;
      ++placed_;
      for (uint32_t next : events_[at].next) {
        if (--events_[next].pending != 0) {
          continue;
        }
        if (events_[next].node != kNone) {
          waiting_.push_back(next);
        } else if (!is_leaf_[next]) {
          stack.push_back(next);
        }
      }
    }
  }

  // The tests that hold in the current state take place now, and so on while something changes
  void settle()
  {
    for (bool changed = true; changed; ) {
      changed = false;
      std::vector<uint32_t> waiting;
      waiting.swap(waiting_);
      for (uint32_t e : waiting) {
        if (precondition_holds(nodes_[events_[e].node])) {
          applied(nodes_[events_[e].node]);
          place(e);
          changed = true;
        } else {
          waiting_.push_back(e);
        }
      }
    }
    conditions();
  }

  // The values of the parameters of the method of a node, by name
  val::Bindings parameters(const Node & n) const
  {
    val::Bindings env;
    const hddl::MethodInfo & method = m_.methods[n.method];
    for (size_t p = 0; p < method.params.size() && !n.bindings.empty(); ++p) {
      if (const auto symbol = m_.pddl.symbols.find(method.params[p].name)) {
        env.emplace_back(*symbol, n.bindings[n.held][p]);
      }
    }
    return env;
  }

  // A method is applied now: the functions in its duration constraints are read in this state
  void applied(Node & n)
  {
    n.tested = now_;
    const hddl::MethodInfo & method = m_.methods[n.method];
    if (method.durations.empty()) {
      return;
    }
    val::Bindings env = parameters(n);
    std::vector<ast::ExprId> pending;
    for (const hddl::DurationConstraint & d : method.durations) {
      pending.push_back(d.lhs);
      pending.push_back(d.rhs);
    }
    while (!pending.empty()) {
      const ast::ExprId id = pending.back();
      pending.pop_back();
      const ast::Expr & e = m_.pddl.nodes.exprs[id];
      pending.insert(pending.end(), e.children.begin(), e.children.end());
      const bool of_step = std::any_of(method.duration_of.begin(), method.duration_of.end(),
          [&](const auto & d) {return d.first == id;});
      if (e.kind != ast::ExprKind::Function || of_step) {
        continue;
      }
      try {
        n.values.emplace_back(
          id, world_.domain_eval.number(id, val::Evaluator::Ctx{state_}, env));
      } catch (const val::EvalError & error) {
        invalid("step " + show(n) + ": a duration of method " + method.name +
          " cannot be computed: " + error.message);
      } catch (const val::Unsupported & u) {
        invalid("unsupported: " + u.message);
      }
    }
  }

  // The value of an expression of a duration constraint, once the times are known
  double duration_value(const Node & n, ast::ExprId id) const
  {
    const hddl::MethodInfo & method = m_.methods[n.method];
    const ast::Expr & e = m_.pddl.nodes.exprs[id];
    auto child = [&](size_t i) {return duration_value(n, e.children.at(i));};
    switch (e.kind) {
      case ast::ExprKind::Number:
        return e.number;
      case ast::ExprKind::Duration:
        return n.to - n.from;
      case ast::ExprKind::Function:
        for (const auto & [expr, step] : method.duration_of) {
          if (expr == id) {
            return nodes_[n.children[step]].to - nodes_[n.children[step]].from;
          }
        }
        for (const auto & [expr, value] : n.values) {
          if (expr == id) {
            return value;
          }
        }
        return 0.0;
      case ast::ExprKind::Add: {
          double total = 0.0;
          for (size_t i = 0; i < e.children.size(); ++i) {
            total += child(i);
          }
          return total;
        }
      case ast::ExprKind::Mul: {
          double total = 1.0;
          for (size_t i = 0; i < e.children.size(); ++i) {
            total *= child(i);
          }
          return total;
        }
      case ast::ExprKind::Sub:
        return child(0) - child(1);
      case ast::ExprKind::Div:
        return child(0) / child(1);
      default:
        return -child(0);
    }
  }

  // An action starts: so do the tasks above it that had not
  void started(const Node & leaf)
  {
    for (uint32_t up = leaf.parent; up != kNone && !nodes_[up].began; up = nodes_[up].parent) {
      nodes_[up].began = true;
      if (m_.methods[nodes_[up].method].condition.valid()) {
        lasting_.push_back(up);
      }
    }
  }

  // The conditions of the durative methods under way: over all while their task lasts, at end
  // once its last action has ended
  void conditions()
  {
    for (size_t i = 0; i < lasting_.size(); ) {
      const Node & n = nodes_[lasting_[i]];
      const hddl::MethodInfo & method = m_.methods[n.method];
      const bool ended = events_[n.end].placed;
      val::Bindings env = parameters(n);
      const val::Evaluator::Ctx c{
        state_, nullptr, ended ? val::Phase::End : val::Phase::Invariant};
      bool ok = false;
      try {
        ok = world_.domain_eval.holds(method.condition, c, env);
      } catch (const val::EvalError &) {
        ok = false;
      } catch (const val::Unsupported & u) {
        invalid("unsupported: " + u.message);
      }
      if (!ok) {
        invalid("step " + show(n) + ": the condition of method " + method.name +
          (ended ? " at end" : " over all") + " does not hold");
      }
      if (ended) {
        lasting_.erase(lasting_.begin() + static_cast<std::ptrdiff_t>(i));
      } else {
        ++i;
      }
    }
  }

  // When each task starts and ends, and what the networks and the methods say of those times
  void check_times()
  {
    for (size_t k = order_.size(); k-- > 0; ) {
      Node & n = nodes_[order_[k]];
      if (n.primitive) {
        n.from = times_[n.position].time;
        n.to = n.from + times_[n.position].duration;
        continue;
      }
      n.from = n.to = n.tested;
      bool first = true;
      for (uint32_t c : n.children) {
        const Node & child = nodes_[c];
        n.from = first ? child.from : std::min(n.from, child.from);
        n.to = first ? child.to : std::max(n.to, child.to);
        first = false;
      }
    }
    constexpr double kSame = 1e-9;
    auto network = [&](const hddl::Network & net, const std::vector<uint32_t> & steps,
        const std::string & where) {
        for (const hddl::TimeConstraint & t : net.times) {
          const Node & a = nodes_[steps[t.a]];
          const Node & b = nodes_[steps[t.b]];
          const double left = t.a_end ? a.to : a.from;
          const double right = t.b_end ? b.to : b.from;
          const bool ok = t.op == ast::CompareOp::Lt ? left < right - kSame :
            t.op == ast::CompareOp::Le ? left <= right + kSame : std::abs(left - right) <= kSame;
          if (!ok) {
            invalid("steps " + show(a) + " and " + show(b) + ": their times break an ordering "
              "constraint of " + where);
          }
        }
      };
    network(m_.initial, roots_, "the initial task network");
    // Durations are written with three decimals
    constexpr double kDuration = 1e-3;
    for (const Node & n : nodes_) {
      if (n.primitive) {
        continue;
      }
      const hddl::MethodInfo & method = m_.methods[n.method];
      network(method.network, n.children, "method " + method.name);
      for (const hddl::DurationConstraint & c : method.durations) {
        const double left = duration_value(n, c.lhs);
        const double right = duration_value(n, c.rhs);
        bool ok = false;
        switch (c.op) {
          case ast::CompareOp::Eq: ok = std::abs(left - right) <= kDuration; break;
          case ast::CompareOp::Lt: ok = left < right - kSame; break;
          case ast::CompareOp::Le: ok = left <= right + kDuration; break;
          case ast::CompareOp::Gt: ok = left > right + kSame; break;
          default: ok = left >= right - kDuration; break;
        }
        if (!ok) {
          invalid("step " + show(n) + ": its durations break a duration constraint of method " +
            method.name);
        }
      }
    }
  }

  void starts(const Node & n)
  {
    if (events_[n.start].pending != 0) {
      invalid("step " + show(n) + " comes too early: " + blocker(n.start));
    }
  }

  // An action without duration: applicable, and done
  void instant(const Node & n, Timed & when)
  {
    val::Bindings env;
    const ast::Action & a = m_.pddl.actions[m_.tasks[n.task].action];
    for (size_t p = 0; p < a.params.size(); ++p) {
      env.emplace_back(a.params[p].name, n.args[p]);
    }
    if (a.precondition.valid() && !holds(world_.domain_eval, a.precondition, env)) {
      invalid("step " + show(n) + " is not applicable: its precondition does not hold");
    }
    if (a.effect.valid()) {
      apply(a.effect, val::Evaluator::Ctx{state_}, env, n);
    }
    if (timed_ && when.duration != 0.0) {
      invalid("step " + show(n) + " is an action without duration");
    }
    when.duration = 0.0;
  }

  // A plan without times: each step when the one before has ended
  void one_after_another()
  {
    double clock = 0.0;
    for (uint32_t index : sequence_) {
      const Node & n = nodes_[index];
      starts(n);
      Timed & when = times_[n.position];
      when.time = n.position == 0 ? 0.0 : clock + kEpsilon;
      now_ = when.time;
      started(n);
      if (m_.tasks[n.task].durative) {
        temporal_ = true;
        const ast::DurativeAction & a = m_.pddl.durative_actions[m_.tasks[n.task].action];
        Running running = begin(a, n, when);
        place(n.start);
        settle();
        check(a, n, running, val::Phase::Invariant, "over all");
        now_ = when.time + when.duration;
        finish(a, n, running);
      } else {
        instant(n, when);
      }
      clock = when.time + when.duration;
      place(n.end);
      settle();
    }
  }

  // The timed initial literals of the problem, in the order of time
  struct Literal
  {
    double time;
    bool add;
    val::Key atom;
  };

  void timed_literals()
  {
    val::Bindings none;
    const val::Evaluator::Ctx c{state_};
    for (const ast::InitItem & item : m_.problem.init) {
      if (!item.time) {
        if (item.negated) {
          invalid("unsupported: a negative literal in the initial state");
        }
        continue;
      }
      const auto predicate = item.kind == ast::InitItem::Kind::Fact ?
        m_.domain.predicates.find(m_.problem.symbols.name(item.name)) : std::nullopt;
      if (!predicate) {
        invalid("unsupported: timed initial values of fluents");
      }
      val::Key key{predicate->value()};
      for (ast::TermId a : item.args) {
        key.push_back(world_.problem_eval.object_of(a, c, none));
      }
      literals_.push_back(Literal{*item.time, !item.negated, std::move(key)});
    }
    std::stable_sort(literals_.begin(), literals_.end(),
      [](const Literal & a, const Literal & b) {return a.time < b.time;});
  }

  // The literals up to a time take place; true if some did
  bool literals_until(double time)
  {
    bool some = false;
    for (; next_literal_ < literals_.size() && literals_[next_literal_].time <= time;
      ++next_literal_)
    {
      const Literal & l = literals_[next_literal_];
      if (l.add) {
        state_.atoms.insert(l.atom);
      } else {
        state_.atoms.erase(l.atom);
      }
      some = true;
    }
    if (some) {
      state_.changed();
    }
    return some;
  }

  // A plan without times in a problem with timed initial literals: each step when the one
  // before has ended, or after a literal if it cannot start before. Nothing else takes place
  // while a step runs but the literals
  void one_after_another_waiting()
  {
    double clock = 0.0;
    for (uint32_t index : sequence_) {
      const Node & n = nodes_[index];
      Timed & when = times_[n.position];
      const bool lasts = m_.tasks[n.task].durative;
      // When it may start: now, or once a literal has taken place
      std::vector<double> candidates{n.position == 0 ? 0.0 : clock + kEpsilon};
      for (size_t l = next_literal_; l < literals_.size(); ++l) {
        if (literals_[l].time + kEpsilon > candidates.back()) {
          candidates.push_back(literals_[l].time + kEpsilon);
        }
      }
      bool done = false;
      std::string why;
      for (size_t c = 0; c < candidates.size() && !done; ++c) {
        when.time = candidates[c];
        now_ = when.time;
        if (literals_until(when.time - kEpsilon)) {
          settle();
        }
        if (events_[n.start].pending != 0) {
          why = "step " + show(n) + " comes too early: " + blocker(n.start);
          continue;
        }
        const val::State before = state_;
        const size_t literal = next_literal_;
        try {
          if (lasts) {
            const ast::DurativeAction & a = m_.pddl.durative_actions[m_.tasks[n.task].action];
            Running running = begin(a, n, when);
            // A literal while it runs may break what must hold over all of it
            while (next_literal_ < literals_.size() &&
              literals_[next_literal_].time < when.time + when.duration)
            {
              check(a, n, running, val::Phase::Invariant, "over all");
              literals_until(literals_[next_literal_].time);
            }
            check(a, n, running, val::Phase::Invariant, "over all");
            literals_until(when.time + when.duration - kEpsilon);
            finish(a, n, running);
          } else {
            instant(n, when);
          }
          done = true;
        } catch (const Invalid & e) {
          why = e.reason;
          state_ = before;
          next_literal_ = literal;
        }
      }
      if (!done) {
        invalid(why);
      }
      clock = when.time + when.duration;
      now_ = clock;
      started(n);
      if (lasts) {
        place(n.start);
      }
      place(n.end);
      settle();
    }
  }

  // A plan with times: the starts and the ends of its steps in the order of time; at the same
  // time the ends first, then as the plan lists the steps. What must hold while an action
  // runs, and the rules on what happens at once, are left to Kairos' validator
  // (validate_times())
  void happenings()
  {
    struct Happening
    {
      double time;
      int kind;  // 0: an end, 1: the literals of the time, 2: a start
      uint32_t position;
    };
    std::vector<Happening> all;
    for (uint32_t index : sequence_) {
      const Node & n = nodes_[index];
      const Timed & when = times_[n.position];
      all.push_back(Happening{when.time, 2, n.position});
      if (m_.tasks[n.task].durative) {
        temporal_ = true;
        all.push_back(Happening{when.time + when.duration, 0, n.position});
      }
    }
    for (const Literal & l : literals_) {
      temporal_ = true;
      all.push_back(Happening{l.time, 1, 0});
    }
    std::stable_sort(all.begin(), all.end(), [](const Happening & a, const Happening & b) {
        return a.time != b.time ? a.time < b.time : a.kind < b.kind;
      });
    std::vector<Running> running(sequence_.size());
    for (const Happening & h : all) {
      now_ = h.time;
      if (h.kind == 1) {
        if (literals_until(h.time)) {
          settle();
        }
        continue;
      }
      const Node & n = nodes_[sequence_[h.position]];
      Timed & when = times_[n.position];
      if (!m_.tasks[n.task].durative) {
        starts(n);
        started(n);
        instant(n, when);
      } else if (h.kind == 2) {
        starts(n);
        started(n);
        running[h.position] = begin(
          m_.pddl.durative_actions[m_.tasks[n.task].action], n, when);
      } else {
        finish(m_.pddl.durative_actions[m_.tasks[n.task].action], n, running[h.position]);
      }
      place(h.kind == 2 && m_.tasks[n.task].durative ? n.start : n.end);
      settle();
    }
  }

  void execute()
  {
    is_leaf_.assign(events_.size(), false);
    for (uint32_t n : sequence_) {
      is_leaf_[nodes_[n].start] = true;
      is_leaf_[nodes_[n].end] = true;
    }
    val::Bindings env;
    try {
      world_.initial_state(state_);
      timed_literals();
    } catch (const val::EvalError & e) {
      invalid("unsupported: this initial state: " + e.message);
    }
    for (uint32_t e = 0; e < events_.size(); ++e) {
      if (events_[e].pending == 0 && !events_[e].placed && !is_leaf_[e] &&
        events_[e].node == kNone)
      {
        place(e);
      }
    }
    settle();
    if (timed_) {
      happenings();
    } else if (!literals_.empty()) {
      temporal_ = true;
      one_after_another_waiting();
    } else {
      one_after_another();
    }
    if (placed_ != events_.size()) {
      for (uint32_t e : waiting_) {
        const Node & n = nodes_[events_[e].node];
        invalid("step " + show(n) + ": the precondition of method " +
          m_.methods[n.method].name + " never holds where the ordering allows it");
      }
      invalid("the ordering constraints cannot be met");
    }
    if (m_.problem.goal.valid()) {
      env.clear();
      if (!holds(world_.problem_eval, m_.problem.goal, env)) {
        invalid("the goal does not hold at the end of the plan");
      }
    }
    try {
      check_times();
    } catch (const Invalid & e) {
      times_broken_ = e.reason;  // said once the rest is known to be right
    }
    if (temporal_) {
      validate_times();
    }
    // The metric, in the state the plan ends in. Time is not read yet: no value then
    if (m_.problem.metric && !uses_time(m_.problem.metric->expr)) {
      env.clear();
      try {
        metric_ = world_.problem_eval.number(
          m_.problem.metric->expr, val::Evaluator::Ctx{state_}, env);
      } catch (const val::EvalError &) {
        metric_.reset();
      } catch (const val::Unsupported &) {
        metric_.reset();
      }
    }
  }

  bool uses_time(ast::ExprId id) const
  {
    const ast::Expr & e = m_.problem.nodes.exprs[id];
    switch (e.kind) {
      case ast::ExprKind::Number:
      case ast::ExprKind::Function:
        return false;
      case ast::ExprKind::Add:
      case ast::ExprKind::Sub:
      case ast::ExprKind::Mul:
      case ast::ExprKind::Div:
      case ast::ExprKind::Neg:
        return std::any_of(e.children.begin(), e.children.end(),
                 [&](ast::ExprId c) {return uses_time(c);});
      default:
        return true;
    }
  }

  // Why an event cannot take place yet, for the message
  std::string blocker(uint32_t e) const
  {
    for (uint32_t w : waiting_) {
      const Node & n = nodes_[events_[w].node];
      (void)e;
      return "the precondition of method " + m_.methods[n.method].name + " of step " + n.id +
             " does not hold, or a task ordered before it is not finished";
    }
    return "a task ordered before it is not finished";
  }

  const Model & m_;
  PlanText plan_;
  static constexpr size_t kListed = 10000;      // bindings of a method listed beforehand
  static constexpr size_t kChoices = 50000000;  // steps choosing them from the state instead

  val::World world_;
  std::unordered_map<std::string, size_t> task_index_;
  std::unordered_map<std::string, size_t> method_index_;
  std::unordered_map<std::string, uint32_t> node_index_;
  std::vector<Node> nodes_;       // one per plan line, in the same order
  std::vector<uint32_t> sequence_;  // the primitive steps in plan order
  std::vector<uint32_t> roots_;     // in the order of the initial task network
  std::vector<uint32_t> order_;     // parents before children
  std::vector<Event> events_;
  std::vector<bool> is_leaf_;
  std::vector<uint32_t> waiting_;   // tests whose predecessors are placed
  size_t placed_ = 0;
  val::State state_;
  static constexpr double kEpsilon = 0.001;  // between a step and the next, as in PDDL 2.1
  std::vector<Timed> times_;           // of the actions, in plan order
  std::vector<bool> durations_given_;  // by the plan
  std::string times_broken_;
  double now_ = 0.0;                // the time of what is taking place
  std::vector<uint32_t> lasting_;   // nodes of durative methods with a condition, under way
  std::vector<Literal> literals_;  // the timed initial literals, in the order of time
  size_t next_literal_ = 0;        // the first that has not taken place
  bool timed_ = false;     // the plan gives the times
  bool temporal_ = false;  // some action is durative
  std::optional<double> metric_;
};

}  // namespace

Outcome verify(const hddl::Model & model, std::string_view plan, Tree * tree, Order * order)
{
  try {
    Verifier verifier(model, plan);
    verifier.run();
    if (tree != nullptr) {
      *tree = verifier.tree();
    }
    if (order != nullptr) {
      verifier.order(*order);
    }
    if (!verifier.times_broken().empty()) {
      Outcome broken{false, verifier.times_broken(), std::nullopt, {}};
      broken.only_times = true;
      return broken;
    }
    return Outcome{true, "", verifier.metric(), verifier.times()};
  } catch (const Invalid & e) {
    return Outcome{false, e.reason, std::nullopt, {}};
  }
}

}  // namespace klimax::verifier

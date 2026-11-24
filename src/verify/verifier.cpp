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

namespace klimax::verifier
{

namespace
{

namespace ast = kairos::ast;
namespace sem = kairos::sem;
using hddl::Arg;
using hddl::Model;
using hddl::TaskId;
using kairos::Symbol;

constexpr uint32_t kNone = std::numeric_limits<uint32_t>::max();

// Thrown with the reason the plan is rejected
struct Invalid
{
  std::string reason;
};

[[noreturn]] void invalid(std::string reason) {throw Invalid{std::move(reason)};}

// --- The plan as written ----------------------------------------------------------------------

struct PlanLine
{
  std::string id;
  std::string name;
  std::vector<std::string> args;
  bool abstract = false;
  std::string method;
  std::vector<std::string> children;
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
    if (w.size() < 2) {
      invalid("plan line '" + line + "' has no task");
    }
    PlanLine entry;
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

// --- States and formulas ----------------------------------------------------------------------

using Atom = std::vector<uint32_t>;  // predicate, then objects

struct AtomHash
{
  size_t operator()(const Atom & a) const noexcept
  {
    size_t h = 1469598103934665603ull;
    for (uint32_t v : a) {
      h = (h ^ v) * 1099511628211ull;
    }
    return h;
  }
};

using State = std::unordered_set<Atom, AtomHash>;

// Values of the variables in scope: innermost last
using Env = std::vector<std::pair<Symbol, uint32_t>>;

// Evaluates the formulas and effects of one file (the domain's or the problem's) on a state
class Interpreter
{
public:
  Interpreter(const Model & model, const ast::Nodes & nodes, const kairos::SymbolTable & symbols)
  : m_(model), nodes_(nodes), symbols_(symbols) {}

  bool holds(ast::FormulaId id, const State & state, Env & env)
  {
    const ast::Formula & f = nodes_.formulas[id];
    switch (f.kind) {
      case ast::FormulaKind::And:
        return std::all_of(f.children.begin(), f.children.end(),
                 [&](ast::FormulaId c) {return holds(c, state, env);});
      case ast::FormulaKind::Or:
        return std::any_of(f.children.begin(), f.children.end(),
                 [&](ast::FormulaId c) {return holds(c, state, env);});
      case ast::FormulaKind::Not:
        return !holds(f.children.at(0), state, env);
      case ast::FormulaKind::Imply:
        return !holds(f.children.at(0), state, env) || holds(f.children.at(1), state, env);
      case ast::FormulaKind::Exists:
        return quantified(f.params, 0, env, false, [&]() {
                   return holds(f.children.at(0), state, env);
                 });
      case ast::FormulaKind::Forall:
        return quantified(f.params, 0, env, true, [&]() {
                   return holds(f.children.at(0), state, env);
                 });
      case ast::FormulaKind::Atom:
        return state.count(atom(f.name, f.args, env)) != 0;
      case ast::FormulaKind::Compare: {
          const ast::Expr & l = nodes_.exprs[f.lhs];
          const ast::Expr & r = nodes_.exprs[f.rhs];
          if (f.compare != ast::CompareOp::Eq || l.kind != ast::ExprKind::Term ||
            r.kind != ast::ExprKind::Term)
          {
            invalid("unsupported: numeric conditions");
          }
          return object(l.term, env) == object(r.term, env);
        }
      default:
        invalid("unsupported: temporal conditions, preferences or trajectory constraints");
    }
  }

  // Collects what `id` adds and deletes when applied in `state`
  void effect(
    ast::EffectId id, const State & state, Env & env, std::vector<Atom> & adds,
    std::vector<Atom> & deletes)
  {
    const ast::Effect & e = nodes_.effects[id];
    switch (e.kind) {
      case ast::EffectKind::And:
        for (ast::EffectId c : e.children) {
          effect(c, state, env, adds, deletes);
        }
        return;
      case ast::EffectKind::Forall:
        quantified(e.params, 0, env, true, [&]() {
            effect(e.children.at(0), state, env, adds, deletes);
            return true;
          });
        return;
      case ast::EffectKind::When:
        if (holds(e.condition, state, env)) {
          effect(e.children.at(0), state, env, adds, deletes);
        }
        return;
      case ast::EffectKind::Add:
        adds.push_back(atom(e.name, e.args, env));
        return;
      case ast::EffectKind::Delete:
        deletes.push_back(atom(e.name, e.args, env));
        return;
      case ast::EffectKind::Assign: {
          // Action costs do not change which plans are valid; anything else is numeric
          const ast::Expr & target = nodes_.exprs[e.target];
          if (std::string_view(symbols_.name(target.name)) == "total-cost") {
            return;
          }
          invalid("unsupported: numeric effects");
        }
      default:
        invalid("unsupported: temporal effects");
    }
  }

  Atom atom(Symbol predicate, const std::vector<ast::TermId> & args, const Env & env)
  {
    Atom a;
    a.reserve(args.size() + 1);
    a.push_back(this->predicate(predicate));
    for (ast::TermId t : args) {
      a.push_back(object(t, env));
    }
    return a;
  }

  uint32_t object(ast::TermId id, const Env & env)
  {
    const ast::Term & t = nodes_.terms[id];
    if (t.kind == ast::TermKind::Variable) {
      for (auto it = env.rbegin(); it != env.rend(); ++it) {
        if (it->first == t.name) {
          return it->second;
        }
      }
      invalid("internal: unbound variable " + std::string(symbols_.name(t.name)));
    }
    if (t.kind != ast::TermKind::Name) {
      invalid("unsupported: object fluents");
    }
    return object(t.name);
  }

  uint32_t object(Symbol name)
  {
    uint32_t & cached = slot(objects_, name);
    if (cached == kNone) {
      auto found = m_.objects.objects.find(symbols_.name(name));
      if (!found) {
        invalid("internal: unknown object " + std::string(symbols_.name(name)));
      }
      cached = found->value();
    }
    return cached;
  }

  uint32_t predicate(Symbol name)
  {
    uint32_t & cached = slot(predicates_, name);
    if (cached == kNone) {
      auto found = m_.domain.predicates.find(symbols_.name(name));
      if (!found) {
        invalid("internal: unknown predicate " + std::string(symbols_.name(name)));
      }
      if (m_.domain.predicates.items[*found].derived) {
        invalid("unsupported: derived predicates");
      }
      cached = found->value();
    }
    return cached;
  }

  // The objects that fit a typed variable
  std::vector<uint32_t> domain_of(const ast::TypedName & variable)
  {
    sem::TypeSpec spec;
    for (Symbol t : variable.types) {
      if (auto id = m_.domain.types.find(symbols_.name(t))) {
        spec.push_back(*id);
      }
    }
    if (spec.empty()) {
      spec.push_back(m_.domain.types.object());
    }
    std::vector<uint32_t> result;
    for (uint32_t o = 0; o < m_.objects.objects.items.size(); ++o) {
      if (m_.domain.types.fits(m_.objects.objects.items[sem::ObjectId(o)].types, spec)) {
        result.push_back(o);
      }
    }
    return result;
  }

private:
  static uint32_t & slot(std::vector<uint32_t> & cache, Symbol s)
  {
    if (cache.size() <= s.index()) {
      cache.resize(s.index() + 1, kNone);
    }
    return cache[s.index()];
  }

  // `body` for every value of params[at..]: all of them (forall) or some (exists)
  template<class F>
  bool quantified(
    const std::vector<ast::TypedName> & params, size_t at, Env & env, bool all, F && body)
  {
    if (at == params.size()) {
      return body();
    }
    for (uint32_t o : domain_of(params[at])) {
      env.emplace_back(params[at].name, o);
      const bool r = quantified(params, at + 1, env, all, body);
      env.pop_back();
      if (r != all) {
        return r;
      }
    }
    return all;
  }

  const Model & m_;
  const ast::Nodes & nodes_;
  const kairos::SymbolTable & symbols_;
  std::vector<uint32_t> objects_;
  std::vector<uint32_t> predicates_;
};

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
  // Events of the node in the precedence graph
  uint32_t start = 0;
  uint32_t test = kNone;
  uint32_t end = 0;
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
    domain_(model, model.pddl.nodes, model.pddl.symbols),
    problem_(model, model.problem.nodes, model.problem.symbols) {}

  void run()
  {
    if (!m_.has_htn) {
      invalid("unsupported: a problem without :htn is a PDDL problem");
    }
    if (!m_.pddl.durative_actions.empty() || !m_.pddl.derived.empty() ||
      m_.problem.constraints.valid() || m_.pddl.constraints.valid())
    {
      invalid("unsupported: durative actions, derived predicates or trajectory constraints");
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
      if (n.primitive) {
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

  // Every way of completing `binding` that respects types and variable constraints
  std::vector<std::vector<uint32_t>> complete(
    const std::vector<hddl::Param> & params, const hddl::Network & net,
    std::vector<uint32_t> binding)
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
    auto consistent = [&](const std::vector<uint32_t> & b) {
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
      };
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
      if (consistent(binding)) {
        result.push_back(binding);
        if (result.size() > 100000) {
          invalid("too many values for the free parameters of a method");
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
    n.bindings = complete(method.params, method.network, binding);
    if (n.bindings.empty()) {
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
      if (n.primitive) {
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

  bool precondition_holds(const Node & n)
  {
    const hddl::MethodInfo & method = m_.methods[n.method];
    if (method.precondition_action < 0) {
      return true;
    }
    const ast::Action & a = m_.pddl.actions[static_cast<size_t>(method.precondition_action)];
    for (const std::vector<uint32_t> & binding : n.bindings) {
      Env env;
      for (size_t p = 0; p < a.params.size(); ++p) {
        env.emplace_back(a.params[p].name, binding[p]);
      }
      if (domain_.holds(a.precondition, state_, env)) {
        return true;
      }
    }
    return false;
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
          place(e);
          changed = true;
        } else {
          waiting_.push_back(e);
        }
      }
    }
  }

  void execute()
  {
    is_leaf_.assign(events_.size(), false);
    for (uint32_t n : sequence_) {
      is_leaf_[nodes_[n].start] = true;
    }
    Env env;
    for (const ast::InitItem & item : m_.problem.init) {
      if (item.kind != ast::InitItem::Kind::Fact) {
        // Costs do not change which plans are valid
        if (std::string_view(m_.problem.symbols.name(item.name)) == "total-cost" ||
          !function_changes(item.name))
        {
          continue;
        }
        invalid("unsupported: numeric fluents");
      }
      if (item.time || item.negated) {
        invalid("unsupported: timed initial literals");
      }
      state_.insert(problem_.atom(item.name, item.args, env));
    }
    for (uint32_t e = 0; e < events_.size(); ++e) {
      if (events_[e].pending == 0 && !events_[e].placed && !is_leaf_[e] &&
        events_[e].node == kNone)
      {
        place(e);
      }
    }
    settle();
    for (uint32_t index : sequence_) {
      const Node & n = nodes_[index];
      if (events_[n.start].pending != 0) {
        invalid("step " + show(n) + " comes too early: " + blocker(n.start));
      }
      const ast::Action & a = m_.pddl.actions[m_.tasks[n.task].action];
      env.clear();
      for (size_t p = 0; p < a.params.size(); ++p) {
        env.emplace_back(a.params[p].name, n.args[p]);
      }
      if (a.precondition.valid() && !domain_.holds(a.precondition, state_, env)) {
        invalid("step " + show(n) + " is not applicable: its precondition does not hold");
      }
      if (a.effect.valid()) {
        std::vector<Atom> adds;
        std::vector<Atom> deletes;
        domain_.effect(a.effect, state_, env, adds, deletes);
        for (const Atom & d : deletes) {
          state_.erase(d);
        }
        for (Atom & add : adds) {
          state_.insert(std::move(add));
        }
      }
      place(n.start);
      settle();
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
      if (!problem_.holds(m_.problem.goal, state_, env)) {
        invalid("the goal does not hold at the end of the plan");
      }
    }
  }

  bool function_changes(Symbol name) const
  {
    auto f = m_.domain.functions.find(m_.problem.symbols.name(name));
    return !f || m_.domain.functions.items[*f].changes;
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
  Interpreter domain_;
  Interpreter problem_;
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
  State state_;
};

}  // namespace

Outcome verify(const hddl::Model & model, std::string_view plan)
{
  try {
    Verifier(model, plan).run();
  } catch (const Invalid & e) {
    return Outcome{false, e.reason};
  }
  return Outcome{true, ""};
}

}  // namespace klimax::verifier

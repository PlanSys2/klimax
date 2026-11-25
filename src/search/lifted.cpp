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


#include "search/lifted.hpp"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <set>
#include <string>
#include <string_view>
#include <unordered_set>
#include <utility>
#include <vector>

#include "verify/interpreter.hpp"

namespace klimax::search
{

namespace
{

namespace ast = kairos::ast;
namespace sem = kairos::sem;
using hddl::Arg;
using hddl::Model;
using hddl::TaskId;
using verifier::Atom;
using verifier::Env;
using verifier::Interpreter;
using verifier::kNone;
using verifier::State;

uint64_t mix(uint64_t x)
{
  x ^= x >> 33;
  x *= 0xff51afd7ed558ccdull;
  x ^= x >> 33;
  x *= 0xc4ceb9fe1a85ec53ull;
  return x ^ (x >> 33);
}

uint64_t hash_of(const Atom & atom)
{
  uint64_t h = 0x9e3779b97f4a7c15ull;
  for (uint32_t v : atom) {
    h = mix(h ^ v);
  }
  return h;
}

// An atom that a precondition requires, over the parameters of its method or action
struct Required
{
  uint32_t predicate;
  std::vector<Arg> args;
};

// What a method or an action asks of its parameters (the initial task network too)
struct Schema
{
  std::vector<const sem::TypeSpec *> types;  // by parameter
  std::vector<bool> needed;      // a value is needed before going on: the rest can wait
  std::vector<Required> required;
  const ast::Action * condition = nullptr;   // its precondition, if any
  std::vector<uint32_t> condition_params;    // the parameter of each one of `condition`
  const hddl::Network * network = nullptr;   // methods: constraints and subtasks
  std::vector<uint32_t> order;               // its steps in sequence
};

// A value is an object or, from kVariable on, a variable that takes one later
constexpr uint32_t kVariable = 0x80000000u;

struct Variable
{
  uint32_t value = kNone;  // an object, or another variable it was made equal to
  std::vector<const sem::TypeSpec *> types;
};

// A pending task; the ones after it follow by `next`
struct Cell
{
  uint32_t task;
  uint32_t args;   // in args_
  uint32_t next;
  uint32_t node;   // its place in the decomposition
  uint64_t hash;   // of the task, its arguments and everything after
};

struct Node
{
  uint32_t task;
  uint32_t args;
  uint32_t parent;
  uint32_t method = kNone;
};

// A way of decomposing a task or of doing an action: values for the parameters (kNone: left
// for later) and for variables that were pending
struct Choice
{
  uint32_t method = kNone;
  std::vector<uint32_t> binding;
  std::vector<std::pair<uint32_t, uint32_t>> assigned;                  // variable, value
  std::vector<std::pair<uint32_t, const sem::TypeSpec *>> narrowed;     // variable, type
};

enum class Kind : uint8_t {Root, Method, Action};

// A choice point: what to go back to, and the ways not tried yet
struct Frame
{
  Kind kind = Kind::Method;
  uint32_t cell = kNone;
  size_t cells;
  size_t args;
  size_t nodes;
  size_t sequence;
  size_t trail;
  size_t variables;
  size_t bound;
  size_t typed;
  uint64_t state_hash;
  uint64_t variables_hash;
  std::vector<Choice> choices;
  size_t next = 0;
};

}  // namespace

class LiftedSearch::Impl
{
public:
  explicit Impl(const Model & model)
  : m_(model), domain_(model, model.pddl.nodes, model.pddl.symbols),
    problem_(model, model.problem.nodes, model.problem.symbols) {}

  Result run(kairos::Budget * budget)
  {
    if (!finished_) {
      budget_ = budget;
      out_ = false;
      last_ = Result();
      search(last_);
      last_.expansions = expansions_;
    }
    return last_;
  }

private:
  // Leaves `finished_` false when the budget ran out: the next call goes on from there
  void search(Result & result)
  {
    finished_ = true;
    if (!started_) {
      if (!m_.total_order) {
        result.message = "the search that does not ground takes totally ordered problems only";
        return;
      }
      if (!m_.pddl.durative_actions.empty() || !m_.pddl.derived.empty() ||
        m_.problem.constraints.valid() || m_.pddl.constraints.valid())
      {
        result.message = "durative actions, derived predicates or trajectory constraints";
        return;
      }
      prepare();
      initial_state();
      started_ = true;
    }
    uint32_t & current = current_;
    bool & forward = forward_;  // false: go back to the last choice point with a way not tried
    while (true) {
      if (out_ || (budget_ != nullptr && budget_->exhausted())) {
        result.outcome = Outcome::OutOfBudget;
        finished_ = false;
        return;
      }
      if (cells_.size() > kCells || stored_ > kStored) {
        result.outcome = Outcome::OutOfBudget;
        result.message = "the search that does not ground ran out of its memory";
        return;
      }
      if (frames_.empty() && !rooted_) {
        Frame first;
        first.kind = Kind::Root;
        mark(first);
        Choice start;
        start.binding.assign(m_.initial_params.size(), kNone);
        choices(initial_, std::move(start), first.choices);
        if (!out_) {
          rooted_ = true;
          push(std::move(first));
        }
        continue;
      }
      if (!forward) {
        while (!frames_.empty()) {
          Frame & frame = frames_.back();
          restore(frame);
          if (frame.next < frame.choices.size()) {
            if (take(frame, current)) {
              forward = true;
              break;
            }
            continue;  // that way does not fit: the next one
          }
          stored_ -= frame.choices.size();
          frames_.pop_back();
        }
        if (!forward) {
          result.outcome = complete_ ? Outcome::Unsolvable : Outcome::OutOfBudget;
          if (!complete_) {
            result.message = "the search that does not ground left part of the space out";
          }
          return;
        }
        continue;
      }
      forward = false;
      if (current == kNone) {
        Env env;
        if ((!m_.problem.goal.valid() || problem_.holds(m_.problem.goal, state_, env)) &&
          settle())
        {
          result.outcome = Outcome::Solved;
          result.plan = plan();
          result.steps = sequence_.size();
          result.plans = 1;
          result.message = "depth first, not grounded";
          return;
        }
        continue;
      }
      const Cell cell = cells_[current];
      ++expansions_;
      const hddl::TaskInfo & task = m_.tasks[TaskId(cell.task)];
      if (task.primitive) {
        const Schema & schema = actions_[cell.task];
        Choice choice;
        choice.binding.resize(task.params.size());
        bool pending = false;
        for (size_t i = 0; i < task.params.size(); ++i) {
          choice.binding[i] = value(args_[cell.args + i]);
          pending = pending || choice.binding[i] >= kVariable;
        }
        if (!pending) {
          if (apply(cell.task, choice.binding)) {
            sequence_.push_back(cell.node);
            current = cell.next;
            forward = true;
          }
          continue;
        }
        // Arguments still to choose: the precondition says which ones are possible
        Frame frame;
        frame.kind = Kind::Action;
        frame.cell = current;
        mark(frame);
        choices(schema, std::move(choice), frame.choices);
        if (out_) {
          --expansions_;
          forward = true;  // this task again, the next time
        } else {
          push(std::move(frame));
        }
        continue;
      }
      const uint64_t key = mix(state_hash_ ^ cell.hash ^ variables_hash_);
      if (!fresh(key)) {
        continue;
      }
      Frame frame;
      frame.cell = current;
      mark(frame);
      for (uint32_t m : m_.methods_of[cell.task]) {
        const hddl::MethodInfo & method = m_.methods[m];
        Choice choice;
        choice.method = m;
        choice.binding.assign(method.params.size(), kNone);
        if (unify(method, methods_[m], cell, choice)) {
          choices(methods_[m], std::move(choice), frame.choices);
        }
      }
      if (out_) {
        seen_.erase(key);
        --expansions_;
        forward = true;
      } else {
        push(std::move(frame));
      }
    }
  }

  static constexpr size_t kChoices = 100000;   // ways of decomposing one task
  static constexpr size_t kSeen = 5000000;     // tasks and states remembered
  static constexpr size_t kCells = 5000000;    // pending tasks made along one branch
  static constexpr size_t kStored = 2000000;   // ways not tried yet, in all the choice points

  void push(Frame && frame)
  {
    stored_ += frame.choices.size();
    frames_.push_back(std::move(frame));
  }

  const sem::ObjectInfo & object(uint32_t o) const
  {
    return m_.objects.objects.items[sem::ObjectId(o)];
  }

  bool fits(uint32_t o, const sem::TypeSpec & types) const
  {
    return types.empty() || m_.domain.types.fits(object(o).types, types);
  }

  bool fits(uint32_t o, const Variable & variable) const
  {
    return std::all_of(variable.types.begin(), variable.types.end(),
             [&](const sem::TypeSpec * t) {return fits(o, *t);});
  }

  // What a value stands for now: an object, or a variable with none yet
  uint32_t value(uint32_t v) const
  {
    while (v >= kVariable && v != kNone && variables_[v - kVariable].value != kNone) {
      v = variables_[v - kVariable].value;
    }
    return v;
  }

  // The same, with what `choice` gives to variables
  uint32_t value(const Choice & choice, uint32_t v) const
  {
    for (bool again = true; again; ) {
      v = value(v);
      again = false;
      if (v >= kVariable && v != kNone) {
        for (const auto & [variable, to] : choice.assigned) {
          if (variable == v) {
            v = to;
            again = true;
            break;
          }
        }
      }
    }
    return v;
  }

  // Gives `to` (an object) to parameter `p` of the schema, which may be a pending variable
  bool give(const Schema & schema, Choice & choice, uint32_t p, uint32_t to) const
  {
    const uint32_t now = choice.binding[p] == kNone ? kNone : value(choice, choice.binding[p]);
    if (now < kVariable) {
      return now == to;
    }
    if (!fits(to, *schema.types[p])) {
      return false;
    }
    if (now == kNone) {
      choice.binding[p] = to;
      return true;
    }
    if (!fits(to, variables_[now - kVariable])) {
      return false;
    }
    choice.assigned.emplace_back(now, to);
    return true;
  }

  bool open(const Choice & choice, uint32_t p) const
  {
    return choice.binding[p] == kNone || value(choice, choice.binding[p]) >= kVariable;
  }

  std::vector<Required> required_of(
    const ast::Action & a, const std::vector<uint32_t> & params)
  {
    std::vector<Required> result;
    if (!a.precondition.valid()) {
      return result;
    }
    std::vector<ast::FormulaId> atoms;
    domain_.required(a.precondition, atoms);
    for (ast::FormulaId id : atoms) {
      const ast::Formula & f = domain_.formula(id);
      Required r{domain_.predicate(f.name), {}};
      bool usable = true;
      for (ast::TermId t : f.args) {
        const ast::Term & term = domain_.term(t);
        if (term.kind == ast::TermKind::Name) {
          r.args.push_back(Arg{false, domain_.object(term.name)});
          continue;
        }
        size_t p = 0;
        while (p < a.params.size() && a.params[p].name != term.name) {
          ++p;
        }
        if (term.kind != ast::TermKind::Variable || p == a.params.size()) {
          usable = false;
          break;
        }
        r.args.push_back(Arg{true, params[p]});
      }
      if (usable) {
        result.push_back(std::move(r));
      }
    }
    return result;
  }

  void network(Schema & schema, const std::vector<hddl::Param> & params, const hddl::Network & net)
  {
    schema.network = &net;
    schema.order = order_of(net);
    schema.needed.assign(params.size(), false);
    for (const hddl::Param & p : params) {
      schema.types.push_back(&p.types);
    }
    for (const hddl::VarConstraint & c : net.constraints) {
      for (const Arg & x : {c.lhs, c.rhs}) {
        if (x.variable) {
          schema.needed[x.index] = true;
        }
      }
    }
    for (const hddl::SortConstraint & c : net.sorts) {
      schema.needed[c.param] = true;
    }
  }

  void prepare()
  {
    network(initial_, m_.initial_params, m_.initial);
    methods_.resize(m_.methods.size());
    for (size_t m = 0; m < m_.methods.size(); ++m) {
      const hddl::MethodInfo & method = m_.methods[m];
      Schema & schema = methods_[m];
      network(schema, method.params, method.network);
      if (method.precondition_action >= 0) {
        const ast::Action & a = m_.pddl.actions[static_cast<size_t>(method.precondition_action)];
        if (a.precondition.valid()) {
          schema.condition = &a;
          schema.condition_params = method.precondition_params;
          schema.required = required_of(a, schema.condition_params);
          for (uint32_t p : schema.condition_params) {
            schema.needed[p] = true;
          }
        }
      }
    }
    actions_.resize(m_.tasks.size());
    for (uint32_t t = 0; t < m_.tasks.size(); ++t) {
      const hddl::TaskInfo & task = m_.tasks[TaskId(t)];
      if (!task.primitive) {
        continue;
      }
      Schema & schema = actions_[t];
      const ast::Action & a = m_.pddl.actions[task.action];
      for (const sem::TypeSpec & types : task.params) {
        schema.types.push_back(&types);
      }
      schema.needed.assign(task.params.size(), true);
      schema.condition_params.resize(task.params.size());
      for (uint32_t p = 0; p < task.params.size(); ++p) {
        schema.condition_params[p] = p;
      }
      if (a.precondition.valid()) {
        schema.condition = &a;
        schema.required = required_of(a, schema.condition_params);
      }
    }
  }

  static std::vector<uint32_t> order_of(const hddl::Network & net)
  {
    const uint32_t n = static_cast<uint32_t>(net.steps.size());
    std::vector<uint32_t> before(n, 0);
    for (auto [a, b] : net.order) {
      (void)a;
      ++before[b];
    }
    std::vector<uint32_t> order;
    std::vector<bool> done(n, false);
    while (order.size() < n) {
      uint32_t s = 0;
      while (done[s] || before[s] != 0) {
        ++s;
      }
      done[s] = true;
      order.push_back(s);
      for (auto [a, b] : net.order) {
        if (a == s) {
          --before[b];
        }
      }
    }
    return order;
  }

  void initial_state()
  {
    facts_.resize(m_.domain.predicates.items.size());
    Env env;
    for (const ast::InitItem & item : m_.problem.init) {
      if (item.kind != ast::InitItem::Kind::Fact) {
        const std::string_view name = m_.problem.symbols.name(item.name);
        auto f = m_.domain.functions.find(name);
        if (name == "total-cost" || (f && !m_.domain.functions.items[*f].changes)) {
          continue;
        }
        verifier::invalid("unsupported: numeric fluents");
      }
      if (item.time || item.negated) {
        verifier::invalid("unsupported: timed initial literals");
      }
      add(problem_.atom(item.name, item.args, env));
    }
    trail_.clear();
  }

  bool add(Atom atom)
  {
    if (!state_.insert(atom).second) {
      return false;
    }
    state_hash_ ^= hash_of(atom);
    facts_[atom[0]].insert(atom);
    trail_.emplace_back(true, std::move(atom));
    return true;
  }

  bool remove(const Atom & atom)
  {
    if (state_.erase(atom) == 0) {
      return false;
    }
    state_hash_ ^= hash_of(atom);
    facts_[atom[0]].erase(atom);
    trail_.emplace_back(false, atom);
    return true;
  }

  void mark(Frame & frame) const
  {
    frame.cells = cells_.size();
    frame.args = args_.size();
    frame.nodes = nodes_.size();
    frame.sequence = sequence_.size();
    frame.trail = trail_.size();
    frame.variables = variables_.size();
    frame.bound = bound_.size();
    frame.typed = typed_.size();
    frame.state_hash = state_hash_;
    frame.variables_hash = variables_hash_;
  }

  void restore(const Frame & frame)
  {
    while (trail_.size() > frame.trail) {
      auto & [added, atom] = trail_.back();
      if (added) {
        state_.erase(atom);
        facts_[atom[0]].erase(atom);
      } else {
        facts_[atom[0]].insert(atom);
        state_.insert(std::move(atom));
      }
      trail_.pop_back();
    }
    state_hash_ = frame.state_hash;
    while (typed_.size() > frame.typed) {
      variables_[typed_.back()].types.pop_back();
      typed_.pop_back();
    }
    while (bound_.size() > frame.bound) {
      variables_[bound_.back()].value = kNone;
      bound_.pop_back();
    }
    variables_hash_ = frame.variables_hash;
    variables_.resize(frame.variables);
    cells_.resize(frame.cells);
    args_.resize(frame.args);
    nodes_.resize(frame.nodes);
    sequence_.resize(frame.sequence);
  }

  bool fresh(uint64_t key)
  {
    if (seen_.size() >= kSeen) {
      complete_ = false;
      return seen_.count(key) == 0;
    }
    return seen_.insert(key).second;
  }

  // The variable (a reference to it) takes a value now: an object or another variable
  bool bind(uint32_t reference, uint32_t to)
  {
    Variable & variable = variables_[reference - kVariable];
    if (to < kVariable) {
      if (!fits(to, variable)) {
        return false;
      }
    } else {
      for (const sem::TypeSpec * t : variable.types) {
        narrow(to, t);
      }
    }
    variable.value = to;
    bound_.push_back(reference - kVariable);
    variables_hash_ ^= mix((uint64_t{reference} << 32) | to);
    return true;
  }

  void narrow(uint32_t reference, const sem::TypeSpec * types)
  {
    if (!types->empty()) {
      variables_[reference - kVariable].types.push_back(types);
      typed_.push_back(reference - kVariable);
    }
  }

  // Makes two values the same, now
  bool same(uint32_t a, uint32_t b)
  {
    a = value(a);
    b = value(b);
    if (a == b) {
      return true;
    }
    if (a >= kVariable) {
      return bind(a, b);
    }
    return b >= kVariable && bind(b, a);
  }

  // The parameters of the method that the arguments of the task fix
  bool unify(
    const hddl::MethodInfo & method, const Schema & schema, const Cell & cell,
    Choice & choice) const
  {
    for (size_t i = 0; i < method.task_args.size(); ++i) {
      const Arg & x = method.task_args[i];
      const uint32_t given = value(choice, args_[cell.args + i]);
      if (!x.variable) {
        if (given < kVariable) {
          if (given != x.index) {
            return false;
          }
        } else {
          if (!fits(x.index, variables_[given - kVariable])) {
            return false;
          }
          choice.assigned.emplace_back(given, x.index);
        }
        continue;
      }
      const uint32_t p = x.index;
      if (choice.binding[p] == kNone) {
        if (given < kVariable) {
          if (!fits(given, *schema.types[p])) {
            return false;
          }
        } else {
          choice.narrowed.emplace_back(given, schema.types[p]);
        }
        choice.binding[p] = given;
        continue;
      }
      const uint32_t now = value(choice, choice.binding[p]);
      if (now == given) {
        continue;
      }
      if (now < kVariable && given < kVariable) {
        return false;
      }
      if (given >= kVariable) {
        if (now < kVariable && !fits(now, variables_[given - kVariable])) {
          return false;
        }
        choice.assigned.emplace_back(given, now);
      } else {
        if (!fits(given, variables_[now - kVariable])) {
          return false;
        }
        choice.assigned.emplace_back(now, given);
      }
    }
    return true;
  }

  // Every way of completing `choice` for the schema: values from the atoms that its
  // precondition requires, by type for the other parameters that need one now; the constraints,
  // the types of the subtasks' arguments and the precondition hold
  void choices(const Schema & schema, Choice choice, std::vector<Choice> & out)
  {
    if (out_ || ((++work_ & 4095) == 0 && budget_ != nullptr && budget_->exhausted())) {
      out_ = true;
      return;
    }
    for (const Required & r : schema.required) {
      if (std::none_of(r.args.begin(), r.args.end(),
        [&](const Arg & x) {return x.variable && open(choice, x.index);}))
      {
        continue;
      }
      // Nothing below changes the state
      for (const Atom & atom : facts_[r.predicate]) {
        if (atom.size() != r.args.size() + 1) {
          continue;
        }
        Choice next = choice;
        bool match = true;
        for (size_t i = 0; match && i < r.args.size(); ++i) {
          const Arg & x = r.args[i];
          match = x.variable ? give(schema, next, x.index, atom[i + 1]) : x.index == atom[i + 1];
        }
        if (match) {
          choices(schema, std::move(next), out);
        }
      }
      return;
    }
    for (uint32_t p = 0; p < schema.types.size(); ++p) {
      if (!schema.needed[p] || !open(choice, p)) {
        continue;
      }
      for (uint32_t o = 0; o < m_.objects.objects.items.size(); ++o) {
        Choice next = choice;
        if (give(schema, next, p, o)) {
          choices(schema, std::move(next), out);
        }
      }
      return;
    }
    if (out.size() >= kChoices) {
      complete_ = false;
      return;
    }
    auto of = [&](const Arg & x) {return x.variable ? value(choice, choice.binding[x.index]) : x.index;};
    if (schema.network != nullptr) {
      for (const hddl::VarConstraint & c : schema.network->constraints) {
        if ((of(c.lhs) == of(c.rhs)) != c.equal) {
          return;
        }
      }
      for (const hddl::SortConstraint & c : schema.network->sorts) {
        const uint32_t o = value(choice, choice.binding[c.param]);
        if (m_.domain.types.fits(object(o).types, c.types) == c.negated) {
          return;
        }
      }
      for (const hddl::Step & step : schema.network->steps) {
        const hddl::TaskInfo & task = m_.tasks[step.task];
        for (size_t i = 0; i < step.args.size(); ++i) {
          if (step.args[i].variable && choice.binding[step.args[i].index] == kNone) {
            continue;
          }
          const uint32_t o = of(step.args[i]);
          if (o < kVariable && !fits(o, task.params[i])) {
            return;
          }
        }
      }
    }
    if (schema.condition != nullptr && schema.network != nullptr) {
      Env env;
      for (size_t p = 0; p < schema.condition->params.size(); ++p) {
        env.emplace_back(schema.condition->params[p].name,
          value(choice, choice.binding[schema.condition_params[p]]));
      }
      if (!domain_.holds(schema.condition->precondition, state_, env)) {
        return;
      }
    }
    out.push_back(std::move(choice));
  }

  // Goes the next way of `frame`: false if it does not fit after all (the frame is to be
  // restored). `current` is then the first pending task
  bool take(Frame & frame, uint32_t & current)
  {
    Choice choice = std::move(frame.choices[frame.next++]);
    for (const auto & [variable, to] : choice.assigned) {
      if (!same(variable, to)) {
        return false;
      }
    }
    for (const auto & [variable, types] : choice.narrowed) {
      const uint32_t now = value(variable);
      if (now < kVariable) {
        if (!fits(now, *types)) {
          return false;
        }
      } else {
        narrow(now, types);
      }
    }
    if (frame.kind == Kind::Action) {
      const Cell cell = cells_[frame.cell];
      for (uint32_t & v : choice.binding) {
        v = value(v);
      }
      if (!apply(cell.task, choice.binding)) {
        return false;
      }
      sequence_.push_back(cell.node);
      current = cell.next;
      return true;
    }
    const bool root = frame.kind == Kind::Root;
    const Schema & schema = root ? initial_ : methods_[choice.method];
    const hddl::Network & net = *schema.network;
    uint32_t next = root ? kNone : cells_[frame.cell].next;
    const uint32_t parent = root ? kNone : cells_[frame.cell].node;
    if (!root) {
      nodes_[parent].method = choice.method;
    }
    // The parameters left for later become variables
    for (uint32_t p = 0; p < choice.binding.size(); ++p) {
      if (choice.binding[p] == kNone) {
        choice.binding[p] = kVariable + static_cast<uint32_t>(variables_.size());
        variables_.emplace_back();
        narrow(choice.binding[p], schema.types[p]);
      }
    }
    // Nodes in the order of the method, as the format of the plan asks; cells from the last
    const uint32_t first_node = static_cast<uint32_t>(nodes_.size());
    for (uint32_t s : schema.order) {
      const hddl::Step & step = net.steps[s];
      const hddl::TaskInfo & task = m_.tasks[step.task];
      Node node;
      node.task = step.task.value();
      node.args = static_cast<uint32_t>(args_.size());
      node.parent = parent;
      for (size_t i = 0; i < step.args.size(); ++i) {
        const Arg & x = step.args[i];
        const uint32_t v = x.variable ? value(choice.binding[x.index]) : x.index;
        if (v < kVariable) {
          if (!fits(v, task.params[i])) {
            return false;
          }
        } else {
          narrow(v, &task.params[i]);
        }
        args_.push_back(v);
      }
      nodes_.push_back(node);
    }
    for (uint32_t k = static_cast<uint32_t>(schema.order.size()); k-- > 0; ) {
      const Node & node = nodes_[first_node + k];
      Cell cell;
      cell.task = node.task;
      cell.args = node.args;
      cell.next = next;
      cell.node = first_node + k;
      uint64_t h = mix((next == kNone ? 0 : cells_[next].hash) + node.task + 1);
      for (size_t i = 0; i < net.steps[schema.order[k]].args.size(); ++i) {
        h = mix(h ^ args_[node.args + i]);
      }
      cell.hash = h;
      next = static_cast<uint32_t>(cells_.size());
      cells_.push_back(cell);
    }
    current = next;
    return true;
  }

  bool apply(uint32_t task, const std::vector<uint32_t> & args)
  {
    const ast::Action & a = m_.pddl.actions[m_.tasks[TaskId(task)].action];
    Env env;
    for (size_t p = 0; p < a.params.size(); ++p) {
      env.emplace_back(a.params[p].name, args[p]);
    }
    if (a.precondition.valid() && !domain_.holds(a.precondition, state_, env)) {
      return false;
    }
    if (a.effect.valid()) {
      std::vector<Atom> adds;
      std::vector<Atom> deletes;
      domain_.effect(a.effect, state_, env, adds, deletes);
      for (const Atom & d : deletes) {
        remove(d);
      }
      for (Atom & atom : adds) {
        add(std::move(atom));
      }
    }
    return true;
  }

  // At the end: a variable that nothing fixed takes any object of its types
  bool settle()
  {
    for (uint32_t v = 0; v < variables_.size(); ++v) {
      if (value(kVariable + v) < kVariable) {
        continue;
      }
      const uint32_t reference = value(kVariable + v);
      bool done = false;
      for (uint32_t o = 0; o < m_.objects.objects.items.size() && !done; ++o) {
        done = fits(o, variables_[reference - kVariable]) && bind(reference, o);
      }
      if (!done) {
        return false;
      }
    }
    return true;
  }

  std::string plan() const
  {
    auto text = [&](uint32_t n) {
        const Node & node = nodes_[n];
        const hddl::TaskInfo & task = m_.tasks[TaskId(node.task)];
        std::string out = std::to_string(n + 1) + " " + task.name;
        for (size_t i = 0; i < task.params.size(); ++i) {
          out += " " + object(value(args_[node.args + i])).name;
        }
        return out;
      };
    std::string out = "==>\n";
    for (uint32_t n : sequence_) {
      out += text(n) + "\n";
    }
    std::vector<std::string> children(nodes_.size());
    std::string roots = "root";
    for (uint32_t n = 0; n < nodes_.size(); ++n) {
      (nodes_[n].parent == kNone ? roots : children[nodes_[n].parent]) +=
        " " + std::to_string(n + 1);
    }
    out += roots + "\n";
    for (uint32_t n = 0; n < nodes_.size(); ++n) {
      if (!m_.tasks[TaskId(nodes_[n].task)].primitive) {
        out += text(n) + " -> " + m_.methods[nodes_[n].method].name + children[n] + "\n";
      }
    }
    return out + "<==\n";
  }

  const Model & m_;
  kairos::Budget * budget_ = nullptr;
  bool started_ = false;
  bool rooted_ = false;
  bool finished_ = false;
  Result last_;
  uint32_t current_ = kNone;                       // the first pending task
  bool forward_ = false;
  size_t stored_ = 0;
  Interpreter domain_;
  Interpreter problem_;
  Schema initial_;
  std::vector<Schema> methods_;
  std::vector<Schema> actions_;                    // by task; the primitive ones
  State state_;
  std::vector<std::set<Atom>> facts_;              // the state by predicate, in a fixed order
  uint64_t state_hash_ = 0;
  std::vector<std::pair<bool, Atom>> trail_;       // added (true) or removed, to undo
  std::vector<Variable> variables_;
  std::vector<uint32_t> bound_;                    // variables given a value, to undo
  std::vector<uint32_t> typed_;                    // variables given one more type, to undo
  uint64_t variables_hash_ = 0;                    // of the values given
  std::vector<Cell> cells_;
  std::vector<uint32_t> args_;
  std::vector<Node> nodes_;
  std::vector<uint32_t> sequence_;                 // nodes of the actions done
  std::vector<Frame> frames_;
  std::unordered_set<uint64_t> seen_;
  bool complete_ = true;
  bool out_ = false;                               // the budget ran out while choosing
  uint64_t work_ = 0;
  uint64_t expansions_ = 0;
};

LiftedSearch::LiftedSearch(const hddl::Model & model)
: impl_(std::make_unique<Impl>(model)) {}

LiftedSearch::~LiftedSearch() = default;

Result LiftedSearch::run(kairos::Budget * budget)
{
  if (unsupported_.empty()) {
    try {
      return impl_->run(budget);
    } catch (const verifier::Invalid & unsupported) {
      unsupported_ = unsupported.reason;
    }
  }
  Result result;
  result.message = unsupported_;
  return result;
}

Result lifted(const hddl::Model & model, kairos::Budget * budget)
{
  return LiftedSearch(model).run(budget);
}

}  // namespace klimax::search

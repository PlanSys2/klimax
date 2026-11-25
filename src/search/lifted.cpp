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
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iterator>
#include <memory>
#include <set>
#include <string>
#include <string_view>
#include <unordered_map>
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

  bool operator==(const Required & other) const
  {
    return predicate == other.predicate && args.size() == other.args.size() &&
           std::equal(args.begin(), args.end(), other.args.begin(),
             [](const Arg & a, const Arg & b) {
               return a.variable == b.variable && a.index == b.index;
             });
  }
};

// An atom that an action adds or deletes, over the parameters of the method it is a step of
struct Touch
{
  bool add;
  uint32_t predicate;
  std::vector<Arg> args;
};

// What a method or an action asks of its parameters (the initial task network too)
struct Schema
{
  std::vector<const sem::TypeSpec *> types;  // by parameter
  std::vector<bool> needed;      // a value is needed before going on: the rest can wait
  std::vector<Required> required;
  size_t own = 0;                // the first ones are of its precondition; the rest, pulled up
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
  std::vector<uint32_t> watch;  // the atoms in constraints_ that wait for its value
};

// A pending task; the ones after it follow by `next`
struct Cell
{
  uint32_t task;
  uint32_t args;   // in args_
  uint32_t next;
  uint32_t node;   // its place in the decomposition
  uint64_t hash;   // of the task, its arguments and everything after
  bool open;       // some argument, here or after, was a variable when it was made
};

struct Node
{
  uint32_t task;
  uint32_t args;
  uint32_t parent;
  uint32_t method = kNone;
  uint64_t state = 0;    // the hash of the state when it was decomposed
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

// Where the enumeration of the ways of a choice point is: the values given so far and what
// gives the next one (the atoms of a predicate, the objects, or nothing more: a leaf)
struct Level
{
  Choice choice;
  enum class By : uint8_t {Fact, Object, Leaf} by = By::Leaf;
  uint32_t index = 0;    // of the required atom or of the parameter
  Atom last;             // Fact: the last atom tried, if `any`
  bool any = false;
  uint32_t object = 0;   // Object: the next one to try
};

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
  size_t numbers;
  size_t variables;
  size_t bound;
  size_t typed;
  size_t watched;
  size_t constraints;
  uint64_t state_hash;
  uint64_t variables_hash;
  Choice start;               // Root, Action: what is known of the parameters
  bool started = false;
  size_t method = 0;          // Method: the next one of the task to try
  std::vector<uint32_t> order;  // Method: the methods in the order to try them, if not the domain's
  std::vector<Level> levels;
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
      run_from_ = std::chrono::steady_clock::now();
      search(last_);
      spent_ = seconds();
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
      // A partially ordered network is taken in one of the orders it allows: a plan found is a
      // plan, and none found proves nothing
      complete_ = m_.total_order;
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
      if (improving_ && (out_ || ((++ticks_ & 255u) == 0 && (seconds() > allowance_ ||
        (budget_ != nullptr && budget_->exhausted())))))
      {
        result = best_;  // no time for a shorter one
        return;
      }
      if (out_ || (budget_ != nullptr && budget_->exhausted())) {
        result.outcome = Outcome::OutOfBudget;
        finished_ = false;
        return;
      }
      if (cells_.size() > kCells && improving_) {
        result = best_;
        return;
      }
      if (cells_.size() > kCells) {
        result.outcome = Outcome::OutOfBudget;
        result.message = "the search that does not ground ran out of its memory";
        return;
      }
      if (frames_.empty() && !rooted_) {
        Frame first;
        first.kind = Kind::Root;
        mark(first);
        first.start.binding.assign(m_.initial_params.size(), kNone);
        rooted_ = true;
        frames_.push_back(std::move(first));
        continue;
      }
      if (!forward) {
        while (!frames_.empty()) {
          Frame & frame = frames_.back();
          restore(frame);
          Choice choice;
          if (produce(frame, choice)) {
            if (take(frame, std::move(choice), current)) {
              forward = true;
              break;
            }
            continue;  // that way does not fit: the next one
          }
          if (out_) {
            break;  // the budget ran out while choosing: from this choice point the next time
          }
          frames_.pop_back();
        }
        if (out_) {
          continue;
        }
        if (!forward && cut_ && repeats_ >= kRepeats) {
          complete_ = false;
          cut_ = false;
        }
        if (!forward && cut_) {
          // Something was left out for coming back to the same task: once more is allowed now
          ++repeats_;
          cut_ = false;
          rooted_ = false;
          seen_.clear();
          current = kNone;
          continue;
        }
        if (!forward && improving_) {
          // Nothing that short: one more try, less ambitious, or the plan there is
          if (++stage_ > 1 || !shorter()) {
            result = best_;
            return;
          }
          continue;
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
          best_.outcome = Outcome::Solved;
          best_.plan = plan();
          best_.steps = sequence_.size();
          ++best_.plans;
          best_.message = "depth first, not grounded";
          // The first plan of a search in depth may be many times longer than needed: for
          // half as long as it took, within limits, plans half as long are looked for
          if (!improving_) {
            improving_ = true;
            allowance_ = seconds() + std::clamp(0.5 * seconds(), kImprove, 25.0 * kImprove);
          }
          stage_ = 0;
          if (!shorter()) {
            result = best_;
            return;
          }
        }
        continue;
      }
      const Cell cell = cells_[current];
      ++expansions_;
      if (trace_ && (expansions_ & 0x7ffff) == 0) {
        std::string doing;
        for (size_t i = sequence_.size() > 12 ? sequence_.size() - 12 : 0; i < sequence_.size(); ++i) {
          const Node & n = nodes_[sequence_[i]];
          doing += " " + m_.tasks[TaskId(n.task)].name;
          for (size_t k = 0; k < m_.tasks[TaskId(n.task)].params.size(); ++k) {
            const uint32_t v = value(args_[n.args + k]);
            doing += v < kVariable ? "_" + object(v).name : "_?";
          }
        }
        std::fprintf(stderr, "lifted: %llu expansions, %zu actions, %zu frames, %zu cells, "
          "repeats %u:%s\n", static_cast<unsigned long long>(expansions_), sequence_.size(),
          frames_.size(), cells_.size(), repeats_, doing.c_str());
      }
      const hddl::TaskInfo & task = m_.tasks[TaskId(cell.task)];
      if (task.primitive) {
        Choice choice;
        choice.binding.resize(task.params.size());
        bool pending = false;
        for (size_t i = 0; i < task.params.size(); ++i) {
          choice.binding[i] = value(args_[cell.args + i]);
          pending = pending || choice.binding[i] >= kVariable;
        }
        if (sequence_.size() >= longest_) {
          continue;
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
        frame.start = std::move(choice);
        frames_.push_back(std::move(frame));
        continue;
      }
      if (again(cell)) {
        cut_ = true;
        continue;
      }
      // The task with the values its arguments have now. The values given to variables count
      // only if some pending task has one: otherwise the future does not depend on them
      uint64_t pending = mix((cell.next == kNone ? 0 : cells_[cell.next].hash) + cell.task + 1);
      bool open = cell.next != kNone && cells_[cell.next].open;
      for (size_t i = 0; i < task.params.size(); ++i) {
        const uint32_t v = value(args_[cell.args + i]);
        pending = mix(pending ^ v);
        open = open || v >= kVariable;
      }
      const uint64_t key = mix(state_hash_ ^ pending ^ (open ? variables_hash_ : 0));
      if (!fresh(key)) {
        continue;
      }
      nodes_[cell.node].state = state_hash_;
      Frame frame;
      frame.cell = current;
      mark(frame);
      if (!goal_.empty() && m_.methods_of[cell.task].size() > 1) {
        by_goal(cell, frame.order);
      }
      frames_.push_back(std::move(frame));
    }
  }

  // The methods of the task, first those whose actions undo less of the goal: the atoms of
  // the goal that hold now and an action of the method deletes, less those that do not hold
  // and one adds. A parameter that the task does not fix matches anything. Ties in the order
  // of the domain
  void by_goal(const Cell & cell, std::vector<uint32_t> & order) const
  {
    const std::vector<uint32_t> & methods = m_.methods_of[cell.task];
    std::vector<std::pair<int, uint32_t>> ranked;
    bool differ = false;
    for (uint32_t i = 0; i < methods.size(); ++i) {
      const uint32_t m = methods[i];
      int undone = 0;
      Choice choice;
      choice.method = m;
      choice.binding.assign(m_.methods[m].params.size(), kNone);
      if (!touches_[m].empty() && unify(m_.methods[m], methods_[m], cell, choice)) {
        for (const Touch & touch : touches_[m]) {
          for (const Atom & wanted : goal_) {
            if (wanted[0] != touch.predicate || wanted.size() != touch.args.size() + 1 ||
              (state_.count(wanted) != 0) == touch.add)
            {
              continue;
            }
            bool match = true;
            for (size_t k = 0; match && k < touch.args.size(); ++k) {
              const Arg & x = touch.args[k];
              const uint32_t v = !x.variable ? x.index :
                choice.binding[x.index] == kNone ? kNone : value(choice, choice.binding[x.index]);
              match = v >= kVariable || v == wanted[k + 1];
            }
            undone += match ? (touch.add ? -1 : 1) : 0;
          }
        }
      }
      differ = differ || undone != 0;
      ranked.emplace_back(undone, i);
    }
    if (!differ) {
      return;
    }
    std::sort(ranked.begin(), ranked.end());
    for (const auto & [undone, i] : ranked) {
      (void)undone;
      order.push_back(methods[i]);
    }
  }

  static constexpr double kImprove = 0.002;    // seconds, at least, looking for a shorter plan

  double seconds() const
  {
    return spent_ + std::chrono::duration<double>(
      std::chrono::steady_clock::now() - run_from_).count();
  }

  // Starts again for a plan shorter than the one found: half as long, then three quarters.
  // False if there is nothing shorter to try
  bool shorter()
  {
    const size_t target = stage_ == 0 ? best_.steps / 2 : best_.steps * 3 / 4;
    if (best_.steps == 0 || target >= best_.steps || (stage_ == 1 && target == best_.steps / 2)) {
      return false;
    }
    while (!frames_.empty()) {
      restore(frames_.back());
      frames_.pop_back();
    }
    longest_ = target;
    rooted_ = false;
    cut_ = false;
    seen_.clear();
    current_ = kNone;
    forward_ = false;
    return true;
  }

  static constexpr size_t kSeen = 5000000;     // tasks and states remembered
  static constexpr size_t kCells = 5000000;    // pending tasks made along one branch
  static constexpr uint32_t kAncestors = 64;   // looked at for a recursion without progress
  static constexpr uint32_t kRepeats = 3;      // of a task from itself with nothing done

  // The task comes from itself, with the same arguments and in the same state, more than
  // `repeats_` times: a recursion that does not progress. Looks kAncestors back
  bool again(const Cell & cell) const
  {
    const hddl::TaskInfo & task = m_.tasks[TaskId(cell.task)];
    uint32_t times = 0;
    uint32_t back = 0;
    for (uint32_t n = nodes_[cell.node].parent; n != kNone && back < kAncestors;
      n = nodes_[n].parent, ++back)
    {
      const Node & node = nodes_[n];
      if (node.task != cell.task || node.state != state_hash_) {
        continue;
      }
      // Two arguments with no value yet are the same one: any value the one can take, the
      // other can
      bool same = true;
      for (size_t i = 0; same && i < task.params.size(); ++i) {
        const uint32_t a = value(args_[node.args + i]);
        const uint32_t b = value(args_[cell.args + i]);
        same = a == b || (a >= kVariable && b >= kVariable);
      }
      if (same && ++times > repeats_) {
        return true;
      }
    }
    return false;
  }

  const Schema & schema_of(const Frame & frame, const Choice & choice) const
  {
    return frame.kind == Kind::Root ? initial_ :
           frame.kind == Kind::Action ? actions_[cells_[frame.cell].task] : methods_[choice.method];
  }

  // What gives the next value to `choice`: an atom that the precondition requires with a
  // parameter still open, then a parameter that needs a value now
  Level level(const Schema & schema, Choice choice) const
  {
    Level l;
    for (uint32_t r = 0; r < schema.own && l.by == Level::By::Leaf; ++r) {
      const std::vector<Arg> & args = schema.required[r].args;
      if (std::any_of(args.begin(), args.end(),
        [&](const Arg & x) {return x.variable && open(choice, x.index);}))
      {
        l.by = Level::By::Fact;
        l.index = r;
      }
    }
    for (uint32_t p = 0; p < schema.types.size() && l.by == Level::By::Leaf; ++p) {
      if (schema.needed[p] && open(choice, p)) {
        l.by = Level::By::Object;
        l.index = p;
      }
    }
    l.choice = std::move(choice);
    return l;
  }

  // The first level of the next thing to enumerate in the choice point; false: nothing more
  bool seed(Frame & frame)
  {
    if (frame.kind != Kind::Method) {
      if (frame.started) {
        return false;
      }
      frame.started = true;
      frame.levels.push_back(level(schema_of(frame, frame.start), frame.start));
      return true;
    }
    const Cell & cell = cells_[frame.cell];
    const std::vector<uint32_t> & methods =
      frame.order.empty() ? m_.methods_of[cell.task] : frame.order;
    while (frame.method < methods.size()) {
      const uint32_t m = methods[frame.method++];
      Choice choice;
      choice.method = m;
      choice.binding.assign(m_.methods[m].params.size(), kNone);
      if (unify(m_.methods[m], methods_[m], cell, choice)) {
        frame.levels.push_back(level(methods_[m], std::move(choice)));
        return true;
      }
    }
    return false;
  }

  // The next way of the choice point, in `out`: values from the atoms that the precondition
  // requires, by type for the other parameters that need one now; the constraints, the types
  // of the subtasks' arguments and the precondition hold. False: no more, or out of budget
  // (`out_`: it goes on from the same place the next time). The state is that of the frame
  bool produce(Frame & frame, Choice & out)
  {
    while (true) {
      if ((++work_ & 1023) == 0 && budget_ != nullptr && budget_->exhausted()) {
        out_ = true;
        return false;
      }
      if (frame.levels.empty()) {
        if (!seed(frame)) {
          return false;
        }
        continue;
      }
      Level & l = frame.levels.back();
      const Schema & schema = schema_of(frame, l.choice);
      if (l.by == Level::By::Leaf) {
        out = std::move(l.choice);
        frame.levels.pop_back();
        if (valid(schema, out)) {
          return true;
        }
        continue;
      }
      Choice next;
      bool found = false;
      if (l.by == Level::By::Fact) {
        const Required & r = schema.required[l.index];
        const std::set<Atom> & atoms = facts_[r.predicate];
        for (auto it = l.any ? atoms.upper_bound(l.last) : atoms.begin();
          it != atoms.end() && !found; ++it)
        {
          const Atom & atom = *it;
          if (atom.size() != r.args.size() + 1) {
            continue;
          }
          next = l.choice;
          found = true;
          for (size_t i = 0; found && i < r.args.size(); ++i) {
            const Arg & x = r.args[i];
            found = x.variable ? give(schema, next, x.index, atom[i + 1]) : x.index == atom[i + 1];
          }
          if (found) {
            l.last = atom;
            l.any = true;
          }
        }
      } else {
        for (; l.object < m_.objects.objects.items.size() && !found; ++l.object) {
          next = l.choice;
          found = give(schema, next, l.index, l.object);
        }
      }
      if (!found) {
        frame.levels.pop_back();
        continue;
      }
      frame.levels.push_back(level(schema, std::move(next)));
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
      schema.own = schema.required.size();
    }
    pull_up();
    touches();
  }

  void touched(ast::EffectId id, const ast::Action & a, std::vector<Touch> & out)
  {
    const ast::Effect & e = m_.pddl.nodes.effects[id];
    if (e.kind == ast::EffectKind::And) {
      for (ast::EffectId c : e.children) {
        touched(c, a, out);
      }
      return;
    }
    if (e.kind != ast::EffectKind::Add && e.kind != ast::EffectKind::Delete) {
      return;
    }
    Touch touch{e.kind == ast::EffectKind::Add, domain_.predicate(e.name), {}};
    for (ast::TermId t : e.args) {
      const ast::Term & term = domain_.term(t);
      if (term.kind == ast::TermKind::Name) {
        touch.args.push_back(Arg{false, domain_.object(term.name)});
        continue;
      }
      size_t p = 0;
      while (p < a.params.size() && a.params[p].name != term.name) {
        ++p;
      }
      if (term.kind != ast::TermKind::Variable || p == a.params.size()) {
        return;
      }
      touch.args.push_back(Arg{true, static_cast<uint32_t>(p)});
    }
    out.push_back(std::move(touch));
  }

  // The atoms of the goal, and what the actions of each method add and delete, over the
  // parameters of the method: only for problems with a goal
  void touches()
  {
    if (!m_.problem.goal.valid()) {
      return;
    }
    std::vector<ast::FormulaId> atoms;
    problem_.required(m_.problem.goal, atoms);
    Env env;
    for (ast::FormulaId id : atoms) {
      const ast::Formula & f = problem_.formula(id);
      bool ground = true;
      for (ast::TermId t : f.args) {
        ground = ground && problem_.term(t).kind == ast::TermKind::Name;
      }
      if (ground) {
        goal_.push_back(problem_.atom(f.name, f.args, env));
      }
    }
    touches_.resize(m_.methods.size());
    if (goal_.empty()) {
      return;
    }
    for (size_t m = 0; m < m_.methods.size(); ++m) {
      for (const hddl::Step & step : m_.methods[m].network.steps) {
        const hddl::TaskInfo & task = m_.tasks[step.task];
        if (!task.primitive || !m_.pddl.actions[task.action].effect.valid()) {
          continue;
        }
        std::vector<Touch> of_action;
        touched(m_.pddl.actions[task.action].effect, m_.pddl.actions[task.action], of_action);
        for (Touch & touch : of_action) {
          for (Arg & x : touch.args) {
            if (x.variable) {
              x = step.args[x.index];
            }
          }
          touches_[m].push_back(std::move(touch));
        }
      }
    }
  }

  void changed(ast::EffectId id, std::vector<bool> & changes)
  {
    const ast::Effect & e = m_.pddl.nodes.effects[id];
    if (e.kind == ast::EffectKind::Add || e.kind == ast::EffectKind::Delete) {
      changes[domain_.predicate(e.name)] = true;
    }
    for (ast::EffectId c : e.children) {
      changed(c, changes);
    }
  }

  // The atoms of `from`, over the parameters of a task, for a step with those arguments
  static void through(
    const std::vector<Required> & from, const std::vector<Arg> & args, std::vector<Required> & to)
  {
    for (Required r : from) {
      for (Arg & x : r.args) {
        if (x.variable) {
          x = args[x.index];
        }
      }
      if (std::find(to.begin(), to.end(), r) == to.end()) {
        to.push_back(std::move(r));
      }
    }
  }

  // An atom of a predicate that no action changes, required by an action, is required by
  // every method that leads to it: checked there, a wrong choice of parameters is seen at
  // once and not when the action comes. For a compound task, what all its methods require
  void pull_up()
  {
    std::vector<bool> changes(m_.domain.predicates.items.size(), false);
    for (const ast::Action & a : m_.pddl.actions) {
      if (a.effect.valid()) {
        changed(a.effect, changes);
      }
    }
    auto fixed_of = [&](const std::vector<Required> & all) {
        std::vector<Required> fixed;
        std::copy_if(all.begin(), all.end(), std::back_inserter(fixed),
          [&](const Required & r) {return !changes[r.predicate];});
        return fixed;
      };
    auto with_steps = [&](const Schema & schema, const std::vector<std::vector<Required>> & of) {
        std::vector<Required> all = fixed_of(schema.required);
        for (const hddl::Step & step : schema.network->steps) {
          through(of[step.task.value()], step.args, all);
        }
        return all;
      };
    std::vector<std::vector<Required>> of(m_.tasks.size());  // by task, over its parameters
    for (uint32_t t = 0; t < m_.tasks.size(); ++t) {
      if (m_.tasks[TaskId(t)].primitive) {
        of[t] = fixed_of(actions_[t].required);
      }
    }
    for (int round = 0; round < 4; ++round) {
      for (uint32_t t = 0; t < m_.tasks.size(); ++t) {
        if (m_.tasks[TaskId(t)].primitive) {
          continue;
        }
        std::vector<Required> common;
        bool first = true;
        for (uint32_t m : m_.methods_of[t]) {
          const hddl::MethodInfo & method = m_.methods[m];
          std::vector<Required> mine;
          for (Required r : with_steps(methods_[m], of)) {
            bool usable = true;
            for (Arg & x : r.args) {
              if (!x.variable) {
                continue;
              }
              size_t i = 0;
              while (i < method.task_args.size() &&
                !(method.task_args[i].variable && method.task_args[i].index == x.index))
              {
                ++i;
              }
              usable = usable && i < method.task_args.size();
              x.index = static_cast<uint32_t>(i);
            }
            if (usable) {
              mine.push_back(std::move(r));
            }
          }
          if (first) {
            common = std::move(mine);
            first = false;
          } else {
            common.erase(std::remove_if(common.begin(), common.end(),
              [&](const Required & r) {
                return std::find(mine.begin(), mine.end(), r) == mine.end();
              }), common.end());
          }
        }
        of[t] = std::move(common);
      }
    }
    auto extend = [&](Schema & schema) {
        schema.own = schema.required.size();
        for (const hddl::Step & step : schema.network->steps) {
          through(of[step.task.value()], step.args, schema.required);
        }
      };
    extend(initial_);
    std::for_each(methods_.begin(), methods_.end(), extend);
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
      if (item.time || item.negated) {
        verifier::invalid("unsupported: timed initial literals");
      }
      if (item.kind != ast::InitItem::Kind::Fact) {
        if (m_.problem.symbols.name(item.name) == std::string_view("total-cost")) {
          continue;
        }
        Atom fluent{problem_.function(item.name)};
        for (ast::TermId t : item.args) {
          fluent.push_back(problem_.object(t, env));
        }
        const std::optional<double> value = problem_.number(item.value, env);
        if (!value) {
          verifier::invalid("unsupported: this initial value");
        }
        set(std::move(fluent), *value);
        continue;
      }
      add(problem_.atom(item.name, item.args, env));
    }
    domain_.numbers(&values_);
    problem_.numbers(&values_);
    trail_.clear();
    numbers_.clear();
  }

  static uint64_t hash_with(const Atom & fluent, double value)
  {
    uint64_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&bits, &value, sizeof(bits));
    return mix(hash_of(fluent) ^ mix(bits + 0x9e3779b97f4a7c15ull));
  }

  // A numeric fluent takes a value
  void set(Atom fluent, double value)
  {
    auto [it, fresh] = values_.emplace(fluent, value);
    if (!fresh && it->second == value) {
      return;
    }
    if (!fresh) {
      state_hash_ ^= hash_with(fluent, it->second);
    }
    numbers_.push_back(Number{std::move(fluent), !fresh, fresh ? 0.0 : it->second});
    it->second = value;
    state_hash_ ^= hash_with(numbers_.back().fluent, value);
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
    frame.numbers = numbers_.size();
    frame.variables = variables_.size();
    frame.bound = bound_.size();
    frame.typed = typed_.size();
    frame.watched = watched_.size();
    frame.constraints = constraints_.size();
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
    while (numbers_.size() > frame.numbers) {
      Number & n = numbers_.back();
      if (n.had) {
        values_[n.fluent] = n.old;
      } else {
        values_.erase(n.fluent);
      }
      numbers_.pop_back();
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
    while (watched_.size() > frame.watched) {
      variables_[watched_.back()].watch.pop_back();
      watched_.pop_back();
    }
    constraints_.resize(frame.constraints);
    variables_.resize(frame.variables);
    cells_.resize(frame.cells);
    args_.resize(frame.args);
    nodes_.resize(frame.nodes);
    sequence_.resize(frame.sequence);
  }

  // Under a bound on the length, the same again counts only with fewer actions done
  bool fresh(uint64_t key)
  {
    const uint32_t actions = static_cast<uint32_t>(sequence_.size());
    if (seen_.size() >= kSeen) {
      complete_ = false;
      const auto it = seen_.find(key);
      return it == seen_.end() || actions < it->second;
    }
    const auto [it, fresh] = seen_.emplace(key, actions);
    if (!fresh) {
      if (it->second <= actions) {
        return false;
      }
      it->second = actions;
    }
    return true;
  }

  // The variable (a reference to it) takes a value now: an object or another variable
  bool bind(uint32_t reference, uint32_t to)
  {
    Variable & variable = variables_[reference - kVariable];
    if (to < kVariable) {
      if (!fits(to, variable)) {
        return false;
      }
      // The atoms that waited for this value, if it was the last one they waited for
      variable.value = to;
      for (uint32_t c : variable.watch) {
        Atom atom = constraints_[c];
        bool known = true;
        for (size_t i = 1; i < atom.size(); ++i) {
          atom[i] = value(atom[i]);
          known = known && atom[i] < kVariable;
        }
        if (known && state_.count(atom) == 0) {
          variable.value = kNone;
          return false;
        }
      }
    } else {
      for (const sem::TypeSpec * t : variable.types) {
        narrow(to, t);
      }
      const std::vector<uint32_t> waiting = variable.watch;
      for (uint32_t c : waiting) {
        variables_[to - kVariable].watch.push_back(c);
        watched_.push_back(to - kVariable);
      }
    }
    variables_[reference - kVariable].value = to;
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

  // The constraints, the types of the subtasks' arguments and the precondition hold
  bool valid(const Schema & schema, const Choice & choice)
  {
    auto of = [&](const Arg & x) {return x.variable ? value(choice, choice.binding[x.index]) : x.index;};
    if (schema.network != nullptr) {
      for (const hddl::VarConstraint & c : schema.network->constraints) {
        if ((of(c.lhs) == of(c.rhs)) != c.equal) {
          return false;
        }
      }
      for (const hddl::SortConstraint & c : schema.network->sorts) {
        const uint32_t o = value(choice, choice.binding[c.param]);
        if (m_.domain.types.fits(object(o).types, c.types) == c.negated) {
          return false;
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
            return false;
          }
        }
      }
    }
    // What was pulled up, where its values are known: the rest waits (see take())
    for (size_t r = schema.own; r < schema.required.size(); ++r) {
      Atom atom{schema.required[r].predicate};
      bool known = true;
      for (const Arg & x : schema.required[r].args) {
        atom.push_back(x.variable && choice.binding[x.index] == kNone ? kNone : of(x));
        known = known && atom.back() < kVariable;
      }
      if (known && state_.count(atom) == 0) {
        return false;
      }
    }
    if (schema.condition != nullptr && schema.network != nullptr) {
      Env env;
      for (size_t p = 0; p < schema.condition->params.size(); ++p) {
        env.emplace_back(schema.condition->params[p].name,
          value(choice, choice.binding[schema.condition_params[p]]));
      }
      if (!domain_.holds(schema.condition->precondition, state_, env)) {
        return false;
      }
    }
    return true;
  }

  // Goes that way of `frame`: false if it does not fit after all (the frame is to be
  // restored). `current` is then the first pending task
  bool take(const Frame & frame, Choice choice, uint32_t & current)
  {
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
    // What was pulled up and has a variable waits for its value
    for (size_t r = schema.own; r < schema.required.size(); ++r) {
      Atom atom{schema.required[r].predicate};
      for (const Arg & x : schema.required[r].args) {
        atom.push_back(x.variable ? value(choice.binding[x.index]) : x.index);
      }
      const uint32_t c = static_cast<uint32_t>(constraints_.size());
      bool waits = false;
      for (size_t i = 1; i < atom.size(); ++i) {
        const auto here = atom.begin() + static_cast<std::ptrdiff_t>(i);
        if (atom[i] >= kVariable && std::find(atom.begin() + 1, here, atom[i]) == here) {
          variables_[atom[i] - kVariable].watch.push_back(c);
          watched_.push_back(atom[i] - kVariable);
          waits = true;
        }
      }
      if (waits) {
        constraints_.push_back(std::move(atom));
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
      cell.open = next != kNone && cells_[next].open;
      for (size_t i = 0; i < net.steps[schema.order[k]].args.size(); ++i) {
        h = mix(h ^ args_[node.args + i]);
        cell.open = cell.open || args_[node.args + i] >= kVariable;
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
      std::vector<verifier::Change> changes;
      domain_.effect(a.effect, state_, env, adds, deletes, &changes);
      for (const verifier::Change & change : changes) {
        if (change.value != change.value) {
          return false;  // it reads a number with no value
        }
      }
      for (const Atom & d : deletes) {
        remove(d);
      }
      for (Atom & atom : adds) {
        add(std::move(atom));
      }
      for (verifier::Change & change : changes) {
        set(std::move(change.fluent), change.value);
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
  bool trace_ = std::getenv("KLIMAX_TRACE_SEARCH") != nullptr;
  size_t longest_ = ~size_t{0};                    // actions allowed: fewer than the plan found
  bool improving_ = false;                         // a plan was found: looking for a shorter one
  Result best_;
  int stage_ = 0;                                  // 0: half as long; 1: three quarters
  double allowance_ = 0.0;                         // seconds of search until which to improve
  double spent_ = 0.0;                             // seconds of search in the calls before
  std::chrono::steady_clock::time_point run_from_;
  uint32_t ticks_ = 0;
  uint32_t repeats_ = 0;                           // of a task from itself, allowed
  bool cut_ = false;                               // some were left out for that
  Interpreter domain_;
  Interpreter problem_;
  Schema initial_;
  std::vector<Schema> methods_;
  std::vector<Atom> goal_;                         // the atoms that the goal requires
  std::vector<std::vector<Touch>> touches_;        // by method: what its actions add and delete
  std::vector<Schema> actions_;                    // by task; the primitive ones
  State state_;
  std::vector<std::set<Atom>> facts_;              // the state by predicate, in a fixed order
  uint64_t state_hash_ = 0;
  verifier::Values values_;                        // of the numeric fluents
  struct Number
  {
    Atom fluent;
    bool had;
    double old;
  };
  std::vector<Number> numbers_;                    // values given, to undo
  std::vector<std::pair<bool, Atom>> trail_;       // added (true) or removed, to undo
  std::vector<Variable> variables_;
  std::vector<uint32_t> bound_;                    // variables given a value, to undo
  std::vector<Atom> constraints_;                  // pulled up, waiting for variables
  std::vector<uint32_t> watched_;                  // variables given one more to watch, to undo
  std::vector<uint32_t> typed_;                    // variables given one more type, to undo
  uint64_t variables_hash_ = 0;                    // of the values given
  std::vector<Cell> cells_;
  std::vector<uint32_t> args_;
  std::vector<Node> nodes_;
  std::vector<uint32_t> sequence_;                 // nodes of the actions done
  std::vector<Frame> frames_;
  std::unordered_map<uint64_t, uint32_t> seen_;    // the fewest actions done when seen
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

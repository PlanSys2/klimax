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

#include "hddl/model.hpp"

#include <algorithm>
#include <functional>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "semantic/analyzer.hpp"

namespace klimax::hddl
{

namespace
{

using kairos::Diagnostic;

// Prefix of the actions that stand for method preconditions; ':' cannot be in a PDDL name
constexpr const char * kPreconditionPrefix = "method:";

class Analyzer
{
public:
  Analyzer(Domain domain, Problem problem)
  : methods_(std::move(domain.methods)), task_decls_(std::move(domain.tasks)),
    htn_(std::move(problem.htn)), htn_params_(std::move(problem.htn_params)),
    htn_loc_(problem.htn_loc)
  {
    m_.pddl = std::move(domain.pddl);
    m_.problem = std::move(problem.pddl);
    m_.has_htn = problem.has_htn;
  }

  kairos::Result<Model> run()
  {
    function_terms();
    add_precondition_actions();
    auto dm = kairos::sem::analyze_domain(m_.pddl);
    take(dm.diagnostics());
    if (!dm.ok()) {
      return fail();
    }
    m_.domain = std::move(dm).value();
    file_ = &m_.pddl.file;
    symbols_ = &m_.pddl.symbols;
    nodes_ = &m_.pddl.nodes;
    objects_ = &m_.domain.constants;
    declare_tasks();
    for (size_t i = 0; i < methods_.size(); ++i) {
      analyze_method(i);
    }

    auto pm = kairos::sem::analyze_problem(m_.problem, m_.pddl, m_.domain);
    take(pm.diagnostics());
    if (!pm.ok()) {
      return fail();
    }
    m_.objects = std::move(pm).value();
    file_ = &m_.problem.file;
    symbols_ = &m_.problem.symbols;
    nodes_ = &m_.problem.nodes;
    objects_ = &m_.objects.objects;
    analyze_initial();
    if (errors_) {
      return fail();
    }

    m_.has_goal = m_.problem.goal.valid();

    m_.methods_of.assign(m_.tasks.size(), {});
    m_.total_order = !m_.has_htn || m_.initial.total_order;
    for (size_t i = 0; i < m_.methods.size(); ++i) {
      m_.methods_of[m_.methods[i].task.index()].push_back(static_cast<uint32_t>(i));
      m_.total_order = m_.total_order && m_.methods[i].network.total_order;
    }
    return kairos::Result<Model>::success(std::move(m_), std::move(diagnostics_));
  }

private:
  using Objects = sem::Table<sem::ObjectId, sem::ObjectInfo>;

  kairos::Result<Model> fail() {return kairos::Result<Model>::failure(std::move(diagnostics_));}

  void take(const std::vector<Diagnostic> & more)
  {
    diagnostics_.insert(diagnostics_.end(), more.begin(), more.end());
  }

  void error(const std::string & code, const std::string & message, ast::Loc loc)
  {
    diagnostics_.push_back(
      Diagnostic::error(code, message, kairos::SourceLocation{*file_, loc.line, loc.column}));
    errors_ = true;
  }

  std::string name(kairos::Symbol s) const {return std::string(symbols_->name(s));}

  // One action per method precondition, checked and grounded by Kairos as any other
  // A function term as the argument of a subtask, (go ?t (in ?b)), is the value of the fluent
  // when the method is applied: a new parameter of the method, which its precondition makes
  // equal to it
  void function_terms()
  {
    size_t fresh = 0;
    for (Method & method : methods_) {
      for (Subtask & subtask : method.network.subtasks) {
        for (ast::TermId & arg : subtask.args) {
          const ast::Term term = m_.pddl.nodes.terms[arg];
          if (term.kind != ast::TermKind::Function) {
            continue;
          }
          const auto declared = std::find_if(m_.pddl.functions.begin(), m_.pddl.functions.end(),
              [&](const ast::FunctionDecl & f) {return f.name == term.name;});
          if (declared == m_.pddl.functions.end() || declared->type.empty()) {
            continue;  // reported where the argument is read
          }
          ast::TypedName param;
          param.name = m_.pddl.symbols.intern("?klimax-value-" + std::to_string(fresh++));
          param.types = declared->type;
          param.loc = term.loc;
          param.type_loc = term.loc;
          method.params.push_back(param);
          ast::Term variable;
          variable.kind = ast::TermKind::Variable;
          variable.name = param.name;
          variable.loc = term.loc;
          const ast::TermId value = m_.pddl.nodes.terms.push_back(variable);
          ast::Expr fluent;
          fluent.kind = ast::ExprKind::Function;
          fluent.name = term.name;
          fluent.args = term.args;
          fluent.loc = term.loc;
          ast::Expr same;
          same.kind = ast::ExprKind::Term;
          same.term = value;
          same.loc = term.loc;
          ast::Formula equal;
          equal.kind = ast::FormulaKind::Compare;
          equal.compare = ast::CompareOp::Eq;
          equal.lhs = m_.pddl.nodes.exprs.push_back(std::move(fluent));
          equal.rhs = m_.pddl.nodes.exprs.push_back(std::move(same));
          equal.loc = term.loc;
          ast::FormulaId condition = m_.pddl.nodes.formulas.push_back(std::move(equal));
          if (method.durative) {
            ast::Formula at_start;
            at_start.kind = ast::FormulaKind::AtStart;
            at_start.children.push_back(condition);
            at_start.loc = term.loc;
            condition = m_.pddl.nodes.formulas.push_back(std::move(at_start));
          }
          ast::Formula both;
          both.kind = ast::FormulaKind::And;
          both.loc = term.loc;
          if (method.precondition.valid()) {
            both.children.push_back(method.precondition);
          }
          both.children.push_back(condition);
          method.precondition = m_.pddl.nodes.formulas.push_back(std::move(both));
          arg = value;
        }
      }
    }
  }

  void add_precondition_actions()
  {
    m_.user_actions = m_.pddl.actions.size();
    precondition_action_.assign(methods_.size(), -1);
    for (size_t i = 0; i < methods_.size(); ++i) {
      const Method & method = methods_[i];
      if (!method.precondition.valid()) {
        precondition_params_.emplace_back();
        continue;
      }
      ast::Action a;
      a.name = m_.pddl.symbols.intern(
        kPreconditionPrefix + std::string(m_.pddl.symbols.name(method.name)) + "#" +
        std::to_string(i));
      // Only the parameters the precondition mentions: the others would multiply its instances
      std::unordered_set<uint32_t> used;
      variables_of(method.precondition, used);
      precondition_params_.emplace_back();
      for (uint32_t p = 0; p < method.params.size(); ++p) {
        if (used.count(method.params[p].name.value()) != 0) {
          a.params.push_back(method.params[p]);
          precondition_params_.back().push_back(p);
        }
      }
      a.precondition = method.precondition;
      a.loc = method.loc;
      if (method.durative) {
        // The search tests what must hold at start; Kairos also reads the rest, as an action
        // that nothing asks for, so that its names and types are checked
        ast::Action rest = a;
        rest.name = m_.pddl.symbols.intern(
          std::string(kPreconditionPrefix) + "later " +
          std::string(m_.pddl.symbols.name(method.name)) + "#" + std::to_string(i));
        a.precondition = part(method.precondition, true);
        rest.precondition = part(method.precondition, false);
        later_.push_back(std::move(rest));
      }
      precondition_action_[i] = static_cast<int32_t>(m_.pddl.actions.size());
      m_.pddl.actions.push_back(std::move(a));
    }
    // The conditions of the constraints on states too: read by Kairos, asked for by nothing
    for (size_t i = 0; i < methods_.size(); ++i) {
      const Method & method = methods_[i];
      size_t k = 0;
      for (const Constraint & c : method.network.constraints) {
        for (ast::FormulaId formula : {c.first, c.second}) {
          if (!formula.valid()) {
            continue;
          }
          ast::Action a;
          a.name = m_.pddl.symbols.intern(
            std::string(kPreconditionPrefix) + "constraint " +
            std::string(m_.pddl.symbols.name(method.name)) + "#" + std::to_string(i) + "#" +
            std::to_string(k++));
          std::unordered_set<uint32_t> used;
          variables_of(formula, used);
          std::vector<uint32_t> positions;
          for (uint32_t p = 0; p < method.params.size(); ++p) {
            if (used.count(method.params[p].name.value()) != 0) {
              a.params.push_back(method.params[p]);
              positions.push_back(p);
            }
          }
          a.precondition = formula;
          a.loc = c.loc;
          constraint_action_[formula.value()] = {later_.size(), std::move(positions)};
          later_.push_back(std::move(a));
        }
      }
    }
    later_from_ = m_.pddl.actions.size();
    for (ast::Action & a : later_) {
      m_.pddl.actions.push_back(std::move(a));
    }
  }

  // Of the condition of a durative method, what must hold at start, or the rest (over all and
  // at end), as one conjunction without the words of time
  ast::FormulaId part(ast::FormulaId id, bool at_start)
  {
    ast::Formula all;
    all.kind = ast::FormulaKind::And;
    all.loc = m_.pddl.nodes.formulas[id].loc;
    std::vector<ast::FormulaId> pending{id};
    while (!pending.empty()) {
      const ast::FormulaId current = pending.back();
      const ast::Formula f = m_.pddl.nodes.formulas[current];
      pending.pop_back();
      if (f.kind == ast::FormulaKind::And) {
        pending.insert(pending.end(), f.children.begin(), f.children.end());
      } else if (f.kind == ast::FormulaKind::AtStart) {
        if (at_start) {
          all.children.insert(all.children.end(), f.children.begin(), f.children.end());
        }
      } else if (f.kind == ast::FormulaKind::OverAll || f.kind == ast::FormulaKind::AtEnd) {
        if (!at_start) {
          all.children.insert(all.children.end(), f.children.begin(), f.children.end());
        }
      } else if (at_start) {
        all.children.push_back(current);  // without a word of time: at start
      }
    }
    return m_.pddl.nodes.formulas.push_back(std::move(all));
  }

  // The variables a formula mentions (bound ones included: a superset is enough)
  void variables_of(ast::FormulaId id, std::unordered_set<uint32_t> & out) const
  {
    const ast::Formula & f = m_.pddl.nodes.formulas[id];
    for (ast::FormulaId c : f.children) {
      variables_of(c, out);
    }
    for (ast::TermId t : f.args) {
      variables_of(t, out);
    }
    for (ast::ExprId e : {f.lhs, f.rhs}) {
      if (e.valid()) {
        variables_of(e, out);
      }
    }
  }

  void variables_of(ast::ExprId id, std::unordered_set<uint32_t> & out) const
  {
    const ast::Expr & e = m_.pddl.nodes.exprs[id];
    if (e.term.valid()) {
      variables_of(e.term, out);
    }
    for (ast::TermId t : e.args) {
      variables_of(t, out);
    }
    for (ast::ExprId c : e.children) {
      variables_of(c, out);
    }
  }

  void variables_of(ast::TermId id, std::unordered_set<uint32_t> & out) const
  {
    const ast::Term & t = m_.pddl.nodes.terms[id];
    if (t.kind == ast::TermKind::Variable) {
      out.insert(t.name.value());
    }
    for (ast::TermId a : t.args) {
      variables_of(a, out);
    }
  }

  sem::TypeSpec spec(const ast::TypedName & typed)
  {
    if (typed.types.empty()) {
      return {m_.domain.types.object()};
    }
    sem::TypeSpec result;
    for (kairos::Symbol t : typed.types) {
      if (auto id = m_.domain.types.find(name(t))) {
        result.push_back(*id);
      } else {
        error("hddl.unknown_type", "unknown type " + name(t), typed.type_loc);
        return {};  // unknown: accepted by every later check
      }
    }
    return result;
  }

  std::vector<sem::TypeSpec> specs(const std::vector<ast::TypedName> & params)
  {
    std::vector<sem::TypeSpec> result;
    for (const ast::TypedName & p : params) {
      result.push_back(spec(p));
    }
    return result;
  }

  void declare_tasks()
  {
    auto add = [&](TaskInfo info) {
        if (!task_index_.emplace(info.name, m_.tasks.size()).second) {
          const TaskInfo & first = m_.tasks[TaskId(static_cast<uint32_t>(task_index_[info.name]))];
          error(first.primitive != info.primitive ? "hddl.task_is_action" : "hddl.duplicate_task",
            first.primitive != info.primitive ?
            info.name + " is both an action and an abstract task" :
            "task " + info.name + " declared more than once", info.loc);
          return;
        }
        m_.tasks.push_back(std::move(info));
      };
    for (uint32_t i = 0; i < m_.user_actions; ++i) {
      const ast::Action & a = m_.pddl.actions[i];
      add(TaskInfo{name(a.name), specs(a.params), true, false, i, a.loc});
    }
    for (uint32_t i = 0; i < m_.pddl.durative_actions.size(); ++i) {
      const ast::DurativeAction & a = m_.pddl.durative_actions[i];
      add(TaskInfo{name(a.name), specs(a.params), true, true, i, a.loc});
    }
    for (const Task & t : task_decls_) {
      add(TaskInfo{name(t.name), specs(t.params), false, false, 0, t.loc});
    }
  }

  // The parameters of a method or of :htn
  std::vector<Param> declare_params(const std::vector<ast::TypedName> & params)
  {
    std::vector<Param> result;
    vars_.clear();
    for (const ast::TypedName & p : params) {
      if (!vars_.emplace(name(p.name), static_cast<uint32_t>(result.size())).second) {
        error("hddl.duplicate_parameter", "parameter " + name(p.name) + " repeated", p.loc);
        continue;
      }
      result.push_back(Param{name(p.name), spec(p)});
    }
    return result;
  }

  // A variable of the enclosing parameters or an object; nullopt after reporting an error
  std::optional<Arg> resolve(ast::TermId id, const std::vector<Param> & params)
  {
    const ast::Term & term = nodes_->terms[id];
    if (term.kind == ast::TermKind::Variable) {
      auto it = vars_.find(name(term.name));
      if (it == vars_.end()) {
        error("hddl.undeclared_variable", "variable " + name(term.name) +
          " is not a parameter here", term.loc);
        return std::nullopt;
      }
      (void)params;
      return Arg{true, it->second};
    }
    if (term.kind == ast::TermKind::Name) {
      if (auto object = objects_->find(name(term.name))) {
        return Arg{false, object->value()};
      }
      error("hddl.unknown_object", "unknown object " + name(term.name), term.loc);
      return std::nullopt;
    }
    error("hddl.term", "a function term is not allowed here", term.loc);
    return std::nullopt;
  }

  const sem::TypeSpec & types_of(const Arg & arg, const std::vector<Param> & params) const
  {
    return arg.variable ? params[arg.index].types :
           objects_->items[sem::ObjectId(arg.index)].types;
  }

  // The arguments of a use of `task`, checked against its parameters
  std::vector<Arg> resolve_args(
    const TaskInfo & task, const std::vector<ast::TermId> & terms,
    const std::vector<Param> & params, ast::Loc loc)
  {
    std::vector<Arg> args;
    if (terms.size() != task.params.size()) {
      error("hddl.arity", "task " + task.name + " takes " + std::to_string(task.params.size()) +
        " arguments, " + std::to_string(terms.size()) + " given", loc);
      return args;
    }
    for (size_t i = 0; i < terms.size(); ++i) {
      std::optional<Arg> arg = resolve(terms[i], params);
      if (!arg) {
        args.push_back(Arg{});
        continue;
      }
      const sem::TypeSpec & have = types_of(*arg, params);
      const sem::TypeSpec & want = task.params[i];
      const bool fits = have.empty() || want.empty() ||
        (arg->variable ? m_.domain.types.compatible(have, want) :
        m_.domain.types.fits(have, want));
      if (!fits) {
        const ast::Term & term = nodes_->terms[terms[i]];
        error("hddl.type_mismatch", name(term.name) + " is " +
          m_.domain.types.to_string(have) + ", but argument " + std::to_string(i + 1) + " of " +
          task.name + " is " + m_.domain.types.to_string(want), term.loc);
      }
      args.push_back(*arg);
    }
    return args;
  }

  Network analyze_network(
    const TaskNetwork & in, const std::vector<Param> & params,
    std::unordered_map<std::string, uint32_t> * ids_out = nullptr)
  {
    Network out;
    std::unordered_map<std::string, uint32_t> ids;
    for (const Subtask & s : in.subtasks) {
      Step step;
      step.loc = s.loc;
      auto it = task_index_.find(name(s.task));
      if (it == task_index_.end()) {
        error("hddl.unknown_task", "unknown task or action " + name(s.task), s.loc);
      } else {
        step.task = TaskId(static_cast<uint32_t>(it->second));
        step.args = resolve_args(m_.tasks[step.task], s.args, params, s.loc);
      }
      if (s.id.valid()) {
        step.id = name(s.id);
        if (!ids.emplace(step.id, static_cast<uint32_t>(out.steps.size())).second) {
          error("hddl.duplicate_subtask_id", "subtask id " + step.id + " repeated", s.loc);
        }
      }
      out.steps.push_back(std::move(step));
    }
    const uint32_t n = static_cast<uint32_t>(out.steps.size());
    if (in.ordered) {
      for (uint32_t i = 0; i + 1 < n; ++i) {
        out.order.emplace_back(i, i + 1);
      }
    }
    for (const Order & o : in.ordering) {
      auto a = ids.find(name(o.before));
      auto b = ids.find(name(o.after));
      if (a == ids.end() || b == ids.end()) {
        error("hddl.unknown_subtask_id", "unknown subtask id " +
          name(a == ids.end() ? o.before : o.after), o.loc);
        continue;
      }
      order(o, a->second, b->second, out);
    }
    if (ids_out != nullptr) {
      *ids_out = ids;
    }
    out.total_order = check_order(out, n, in.ordering.empty() ? ast::Loc{} : in.ordering[0].loc);

    for (const Constraint & c : in.constraints) {
      if (c.first.valid()) {
        StateConstraint state;
        state.kind = c.kind;
        state.first = c.first;
        state.second = c.second;
        state.from = c.from;
        state.to = c.to;
        if (const auto action = constraint_action_.find(c.first.value());
          action != constraint_action_.end() && in_method_)
        {
          state.action = static_cast<int32_t>(later_from_ + action->second.first);
          state.action_params = action->second.second;
        }
        bool known = true;
        for (auto [symbol, step] : {std::pair(c.a, &state.a), std::pair(c.b, &state.b)}) {
          if (symbol.valid()) {
            const auto found = ids.find(name(symbol));
            if (found == ids.end()) {
              error("hddl.unknown_subtask_id", "unknown subtask id " + name(symbol), c.loc);
              known = false;
            } else {
              *step = found->second;
            }
          }
        }
        if (c.kind == ConstraintKind::HoldDuring && c.to < c.from) {
          error("hddl.constraint", "hold-during: the second time is before the first", c.loc);
          known = false;
        }
        if (known) {
          out.states.push_back(state);
        }
        continue;
      }
      std::optional<Arg> lhs = resolve(c.lhs, params);
      if (c.kind == ConstraintKind::Equal || c.kind == ConstraintKind::Different) {
        std::optional<Arg> rhs = resolve(c.rhs, params);
        if (lhs && rhs) {
          out.constraints.push_back(VarConstraint{c.kind == ConstraintKind::Equal, *lhs, *rhs});
        }
        continue;
      }
      sem::TypeSpec types;
      for (kairos::Symbol t : c.types) {
        if (auto id = m_.domain.types.find(name(t))) {
          types.push_back(*id);
        } else {
          error("hddl.unknown_type", "unknown type " + name(t), c.loc);
        }
      }
      if (lhs && !lhs->variable) {
        error("hddl.constraint", "a sort constraint needs a variable", c.loc);
      } else if (lhs && !types.empty()) {
        out.sorts.push_back(SortConstraint{lhs->index, types, c.kind == ConstraintKind::NotSort});
      }
    }
    return out;
  }

  // Rejects cycles; true if the order leaves exactly one sequence
  bool check_order(const Network & net, uint32_t n, ast::Loc loc)
  {
    std::vector<std::vector<uint32_t>> next(n);
    std::vector<uint32_t> pending(n, 0);
    for (auto [a, b] : net.order) {
      next[a].push_back(b);
      ++pending[b];
    }
    std::vector<uint32_t> ready;
    for (uint32_t i = 0; i < n; ++i) {
      if (pending[i] == 0) {
        ready.push_back(i);
      }
    }
    bool total = true;
    uint32_t done = 0;
    while (!ready.empty()) {
      total = total && ready.size() == 1;
      uint32_t at = ready.back();
      ready.pop_back();
      ++done;
      for (uint32_t b : next[at]) {
        if (--pending[b] == 0) {
          ready.push_back(b);
        }
      }
    }
    if (done != n) {
      error("hddl.cyclic_ordering", "the ordering constraints form a cycle", loc);
      return false;
    }
    return total;
  }

  // An ordering constraint of HDDL 2.1 as a comparison of two time points with <, <= or =.
  // An id alone is the point on its side: (< a b) is "a ends, then b starts", the ordering of
  // HDDL 1.0, which does not ask for time in between; (> a b) is (< b a)
  void order(const Order & o, uint32_t before, uint32_t after, Network & out)
  {
    const bool plain = o.before_point == Point::Task && o.after_point == Point::Task &&
      !o.negated;
    if (o.op == ast::CompareOp::Eq && (o.negated || o.before_point == Point::Task ||
      o.after_point == Point::Task))
    {
      error("hddl.ordering", o.negated ?
        "'not' of an equality between time points is not supported" :
        "'=' compares time points: write (start id) or (end id)", o.loc);
      return;
    }
    // The points as written, with the smaller side of the comparison first
    const bool greater = o.op == ast::CompareOp::Gt || o.op == ast::CompareOp::Ge;
    TimeConstraint t;
    t.a = greater ? after : before;
    t.b = greater ? before : after;
    const Point a_point = greater ? o.after_point : o.before_point;
    const Point b_point = greater ? o.before_point : o.after_point;
    t.a_end = a_point != Point::Start;  // the smaller side: a task is its end
    t.b_end = b_point == Point::End;    // the larger side: its start
    t.op = o.op == ast::CompareOp::Eq ? ast::CompareOp::Eq :
      (o.op == ast::CompareOp::Lt || o.op == ast::CompareOp::Gt) ? ast::CompareOp::Lt :
      ast::CompareOp::Le;
    if (o.negated) {
      // not (x < y) is y <= x, and not (x <= y) is y < x
      std::swap(t.a, t.b);
      std::swap(t.a_end, t.b_end);
      t.op = t.op == ast::CompareOp::Lt ? ast::CompareOp::Le : ast::CompareOp::Lt;
    }
    if (t.a == t.b && t.a_end && !t.b_end) {
      error("hddl.ordering", "a subtask cannot end before it starts", o.loc);
      return;
    }
    const bool precedence = t.a_end && !t.b_end && t.op != ast::CompareOp::Eq && t.a != t.b;
    if (precedence) {
      out.order.emplace_back(t.a, t.b);
    }
    // Between tasks, the ordering is all there is to it
    if (!(precedence && (plain || t.op == ast::CompareOp::Le))) {
      out.times.push_back(t);
    }
  }

  // An expression of a duration constraint: numbers, ?duration, (duration id) and the numeric
  // functions of the domain
  void check_duration(
    ast::ExprId id, const std::unordered_map<std::string, uint32_t> & ids, ast::Loc loc,
    MethodInfo & out)
  {
    const ast::Expr & e = m_.pddl.nodes.exprs[id];
    switch (e.kind) {
      case ast::ExprKind::Number:
      case ast::ExprKind::Duration:
        return;
      case ast::ExprKind::Function: {
          if (name(e.name) == "duration" && e.args.size() == 1 &&
            m_.pddl.nodes.terms[e.args[0]].kind == ast::TermKind::Name)
          {
            const auto step = ids.find(name(m_.pddl.nodes.terms[e.args[0]].name));
            if (step == ids.end()) {
              error("hddl.unknown_subtask_id", "unknown subtask id " +
                name(m_.pddl.nodes.terms[e.args[0]].name), loc);
            } else {
              out.duration_of.emplace_back(id, step->second);
            }
            return;
          }
          const auto f = m_.domain.functions.find(name(e.name));
          if (!f || !m_.domain.functions.items[*f].numeric ||
            m_.domain.functions.items[*f].params.size() != e.args.size())
          {
            error("hddl.duration", name(e.name) + " is not a numeric function with these "
              "arguments", loc);
          }
          return;
        }
      case ast::ExprKind::Add:
      case ast::ExprKind::Sub:
      case ast::ExprKind::Mul:
      case ast::ExprKind::Div:
      case ast::ExprKind::Neg:
        for (ast::ExprId c : e.children) {
          check_duration(c, ids, loc, out);
        }
        return;
      default:
        error("hddl.duration", "a duration is compared with ?duration, (duration id) or a "
          "numeric expression", loc);
    }
  }

  void analyze_method(size_t index)
  {
    const Method & in = methods_[index];
    MethodInfo out;
    out.name = name(in.name);
    out.loc = in.loc;
    out.precondition_action = precondition_action_[index];
    out.precondition_params = precondition_params_[index];
    if (!method_names_.insert(out.name).second) {
      error("hddl.duplicate_method", "method " + out.name + " declared more than once", in.loc);
    }
    out.params = declare_params(in.params);
    auto it = task_index_.find(name(in.task));
    if (it == task_index_.end()) {
      error("hddl.unknown_task", "unknown task " + name(in.task), in.task_loc);
      return;
    }
    out.task = TaskId(static_cast<uint32_t>(it->second));
    if (m_.tasks[out.task].primitive) {
      error("hddl.method_for_action", name(in.task) + " is an action: only abstract tasks have "
        "methods", in.task_loc);
      return;
    }
    out.task_args = resolve_args(m_.tasks[out.task], in.task_args, out.params, in.task_loc);
    std::unordered_map<std::string, uint32_t> ids;
    in_method_ = true;
    out.network = analyze_network(in.network, out.params, &ids);
    in_method_ = false;
    if (in.durative) {
      out.condition = in.precondition;
      for (const MethodDuration & d : in.duration) {
        check_duration(d.lhs, ids, d.loc, out);
        check_duration(d.rhs, ids, d.loc, out);
        out.durations.push_back(DurationConstraint{d.op, d.lhs, d.rhs});
      }
    }
    m_.methods.push_back(std::move(out));
  }

  void analyze_initial()
  {
    if (!m_.has_htn) {
      if (!m_.problem.goal.valid()) {
        error("hddl.nothing_to_do", "the problem has neither :htn nor :goal", m_.problem.loc);
      }
      return;
    }
    m_.initial_params = declare_params(htn_params_);
    m_.initial = analyze_network(htn_, m_.initial_params);
    if (m_.initial.steps.empty() && !m_.problem.goal.valid()) {
      error("hddl.nothing_to_do", "the problem has no initial task and no :goal", htn_loc_);
    }
  }

  Model m_;
  std::vector<Method> methods_;
  std::vector<Task> task_decls_;
  TaskNetwork htn_;
  std::vector<ast::TypedName> htn_params_;
  ast::Loc htn_loc_;

  std::vector<Diagnostic> diagnostics_;
  bool errors_ = false;
  std::vector<int32_t> precondition_action_;
  std::vector<ast::Action> later_;  // what durative methods ask over all and at end
  size_t later_from_ = 0;           // where they start in pddl.actions
  // The action of the condition of a constraint on states, by formula: place in later_ and
  // the parameters of the method that are its own
  std::unordered_map<uint32_t, std::pair<size_t, std::vector<uint32_t>>> constraint_action_;
  bool in_method_ = false;          // the network being read is that of a method
  std::vector<std::vector<uint32_t>> precondition_params_;
  std::unordered_map<std::string, size_t> task_index_;
  std::unordered_set<std::string> method_names_;
  std::unordered_map<std::string, uint32_t> vars_;

  // What names are resolved against: the domain's, then the problem's
  const std::string * file_ = nullptr;
  const kairos::SymbolTable * symbols_ = nullptr;
  const ast::Nodes * nodes_ = nullptr;
  const Objects * objects_ = nullptr;
};

}  // namespace

kairos::Result<Model> analyze(Domain domain, Problem problem)
{
  return Analyzer(std::move(domain), std::move(problem)).run();
}

}  // namespace klimax::hddl

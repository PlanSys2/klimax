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

    // Kairos expects a goal: a problem without one gets the empty conjunction
    m_.has_goal = m_.problem.goal.valid();
    if (!m_.has_goal) {
      m_.problem.goal = m_.problem.nodes.formulas.push_back(ast::Formula{});
    }

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
      precondition_action_[i] = static_cast<int32_t>(m_.pddl.actions.size());
      m_.pddl.actions.push_back(std::move(a));
    }
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

  Network analyze_network(const TaskNetwork & in, const std::vector<Param> & params)
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
      out.order.emplace_back(a->second, b->second);
    }
    out.total_order = check_order(out, n, in.ordering.empty() ? ast::Loc{} : in.ordering[0].loc);

    for (const Constraint & c : in.constraints) {
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
    out.network = analyze_network(in.network, out.params);
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

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

#ifndef KLIMAX__VERIFY__INTERPRETER_HPP_
#define KLIMAX__VERIFY__INTERPRETER_HPP_

#include <algorithm>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "hddl/model.hpp"

// Internal: states as sets of atoms, and the formulas and effects of a file evaluated on them.
// Shared by the verifier and by the search that does not ground.

namespace klimax::verifier
{

namespace ast = kairos::ast;
namespace sem = kairos::sem;
using hddl::Arg;
using hddl::Model;
using hddl::TaskId;
using kairos::Symbol;

inline constexpr uint32_t kNone = std::numeric_limits<uint32_t>::max();

// Thrown with the reason the plan is rejected, or the feature that is not supported
struct Invalid
{
  std::string reason;
};

[[noreturn]] inline void invalid(std::string reason) {throw Invalid{std::move(reason)};}

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

// The values of the numeric fluents that have one: function, then objects
using Values = std::unordered_map<Atom, double, AtomHash>;

// A fluent takes a value; not a number if it could not be computed (it read a fluent with no
// value, or divided by zero)
struct Change
{
  Atom fluent;
  double value;
};

// Values of the variables in scope: innermost last
using Env = std::vector<std::pair<Symbol, uint32_t>>;

// Evaluates the formulas and effects of one file (the domain's or the problem's) on a state
class Interpreter
{
public:
  Interpreter(const Model & model, const ast::Nodes & nodes, const kairos::SymbolTable & symbols)
  : m_(model), nodes_(nodes), symbols_(symbols) {}

  /// The values that numeric conditions and effects read from then on (none: no numbers)
  void numbers(const Values * values) {values_ = values;}

  /// Whether (total-cost) is kept like any other fluent. Not by default: a cost that nothing
  /// reads does not change which plans are valid, and tells apart states that are the same
  void costs(bool kept) {costs_ = kept;}

  // The value of a numeric expression, if all it reads has one
  std::optional<double> number(ast::ExprId id, const Env & env)
  {
    const ast::Expr & e = nodes_.exprs[id];
    auto child = [&](size_t i) {return number(e.children.at(i), env);};
    switch (e.kind) {
      case ast::ExprKind::Number:
        return e.number;
      case ast::ExprKind::Function: {
          if (values_ == nullptr) {
            invalid("unsupported: numeric fluents");
          }
          const auto found = values_->find(fluent(e, env));
          return found == values_->end() ? std::nullopt : std::optional<double>(found->second);
        }
      case ast::ExprKind::Add:
      case ast::ExprKind::Mul: {
          double total = e.kind == ast::ExprKind::Add ? 0.0 : 1.0;
          for (size_t i = 0; i < e.children.size(); ++i) {
            const std::optional<double> v = child(i);
            if (!v) {
              return std::nullopt;
            }
            total = e.kind == ast::ExprKind::Add ? total + *v : total * *v;
          }
          return total;
        }
      case ast::ExprKind::Sub:
      case ast::ExprKind::Div: {
          const std::optional<double> a = child(0);
          const std::optional<double> b = child(1);
          if (!a || !b || (e.kind == ast::ExprKind::Div && *b == 0.0)) {
            return std::nullopt;
          }
          return e.kind == ast::ExprKind::Sub ? *a - *b : *a / *b;
        }
      case ast::ExprKind::Neg: {
          const std::optional<double> a = child(0);
          return a ? std::optional<double>(-*a) : std::nullopt;
        }
      default:
        invalid("unsupported: this numeric expression");
    }
  }

  // The fluent that a function expression names: function, then objects
  Atom fluent(const ast::Expr & e, const Env & env)
  {
    Atom key;
    key.reserve(e.args.size() + 1);
    key.push_back(function(e.name));
    for (ast::TermId t : e.args) {
      key.push_back(object(t, env));
    }
    return key;
  }

  uint32_t function(Symbol name)
  {
    uint32_t & cached = slot(functions_, name);
    if (cached == kNone) {
      auto found = m_.domain.functions.find(symbols_.name(name));
      if (!found) {
        invalid("internal: unknown function " + std::string(symbols_.name(name)));
      }
      if (!m_.domain.functions.items[*found].numeric) {
        invalid("unsupported: object fluents");
      }
      cached = found->value();
    }
    return cached;
  }

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
          if (l.kind == ast::ExprKind::Term && r.kind == ast::ExprKind::Term) {
            if (f.compare != ast::CompareOp::Eq) {
              invalid("unsupported: an order between objects");
            }
            return object(l.term, env) == object(r.term, env);
          }
          // A comparison that reads a fluent with no value does not hold
          const std::optional<double> a = number(f.lhs, env);
          const std::optional<double> b = number(f.rhs, env);
          if (!a || !b) {
            return false;
          }
          switch (f.compare) {
            case ast::CompareOp::Eq: return *a == *b;
            case ast::CompareOp::Lt: return *a < *b;
            case ast::CompareOp::Le: return *a <= *b;
            case ast::CompareOp::Gt: return *a > *b;
            default: return *a >= *b;
          }
        }
      default:
        invalid("unsupported: temporal conditions, preferences or trajectory constraints");
    }
  }

  // The atoms that `id` requires: those of its conjunctions, outside any other connective
  void required(ast::FormulaId id, std::vector<ast::FormulaId> & atoms) const
  {
    const ast::Formula & f = nodes_.formulas[id];
    if (f.kind == ast::FormulaKind::Atom) {
      atoms.push_back(id);
    } else if (f.kind == ast::FormulaKind::And) {
      for (ast::FormulaId c : f.children) {
        required(c, atoms);
      }
    }
  }

  const ast::Formula & formula(ast::FormulaId id) const {return nodes_.formulas[id];}
  const ast::Term & term(ast::TermId id) const {return nodes_.terms[id];}

  // Collects what `id` adds and deletes when applied in `state`, and the values that the
  // fluents take (all read before any is given), if `changes` is there to collect them
  void effect(
    ast::EffectId id, const State & state, Env & env, std::vector<Atom> & adds,
    std::vector<Atom> & deletes, std::vector<Change> * changes = nullptr)
  {
    const ast::Effect & e = nodes_.effects[id];
    switch (e.kind) {
      case ast::EffectKind::And:
        for (ast::EffectId c : e.children) {
          effect(c, state, env, adds, deletes, changes);
        }
        return;
      case ast::EffectKind::Forall:
        quantified(e.params, 0, env, true, [&]() {
            effect(e.children.at(0), state, env, adds, deletes, changes);
            return true;
          });
        return;
      case ast::EffectKind::When:
        if (holds(e.condition, state, env)) {
          effect(e.children.at(0), state, env, adds, deletes, changes);
        }
        return;
      case ast::EffectKind::Add:
        adds.push_back(atom(e.name, e.args, env));
        return;
      case ast::EffectKind::Delete:
        deletes.push_back(atom(e.name, e.args, env));
        return;
      case ast::EffectKind::Assign: {
          const ast::Expr & target = nodes_.exprs[e.target];
          if (!costs_ && std::string_view(symbols_.name(target.name)) == "total-cost") {
            return;
          }
          if (changes == nullptr || target.kind != ast::ExprKind::Function) {
            invalid("unsupported: numeric effects");
          }
          Change change{fluent(target, env), std::numeric_limits<double>::quiet_NaN()};
          const std::optional<double> by = number(e.value, env);
          const std::optional<double> old =
            e.op == ast::AssignOp::Assign ? std::optional<double>(0.0) : number(e.target, env);
          if (by && old) {
            switch (e.op) {
              case ast::AssignOp::Assign: change.value = *by; break;
              case ast::AssignOp::Increase: change.value = *old + *by; break;
              case ast::AssignOp::Decrease: change.value = *old - *by; break;
              case ast::AssignOp::ScaleUp: change.value = *old * *by; break;
              default:
                change.value = *by == 0.0 ? change.value : *old / *by;
                break;
            }
          }
          changes->push_back(std::move(change));
          return;
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
  std::vector<uint32_t> functions_;
  const Values * values_ = nullptr;
  bool costs_ = false;
};

}  // namespace klimax::verifier

#endif  // KLIMAX__VERIFY__INTERPRETER_HPP_

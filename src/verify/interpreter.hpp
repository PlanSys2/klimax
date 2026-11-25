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
#include <string>
#include <string_view>
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

}  // namespace klimax::verifier

#endif  // KLIMAX__VERIFY__INTERPRETER_HPP_

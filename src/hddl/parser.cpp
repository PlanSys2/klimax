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

#include "hddl/parser.hpp"

#include <algorithm>
#include <array>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "pddl/parser.hpp"

namespace klimax::hddl
{

namespace
{

using kairos::Diagnostic;
using kairos::pddl::Fragments;
using kairos::pddl::Token;
using kairos::pddl::TokenKind;

// Requirements of HDDL 1.0 and 2.1 that Kairos does not know
constexpr std::array<std::string_view, 8> kRequirements = {
  ":hierarchy", ":method-preconditions", ":htn-method-prec", ":method-constraints",
  ":durative-methods", ":numeric-fluent", ":htn", ":partial-order"};

std::string lower(std::string_view text) {return kairos::SymbolTable::to_lower(text);}

ast::Loc loc_of(const Token & t) {return ast::Loc{t.line, t.column};}

bool is_operator(const Token & t, std::string_view text)
{
  return t.kind == TokenKind::Operator && t.text == text;
}

bool is_word(const Token & t, std::string_view word)
{
  return t.kind == TokenKind::Name && lower(t.text) == word;
}

// "(" [and] item* ")" where a single item may come without "and"; "()" is empty.
// `item` is called after the item's "(" and head token have been consumed.
template<class F>
void list_of(Fragments & p, const char * what, F && item)
{
  Token open = p.expect(TokenKind::LParen, std::string("'(' starting ") + what);
  if (p.at_close()) {
    p.next();
    return;
  }
  Token head = p.next();
  if (is_word(head, "and")) {
    while (!p.at_close()) {
      Token inner = p.expect(TokenKind::LParen, std::string("'(' starting ") + what);
      item(inner, p.next());
    }
    p.next();
    return;
  }
  item(open, head);
}

bool comparison(const Token & t, ast::CompareOp & op)
{
  static const std::array<std::pair<std::string_view, ast::CompareOp>, 5> kOps = {{
    {"<", ast::CompareOp::Lt}, {"<=", ast::CompareOp::Le}, {"=", ast::CompareOp::Eq},
    {">=", ast::CompareOp::Ge}, {">", ast::CompareOp::Gt}}};
  for (const auto & [text, value] : kOps) {
    if (is_operator(t, text)) {
      op = value;
      return true;
    }
  }
  return false;
}

// After "(" head: "(task args...)" or "(id (task args...))"
Subtask parse_subtask(Fragments & p, const Token & open, const Token & head)
{
  if (head.kind != TokenKind::Name) {
    p.error("hddl.subtask", "expected a task name or a subtask id", head);
  }
  Subtask s;
  s.loc = loc_of(open);
  if (p.peek().kind == TokenKind::LParen) {
    s.id = p.intern(head.text);
    p.next();
    s.task = p.intern(p.expect(TokenKind::Name, "task name").text);
    while (!p.at_close()) {
      s.args.push_back(p.term());
    }
    p.next();
    p.expect(TokenKind::RParen, "')' closing the subtask");
    return s;
  }
  s.task = p.intern(head.text);
  while (!p.at_close()) {
    s.args.push_back(p.term());
  }
  p.next();
  return s;
}

// After "(" head: "(= t1 t2)", "(sortof ?v - type)" or "(not <one of those>)"
Constraint parse_constraint(Fragments & p, const Token & open, const Token & head)
{
  Constraint c;
  c.loc = loc_of(open);
  bool negated = false;
  Token op = head;
  if (is_word(head, "not")) {
    negated = true;
    p.expect(TokenKind::LParen, "'(' after 'not'");
    op = p.next();
  }
  if (is_operator(op, "=")) {
    c.kind = negated ? ConstraintKind::Different : ConstraintKind::Equal;
    c.lhs = p.term();
    c.rhs = p.term();
  } else if (is_word(op, "sortof") || is_word(op, "typeof")) {
    c.kind = negated ? ConstraintKind::NotSort : ConstraintKind::Sort;
    c.lhs = p.term();
    Token dash = p.next();
    if (!is_operator(dash, "-")) {
      p.error("hddl.constraint", "expected '- type' in a sort constraint", dash);
    }
    c.types.push_back(p.intern(p.expect(TokenKind::Name, "type name").text));
  } else {
    p.error("hddl.constraint", "unsupported constraint: expected '=', 'sortof' or 'not'", op);
  }
  p.expect(TokenKind::RParen, "')' closing the constraint");
  if (negated) {
    p.expect(TokenKind::RParen, "')' closing 'not'");
  }
  return c;
}

// The keys of a task network, shared by methods and :htn. True if `key` was one of them.
bool parse_network_key(Fragments & p, const Token & key, const std::string & k, TaskNetwork & n)
{
  if (k == ":subtasks" || k == ":tasks" || k == ":ordered-subtasks" || k == ":ordered-tasks") {
    n.ordered = k.rfind(":ordered", 0) == 0;
    list_of(p, "a subtask", [&](const Token & open, const Token & head) {
        n.subtasks.push_back(parse_subtask(p, open, head));
      });
    return true;
  }
  if (k == ":ordering" || k == ":order") {
    list_of(p, "an ordering constraint", [&](const Token & open, const Token & head) {
        Order o;
        o.loc = loc_of(open);
        Token op = head;
        if (is_word(head, "not")) {
          o.negated = true;
          p.expect(TokenKind::LParen, "'(' after 'not'");
          op = p.next();
        }
        if (!comparison(op, o.op)) {
          p.error("hddl.ordering", "expected '<', '<=', '=', '>=' or '>' in an ordering "
            "constraint", op);
        }
        // An id, (start id) or (end id)
        auto side = [&](Symbol & id, Point & point) {
            if (p.peek().kind != TokenKind::LParen) {
              id = p.intern(p.expect(TokenKind::Name, "subtask id").text);
              return;
            }
            p.next();
            const Token which = p.next();
            if (!is_word(which, "start") && !is_word(which, "end")) {
              p.error("hddl.ordering", "expected 'start' or 'end' before a subtask id", which);
            }
            point = is_word(which, "start") ? Point::Start : Point::End;
            id = p.intern(p.expect(TokenKind::Name, "subtask id").text);
            p.expect(TokenKind::RParen, "')' closing the time point");
          };
        side(o.before, o.before_point);
        side(o.after, o.after_point);
        p.expect(TokenKind::RParen, "')' closing the ordering constraint");
        if (o.negated) {
          p.expect(TokenKind::RParen, "')' closing 'not'");
        }
        n.ordering.push_back(o);
      });
    return true;
  }
  if (k == ":constraints") {
    list_of(p, "a constraint", [&](const Token & open, const Token & head) {
        n.constraints.push_back(parse_constraint(p, open, head));
      });
    return true;
  }
  if (k == ":causallinks" || k == ":causal-links") {
    p.error("hddl.causal_links", "causal links are not part of HDDL and are not supported", key);
  }
  return false;
}

// Reads the next key of a section, rejecting repetitions
Token next_key(Fragments & p, std::set<std::string> & seen, std::string & k, const char * what)
{
  Token key = p.expect(TokenKind::Keyword, std::string(what) + " keyword");
  k = lower(key.text);
  // The spellings of one key count as the same key
  std::string canonical = k;
  if (k == ":tasks" || k == ":ordered-subtasks" || k == ":ordered-tasks") {
    canonical = ":subtasks";
  } else if (k == ":order") {
    canonical = ":ordering";
  }
  if (!seen.insert(canonical).second) {
    p.error("parse.duplicate_key", k + " appears more than once", key);
  }
  return key;
}

Task parse_task(Fragments & p, const Token & keyword)
{
  Task t;
  t.loc = loc_of(keyword);
  t.name = p.intern(p.expect(TokenKind::Name, "task name").text);
  std::set<std::string> seen;
  std::string k;
  while (!p.at_close()) {
    Token key = next_key(p, seen, k, "task");
    if (k == ":parameters") {
      t.params = p.parameters();
    } else if (k == ":precondition" || k == ":effect") {
      // Some domains write empty ones; an abstract task has neither
      p.expect(TokenKind::LParen, "'()'");
      if (!p.at_close()) {
        p.error("hddl.task_with_" + k.substr(1), "an abstract task has no " + k.substr(1), key);
      }
      p.next();
    } else {
      p.error("parse.unknown_key", "unknown key " + k + " in task", key);
    }
  }
  p.next();
  return t;
}

// "(op x y)", "(at start (op x y))", "(at end (op x y))", "(and ...)" of those, or "()"
void parse_durations(Fragments & p, std::vector<MethodDuration> & out)
{
  list_of(p, "a duration constraint", [&](const Token & open, const Token & head) {
      MethodDuration d;
      d.loc = loc_of(open);
      Token op = head;
      const bool timed = is_word(head, "at");
      if (timed) {
        // When the expressions are evaluated: Klimax reads them at the start (semantics.md)
        const Token which = p.next();
        if (!is_word(which, "start") && !is_word(which, "end")) {
          p.error("hddl.duration", "expected 'start' or 'end' after 'at'", which);
        }
        p.expect(TokenKind::LParen, "'(' starting the duration constraint");
        op = p.next();
      }
      if (!comparison(op, d.op)) {
        p.error("hddl.duration", "expected '<', '<=', '=', '>=' or '>' in a duration "
          "constraint", op);
      }
      d.lhs = p.expression();
      d.rhs = p.expression();
      p.expect(TokenKind::RParen, "')' closing the duration constraint");
      if (timed) {
        p.expect(TokenKind::RParen, "')' closing 'at'");
      }
      out.push_back(d);
    });
}

Method parse_method(Fragments & p, const Token & keyword, bool durative)
{
  Method m;
  m.durative = durative;
  m.loc = loc_of(keyword);
  m.name = p.intern(p.expect(TokenKind::Name, "method name").text);
  std::set<std::string> seen;
  std::string k;
  while (!p.at_close()) {
    Token key = next_key(p, seen, k, "method");
    if (k == ":parameters") {
      m.params = p.parameters();
    } else if (k == ":task") {
      Token open = p.expect(TokenKind::LParen, "'(' starting the task");
      m.task_loc = loc_of(open);
      m.task = p.intern(p.expect(TokenKind::Name, "task name").text);
      while (!p.at_close()) {
        m.task_args.push_back(p.term());
      }
      p.next();
    } else if (k == ":precondition" && !durative) {
      m.precondition = p.condition(false, false);
    } else if ((k == ":condition" || k == ":precondition") && durative) {
      m.precondition = p.condition(true, false);
    } else if (k == ":duration" && durative) {
      parse_durations(p, m.duration);
    } else if (!parse_network_key(p, key, k, m.network)) {
      p.error("parse.unknown_key", "unknown key " + k + " in method", key);
    }
  }
  if (!m.task.valid()) {
    p.error("hddl.method_without_task", "method without :task", keyword);
  }
  p.next();
  return m;
}

// Kairos warns about requirements it does not know: those of HDDL are known here
std::vector<Diagnostic> without_hddl_requirements(std::vector<Diagnostic> diagnostics)
{
  std::erase_if(diagnostics, [](const Diagnostic & d) {
      if (d.code != "parse.unknown_requirement") {
        return false;
      }
      return std::any_of(kRequirements.begin(), kRequirements.end(), [&](std::string_view r) {
          const std::string prefix = "unknown requirement " + std::string(r);
          return d.message == prefix || d.message.rfind(prefix + " ", 0) == 0 ||
                 d.message.rfind(prefix + ";", 0) == 0 || d.message.rfind(prefix + ",", 0) == 0;
        });
    });
  return diagnostics;
}

}  // namespace

kairos::Result<Domain> parse_domain(std::string_view text, std::string_view filename)
{
  Domain domain;
  kairos::pddl::ParseOptions options;
  options.extension = [&](const Token & keyword, Fragments & p) {
      const std::string k = lower(keyword.text);
      if (k == ":task") {
        domain.tasks.push_back(parse_task(p, keyword));
        return true;
      }
      if (k == ":method" || k == ":durative-method") {
        domain.methods.push_back(parse_method(p, keyword, k == ":durative-method"));
        return true;
      }
      return false;
    };
  auto parsed = kairos::pddl::parse_domain(text, filename, options);
  auto diagnostics = without_hddl_requirements(parsed.diagnostics());
  if (!parsed.ok()) {
    return kairos::Result<Domain>::failure(std::move(diagnostics));
  }
  domain.pddl = std::move(parsed).value();
  return kairos::Result<Domain>::success(std::move(domain), std::move(diagnostics));
}

kairos::Result<Problem> parse_problem(std::string_view text, std::string_view filename)
{
  Problem problem;
  kairos::pddl::ParseOptions options;
  options.extension = [&](const Token & keyword, Fragments & p) {
      if (lower(keyword.text) != ":htn") {
        return false;
      }
      if (problem.has_htn) {
        p.error("parse.duplicate_section", "section :htn repeated", keyword);
      }
      problem.has_htn = true;
      problem.htn_loc = loc_of(keyword);
      std::set<std::string> seen;
      std::string k;
      while (!p.at_close()) {
        Token key = next_key(p, seen, k, ":htn");
        if (k == ":parameters") {
          problem.htn_params = p.parameters();
        } else if (!parse_network_key(p, key, k, problem.htn)) {
          p.error("parse.unknown_key", "unknown key " + k + " in :htn", key);
        }
      }
      p.next();
      return true;
    };
  auto parsed = kairos::pddl::parse_problem(text, filename, options);
  auto diagnostics = without_hddl_requirements(parsed.diagnostics());
  if (!parsed.ok()) {
    return kairos::Result<Problem>::failure(std::move(diagnostics));
  }
  problem.pddl = std::move(parsed).value();
  return kairos::Result<Problem>::success(std::move(problem), std::move(diagnostics));
}

}  // namespace klimax::hddl

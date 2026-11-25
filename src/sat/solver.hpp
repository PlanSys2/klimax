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


#ifndef KLIMAX__SAT__SOLVER_HPP_
#define KLIMAX__SAT__SOLVER_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "core/budget.hpp"

namespace klimax::sat
{

/// @brief A literal: variable `v` is 2v, its negation 2v + 1.
using Lit = uint32_t;

inline Lit lit(uint32_t variable, bool negated = false) {return 2 * variable + (negated ? 1 : 0);}
inline Lit neg(Lit l) {return l ^ 1u;}
inline uint32_t var(Lit l) {return l >> 1;}

enum class Answer : uint8_t {Sat, Unsat, Unknown};

/**
 * @brief A SAT solver that learns clauses from conflicts (CDCL), incremental.
 *
 * Clauses are added at any time between calls to solve(); each call may assume literals, which
 * hold for that call only. Two watched literals, first-UIP learning, activities with decay for
 * variables and learnt clauses, saved phases, Luby restarts.
 */
class Solver
{
public:
  uint32_t new_var();
  size_t variables() const {return assigns_.size();}
  size_t clauses() const {return clauses_;}

  /// @brief Adds a clause. False once the clauses are contradictory by themselves.
  bool add(std::vector<Lit> clause);
  bool add(Lit a) {return add(std::vector<Lit>{a});}
  bool add(Lit a, Lit b) {return add(std::vector<Lit>{a, b});}
  bool add(Lit a, Lit b, Lit c) {return add(std::vector<Lit>{a, b, c});}

  /// @brief Unknown when the budget runs out: the next call with the same assumptions, if no
  /// clause was added in between, goes on from there. After Sat, value() gives the model.
  Answer solve(const std::vector<Lit> & assumptions = {}, kairos::Budget * budget = nullptr);

  bool value(uint32_t variable) const {return model_[variable];}
  uint64_t conflicts() const {return conflicts_;}

private:
  using Ref = uint32_t;
  static constexpr Ref kNoRef = 0xffffffffu;
  static constexpr int8_t kUndef = 2;

  struct Watch
  {
    Ref clause;
    Lit blocker;
  };

  // In arena_: size, flags (1 learnt, 2 deleted), activity (float bits), then the literals
  uint32_t size(Ref c) const {return arena_[c];}
  Lit * lits(Ref c) {return &arena_[c + 3];}
  const Lit * lits(Ref c) const {return &arena_[c + 3];}
  float & activity(Ref c);

  int8_t value_of(Lit l) const
  {
    const int8_t v = assigns_[var(l)];
    return v == kUndef ? kUndef : static_cast<int8_t>(v ^ static_cast<int8_t>(l & 1u));
  }
  uint32_t level() const {return static_cast<uint32_t>(limits_.size());}

  Ref store(const std::vector<Lit> & clause, bool learnt);
  void attach(Ref c);
  void enqueue(Lit l, Ref reason);
  Ref propagate();
  void analyze(Ref conflict, std::vector<Lit> & learnt, uint32_t & back);
  void cancel(uint32_t to);
  void bump(uint32_t variable);
  void bump_clause(Ref c);
  void reduce();
  uint32_t pick();
  void heap_insert(uint32_t variable);
  void heap_up(size_t i);
  void heap_down(size_t i);

  bool ok_ = true;
  size_t clauses_ = 0;
  uint64_t conflicts_ = 0;
  uint64_t decisions_ = 0;
  bool paused_ = false;                      // solve() was stopped and can go on
  std::vector<Lit> assumed_;                 // in that call
  uint64_t restart_ = 0;
  uint64_t left_ = 0;                        // conflicts before the next restart
  std::vector<uint32_t> arena_;
  std::vector<Ref> learnts_;
  std::vector<std::vector<Watch>> watches_;  // by literal: the clauses that watch its negation
  std::vector<int8_t> assigns_;              // 1 true, 0 false, kUndef
  std::vector<int8_t> phase_;
  std::vector<uint32_t> levels_;
  std::vector<Ref> reasons_;
  std::vector<Lit> trail_;
  std::vector<uint32_t> limits_;             // where each decision level starts in trail_
  size_t head_ = 0;
  std::vector<double> scores_;
  double increment_ = 1.0;
  float clause_increment_ = 1.0f;
  std::vector<uint32_t> heap_;
  std::vector<uint32_t> heap_at_;            // position in heap_, or kNoRef
  std::vector<uint8_t> seen_;
  std::vector<bool> model_;
  double learnt_limit_ = 0.0;
};

}  // namespace klimax::sat

#endif  // KLIMAX__SAT__SOLVER_HPP_

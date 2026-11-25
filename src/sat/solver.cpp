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


#include "sat/solver.hpp"

#include <algorithm>
#include <cstring>

namespace klimax::sat
{

namespace
{

// The Luby sequence: 1 1 2 1 1 2 4 1 1 2 1 1 2 4 8...
uint64_t luby(uint64_t i)
{
  uint64_t size = 1;
  uint64_t power = 1;
  while (size < i + 1) {
    size = 2 * size + 1;
    power *= 2;
  }
  while (size - 1 != i) {
    size = (size - 1) / 2;
    power /= 2;
    i %= size;
  }
  return power;
}

}  // namespace

float & Solver::activity(Ref c)
{
  static_assert(sizeof(float) == sizeof(uint32_t));
  return *reinterpret_cast<float *>(&arena_[c + 2]);
}

uint32_t Solver::new_var()
{
  const uint32_t v = static_cast<uint32_t>(assigns_.size());
  assigns_.push_back(kUndef);
  phase_.push_back(0);
  levels_.push_back(0);
  reasons_.push_back(kNoRef);
  scores_.push_back(0.0);
  seen_.push_back(0);
  heap_at_.push_back(kNoRef);
  watches_.emplace_back();
  watches_.emplace_back();
  heap_insert(v);
  return v;
}

Solver::Ref Solver::store(const std::vector<Lit> & clause, bool learnt)
{
  const Ref c = static_cast<Ref>(arena_.size());
  arena_.push_back(static_cast<uint32_t>(clause.size()));
  arena_.push_back(learnt ? 1u : 0u);
  arena_.push_back(0u);
  arena_.insert(arena_.end(), clause.begin(), clause.end());
  return c;
}

void Solver::attach(Ref c)
{
  const Lit * l = lits(c);
  watches_[neg(l[0])].push_back(Watch{c, l[1]});
  watches_[neg(l[1])].push_back(Watch{c, l[0]});
}

bool Solver::add(std::vector<Lit> clause)
{
  if (!ok_) {
    return false;
  }
  paused_ = false;
  cancel(0);
  std::sort(clause.begin(), clause.end());
  size_t kept = 0;
  for (size_t i = 0; i < clause.size(); ++i) {
    const Lit l = clause[i];
    if (value_of(l) == 1 || (i + 1 < clause.size() && clause[i + 1] == neg(l) && (l & 1u) == 0)) {
      return true;  // true already, or it has a literal and its negation
    }
    if (value_of(l) == 0 || (kept > 0 && clause[kept - 1] == l)) {
      continue;
    }
    clause[kept++] = l;
  }
  clause.resize(kept);
  if (clause.empty()) {
    ok_ = false;
    return false;
  }
  if (clause.size() == 1) {
    enqueue(clause[0], kNoRef);
    ok_ = propagate() == kNoRef;
    return ok_;
  }
  attach(store(clause, false));
  ++clauses_;
  return true;
}

void Solver::enqueue(Lit l, Ref reason)
{
  const uint32_t v = var(l);
  assigns_[v] = static_cast<int8_t>((l & 1u) ^ 1u);
  levels_[v] = level();
  reasons_[v] = reason;
  trail_.push_back(l);
}

Solver::Ref Solver::propagate()
{
  while (head_ < trail_.size()) {
    const Lit p = trail_[head_++];  // true now: the clauses that watch its negation
    std::vector<Watch> & ws = watches_[p];
    size_t kept = 0;
    for (size_t i = 0; i < ws.size(); ++i) {
      const Watch w = ws[i];
      if (value_of(w.blocker) == 1) {
        ws[kept++] = w;
        continue;
      }
      const Ref c = w.clause;
      if ((arena_[c + 1] & 2u) != 0) {
        continue;  // deleted: the watch goes
      }
      Lit * l = lits(c);
      const Lit falsified = neg(p);
      if (l[0] == falsified) {
        std::swap(l[0], l[1]);
      }
      if (value_of(l[0]) == 1) {
        ws[kept++] = Watch{c, l[0]};
        continue;
      }
      bool moved = false;
      const uint32_t n = size(c);
      for (uint32_t k = 2; k < n; ++k) {
        if (value_of(l[k]) != 0) {
          std::swap(l[1], l[k]);
          watches_[neg(l[1])].push_back(Watch{c, l[0]});
          moved = true;
          break;
        }
      }
      if (moved) {
        continue;
      }
      ws[kept++] = Watch{c, l[0]};
      if (value_of(l[0]) == 0) {
        for (++i; i < ws.size(); ++i) {
          ws[kept++] = ws[i];
        }
        ws.resize(kept);
        head_ = trail_.size();
        return c;
      }
      enqueue(l[0], c);
    }
    ws.resize(kept);
  }
  return kNoRef;
}

void Solver::bump(uint32_t variable)
{
  scores_[variable] += increment_;
  if (scores_[variable] > 1e100) {
    for (double & s : scores_) {
      s *= 1e-100;
    }
    increment_ *= 1e-100;
  }
  if (heap_at_[variable] != kNoRef) {
    heap_up(heap_at_[variable]);
  }
}

void Solver::bump_clause(Ref c)
{
  float & a = activity(c);
  a += clause_increment_;
  if (a > 1e20f) {
    for (Ref l : learnts_) {
      activity(l) *= 1e-20f;
    }
    clause_increment_ *= 1e-20f;
  }
}

// The first unique implication point: a clause with one literal of the current level
void Solver::analyze(Ref conflict, std::vector<Lit> & learnt, uint32_t & back)
{
  learnt.assign(1, 0);
  uint32_t pending = 0;
  size_t index = trail_.size();
  Lit p = 0;
  bool first = true;
  Ref c = conflict;
  do {
    if ((arena_[c + 1] & 1u) != 0) {
      bump_clause(c);
    }
    const Lit * l = lits(c);
    for (uint32_t k = first ? 0 : 1; k < size(c); ++k) {
      const uint32_t v = var(l[k]);
      if (seen_[v] == 0 && levels_[v] > 0) {
        seen_[v] = 1;
        bump(v);
        if (levels_[v] >= level()) {
          ++pending;
        } else {
          learnt.push_back(l[k]);
        }
      }
    }
    first = false;
    while (seen_[var(trail_[--index])] == 0) {
    }
    p = trail_[index];
    c = reasons_[var(p)];
    seen_[var(p)] = 0;
    --pending;
    // The reason has its implied literal first
  } while (pending > 0);
  learnt[0] = neg(p);
  // A literal implied by others of the clause is redundant
  for (size_t i = 1; i < learnt.size(); ++i) {
    seen_[var(learnt[i])] = 1;
  }
  size_t kept = 1;
  const std::vector<Lit> all(learnt.begin() + 1, learnt.end());
  for (size_t i = 1; i < learnt.size(); ++i) {
    const Ref reason = reasons_[var(learnt[i])];
    bool redundant = reason != kNoRef;
    if (redundant) {
      const Lit * l = lits(reason);
      for (uint32_t k = 1; k < size(reason) && redundant; ++k) {
        redundant = seen_[var(l[k])] != 0 || levels_[var(l[k])] == 0;
      }
    }
    if (!redundant) {
      learnt[kept++] = learnt[i];
    }
  }
  learnt.resize(kept);
  for (Lit l : all) {
    seen_[var(l)] = 0;
  }
  back = 0;
  if (learnt.size() > 1) {
    size_t best = 1;
    for (size_t i = 2; i < learnt.size(); ++i) {
      if (levels_[var(learnt[i])] > levels_[var(learnt[best])]) {
        best = i;
      }
    }
    std::swap(learnt[1], learnt[best]);
    back = levels_[var(learnt[1])];
  }
}

void Solver::cancel(uint32_t to)
{
  if (level() <= to) {
    return;
  }
  for (size_t i = trail_.size(); i-- > limits_[to]; ) {
    const uint32_t v = var(trail_[i]);
    phase_[v] = assigns_[v];
    assigns_[v] = kUndef;
    if (heap_at_[v] == kNoRef) {
      heap_insert(v);
    }
  }
  trail_.resize(limits_[to]);
  head_ = trail_.size();
  limits_.resize(to);
}

void Solver::heap_insert(uint32_t variable)
{
  heap_at_[variable] = static_cast<uint32_t>(heap_.size());
  heap_.push_back(variable);
  heap_up(heap_.size() - 1);
}

void Solver::heap_up(size_t i)
{
  const uint32_t v = heap_[i];
  while (i > 0) {
    const size_t parent = (i - 1) / 2;
    if (scores_[heap_[parent]] >= scores_[v]) {
      break;
    }
    heap_[i] = heap_[parent];
    heap_at_[heap_[i]] = static_cast<uint32_t>(i);
    i = parent;
  }
  heap_[i] = v;
  heap_at_[v] = static_cast<uint32_t>(i);
}

void Solver::heap_down(size_t i)
{
  const uint32_t v = heap_[i];
  while (true) {
    size_t child = 2 * i + 1;
    if (child >= heap_.size()) {
      break;
    }
    if (child + 1 < heap_.size() && scores_[heap_[child + 1]] > scores_[heap_[child]]) {
      ++child;
    }
    if (scores_[heap_[child]] <= scores_[v]) {
      break;
    }
    heap_[i] = heap_[child];
    heap_at_[heap_[i]] = static_cast<uint32_t>(i);
    i = child;
  }
  heap_[i] = v;
  heap_at_[v] = static_cast<uint32_t>(i);
}

// The unassigned variable with the highest score; kNoRef if all have a value
uint32_t Solver::pick()
{
  while (!heap_.empty()) {
    const uint32_t v = heap_[0];
    heap_at_[v] = kNoRef;
    heap_[0] = heap_.back();
    heap_.pop_back();
    if (!heap_.empty()) {
      heap_at_[heap_[0]] = 0;
      heap_down(0);
    }
    if (assigns_[v] == kUndef) {
      return v;
    }
  }
  return kNoRef;
}

// Forgets the half of the learnt clauses that took part in fewer conflicts
void Solver::reduce()
{
  std::sort(learnts_.begin(), learnts_.end(),
    [&](Ref a, Ref b) {return activity(a) < activity(b);});
  size_t kept = 0;
  for (size_t i = 0; i < learnts_.size(); ++i) {
    const Ref c = learnts_[i];
    const Lit first = lits(c)[0];
    const bool locked = value_of(first) == 1 && reasons_[var(first)] == c;
    if (i < learnts_.size() / 2 && size(c) > 2 && !locked) {
      arena_[c + 1] |= 2u;
    } else {
      learnts_[kept++] = c;
    }
  }
  learnts_.resize(kept);
}

Answer Solver::solve(const std::vector<Lit> & assumptions, kairos::Budget * budget)
{
  if (!ok_) {
    return Answer::Unsat;
  }
  // A call that the budget stopped goes on where it was, if nothing changed in between
  const bool resumed = paused_ && assumptions == assumed_;
  paused_ = false;
  if (!resumed) {
    cancel(0);
    if (propagate() != kNoRef) {
      ok_ = false;
      return Answer::Unsat;
    }
    assumed_ = assumptions;
    restart_ = 0;
    left_ = 100;
  }
  if (learnt_limit_ < 1.0) {
    learnt_limit_ = std::max(4000.0, static_cast<double>(clauses_) / 3.0);
  }
  std::vector<Lit> learnt;
  uint64_t & left = left_;
  for (; ; left = 100 * luby(++restart_)) {
    while (true) {
      const Ref conflict = propagate();
      if (conflict != kNoRef) {
        ++conflicts_;
        if (level() == 0) {
          ok_ = false;
          return Answer::Unsat;
        }
        uint32_t back = 0;
        analyze(conflict, learnt, back);
        cancel(back);
        if (learnt.size() == 1) {
          enqueue(learnt[0], kNoRef);
        } else {
          const Ref c = store(learnt, true);
          learnts_.push_back(c);
          attach(c);
          bump_clause(c);
          enqueue(learnt[0], c);
        }
        increment_ /= 0.95;
        clause_increment_ /= 0.999f;
        if (left > 0) {
          --left;
        }
        if ((conflicts_ & 255u) == 0 && budget != nullptr && budget->exhausted()) {
          paused_ = true;
          return Answer::Unknown;
        }
        continue;
      }
      if (left == 0) {
        cancel(0);
        break;  // restart
      }
      if (static_cast<double>(learnts_.size()) > learnt_limit_ + static_cast<double>(trail_.size())) {
        reduce();
        learnt_limit_ *= 1.1;
      }
      Lit next = kNoRef;
      while (level() < assumptions.size()) {
        const Lit a = assumptions[level()];
        if (value_of(a) == 1) {
          limits_.push_back(static_cast<uint32_t>(trail_.size()));  // nothing to decide
        } else if (value_of(a) == 0) {
          cancel(0);
          return Answer::Unsat;  // under the assumptions
        } else {
          next = a;
          break;
        }
      }
      if (next == kNoRef) {
        const uint32_t v = pick();
        if (v == kNoRef) {
          model_.assign(assigns_.size(), false);
          for (size_t i = 0; i < assigns_.size(); ++i) {
            model_[i] = assigns_[i] == 1;
          }
          cancel(0);
          return Answer::Sat;
        }
        next = lit(v, phase_[v] != 1);
      }
      limits_.push_back(static_cast<uint32_t>(trail_.size()));
      enqueue(next, kNoRef);
      if ((++decisions_ & 1023u) == 0 && budget != nullptr && budget->exhausted()) {
        paused_ = true;
        return Answer::Unknown;
      }
    }
    if (budget != nullptr && budget->exhausted()) {
      paused_ = true;
      left = 0;
      return Answer::Unknown;
    }
  }
}

}  // namespace klimax::sat

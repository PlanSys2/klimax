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


#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <optional>
#include <random>
#include <vector>

#include "sat/solver.hpp"

namespace klimax::sat
{
namespace
{

// Pigeons in holes: pigeon p is in hole h
void pigeons(Solver & s, uint32_t pigeons, uint32_t holes)
{
  std::vector<std::vector<uint32_t>> in(pigeons, std::vector<uint32_t>(holes));
  for (auto & row : in) {
    std::vector<Lit> somewhere;
    for (uint32_t & v : row) {
      v = s.new_var();
      somewhere.push_back(lit(v));
    }
    s.add(somewhere);
  }
  for (uint32_t h = 0; h < holes; ++h) {
    for (uint32_t a = 0; a < pigeons; ++a) {
      for (uint32_t b = a + 1; b < pigeons; ++b) {
        s.add(lit(in[a][h], true), lit(in[b][h], true));
      }
    }
  }
}

TEST(SatSolverTest, Trivial)
{
  Solver s;
  EXPECT_EQ(s.solve(), Answer::Sat);
  const uint32_t a = s.new_var();
  const uint32_t b = s.new_var();
  s.add(lit(a), lit(b));
  s.add(lit(a, true));
  ASSERT_EQ(s.solve(), Answer::Sat);
  EXPECT_FALSE(s.value(a));
  EXPECT_TRUE(s.value(b));
  EXPECT_FALSE(s.add(lit(b, true)));
  EXPECT_EQ(s.solve(), Answer::Unsat);
}

TEST(SatSolverTest, Pigeons)
{
  Solver fits;
  pigeons(fits, 6, 6);
  EXPECT_EQ(fits.solve(), Answer::Sat);
  Solver crowded;
  pigeons(crowded, 7, 6);
  EXPECT_EQ(crowded.solve(), Answer::Unsat);
  EXPECT_GT(crowded.conflicts(), 0u);
}

TEST(SatSolverTest, AssumptionsHoldForOneCall)
{
  Solver s;
  const uint32_t a = s.new_var();
  const uint32_t b = s.new_var();
  const uint32_t c = s.new_var();
  s.add(lit(a, true), lit(b));
  s.add(lit(b, true), lit(c));
  EXPECT_EQ(s.solve({lit(a), lit(c, true)}), Answer::Unsat);
  ASSERT_EQ(s.solve({lit(a)}), Answer::Sat);
  EXPECT_TRUE(s.value(c));
  ASSERT_EQ(s.solve({lit(c, true)}), Answer::Sat);
  EXPECT_FALSE(s.value(a));
  // More clauses after a call
  s.add(lit(a));
  EXPECT_EQ(s.solve({lit(c, true)}), Answer::Unsat);
  EXPECT_EQ(s.solve(), Answer::Sat);
}

TEST(SatSolverTest, GoesOnAfterTheBudgetRanOut)
{
  Solver whole;
  pigeons(whole, 8, 7);
  ASSERT_EQ(whole.solve(), Answer::Unsat);
  // The same in pieces: each stops after a few looks at the clock
  Solver s;
  pigeons(s, 8, 7);
  using Clock = kairos::BudgetConfig::Clock;
  Clock::time_point now{};
  Answer answer = Answer::Unknown;
  int pieces = 0;
  for (; answer == Answer::Unknown && pieces < 100000; ++pieces) {
    kairos::BudgetConfig config;
    config.clock = [&]() {return now += std::chrono::milliseconds(1);};
    config.deadline = now + std::chrono::milliseconds(3);
    config.check_interval = 1;
    config.check_period = std::chrono::microseconds(0);
    config.memory_probe = []() {return std::optional<size_t>();};
    kairos::Budget budget(config);
    answer = s.solve({}, &budget);
  }
  EXPECT_EQ(answer, Answer::Unsat);
  EXPECT_GT(pieces, 2);
  EXPECT_EQ(s.conflicts(), whole.conflicts());
}

// Against trying every assignment, on small random formulas around the hard ratio
TEST(SatSolverTest, AgreesWithEnumeration)
{
  std::mt19937 random(7);
  for (int round = 0; round < 300; ++round) {
    const uint32_t variables = 4 + static_cast<uint32_t>(round % 9);
    const size_t clauses = static_cast<size_t>(4.3 * variables);
    std::vector<std::vector<Lit>> formula;
    for (size_t i = 0; i < clauses; ++i) {
      std::vector<Lit> clause;
      for (int k = 0; k < 3; ++k) {
        clause.push_back(lit(static_cast<uint32_t>(random() % variables), random() % 2 == 0));
      }
      formula.push_back(clause);
    }
    auto holds = [&](uint32_t bits) {
        for (const auto & clause : formula) {
          bool some = false;
          for (Lit l : clause) {
            some = some || (((bits >> var(l)) & 1u) != 0) == ((l & 1u) == 0);
          }
          if (!some) {
            return false;
          }
        }
        return true;
      };
    bool satisfiable = false;
    for (uint32_t bits = 0; bits < (1u << variables) && !satisfiable; ++bits) {
      satisfiable = holds(bits);
    }
    Solver s;
    for (uint32_t v = 0; v < variables; ++v) {
      s.new_var();
    }
    for (const auto & clause : formula) {
      s.add(clause);
    }
    ASSERT_EQ(s.solve(), satisfiable ? Answer::Sat : Answer::Unsat) << round;
    if (satisfiable) {
      uint32_t bits = 0;
      for (uint32_t v = 0; v < variables; ++v) {
        bits |= s.value(v) ? 1u << v : 0u;
      }
      EXPECT_TRUE(holds(bits)) << round;
    }
  }
}

}  // namespace
}  // namespace klimax::sat

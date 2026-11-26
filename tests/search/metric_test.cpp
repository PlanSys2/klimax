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

#include <algorithm>
#include <random>
#include <string>
#include <vector>

#include "klimax/check.hpp"
#include "klimax/solve.hpp"

namespace klimax
{
namespace
{

// Places joined by roads of some length. From a to d: straight (10), through e (2 + 3) or
// through b and c (1 + 1 + 1)
const char * kDomain =
  "(define (domain d) (:requirements :typing :hierarchy :action-costs)\n"
  " (:types place)\n"
  " (:predicates (at ?l - place) (road ?a ?b - place) (paid))\n"
  " (:functions (total-cost) (len ?a ?b - place) (tolls))\n"
  " (:task go :parameters (?to - place))\n"
  " (:method go-here :parameters (?to - place) :task (go ?to) :precondition (at ?to))\n"
  " (:method go-on :parameters (?from ?mid ?to - place) :task (go ?to)\n"
  "  :precondition (and (at ?from) (road ?from ?mid))\n"
  "  :ordered-subtasks (and (drive ?from ?mid) (go ?to)))\n"
  " (:action drive :parameters (?a ?b - place)\n"
  "  :precondition (and (at ?a) (road ?a ?b))\n"
  "  :effect (and (not (at ?a)) (at ?b) (increase (total-cost) (len ?a ?b))))\n"
  " (:action pay :parameters ()\n"
  "  :effect (and (paid) (increase (total-cost) 1) (increase (tolls) 1))))";

std::string problem(
  const std::string & metric, const std::string & tasks = ":ordered-subtasks (and (go d))",
  const std::string & ab = "1", const std::string & cost = "(= (total-cost) 0)")
{
  return "(define (problem p) (:domain d) (:objects a b c d e - place)\n"
         "(:htn " + tasks + ")\n"
         "(:init (at a) (road a d) (road a b) (road b c) (road c d) (road a e) (road e d)\n" +
         cost + " (= (tolls) 0) (= (len a d) 10) (= (len a b) " + ab + ") (= (len b c) 1)\n"
         "(= (len c d) 1) (= (len a e) 2) (= (len e d) 3))\n" + metric + ")";
}

SolveResult plan(const std::string & p, bool anytime = true)
{
  SolveOptions options;
  options.anytime = anytime;
  return solve(kDomain, p, options);
}

const char * kCost = "(:metric minimize (total-cost))";

TEST(MetricTest, TheValueOfThePlan)
{
  // Any plan, with what it costs
  SolveResult r = plan(problem(kCost), false);
  ASSERT_EQ(r.status, Status::Solved) << r.message;
  ASSERT_TRUE(r.cost.has_value());
  VerifyResult v = verify(kDomain, problem(kCost), r.plan);
  EXPECT_TRUE(v.valid) << v.reason;
  ASSERT_TRUE(v.metric.has_value());
  EXPECT_DOUBLE_EQ(*v.metric, *r.cost);
  // No metric, no value
  r = plan(problem(""), false);
  ASSERT_EQ(r.status, Status::Solved) << r.message;
  EXPECT_FALSE(r.cost.has_value());
}

TEST(MetricTest, TheCheapestIsNotTheShortest)
{
  SolveResult r = plan(problem(kCost));
  ASSERT_EQ(r.status, Status::Solved) << r.message;
  EXPECT_EQ(r.steps, 3u) << r.plan;
  ASSERT_TRUE(r.cost.has_value());
  EXPECT_DOUBLE_EQ(*r.cost, 3.0);
  EXPECT_TRUE(r.optimal);
  // Without a metric, the shortest
  r = plan(problem(""));
  EXPECT_EQ(r.steps, 1u) << r.plan;
  EXPECT_TRUE(r.optimal);
}

TEST(MetricTest, CostsThatAreNotWhole)
{
  SolveResult r = plan(problem(kCost, ":ordered-subtasks (and (go d))", "3.25"));
  ASSERT_EQ(r.status, Status::Solved) << r.message;
  ASSERT_TRUE(r.cost.has_value());
  EXPECT_DOUBLE_EQ(*r.cost, 5.0) << r.plan;  // through e: the other way is now 5.25
  EXPECT_TRUE(r.optimal);
}

TEST(MetricTest, ActionsThatCostNothing)
{
  // Cheaper plans are found, but that none is cheaper is not proved
  SolveResult r = plan(problem(kCost, ":ordered-subtasks (and (go d))", "0"));
  ASSERT_EQ(r.status, Status::Solved) << r.message;
  ASSERT_TRUE(r.cost.has_value());
  EXPECT_DOUBLE_EQ(*r.cost, 2.0) << r.plan;
  EXPECT_FALSE(r.optimal);
}

TEST(MetricTest, CostNotInitialized)
{
  // (total-cost) starts at 0 when the problem does not say
  SolveResult r = plan(problem(kCost, ":ordered-subtasks (and (go d))", "1", ""));
  ASSERT_EQ(r.status, Status::Solved) << r.message;
  ASSERT_TRUE(r.cost.has_value());
  EXPECT_DOUBLE_EQ(*r.cost, 3.0);
}

TEST(MetricTest, ASumOfFluents)
{
  // Tolls count ten times: the metric is still a sum of what the actions add
  const std::string p = problem("(:metric minimize (+ (total-cost) (* 10 (tolls))))",
      ":ordered-subtasks (and (pay) (go d))");
  SolveResult r = plan(p);
  ASSERT_EQ(r.status, Status::Solved) << r.message;
  ASSERT_TRUE(r.cost.has_value());
  EXPECT_DOUBLE_EQ(*r.cost, 14.0) << r.plan;
  EXPECT_TRUE(r.optimal);
}

TEST(MetricTest, PartialOrder)
{
  // Paying and going in any order: the proof is over every interleaving
  SolveResult r = plan(problem(kCost, ":subtasks (and (pay) (go d))"));
  ASSERT_EQ(r.status, Status::Solved) << r.message;
  ASSERT_TRUE(r.cost.has_value());
  EXPECT_DOUBLE_EQ(*r.cost, 4.0) << r.plan;
  EXPECT_TRUE(r.optimal);
}

TEST(MetricTest, AnotherMetric)
{
  // Not a sum of costs to keep low: the best among the plans that are no longer, no proof
  SolveResult r = plan(problem("(:metric maximize (total-cost))"));
  ASSERT_EQ(r.status, Status::Solved) << r.message;
  ASSERT_TRUE(r.cost.has_value());
  EXPECT_DOUBLE_EQ(*r.cost, 10.0) << r.plan;
  EXPECT_FALSE(r.optimal);
}

TEST(MetricTest, EveryPlanGivenIsBetter)
{
  SolveOptions options;
  options.anytime = true;
  std::vector<double> costs;
  const std::string p = problem(kCost);
  options.on_plan = [&](const std::string & found, size_t) {
      VerifyResult v = verify(kDomain, p, found);
      ASSERT_TRUE(v.valid && v.metric.has_value());
      costs.push_back(*v.metric);
    };
  SolveResult r = solve(kDomain, p, options);
  ASSERT_EQ(r.status, Status::Solved) << r.message;
  ASSERT_FALSE(costs.empty());
  for (size_t i = 1; i < costs.size(); ++i) {
    EXPECT_LT(costs[i], costs[i - 1]);
  }
  EXPECT_DOUBLE_EQ(costs.back(), *r.cost);
  EXPECT_EQ(costs.size(), r.plans);
}

TEST(MetricTest, TheVerifierValuesAPlan)
{
  const std::string straight = "==>\n2 drive a d\nroot 1\n1 go d -> go-on 2 3\n"
    "3 go d -> go-here\n<==\n";
  VerifyResult v = verify(kDomain, problem(kCost), straight);
  ASSERT_TRUE(v.valid) << v.reason;
  ASSERT_TRUE(v.metric.has_value());
  EXPECT_DOUBLE_EQ(*v.metric, 10.0);
  v = verify(kDomain, problem("(:metric minimize (- (* 2 (total-cost)) 1))"), straight);
  ASSERT_TRUE(v.metric.has_value());
  EXPECT_DOUBLE_EQ(*v.metric, 19.0);
  v = verify(kDomain, problem(""), straight);
  EXPECT_TRUE(v.valid);
  EXPECT_FALSE(v.metric.has_value());
}

TEST(MetricTest, ShortestPaths)
{
  // Random roads: the cheapest plan costs what the shortest path measures
  std::mt19937 random(7);
  const std::string names = "abcdefgh";
  const size_t n = names.size();
  for (int round = 0; round < 30; ++round) {
    const int least = round % 3 == 0 ? 0 : 1;  // every third with roads of no length
    std::vector<std::vector<int>> far(n, std::vector<int>(n, 1000000));
    std::string init = "(at a) (= (tolls) 0)";
    for (size_t i = 0; i < n; ++i) {
      far[i][i] = 0;
      for (size_t j = 0; j < n; ++j) {
        // Always a way on: b, c... in a row
        if (i != j && (j == i + 1 || random() % 3 == 0)) {
          far[i][j] = least + static_cast<int>(random() % 9) + (j == i + 1 ? 5 : 0);
          const std::string ends = std::string(" ") + names[i] + " " + names[j];
          init += " (road" + ends + ") (= (len" + ends + ") " + std::to_string(far[i][j]) + ")";
        }
      }
    }
    for (size_t k = 0; k < n; ++k) {
      for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) {
          far[i][j] = std::min(far[i][j], far[i][k] + far[k][j]);
        }
      }
    }
    const std::string p = "(define (problem p) (:domain d) (:objects a b c d e f g h - place)\n"
      "(:htn :ordered-subtasks (and (go h)))\n(:init " + init + ")\n" + kCost + ")";
    SolveResult r = plan(p);
    ASSERT_EQ(r.status, Status::Solved) << r.message << p;
    ASSERT_TRUE(r.cost.has_value());
    if (least > 0) {
      EXPECT_TRUE(r.optimal) << p;
      EXPECT_DOUBLE_EQ(*r.cost, far[0][n - 1]) << p << r.plan;
    } else {
      EXPECT_GE(*r.cost, far[0][n - 1]) << p << r.plan;
    }
  }
}

TEST(SolveTest, TimeIsRefusedNotAnswered)
{
  // A durative action is not planned with yet: "not supported", never "no plan"
  const char * domain =
    "(define (domain d) (:requirements :typing :hierarchy :durative-actions)\n"
    " (:predicates (done))\n"
    " (:task t :parameters ())\n"
    " (:method m :parameters () :task (t) :ordered-subtasks (and (work)))\n"
    " (:durative-action work :parameters () :duration (= ?duration 2)\n"
    "  :condition (and) :effect (at end (done))))";
  const char * p = "(define (problem p) (:domain d)\n"
    "(:htn :ordered-subtasks (and (t))) (:init))";
  SolveResult r = solve(domain, p);
  EXPECT_EQ(r.status, Status::Unsupported) << r.message << r.plan;
  EXPECT_FALSE(r.message.empty());
}

}  // namespace
}  // namespace klimax

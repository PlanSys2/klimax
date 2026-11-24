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

// The public API of libklimax for Python (module klimax._klimax; klimax/__init__.py adds the
// Pythonic layer). Planning releases the GIL.

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/string_view.h>
#include <nanobind/stl/vector.h>

#include <string>

#include "klimax/klimax.hpp"

namespace nb = nanobind;
using nb::literals::operator""_a;

NB_MODULE(_klimax, m)
{
  m.attr("__version__") = std::string(klimax::version());

  nb::enum_<klimax::Severity>(m, "Severity")
  .value("ERROR", klimax::Severity::Error)
  .value("WARNING", klimax::Severity::Warning)
  .value("NOTE", klimax::Severity::Note);

  nb::enum_<klimax::Status>(m, "Status")
  .value("SOLVED", klimax::Status::Solved)
  .value("UNSOLVABLE", klimax::Status::Unsolvable)
  .value("TIMEOUT", klimax::Status::Timeout)
  .value("UNSUPPORTED", klimax::Status::Unsupported)
  .value("INVALID_INPUT", klimax::Status::InvalidInput)
  .value("INTERNAL_ERROR", klimax::Status::InternalError);

  nb::class_<klimax::Diagnostic>(m, "Diagnostic")
  .def_ro("severity", &klimax::Diagnostic::severity)
  .def_ro("code", &klimax::Diagnostic::code)
  .def_ro("message", &klimax::Diagnostic::message)
  .def_ro("file", &klimax::Diagnostic::file)
  .def_ro("line", &klimax::Diagnostic::line)
  .def_ro("column", &klimax::Diagnostic::column)
  .def_ro("rendered", &klimax::Diagnostic::rendered)
  .def("__str__", [](const klimax::Diagnostic & d) {return d.rendered;});

  nb::class_<klimax::Summary>(m, "Summary")
  .def_ro("types", &klimax::Summary::types)
  .def_ro("predicates", &klimax::Summary::predicates)
  .def_ro("actions", &klimax::Summary::actions)
  .def_ro("tasks", &klimax::Summary::tasks)
  .def_ro("methods", &klimax::Summary::methods)
  .def_ro("objects", &klimax::Summary::objects)
  .def_ro("initial_tasks", &klimax::Summary::initial_tasks)
  .def_ro("total_order", &klimax::Summary::total_order)
  .def_ro("has_goal", &klimax::Summary::has_goal);

  nb::class_<klimax::CheckResult>(m, "CheckResult")
  .def_ro("ok", &klimax::CheckResult::ok)
  .def_ro("diagnostics", &klimax::CheckResult::diagnostics)
  .def_ro("summary", &klimax::CheckResult::summary);

  nb::class_<klimax::PlanStep>(m, "PlanStep")
  .def_ro("id", &klimax::PlanStep::id)
  .def_ro("task", &klimax::PlanStep::task)
  .def_ro("args", &klimax::PlanStep::args)
  .def_ro("primitive", &klimax::PlanStep::primitive)
  .def_ro("method", &klimax::PlanStep::method)
  .def_ro("params", &klimax::PlanStep::params)
  .def_ro("subtasks", &klimax::PlanStep::subtasks)
  .def_ro("labels", &klimax::PlanStep::labels);

  nb::class_<klimax::SolveResult>(m, "SolveResult")
  .def_ro("status", &klimax::SolveResult::status)
  .def_ro("plan", &klimax::SolveResult::plan)
  .def_ro("steps_of_plan", &klimax::SolveResult::steps_of_plan)
  .def_ro("sequence", &klimax::SolveResult::sequence)
  .def_ro("roots", &klimax::SolveResult::roots)
  .def_ro("root_labels", &klimax::SolveResult::root_labels)
  .def_ro("message", &klimax::SolveResult::message)
  .def_ro("diagnostics", &klimax::SolveResult::diagnostics)
  .def_ro("steps", &klimax::SolveResult::steps)
  .def_ro("ground_ms", &klimax::SolveResult::ground_ms)
  .def_ro("search_ms", &klimax::SolveResult::search_ms)
  .def_ro("expansions", &klimax::SolveResult::expansions);

  nb::class_<klimax::VerifyResult>(m, "VerifyResult")
  .def_ro("valid", &klimax::VerifyResult::valid)
  .def_ro("reason", &klimax::VerifyResult::reason)
  .def_ro("diagnostics", &klimax::VerifyResult::diagnostics);

  m.def("check",
    [](const std::string & domain, const std::string & problem, const std::string & domain_file,
    const std::string & problem_file) {
      return klimax::check(domain, problem, domain_file, problem_file);
    }, "domain"_a, "problem"_a, "domain_file"_a = "domain.hddl",
    "problem_file"_a = "problem.hddl");

  m.def("solve",
    [](const std::string & domain, const std::string & problem, double timeout,
    bool decomposition, const std::string & domain_file, const std::string & problem_file) {
      klimax::SolveOptions options;
      options.seconds = timeout;
      options.decomposition = decomposition;
      nb::gil_scoped_release release;
      return klimax::solve(domain, problem, options, domain_file, problem_file);
    }, "domain"_a, "problem"_a, "timeout"_a = 0.0, "decomposition"_a = true,
    "domain_file"_a = "domain.hddl", "problem_file"_a = "problem.hddl");

  m.def("verify",
    [](const std::string & domain, const std::string & problem, const std::string & plan,
    const std::string & domain_file, const std::string & problem_file) {
      nb::gil_scoped_release release;
      return klimax::verify(domain, problem, plan, domain_file, problem_file);
    }, "domain"_a, "problem"_a, "plan"_a, "domain_file"_a = "domain.hddl",
    "problem_file"_a = "problem.hddl");
}

# Copyright 2026 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Links kairos::core directly: checks that Kairos' internal modules are usable from Klimax
add_executable(test_kairos_core test_kairos_core.cpp)
target_link_libraries(test_kairos_core PRIVATE klimax kairos::core)
klimax_configure_target(test_kairos_core)
add_test(NAME kairos_core COMMAND test_kairos_core)

# Unit tests of the internal modules
find_package(GTest QUIET)
if(NOT GTest_FOUND)
  include(FetchContent)
  FetchContent_Declare(googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.tar.gz)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
endif()
include(GoogleTest)

function(klimax_add_gtest name)
  add_executable(${name} ${ARGN})
  target_link_libraries(${name} PRIVATE klimax_objects GTest::gtest_main)
  klimax_configure_target(${name})
  gtest_discover_tests(${name} DISCOVERY_TIMEOUT 60)
endfunction()

klimax_add_gtest(hddl_parser_test hddl/parser_test.cpp)
klimax_add_gtest(hddl_model_test hddl/model_test.cpp)
klimax_add_gtest(verifier_test hddl/verifier_test.cpp)
klimax_add_gtest(grounder_test hddl/grounder_test.cpp)
klimax_add_gtest(total_order_test search/total_order_test.cpp)

if(KLIMAX_BUILD_CLI)
  add_test(NAME cli_version COMMAND klimax_cli --version)
  set_tests_properties(cli_version PROPERTIES
    PASS_REGULAR_EXPRESSION "^klimax [0-9]+\\.[0-9]+\\.[0-9]+ \\(kairos [0-9]")
  add_test(NAME cli_unknown_argument COMMAND klimax_cli --nonsense)
  set_tests_properties(cli_unknown_argument PROPERTIES WILL_FAIL ON)
  add_test(NAME cli_no_arguments COMMAND klimax_cli)
  set_tests_properties(cli_no_arguments PROPERTIES WILL_FAIL ON)
endif()

if(KLIMAX_BUILD_CLI)
  add_test(NAME cli_check COMMAND klimax_cli --check
    ${CMAKE_CURRENT_SOURCE_DIR}/data/house/domain.hddl
    ${CMAKE_CURRENT_SOURCE_DIR}/data/house/p1-total-order.hddl)
  set_tests_properties(cli_check PROPERTIES
    PASS_REGULAR_EXPRESSION "actions 6, tasks 3, methods 6, objects 7, initial tasks 2, total order")
  add_test(NAME cli_check_missing_file COMMAND klimax_cli --check nowhere.hddl nowhere.hddl)
  set_tests_properties(cli_check_missing_file PROPERTIES WILL_FAIL ON)
endif()

if(KLIMAX_BUILD_CLI)
  foreach(case p1-total-order p2-partial-order)
    add_test(NAME cli_verify_${case} COMMAND klimax_cli --verify
      ${CMAKE_CURRENT_SOURCE_DIR}/data/house/domain.hddl
      ${CMAKE_CURRENT_SOURCE_DIR}/data/house/${case}.hddl
      ${CMAKE_CURRENT_SOURCE_DIR}/data/house/${case}.plan)
    set_tests_properties(cli_verify_${case} PROPERTIES
      PASS_REGULAR_EXPRESSION "Plan verification result: true")
  endforeach()
  # A plan for another problem
  add_test(NAME cli_verify_wrong_plan COMMAND klimax_cli --verify
    ${CMAKE_CURRENT_SOURCE_DIR}/data/house/domain.hddl
    ${CMAKE_CURRENT_SOURCE_DIR}/data/house/p1-total-order.hddl
    ${CMAKE_CURRENT_SOURCE_DIR}/data/house/p2-partial-order.plan)
  set_tests_properties(cli_verify_wrong_plan PROPERTIES
    PASS_REGULAR_EXPRESSION "Plan verification result: false")
endif()

if(KLIMAX_BUILD_CLI)
  # The ground problem contains the known plan
  add_test(NAME cli_ground COMMAND klimax_cli --ground
    ${CMAKE_CURRENT_SOURCE_DIR}/data/house/domain.hddl
    ${CMAKE_CURRENT_SOURCE_DIR}/data/house/p2-partial-order.hddl 0
    ${CMAKE_CURRENT_SOURCE_DIR}/data/house/p2-partial-order.plan)
  set_tests_properties(cli_ground PROPERTIES
    PASS_REGULAR_EXPRESSION "facts 20, actions 30, abstract tasks 18, methods 77"
    FAIL_REGULAR_EXPRESSION "not in the ground problem")
endif()

if(KLIMAX_BUILD_CLI)
  add_test(NAME cli_solve COMMAND klimax_cli
    ${CMAKE_CURRENT_SOURCE_DIR}/data/house/domain.hddl
    ${CMAKE_CURRENT_SOURCE_DIR}/data/house/p1-total-order.hddl)
  set_tests_properties(cli_solve PROPERTIES
    PASS_REGULAR_EXPRESSION "==>.*drop tiago book living.*root.*<==")
  add_test(NAME cli_solve_timeout COMMAND klimax_cli
    ${CMAKE_CURRENT_SOURCE_DIR}/data/house/domain.hddl
    ${CMAKE_CURRENT_SOURCE_DIR}/data/house/p1-total-order.hddl --timeout 5)
  set_tests_properties(cli_solve_timeout PROPERTIES PASS_REGULAR_EXPRESSION "; solved")
  # A partially ordered problem, solved in one of the orders it allows
  add_test(NAME cli_solve_partial_order COMMAND klimax_cli
    ${CMAKE_CURRENT_SOURCE_DIR}/data/house/domain.hddl
    ${CMAKE_CURRENT_SOURCE_DIR}/data/house/p2-partial-order.hddl)
  set_tests_properties(cli_solve_partial_order PROPERTIES
    PASS_REGULAR_EXPRESSION "==>.*move kobuki bedroom hall.*<==")
endif()

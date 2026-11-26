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

# Each target is built two ways:
# - <name>_replay: runs the seed corpus as a regular ctest test (any compiler)
# - <name>: a libFuzzer binary, with KLIMAX_BUILD_FUZZERS=ON and Clang
function(klimax_add_fuzzer name)
  cmake_parse_arguments(ARG "" "CORPUS" "SOURCES" ${ARGN})

  if(KLIMAX_BUILD_TESTS)
    add_executable(${name}_replay ${ARG_SOURCES} replay_main.cpp)
    target_link_libraries(${name}_replay PRIVATE klimax_objects)
    klimax_configure_target(${name}_replay)
    add_test(NAME ${name}_corpus COMMAND ${name}_replay ${ARG_CORPUS})
    set_tests_properties(${name}_corpus PROPERTIES LABELS fuzz)
  endif()

  if(KLIMAX_BUILD_FUZZERS)
    if(NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      message(FATAL_ERROR "KLIMAX_BUILD_FUZZERS requires Clang")
    endif()
    add_executable(${name} ${ARG_SOURCES})
    target_link_libraries(${name} PRIVATE klimax_objects)
    target_compile_options(${name} PRIVATE -fsanitize=fuzzer,address,undefined -g)
    target_link_options(${name} PRIVATE -fsanitize=fuzzer,address,undefined)
  endif()
endfunction()

set(_corpus ${CMAKE_CURRENT_SOURCE_DIR}/corpus)
klimax_add_fuzzer(hddl_parser_fuzzer SOURCES hddl_parser_fuzzer.cpp CORPUS ${_corpus}/parser)
klimax_add_fuzzer(check_fuzzer SOURCES check_fuzzer.cpp CORPUS ${_corpus}/pair)
klimax_add_fuzzer(verify_fuzzer SOURCES verify_fuzzer.cpp CORPUS ${_corpus}/plan)
klimax_add_fuzzer(solve_fuzzer SOURCES solve_fuzzer.cpp CORPUS ${_corpus}/pair)

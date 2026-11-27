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

cmake_minimum_required(VERSION 3.22)

project(klimax
  VERSION 1.0.0
  DESCRIPTION "HTN planner for HDDL 2.1: hierarchical, numeric and temporal"
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(DEFINED BUILD_TESTING)
  set(_klimax_tests_default ${BUILD_TESTING})
else()
  set(_klimax_tests_default ON)
endif()
option(KLIMAX_BUILD_TESTS "Build the tests" ${_klimax_tests_default})
option(KLIMAX_BUILD_CLI "Build the klimax executable" ON)
option(KLIMAX_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)
option(KLIMAX_BUILD_PYTHON "Build the Python module (needs nanobind)" OFF)
option(KLIMAX_BUILD_FUZZERS "Build the libFuzzer targets (needs Clang)" OFF)
set(KLIMAX_SANITIZE "" CACHE STRING "Sanitizers: empty or 'address;undefined'")

include(GNUInstallDirs)

# Klimax is built on Kairos' modules: Kairos must be installed with -DKAIROS_INSTALL_CORE=ON
find_package(kairos 1.0.7 REQUIRED)
if(NOT TARGET kairos::core)
  message(FATAL_ERROR
    "This Kairos has no kairos::core. Build Kairos with -DKAIROS_INSTALL_CORE=ON.")
endif()

function(klimax_configure_target target)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${target} PRIVATE
      -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow
      -Wnon-virtual-dtor -Wold-style-cast -Woverloaded-virtual)
    if(KLIMAX_WARNINGS_AS_ERRORS)
      target_compile_options(${target} PRIVATE -Werror)
    endif()
  endif()
  if(KLIMAX_SANITIZE)
    string(REPLACE ";" "," _sanitizers "${KLIMAX_SANITIZE}")
    target_compile_options(${target} PRIVATE -fsanitize=${_sanitizers} -fno-omit-frame-pointer)
    target_link_options(${target} PRIVATE -fsanitize=${_sanitizers})
  endif()
  # Fuzzing: the library is instrumented too, for libFuzzer to see what an input reaches
  if(KLIMAX_BUILD_FUZZERS)
    target_compile_options(${target} PRIVATE -fsanitize=fuzzer-no-link,address,undefined -g)
    target_link_options(${target} PRIVATE -fsanitize=address,undefined)
  endif()
endfunction()

add_subdirectory(src)

if(KLIMAX_BUILD_TESTS)
  include(CTest)
  add_subdirectory(tests)
endif()

if(KLIMAX_BUILD_PYTHON)
  add_subdirectory(python)
endif()

if(KLIMAX_BUILD_TESTS OR KLIMAX_BUILD_FUZZERS)
  add_subdirectory(fuzz)
endif()

include(CMakePackageConfigHelpers)

install(DIRECTORY include/klimax DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  PATTERN "*.in" EXCLUDE)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/include/klimax/version.hpp
  ${CMAKE_CURRENT_BINARY_DIR}/include/klimax/export.hpp
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/klimax)
install(EXPORT klimaxTargets
  NAMESPACE klimax::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/klimax)
configure_package_config_file(cmake/klimaxConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/klimaxConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/klimax)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/klimaxConfigVersion.cmake
  COMPATIBILITY SameMajorVersion)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/klimaxConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/klimaxConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/klimax)

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
  VERSION 0.0.1
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

include(GNUInstallDirs)

# Klimax is built on Kairos' modules: Kairos must be installed with -DKAIROS_INSTALL_CORE=ON
find_package(kairos 1.0.6 REQUIRED)
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
endfunction()

add_subdirectory(src)

if(KLIMAX_BUILD_TESTS)
  include(CTest)
  add_subdirectory(tests)
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

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

include(GenerateExportHeader)

set(KLIMAX_GENERATED_INCLUDE_DIR ${PROJECT_BINARY_DIR}/include)

# libklimax: Kairos' modules are linked inside it (static), so it has no runtime dependency
# The modules, compiled once: libklimax and the unit tests (which reach internal symbols) share them
add_library(klimax_objects OBJECT
  check.cpp
  schedule.cpp
  hddl/model.cpp
  hddl/parser.cpp
  hddl/split.cpp
  sat/solver.cpp
  search/lifted.cpp
  search/sat.cpp
  search/total_order.cpp
  tdg/compress.cpp
  tdg/grounder.cpp
  tdg/object_fluents.cpp
  verify/verifier.cpp
  version.cpp)
set_target_properties(klimax_objects PROPERTIES
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN ON
  POSITION_INDEPENDENT_CODE ON)
target_compile_definitions(klimax_objects PRIVATE klimax_EXPORTS)
target_include_directories(klimax_objects PUBLIC
  ${PROJECT_SOURCE_DIR}/include ${KLIMAX_GENERATED_INCLUDE_DIR} ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(klimax_objects PUBLIC kairos::core)
klimax_configure_target(klimax_objects)

add_library(klimax SHARED $<TARGET_OBJECTS:klimax_objects>)
add_library(klimax::klimax ALIAS klimax)

generate_export_header(klimax
  BASE_NAME KLIMAX
  EXPORT_MACRO_NAME KLIMAX_API
  EXPORT_FILE_NAME ${KLIMAX_GENERATED_INCLUDE_DIR}/klimax/export.hpp)
configure_file(${PROJECT_SOURCE_DIR}/include/klimax/version.hpp.in
  ${KLIMAX_GENERATED_INCLUDE_DIR}/klimax/version.hpp @ONLY)

set_target_properties(klimax PROPERTIES
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN ON
  POSITION_INDEPENDENT_CODE ON
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR})
target_include_directories(klimax PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${KLIMAX_GENERATED_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_include_directories(klimax PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(klimax PRIVATE kairos::core)
klimax_configure_target(klimax)

install(TARGETS klimax EXPORT klimaxTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

if(KLIMAX_BUILD_CLI)
  add_subdirectory(cli)
endif()

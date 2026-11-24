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

# klimax._klimax (nanobind, stable ABI from Python 3.12). Klimax and Kairos' modules are linked
# inside the module: it has no runtime dependency.
find_package(Python 3.10 REQUIRED COMPONENTS Interpreter Development.Module
  OPTIONAL_COMPONENTS Development.SABIModule)
execute_process(COMMAND ${Python_EXECUTABLE} -m nanobind --cmake_dir
  OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE nanobind_ROOT)
find_package(nanobind CONFIG REQUIRED)

nanobind_add_module(_klimax STABLE_ABI NB_STATIC
  src/klimax_python.cpp $<TARGET_OBJECTS:klimax_objects>)
target_include_directories(_klimax PRIVATE
  ${PROJECT_SOURCE_DIR}/include ${PROJECT_BINARY_DIR}/include)
target_link_libraries(_klimax PRIVATE kairos::core)
target_compile_features(_klimax PRIVATE cxx_std_20)

# The package next to the module, for tests from the build tree
file(GLOB _klimax_py ${CMAKE_CURRENT_SOURCE_DIR}/klimax/*.py ${CMAKE_CURRENT_SOURCE_DIR}/klimax/py.typed)
set(_klimax_pkg ${CMAKE_CURRENT_BINARY_DIR}/klimax)
set_target_properties(_klimax PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${_klimax_pkg})
foreach(f ${_klimax_py})
  get_filename_component(n ${f} NAME)
  configure_file(${f} ${_klimax_pkg}/${n} COPYONLY)
endforeach()

if(DEFINED SKBUILD_PROJECT_NAME)
  install(TARGETS _klimax LIBRARY DESTINATION klimax COMPONENT python)
endif()

if(KLIMAX_BUILD_TESTS)
  add_test(NAME python_tests
    COMMAND ${Python_EXECUTABLE} -m pytest -q ${CMAKE_CURRENT_SOURCE_DIR}/tests)
  set_tests_properties(python_tests PROPERTIES
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR};KLIMAX_TEST_DATA=${PROJECT_SOURCE_DIR}/tests/data"
    LABELS python)
endif()

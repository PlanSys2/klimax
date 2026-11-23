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

if(KLIMAX_BUILD_CLI)
  add_test(NAME cli_version COMMAND klimax_cli --version)
  set_tests_properties(cli_version PROPERTIES
    PASS_REGULAR_EXPRESSION "^klimax [0-9]+\\.[0-9]+\\.[0-9]+ \\(kairos [0-9]")
  add_test(NAME cli_unknown_argument COMMAND klimax_cli --nonsense)
  set_tests_properties(cli_unknown_argument PROPERTIES WILL_FAIL ON)
  add_test(NAME cli_no_arguments COMMAND klimax_cli)
  set_tests_properties(cli_no_arguments PROPERTIES WILL_FAIL ON)
endif()

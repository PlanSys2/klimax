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

# The klimax command, built on the public library only
add_executable(klimax_cli main.cpp)
set_target_properties(klimax_cli PROPERTIES OUTPUT_NAME klimax)
target_link_libraries(klimax_cli PRIVATE klimax)
klimax_configure_target(klimax_cli)
file(RELATIVE_PATH _klimax_bin_to_lib
  ${CMAKE_INSTALL_FULL_BINDIR} ${CMAKE_INSTALL_FULL_LIBDIR})
set_target_properties(klimax_cli PROPERTIES INSTALL_RPATH "$ORIGIN/${_klimax_bin_to_lib}")
install(TARGETS klimax_cli RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

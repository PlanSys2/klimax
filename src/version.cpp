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

#include "klimax/version.hpp"

#include "kairos/version.hpp"

namespace klimax
{

std::string_view version()
{
  return KLIMAX_VERSION_STRING;
}

std::string_view kairos_version()
{
  return kairos::version();
}

}  // namespace klimax

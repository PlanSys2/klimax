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

// The inputs of the fuzz targets that take several texts: domain, problem and plan, one after
// another, each part ending at a line ";;;;" (the last one, at the end)

#ifndef KLIMAX__FUZZ__PARTS_HPP_
#define KLIMAX__FUZZ__PARTS_HPP_

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string_view>
#include <vector>

namespace klimax::fuzz
{

inline std::vector<std::string_view> parts(const uint8_t * data, size_t size, size_t count)
{
  std::string_view rest(reinterpret_cast<const char *>(data), size);
  std::vector<std::string_view> out;
  constexpr std::string_view kEnd = "\n;;;;\n";
  while (out.size() + 1 < count) {
    const size_t at = rest.find(kEnd);
    if (at == std::string_view::npos) {
      break;
    }
    out.push_back(rest.substr(0, at));
    rest.remove_prefix(at + kEnd.size());
  }
  out.push_back(rest);
  out.resize(count);
  return out;
}

[[noreturn]] inline void fail(const char * target, const char * what)
{
  std::fprintf(stderr, "%s invariant violated: %s\n", target, what);
  std::abort();
}

}  // namespace klimax::fuzz

#endif  // KLIMAX__FUZZ__PARTS_HPP_

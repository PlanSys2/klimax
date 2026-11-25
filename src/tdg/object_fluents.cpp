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


#include "tdg/object_fluents.hpp"

#include <algorithm>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace klimax::tdg
{
namespace
{

namespace ground = kairos::ground;
namespace sem = kairos::sem;

class Rewriter
{
public:
  Rewriter(const ground::Task & task, const hddl::Model & model)
  : in_(task), out_(task), m_(model) {}

  bool run(ground::Task & result)
  {
    atoms();
    out_.actions.clear();
    for (const ground::Action & a : in_.actions) {
      if (!action(a)) {
        return false;
      }
    }
    std::vector<ground::Condition> goal;
    if (!expand(in_.goal, goal) || goal.size() != 1 || !in_.always.objects.empty()) {
      return false;
    }
    out_.goal = std::move(goal[0]);
    for (size_t f = 0; f < in_.object_fluents.size(); ++f) {
      if (f < in_.init_objects.size() && in_.init_objects[f]) {
        if (auto a = atom(static_cast<uint32_t>(f), in_.init_objects[f]->value())) {
          out_.init_atoms.push_back(*a);
        }
      }
    }
    out_.object_fluents.clear();
    out_.init_objects.clear();
    result = std::move(out_);
    return true;
  }

private:
  // One predicate per function, one atom per fluent and object of its type
  void atoms()
  {
    std::unordered_map<uint32_t, uint32_t> predicate;  // by function
    for (const ground::FluentInfo & f : in_.object_fluents) {
      const sem::FunctionInfo & info = m_.domain.functions.items[f.function];
      auto [p, fresh] = predicate.emplace(f.function.value(),
          static_cast<uint32_t>(out_.predicate_names.size()));
      if (fresh) {
        out_.predicate_names.push_back(info.name + "=");
        out_.derived.push_back(false);
      }
      first_.push_back(static_cast<uint32_t>(out_.atoms.size()));
      values_.emplace_back();
      for (uint32_t o = 0; o < m_.objects.objects.items.size(); ++o) {
        if (m_.domain.types.fits(m_.objects.objects.items[sem::ObjectId(o)].types, info.result)) {
          ground::AtomInfo a{sem::PredicateId(p->second), f.args};
          a.args.emplace_back(o);
          values_.back().emplace(o, static_cast<uint32_t>(values_.back().size()));
          out_.atoms.push_back(std::move(a));
        }
      }
    }
  }

  // The atom that says the fluent holds the object; none if it never can
  std::optional<ground::AtomId> atom(uint32_t fluent, uint32_t object) const
  {
    const auto found = values_[fluent].find(object);
    return found == values_[fluent].end() ? std::nullopt :
           std::optional<ground::AtomId>(first_[fluent] + found->second);
  }

  // `c` without comparisons of objects, as the cases in which it holds (none: never). False
  // if it cannot be written so
  bool expand(const ground::Condition & c, std::vector<ground::Condition> & cases) const
  {
    ground::Condition base = c;
    base.objects.clear();
    cases.assign(1, std::move(base));
    for (const ground::ObjectCondition & o : c.objects) {
      const ground::ObjectTerm & l = o.lhs.is_fluent ? o.lhs : o.rhs;
      const ground::ObjectTerm & r = o.lhs.is_fluent ? o.rhs : o.lhs;
      if (!l.is_fluent) {
        if ((l.index == r.index) != o.equal) {
          cases.clear();
        }
        continue;
      }
      if (!r.is_fluent) {
        const auto a = atom(l.index, r.index);
        if (!a && o.equal) {
          cases.clear();
        }
        for (ground::Condition & k : cases) {
          if (a) {
            (o.equal ? k.positive : k.negative).push_back(*a);
          }
        }
        continue;
      }
      if (!o.equal) {
        return false;
      }
      // Both hold the same object: one case per object
      std::vector<ground::Condition> next;
      for (const auto & [object, at] : values_[l.index]) {
        const auto other = atom(r.index, object);
        if (!other) {
          continue;
        }
        for (const ground::Condition & k : cases) {
          next.push_back(k);
          next.back().positive.push_back(first_[l.index] + at);
          if (*other != first_[l.index] + at) {
            next.back().positive.push_back(*other);
          }
        }
      }
      cases = std::move(next);
    }
    return true;
  }

  static void conjoin(ground::Condition & c, const ground::Condition & with)
  {
    c.positive.insert(c.positive.end(), with.positive.begin(), with.positive.end());
    c.negative.insert(c.negative.end(), with.negative.begin(), with.negative.end());
    c.numeric.insert(c.numeric.end(), with.numeric.begin(), with.numeric.end());
  }

  void change(
    ground::Action & a, const ground::Effect & from, const ground::Condition & when,
    ground::AtomId atom, bool add, std::optional<std::pair<ground::AtomId, bool>> also = {}) const
  {
    ground::Effect e;
    e.kind = add ? ground::Effect::Kind::Add : ground::Effect::Kind::Delete;
    e.timing = from.timing;
    e.condition = when;
    e.atom = atom;
    if (also) {
      (also->second ? e.condition.positive : e.condition.negative).push_back(also->first);
    }
    a.effects.push_back(std::move(e));
  }

  // The effects of `a` without object fluents, into `made`
  bool effects(const ground::Action & a, ground::Action & made) const
  {
    made.effects.clear();
    for (const ground::Effect & e : a.effects) {
      if (!e.start_condition.empty()) {
        return false;
      }
      std::vector<ground::Condition> cases;
      if (!expand(e.condition, cases)) {
        return false;
      }
      for (const ground::Condition & when : cases) {
        if (e.kind != ground::Effect::Kind::ObjectAssign) {
          made.effects.push_back(e);
          made.effects.back().condition = when;
          continue;
        }
        const auto & held = values_[e.fluent];
        if (e.object && e.object->is_fluent) {
          // A copy: what the other holds, or nothing if it holds nothing this one can
          for (const auto & [object, at] : held) {
            const auto source = atom(e.object->index, object);
            if (source && *source == first_[e.fluent] + at) {
              continue;  // onto itself
            }
            if (source) {
              change(made, e, when, first_[e.fluent] + at, true, std::pair(*source, true));
              change(made, e, when, first_[e.fluent] + at, false, std::pair(*source, false));
            } else {
              change(made, e, when, first_[e.fluent] + at, false);
            }
          }
          continue;
        }
        const std::optional<ground::AtomId> now =
          e.object ? atom(e.fluent, e.object->index) : std::nullopt;
        for (const auto & [object, at] : held) {
          const ground::AtomId other = first_[e.fluent] + at;
          if (now && other == *now) {
            continue;
          }
          if (when.empty()) {
            made.changes.del.push_back(other);
          } else {
            change(made, e, when, other, false);
          }
        }
        if (now && when.empty()) {
          made.changes.add.push_back(*now);
        } else if (now) {
          change(made, e, when, *now, true);
        }
      }
    }
    // An atom deleted and added is added
    std::erase_if(made.changes.del, [&](ground::AtomId d) {
        return std::find(made.changes.add.begin(), made.changes.add.end(), d) !=
               made.changes.add.end();
      });
    return true;
  }

  bool action(const ground::Action & a)
  {
    std::vector<ground::Condition> cases;
    ground::Action made = a;
    if (!expand(a.precondition, cases) || !effects(a, made)) {
      return false;
    }
    for (ground::Condition & c : cases) {
      out_.actions.push_back(made);
      out_.actions.back().precondition = std::move(c);
    }
    return true;
  }

  const ground::Task & in_;
  ground::Task out_;
  const hddl::Model & m_;
  std::vector<uint32_t> first_;  // atom of each fluent's first object
  std::vector<std::unordered_map<uint32_t, uint32_t>> values_;  // object -> its place
};

}  // namespace

bool object_fluents_as_atoms(kairos::ground::Task & task, const hddl::Model & model)
{
  if (task.object_fluents.empty()) {
    return true;
  }
  return Rewriter(task, model).run(task);
}

}  // namespace klimax::tdg

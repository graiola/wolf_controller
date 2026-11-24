/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace wolf_controller::mpc_contract {

constexpr std::size_t kContactCount = 4;
constexpr std::size_t kJointCount = 12;
constexpr std::size_t kForceCount = 3 * kContactCount;
constexpr std::size_t kStateSize = 12 + kJointCount;
constexpr std::size_t kInputSize = kForceCount + kJointCount;

// Contact vectors use LF,LH,RF,RH. OCS2 gait bits use LF,RF,LH,RH.
inline std::int8_t encodeMode(const std::array<bool, kContactCount>& contacts) {
  constexpr std::array<int, kContactCount> weights{{8, 2, 4, 1}};
  int mode = 0;
  for (std::size_t i = 0; i < contacts.size(); ++i)
    mode += contacts[i] ? weights[i] : 0;
  return static_cast<std::int8_t>(mode);
}

inline bool uniqueNames(const std::vector<std::string>& names) {
  for (std::size_t i = 0; i < names.size(); ++i) {
    if (names[i].empty() || std::find(names.begin(), names.begin() + i, names[i]) != names.begin() + i)
      return false;
  }
  return true;
}

// Automatic defaults only apply to the standard WoLF limb naming convention.
// Other robots supply explicit mpc_contact_names/mpc_joint_names parameters.
inline std::vector<std::string> canonicalLegOrder(const std::vector<std::string>& names) {
  const std::array<std::string, kContactCount> prefixes{{"lf", "lh", "rf", "rh"}};
  std::vector<std::string> ordered;
  for (const auto& prefix : prefixes) {
    std::string match;
    for (const auto& name : names) {
      if (name.rfind(prefix, 0) != 0)
        continue;
      if (!match.empty())
        throw std::invalid_argument("Ambiguous MPC limb prefix: " + prefix);
      match = name;
    }
    if (match.empty())
      throw std::invalid_argument("No MPC limb for prefix: " + prefix);
    ordered.push_back(match);
  }
  return ordered;
}

// Reject partial, duplicate and unknown contact commands as one unit. A failed
// validation never changes the previously accepted contact schedule.
inline bool reorderContacts(const std::vector<std::string>& expected,
                            const std::vector<std::string>& names,
                            const std::vector<bool>& states,
                            std::array<bool, kContactCount>& result) {
  if (expected.size() != kContactCount || names.size() != kContactCount ||
      states.size() != kContactCount || !uniqueNames(expected) || !uniqueNames(names))
    return false;
  std::array<bool, kContactCount> candidate{};
  for (std::size_t i = 0; i < names.size(); ++i) {
    const auto found = std::find(expected.begin(), expected.end(), names[i]);
    if (found == expected.end())
      return false;
    candidate[static_cast<std::size_t>(found - expected.begin())] = states[i];
  }
  result = candidate;
  return true;
}

inline bool contactReferenceExpired(bool external_mode, bool has_reference,
                                    double age_seconds, double timeout_seconds) {
  return external_mode && has_reference &&
         (!std::isfinite(age_seconds) || age_seconds < 0.0 || age_seconds > timeout_seconds);
}

inline bool allowMpcStart(bool controller_active, bool has_contacts,
                          bool contacts_adopted, bool contacts_fresh, bool references_fresh) {
  return controller_active && has_contacts && contacts_adopted && contacts_fresh && references_fresh;
}

enum class WatchdogAction { None, CancelPending, StopActive };

inline WatchdogAction referenceWatchdogAction(bool external, bool pending_external,
                                              bool has_contacts, bool contacts_fresh,
                                              bool references_fresh) {
  if (!has_contacts || (contacts_fresh && references_fresh))
    return WatchdogAction::None;
  if (external)
    return WatchdogAction::StopActive;
  return pending_external ? WatchdogAction::CancelPending : WatchdogAction::None;
}

}  // namespace wolf_controller::mpc_contract

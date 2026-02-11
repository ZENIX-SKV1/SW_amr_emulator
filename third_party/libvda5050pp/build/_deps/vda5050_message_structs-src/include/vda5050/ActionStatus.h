// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//


#ifndef INCLUDE_VDA5050_ACTIONSTATUS_H_
#define INCLUDE_VDA5050_ACTIONSTATUS_H_

#include <nlohmann/json.hpp>

namespace vda5050 {
enum class ActionStatus {
  /// Action was received by AGV but the node where it triggers was
  /// not yet reached or the edge where it is active was not
  /// yet entered.
  WAITING,

  /// Action was triggered, preparatory measures are initiated.
  INITIALIZING,

  /// The action is paused because of a pause instantAction or
  /// external trigger (pause button on AGV)
  RUNNING,

  /// The action is paused because of a pause instantAction or external
  /// trigger (pause button on AGV)
  PAUSED,

  /// The action is finished. A result is reported via the
  /// resultDescription
  FINISHED,

  /// Action could not be finished for whatever reason.
  FAILED
};

///
///\brief Write the enum-value to an ostream
///
///\param os the stream
///\param action_status the enum
///\return constexpr std::ostream&
///
constexpr std::ostream &operator<<(std::ostream &os, const ActionStatus &action_status) {
  switch (action_status) {
    case ActionStatus::WAITING:
      os << "WAITING";
      break;
    case ActionStatus::INITIALIZING:
      os << "INITIALIZING";
      break;
    case ActionStatus::RUNNING:
      os << "RUNNING";
      break;
    case ActionStatus::PAUSED:
      os << "PAUSED";
      break;
    case ActionStatus::FINISHED:
      os << "FINISHED";
      break;
    case ActionStatus::FAILED:
      os << "FAILED";
      break;
    default:
      os.setstate(std::ios_base::failbit);
  }
  return os;
}

using json = nlohmann::json;
void to_json(json &j, const ActionStatus &d);
void from_json(const json &j, ActionStatus &d);

}  // namespace vda5050
#endif  // INCLUDE_VDA5050_ACTIONSTATUS_H_

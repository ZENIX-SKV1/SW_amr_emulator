// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//


#ifndef INCLUDE_VDA5050_CONNECTIONSTATE_H_
#define INCLUDE_VDA5050_CONNECTIONSTATE_H_

#include <ostream>
#include <nlohmann/json.hpp>

namespace vda5050 {
enum class ConnectionState {
  /// connection between AGV and broker is active.
  ONLINE,
  /// connection between AGV and broker has gone
  /// offline in a coordinated way.
  OFFLINE,
  /// The connection between AGV and
  /// broker has unexpectedly ended.
  /// (used in e.g. MQTT Last-Will-Message)
  CONNECTIONBROKEN
};

///
///\brief Write the enum-value to an ostream
///
///\param os the stream
///\param connection_state the enum
///\return constexpr std::ostream&
///
constexpr std::ostream &operator<<(std::ostream &os, const ConnectionState &connection_state) {
  switch (connection_state) {
    case ConnectionState::ONLINE:
      os << "ONLINE";
      break;
    case ConnectionState::OFFLINE:
      os << "OFFLINE";
      break;
    case ConnectionState::CONNECTIONBROKEN:
      os << "CONNECTIONBROKEN";
      break;
    default:
      os.setstate(std::ios_base::failbit);
  }
  return os;
}

using json = nlohmann::json;
void to_json(json &j, const ConnectionState &d);
void from_json(const json &j, ConnectionState &d);

}  // namespace vda5050
#endif  // INCLUDE_VDA5050_CONNECTIONSTATE_H_

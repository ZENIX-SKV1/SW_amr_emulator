// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//


#ifndef INCLUDE_VDA5050_CONNECTION_H_
#define INCLUDE_VDA5050_CONNECTION_H_

#include "vda5050/ConnectionState.h"
#include "Header_vda5050.h"
#include <nlohmann/json.hpp>

namespace vda5050 {
/// A message containing connection info
struct Connection {
  /// Message header
  HeaderVDA5050 header;

  /// Enum{ONLINE, OFFLINE, CONNECTIONBROKEN}
  /// ONLINE:  connection between AGV and broker is active.
  /// OFFLINE: connection between AGV and broker has gone
  ///          offline in a coordinated way.
  /// CONNECTIONBROKEN: The connection between AGV and
  ///                   broker has unexpectedly ended.
  ///                   (used in e.g. MQTT Last-Will-Message)
  ConnectionState connectionState = ConnectionState::OFFLINE;

  ///
  ///\brief Equality operator
  ///
  ///\param other the other object to compare to
  ///\return is equal?
  ///
  inline bool operator==(const Connection &other) const {
    return header == other.header && connectionState == other.connectionState;
  }

  ///
  ///\brief inequality operator
  ///
  ///\param other the other object to compare to
  ///\return is not equal?
  ///
  inline bool operator!=(const Connection &other) const { return !this->operator==(other); }
};

using json = nlohmann::json;
void to_json(json &j, const Connection &d);
void from_json(const json &j, Connection &d);

}  // namespace vda5050
#endif  // INCLUDE_VDA5050_CONNECTION_H_

// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3

#ifndef INCLUDE_VDA5050_NETWORK_H_
#define INCLUDE_VDA5050_NETWORK_H_

#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

namespace vda5050 {
struct Network {
  /// Array of Domain Name Servers (DNS) used by the vehicle.
  std::optional<std::vector<std::string>> dnsServers;

  /// Array of Network Time Protocol (NTP) servers used by the vehicle.
  std::optional<std::vector<std::string>> ntpServers;

  /// A priori assigned IP address used to communicate with the MQTT broker.
  /// Note that this IP address should not be modified/changed during operations.
  std::optional<std::string> localIpAddress;

  /// The subnet mask used in the network configuration corresponding to the local IP address.
  std::optional<std::string> netmask;

  /// The default gateway used by the vehicle, corresponding to the local IP address.
  std::optional<std::string> defaultGateway;

  ///
  ///\brief Compare two Network objects
  ///
  ///\param other Network object to compare with
  ///\return true if the objects are equal
  ///
  inline bool operator==(const Network &other) const {
    if (dnsServers != other.dnsServers) return false;
    if (ntpServers != other.ntpServers) return false;
    if (localIpAddress != other.localIpAddress) return false;
    if (netmask != other.netmask) return false;
    if (defaultGateway != other.defaultGateway) return false;
    return true;
  }

  ///
  ///\brief Compare two Network objects
  ///
  ///\param other Network object to compare with
  ///\return true if the objects are not equal
  ///
  inline bool operator!=(const Network &other) const { return !(*this == other); }
};

using json = nlohmann::json;
void to_json(json &j, const Network &d);
void from_json(const json &j, Network &d);

}  // namespace vda5050

#endif  // INCLUDE_VDA5050_NETWORK_H_

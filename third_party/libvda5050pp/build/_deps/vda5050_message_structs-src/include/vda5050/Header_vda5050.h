// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//


#ifndef INCLUDE_VDA5050_HEADER_VDA5050
#define INCLUDE_VDA5050_HEADER_VDA5050

#include <chrono>
#include <cstdint>
#include <string>

#include <nlohmann/json.hpp>

namespace vda5050 {
/// The definition of a vda5050 header sent with each message
struct HeaderVDA5050 {
  /// header ID of the message. The headerId is defined per topic and incremented by
  /// 1 with each sent(but not necessarily received) message.
  uint32_t headerId = 0;

  /// Timestamp after ISO8601 in the format YYYY-MM-DDTHH:mm:ss.ssZ
  /// (e.g.“2017-04-15T11:40:03.12Z”)
  ///
  /// Note: This structure will not be serialized directly, so
  /// a time_point of the system clock is the most convinient datatype
  std::chrono::time_point<std::chrono::system_clock> timestamp;

  /// Version of the protocol[Major].[Minor].[Patch](e.g .1.3.2)
  std::string version;

  /// Manufacturer of the AGV
  std::string manufacturer;

  /// Serial Number of the AGV
  std::string serialNumber;

  inline bool operator==(const HeaderVDA5050 &other) const {
    if (headerId != other.headerId) return false;
    // Compare epoch seconds, since the serialization's precision is seconds
    if (std::chrono::duration_cast<std::chrono::seconds>(timestamp.time_since_epoch()) !=
        std::chrono::duration_cast<std::chrono::seconds>(other.timestamp.time_since_epoch()))
      return false;
    if (version != other.version) return false;
    if (manufacturer != other.manufacturer) return false;
    if (serialNumber != other.serialNumber) return false;
    return true;
  }
  inline bool operator!=(const HeaderVDA5050 &other) const { return !this->operator==(other); }
};

void to_json(nlohmann::json &j, const HeaderVDA5050 &d);
void from_json(const nlohmann::json &j, HeaderVDA5050 &d);

}  // namespace vda5050
#endif /* INCLUDE_VDA5050_HEADER_VDA5050 */

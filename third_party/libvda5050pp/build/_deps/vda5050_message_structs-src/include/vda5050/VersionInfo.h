// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3

#ifndef INCLUDE_VDA5050_VERSIONINFO_H_
#define INCLUDE_VDA5050_VERSIONINFO_H_

#include <nlohmann/json.hpp>
#include <string>

namespace vda5050 {

struct VersionInfo {
  /// Key of the software/hardware version used. (e.g., softwareVersion)
  std::string key;

  /// The version corresponding to the key. (e.g., v1.12.4-beta)
  std::string value;

  ///
  ///\brief Compare two VersionInfo objects
  ///
  ///\param other VersionInfo object to compare with
  ///\return true if the objects are equal
  ///
  inline bool operator==(const VersionInfo &other) const {
    if (key != other.key) return false;
    if (value != other.value) return false;
    return true;
  }

  ///
  ///\brief Compare two VersionInfo objects
  ///
  ///\param other VersionInfo object to compare with
  ///\return true if the objects are not equal
  ///
  inline bool operator!=(const VersionInfo &other) const { return !(*this == other); }
};

using json = nlohmann::json;
void to_json(json &j, const VersionInfo &d);
void from_json(const json &j, VersionInfo &d);

}  // namespace vda5050

#endif  // INCLUDE_VDA5050_VERSIONINFO_H_

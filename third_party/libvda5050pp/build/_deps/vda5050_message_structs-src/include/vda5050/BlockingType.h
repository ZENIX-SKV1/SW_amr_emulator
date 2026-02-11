// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//


#ifndef INCLUDE_VDA5050_BLOCKINGTYPE_H_
#define INCLUDE_VDA5050_BLOCKINGTYPE_H_

#include <ostream>
#include <nlohmann/json.hpp>

namespace vda5050 {
/// Enums for blockingType
enum class BlockingType {
  /// “NONE” – allows driving and other actions
  NONE,
  /// “SOFT” – allows other actions, but not driving
  SOFT,
  /// “HARD” – is the only allowed action at that time.
  HARD
};

///
///\brief Write the enum-value to an ostream
///
///\param os the stream
///\param blocking_type the enum
///\return constexpr std::ostream&
///
constexpr std::ostream &operator<<(std::ostream &os, const BlockingType &blocking_type) {
  switch (blocking_type) {
    case BlockingType::NONE:
      os << "NONE";
      break;
    case BlockingType::SOFT:
      os << "SOFT";
      break;
    case BlockingType::HARD:
      os << "HARD";
      break;
    default:
      os.setstate(std::ios_base::failbit);
  }
  return os;
}

using json = nlohmann::json;
void to_json(json &j, const BlockingType &d);
void from_json(const json &j, BlockingType &d);

}  // namespace vda5050
#endif  // INCLUDE_VDA5050_BLOCKINGTYPE_H_

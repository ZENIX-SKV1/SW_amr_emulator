// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//


#ifndef INCLUDE_VDA5050_NODESTATE_H_
#define INCLUDE_VDA5050_NODESTATE_H_

#include <cstdint>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>

#include "vda5050/NodePosition.h"

namespace vda5050 {
/// VD(M)A 5050 NodeState
struct NodeState {
  /// Unique node identification
  std::string nodeId;

  /// sequenceId to discern multiple nodes with same nodeId
  uint32_t sequenceId = 0;

  /// Additional information on the node
  std::optional<std::string> nodeDescription;

  /// Node position.The object is defined in chapter 6.6
  /// Optional: MC has this information. Can be sent
  /// additionally, e. g. for debugging purposes.
  std::optional<NodePosition> nodePosition;

  /// true indicates that the node is part of the base.
  /// false indicates that the node is part of the horizon.
  bool released = false;

  ///
  ///\brief Equality operator
  ///
  ///\param other the other object to compare to
  ///\return is equal?
  ///
  inline bool operator==(const NodeState &other) const {
    if (this->nodeDescription != other.nodeDescription) return false;
    if (this->nodeId != other.nodeId) return false;
    if (this->nodePosition != other.nodePosition) return false;
    if (this->released != other.released) return false;
    if (this->sequenceId != other.sequenceId) return false;

    return true;
  }

  ///
  ///\brief Inequality operator
  ///
  ///\param other the other object to compare to
  ///\return is not equal?
  ///
  inline bool operator!=(const NodeState &other) const { return !this->operator==(other); }
};

using json = nlohmann::json;
void to_json(json &j, const NodeState &d);
void from_json(const json &j, NodeState &d);

}  // namespace vda5050
#endif  // INCLUDE_VDA5050_NODESTATE_H_

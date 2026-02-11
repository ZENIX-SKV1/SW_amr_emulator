// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//


#ifndef INCLUDE_VDA5050_NODE_H_
#define INCLUDE_VDA5050_NODE_H_

#include <cstdint>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>

#include "vda5050/Action.h"
#include "vda5050/NodePosition.h"

namespace vda5050 {
/// VD(M)A 5050 Node
struct Node {
  /// Unique node identification.
  std::string nodeId;

  /// Number to track the sequence of nodes and edges in an
  /// order and to simplify order updates. The main purpose is
  /// to distinguish between a node which is passed more than
  /// once within one orderId. The variable sequenceId runs
  /// across all nodes and edges of the same order and is reset
  /// when a new orderId is issued.
  uint32_t sequenceId = 0;

  /// Additional information on the node
  std::optional<std::string> nodeDescription;

  /// True indicates that the node is part of the base.
  /// False indicates that the node is part of the horizon.
  bool released = false;

  /// Node position Optional for vehicle-types that do not
  /// require the node position (e.g. line-guided vehicles).
  std::optional<NodePosition> nodePosition;

  /// Array of actions to be executed in node.Empty array if no
  /// actions required. An action triggered by a node will
  /// persist until changed in another node unless restricted by
  /// durationType/durationValue.
  std::vector<Action> actions;

  ///
  ///\brief Equality operator
  ///
  ///\param other the other object to compare to
  ///\return is equal?
  ///
  inline bool operator==(const Node &other) const {
    if (this->actions != other.actions) return false;
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
  inline bool operator!=(const Node &other) const { return !this->operator==(other); }
};

using json = nlohmann::json;
void to_json(json &j, const Node &d);
void from_json(const json &j, Node &d);

}  // namespace vda5050
#endif  // INCLUDE_VDA5050_NODE_H_

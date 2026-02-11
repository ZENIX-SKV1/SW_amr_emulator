// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//


#ifndef INCLUDE_VDA5050_ORDER_H_
#define INCLUDE_VDA5050_ORDER_H_

#include <cstdint>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

#include "Header_vda5050.h"
#include "vda5050/Edge.h"
#include "vda5050/Node.h"

namespace vda5050 {
/// VD(M)A 5050 Order
struct Order {
  /// Message header
  HeaderVDA5050 header;

  /// Order identification. This is to be used to identify
  /// multiple order messages that belong to the same order.
  std::string orderId;

  /// orderUpdate identification. Is unique per orderId. If an order
  /// update is rejected, this field is to be passed in the rejection
  /// message
  uint32_t orderUpdateId = 0;

  /// Unique identifier of the zone set that the AGV has to use for
  /// navigation or that was used by MC for planning.
  std::optional<std::string> zoneSetId;

  /// Array of nodes objects to be traversed for fulfilling the order.
  /// One node is enough for a valid order. Leave edge list empty for
  /// that case.
  std::vector<Node> nodes;

  /// Array of edges to be traversed for fulfilling the order. May
  /// be empty in case only one node is used for an order.
  std::vector<Edge> edges;

  ///
  ///\brief Equality operator
  ///
  ///\param other the other object to compare to
  ///\return is equal?
  ///
  inline bool operator==(const Order &other) const {
    if (this->edges != other.edges) return false;
    if (this->header != other.header) return false;
    if (this->nodes != other.nodes) return false;
    if (this->orderId != other.orderId) return false;
    if (this->orderUpdateId != other.orderUpdateId) return false;
    if (this->zoneSetId != other.zoneSetId) return false;

    return true;
  }

  ///
  ///\brief Inequality operator
  ///
  ///\param other the other object to compare to
  ///\return is not equal?
  ///
  inline bool operator!=(const Order &other) const { return !this->operator==(other); }
};

using json = nlohmann::json;
void to_json(json &j, const Order &d);
void from_json(const json &j, Order &d);

}  // namespace vda5050
#endif  // INCLUDE_VDA5050_ORDER_H_

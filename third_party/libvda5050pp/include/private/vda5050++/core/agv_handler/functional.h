//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_2B_2B_CORE_AGV_HANDLER_FUNCTIONAL_H_
#define VDA5050_2B_2B_CORE_AGV_HANDLER_FUNCTIONAL_H_

#include <vda5050/AGVPosition.h>
#include <vda5050/NodePosition.h>

#include <optional>

namespace vda5050pp::core::agv_handler {

///
///\brief Determine if the AGV is on a node.
///
/// An AGV is considered to be on a node if the deviation radius of the AGV's position
/// is fully enclosed by the Node's deviation radius.
/// If the theta deviation of the node is set, the AGV must also have it's theta inside
/// of that deviation range.
///
///\param agv the agv's position
///\param node  the node object
///\param default_deviation_xy the default deviation radius for the xy position
/// (if the node has none)
///\param default_deviation_theta the default theta deviation (if the node has none)
///\param override_deviation_xy  override the deviation radius for the xy position
///\param override_deviation_theta
///\return true iff the AGV is considered to be on the node.
///
bool isOnNode(const vda5050::AGVPosition &agv, const vda5050::NodePosition &node,
              std::optional<double> default_deviation_xy,
              std::optional<double> default_deviation_theta,
              std::optional<double> override_deviation_xy = std::nullopt,
              std::optional<double> override_deviation_theta = std::nullopt);
}  // namespace vda5050pp::core::agv_handler

#endif  // VDA5050_2B_2B_CORE_AGV_HANDLER_FUNCTIONAL_H_

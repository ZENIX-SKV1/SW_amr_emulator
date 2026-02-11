// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//


#ifndef INCLUDE_VDA5050_EDGE_H_
#define INCLUDE_VDA5050_EDGE_H_

#include <cstdint>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

#include "vda5050/Action.h"
#include "vda5050/Corridor.h"
#include "vda5050/OrientationType.h"
#include "vda5050/Trajectory.h"

namespace vda5050 {

/// Directional connection between two nodes
struct Edge {
  /// Unique edge identification
  std::string edgeId;

  /// Id to track the sequence of nodes and edges in an order and to
  /// simplify order updates.The variable sequenceId runs across
  /// all nodes and edges of the same order and is reset when a new
  /// orderId is issued.
  uint32_t sequenceId = 0;

  /// Additional information on the edge
  std::optional<std::string> edgeDescription;

  /// True indicates that the edge is part of the base.
  /// False indicates that the edge is part of the horizon.
  bool released = false;

  /// nodeID of startNode
  std::string startNodeId;

  /// nodeID of endNode
  std::string endNodeId;

  /// [m/s] Permitted maximum speed on the edge. Speed is defined
  /// by the fastest point of the vehicle.
  std::optional<double> maxSpeed;

  /// [m] Permitted maximal height of the load handling device
  /// on the edge.
  std::optional<double> maxHeight;

  /// [m] Permitted minimal height of the load handling device
  /// on the edge.
  std::optional<double> minHeight;

  /// [rad] Orientation of the AGV on the edge relative to the
  /// tangential direction of the edge (for holonomic vehicles with
  /// more than one driving direction).
  /// Example: orientation Pi/2 rad will lead to a rotation
  /// of 90 degrees.
  ///
  /// If AGV starts in different orientation, rotate the vehicle on
  /// the edge to the desired orientation if rotationAllowed is set
  /// to “true”. If rotationAllowed is “false", rotate before
  /// entering the edge. If that is not possible, reject the order.
  ///
  /// If a trajectory with orientation is defined, follow the
  /// trajectories orientation. If a trajectory without orientation
  /// and the orientation field here is defined, apply the
  /// orientation to the tangent of the trajectory.
  std::optional<double> orientation;

  /// Enum {GLOBAL, TANGENTIAL}
  /// "GLOBAL"- relative to the global project specific map coordinate system
  /// "TANGENTIAL"- tangential to the edge.
  /// If not defined, the default value is "TANGENTIAL".
  std::optional<OrientationType> orientationType;

  /// Sets direction at junctions for line-guided or wire-guided
  /// vehicles, to be defined initially (vehicle individual).
  /// Example: left, right, straight, 433MHz
  std::optional<std::string> direction;

  /// “true”: rotation is allowed on the edge.
  /// “false”: rotation is not allowed on the edge.
  /// Optional: Default to "false"
  std::optional<bool> rotationAllowed;

  /// [rad/s] Maximum rotation speed
  std::optional<double> maxRotationSpeed;

  /// Trajectory JSON-object for this edge as a NURBS.
  /// Defines the curve on which the AGV should move between
  /// startNode and endNode.
  /// Optional: Can be omitted if AGV cannot process trajectories
  /// or if AGV plans its own trajectory.
  std::optional<Trajectory> trajectory;

  /// [m] Length of the path from startNode to endNode.
  /// Optional: This value is used by line-guided AGVs to decrease
  /// their speed before reaching a stop position.
  std::optional<double> length;

  /// Definition of boundaries in which a vehicle can deviate from
  /// its trajectory, e.g., to avoid obstacles.
  std::optional<Corridor> corridor;

  /// Array of actions to be executed on the edge.
  /// An action triggered by an edge will only be active for the
  /// time that the AGV is traversing the edge which triggered
  /// the action. When the AGV leaves the edge, the action will stop
  /// and the state before entering the edge will be restored.
  std::vector<Action> actions;

  ///
  ///\brief Equality operator
  ///
  ///\param other the other object to compare to
  ///\return is equal?
  ///
  inline bool operator==(const Edge &other) const {
    if (this->actions != other.actions) return false;
    if (this->corridor != other.corridor) return false;
    if (this->direction != other.direction) return false;
    if (this->edgeDescription != other.edgeDescription) return false;
    if (this->edgeId != other.edgeId) return false;
    if (this->endNodeId != other.endNodeId) return false;
    if (this->length != other.length) return false;
    if (this->maxHeight != other.maxHeight) return false;
    if (this->maxRotationSpeed != other.maxRotationSpeed) return false;
    if (this->maxSpeed != other.maxSpeed) return false;
    if (this->minHeight != other.minHeight) return false;
    if (this->orientation != other.orientation) return false;
    if (this->orientationType != other.orientationType) return false;
    if (this->released != other.released) return false;
    if (this->rotationAllowed != other.rotationAllowed) return false;
    if (this->sequenceId != other.sequenceId) return false;
    if (this->startNodeId != other.startNodeId) return false;
    if (this->trajectory != other.trajectory) return false;

    return true;
  }

  ///
  ///\brief Inequality operator
  ///
  ///\param other the other object to compare to
  ///\return is not equal?
  ///
  inline bool operator!=(const Edge &other) const { return !this->operator==(other); }
};

using json = nlohmann::json;
void to_json(json &j, const Edge &d);
void from_json(const json &j, Edge &d);

}  // namespace vda5050
#endif  // INCLUDE_VDA5050_EDGE_H_

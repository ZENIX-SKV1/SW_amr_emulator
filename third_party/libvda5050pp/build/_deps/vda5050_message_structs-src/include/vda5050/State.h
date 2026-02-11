// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//

#ifndef INCLUDE_VDA5050_STATE_H_
#define INCLUDE_VDA5050_STATE_H_

#include <cstdint>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

#include "Header_vda5050.h"
#include "vda5050/AGVPosition.h"
#include "vda5050/ActionState.h"
#include "vda5050/BatteryState.h"
#include "vda5050/EdgeState.h"
#include "vda5050/Error.h"
#include "vda5050/Info.h"
#include "vda5050/Load.h"
#include "vda5050/Map.h"
#include "vda5050/NodeState.h"
#include "vda5050/OperatingMode.h"
#include "vda5050/SafetyState.h"
#include "vda5050/Velocity.h"

namespace vda5050 {
/// VD(M)A 5050 State
struct State {
  /// Message header
  HeaderVDA5050 header;

  /// Array of map objects that are currently stored on the vehicle.
  std::optional<std::vector<Map>> maps;

  /// Unique order identification of the current order or the
  /// previous finished order. The orderId is kept until a new
  /// order is received. Empty string ("") if no previous
  /// orderId is available.
  std::string orderId;

  /// Order Update Identification to identify that an order
  /// update has been accepted by the AGV. “0” if no previous
  /// orderUpdateId is available.
  uint32_t orderUpdateId = 0;

  /// Unique ID of the zone set that the AGV currently uses for path
  /// planning. Must be the same as the one used in the order, otherwise
  /// the AGV has to reject the order.
  /// Optional: If the AGV does not use zones,
  /// this field can be omitted.
  std::optional<std::string> zoneSetId;

  /// nodeId of last reached node or, if AGV is currently on a
  /// node, current node (e.g. „node7”).
  /// Empty string ("") if no lastNodeId is available.
  std::string lastNodeId;

  /// sequenceId of the last reached node or, if the AGV is
  /// currently on a node, sequenceId of current node.
  /// “0” if no lastNodeSequenceId is available.
  uint32_t lastNodeSequenceId = 0;

  /// Array of nodeStateObjects that need to be traversed for
  /// fulfilling the order (empty list if idle)
  std::vector<NodeState> nodeStates;

  /// Array of edgeStateObjects that need to be traversed for
  /// fulfilling the order (empty list if idle)
  std::vector<EdgeState> edgeStates;

  /// Current position of the AGV on the map.
  /// Optional: Can only be omitted for AGVs without the
  /// capability to localize themselves, e.g. line guided AGVs.
  std::optional<AGVPosition> agvPosition;

  /// AGV's velocity in vehicle coordinates
  std::optional<Velocity> velocity;

  /// Loads that are currently handled by the AGV.
  /// Optional: If AGV cannot determine load state, leave the
  /// array out of the state. If the AGV can determine the load
  /// state, but the array is empty, the AGV is considered
  /// unloaded.
  std::optional<std::vector<Load>> loads;

  /// True: indicates that the AGV is driving and/or rotating. Other
  /// movements of the AGV (e.g. lift movements) are not included here.
  /// False: indicates that the AGV is neither driving nor rotating
  bool driving = false;

  /// True: AGV is currently in a paused state, either because of the push
  /// of a physical button on the AGV or because of an instantAction.
  /// The AGV can resume the order.
  /// False: The AGV is currently not in a paused state
  std::optional<bool> paused;

  /// True: AGV is almost at the end of the base and will reduce
  /// speed if no new base is transmitted. Trigger for MC to
  /// send new base.
  /// False: no base update required
  std::optional<bool> newBaseRequest;

  /// [m] Used by line guided vehicles to indicate the distance
  /// it has been driving past the „lastNodeIdId“.
  /// Distance is in meters.
  std::optional<double> distanceSinceLastNode;

  /// Contains a list of the current actions and the actions
  /// which are yet to be finished. This may include actions
  /// from previous nodes that are still in progress. When an
  /// action is completed, an updated state message is published
  /// with actionStatus set to finished and if applicable with
  /// the corresponding resultDescription.
  /// The action state is kept until a new order is received.
  std::vector<ActionState> actionStates;

  /// Contains all battery related information.
  BatteryState batteryState;

  /// Enum {automatic, semi-automatic, manual, service, teach-in}
  /// For additional information see chapter 6.11
  OperatingMode operatingMode = OperatingMode::AUTOMATIC;

  /// Array of error - objects. All active errors of the AGV
  /// should be in the list. An empty array indicates that the
  /// AGV has no active errors.
  std::vector<Error> errors;

  /// Array of info-objects.An empty array indicates that the AGV
  /// has no information. This should only be used for visualization
  /// or debugging – it must not be used for logic in master control.
  std::optional<std::vector<Info>> information;

  /// Contains all safetyrelated information.
  SafetyState safetyState;

  ///
  ///\brief Equality operator
  ///
  ///\param other the other object to compare to
  ///\return is equal?
  ///
  inline bool operator==(const State &other) const {
    if (this->header != other.header) return false;
    if (this->maps != other.maps) return false;
    if (this->orderId != other.orderId) return false;
    if (this->orderUpdateId != other.orderUpdateId) return false;
    if (this->zoneSetId != other.zoneSetId) return false;
    if (this->lastNodeId != other.lastNodeId) return false;
    if (this->lastNodeSequenceId != other.lastNodeSequenceId) return false;
    if (this->nodeStates != other.nodeStates) return false;
    if (this->edgeStates != other.edgeStates) return false;
    if (this->agvPosition != other.agvPosition) return false;
    if (this->velocity != other.velocity) return false;
    if (this->loads != other.loads) return false;
    if (this->driving != other.driving) return false;
    if (this->paused != other.paused) return false;
    if (this->newBaseRequest != other.newBaseRequest) return false;
    if (this->distanceSinceLastNode != other.distanceSinceLastNode) return false;
    if (this->actionStates != other.actionStates) return false;
    if (this->batteryState != other.batteryState) return false;
    if (this->operatingMode != other.operatingMode) return false;
    if (this->errors != other.errors) return false;
    if (this->information != other.information) return false;
    if (this->safetyState != other.safetyState) return false;

    return true;
  }

  ///
  ///\brief Inequality operator
  ///
  ///\param other the other object to compare to
  ///\return is not equal?
  ///
  inline bool operator!=(const State &other) const { return !this->operator==(other); }
};

using json = nlohmann::json;
void to_json(json &j, const State &d);
void from_json(const json &j, State &d);

}  // namespace vda5050
#endif  // INCLUDE_VDA5050_STATE_H_

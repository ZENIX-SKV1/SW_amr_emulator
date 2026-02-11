// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_MAXARRAYLENS_H_
#define VDA5050_MAXARRAYLENS_H_

#include <cstdint>
#include <nlohmann/json.hpp>
#include <optional>

namespace vda5050 {

struct MaxArrayLens {
  /// Maximum number of nodes per order processable by the AGV.
  std::optional<uint32_t> orderNodes;

  /// Maximum number of edges per order processable by the AGV.
  std::optional<uint32_t> orderEdges;

  /// Maximum number of actions per node processable by the AGV.
  std::optional<uint32_t> nodeActions;

  /// Maximum number of actions per edge processable by the AGV.
  std::optional<uint32_t> edgeActions;

  /// Maximum number of parameters per action processable by the AGV.
  std::optional<uint32_t> actionActionsParameters;

  /// Maximum number of instant actions per message processable by the AGV.
  std::optional<uint32_t> instantActions;

  /// Maximum number of knots per trajectory processable by the AGV.
  std::optional<uint32_t> trajectoryKnotVector;

  /// Maximum number of control points per trajectory processable by the AGV.
  std::optional<uint32_t> trajectoryControlPoints;

  /// Maximum number of nodeStates sent by the AGV, maximum number of nodes in base of AGV.
  std::optional<uint32_t> stateNodeStates;

  /// Maximum number of edgeStates sent by the AGV, maximum number of edges in base of AGV.
  std::optional<uint32_t> stateEdgeStates;

  /// Maximum number of load-objects sent by the AGV.
  std::optional<uint32_t> stateLoads;

  /// Maximum number of actionStates sent by the AGV.
  std::optional<uint32_t> stateActionStates;

  /// Maximum number of errors sent by the AGV in one state-message.
  std::optional<uint32_t> stateErrors;

  /// Maximum number of informations sent by the AGV in one state-message.
  std::optional<uint32_t> stateInformations;

  /// Maximum number of error references sent by the AGV for each error.
  std::optional<uint32_t> errorErrorReferences;

  /// Maximum number of info references sent by the AGV for each information.
  std::optional<uint32_t> informationsInfoReferences;

  ///
  ///\brief Equality operator
  ///
  ///\param other the other object to compare to
  ///\return is equal?
  ///
  inline bool operator==(const MaxArrayLens &other) const {
    if (this->actionActionsParameters != other.actionActionsParameters) return false;
    if (this->edgeActions != other.edgeActions) return false;
    if (this->errorErrorReferences != other.errorErrorReferences) return false;
    if (this->informationsInfoReferences != other.informationsInfoReferences) return false;
    if (this->instantActions != other.instantActions) return false;
    if (this->nodeActions != other.nodeActions) return false;
    if (this->orderEdges != other.orderEdges) return false;
    if (this->orderNodes != other.orderNodes) return false;
    if (this->stateActionStates != other.stateActionStates) return false;
    if (this->stateEdgeStates != other.stateEdgeStates) return false;
    if (this->stateErrors != other.stateErrors) return false;
    if (this->stateInformations != other.stateInformations) return false;
    if (this->stateLoads != other.stateLoads) return false;
    if (this->stateNodeStates != other.stateNodeStates) return false;
    if (this->trajectoryControlPoints != other.trajectoryControlPoints) return false;
    if (this->trajectoryKnotVector != other.trajectoryKnotVector) return false;

    return true;
  }

  ///
  ///\brief Inequality operator
  ///
  ///\param other the other object to compare to
  ///\return is not equal?
  ///
  inline bool operator!=(const MaxArrayLens &other) const { return !this->operator==(other); }
};

using json = nlohmann::json;
void to_json(json &j, const MaxArrayLens &d);
void from_json(const json &j, MaxArrayLens &d);

}  // namespace vda5050
#endif  // VDA5050_MAXARRAYLENS_H_

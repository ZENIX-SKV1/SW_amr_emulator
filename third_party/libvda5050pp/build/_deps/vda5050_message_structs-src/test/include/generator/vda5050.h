// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//
//

#ifndef TEST_INCLUDE_GENERATOR_VDA5050_H_
#define TEST_INCLUDE_GENERATOR_VDA5050_H_

#include "vda5050/AgvFactsheet.h"
#include "vda5050/Connection.h"
#include "vda5050/InstantActions.h"
#include "vda5050/Order.h"
#include "vda5050/State.h"
#include "vda5050/Visualization.h"

namespace generator {

template <typename Action>
typename std::enable_if_t<std::is_same_v<Action, vda5050::Action>, Action> generate();

template <typename ActionParameter>
typename std::enable_if_t<std::is_same_v<ActionParameter, vda5050::ActionParameter>,
                          ActionParameter>
generate();

template <typename ActionParameterFactsheet>
typename std::enable_if_t<
    std::is_same_v<ActionParameterFactsheet, vda5050::ActionParameterFactsheet>,
    ActionParameterFactsheet>
generate();

template <typename ActionScope>
typename std::enable_if_t<std::is_same_v<ActionScope, vda5050::ActionScope>, ActionScope>
generate();

template <typename ActionState>
typename std::enable_if_t<std::is_same_v<ActionState, vda5050::ActionState>, ActionState>
generate();

template <typename ActionStatus>
typename std::enable_if_t<std::is_same_v<ActionStatus, vda5050::ActionStatus>, ActionStatus>
generate();

template <typename AgvAction>
typename std::enable_if_t<std::is_same_v<AgvAction, vda5050::AgvAction>, AgvAction> generate();

template <typename AgvFactsheet>
typename std::enable_if_t<std::is_same_v<AgvFactsheet, vda5050::AgvFactsheet>, AgvFactsheet>
generate();

template <typename AgvGeometry>
typename std::enable_if_t<std::is_same_v<AgvGeometry, vda5050::AgvGeometry>, AgvGeometry>
generate();

template <typename AGVPosition>
typename std::enable_if_t<std::is_same_v<AGVPosition, vda5050::AGVPosition>, AGVPosition>
generate();

template <typename BatteryState>
typename std::enable_if_t<std::is_same_v<BatteryState, vda5050::BatteryState>, BatteryState>
generate();

template <typename BlockingType>
typename std::enable_if_t<std::is_same_v<BlockingType, vda5050::BlockingType>, BlockingType>
generate();

template <typename BoundingBoxReference>
typename std::enable_if_t<std::is_same_v<BoundingBoxReference, vda5050::BoundingBoxReference>,
                          BoundingBoxReference>
generate();

template <typename Connection>
typename std::enable_if_t<std::is_same_v<Connection, vda5050::Connection>, Connection> generate();

template <typename ConnectionState>
typename std::enable_if_t<std::is_same_v<ConnectionState, vda5050::ConnectionState>,
                          ConnectionState>
generate();

template <typename ControlPoint>
typename std::enable_if_t<std::is_same_v<ControlPoint, vda5050::ControlPoint>, ControlPoint>
generate();

template <typename Corridor>
typename std::enable_if_t<std::is_same_v<Corridor, vda5050::Corridor>, Corridor> generate();

template <typename CorridorRefPoint>
typename std::enable_if_t<std::is_same_v<CorridorRefPoint, vda5050::CorridorRefPoint>,
                          CorridorRefPoint>
generate();

template <typename Edge>
typename std::enable_if_t<std::is_same_v<Edge, vda5050::Edge>, Edge> generate();

template <typename EdgeState>
typename std::enable_if_t<std::is_same_v<EdgeState, vda5050::EdgeState>, EdgeState> generate();

template <typename Envelope2d>
typename std::enable_if_t<std::is_same_v<Envelope2d, vda5050::Envelope2d>, Envelope2d> generate();

template <typename Envelope3d>
typename std::enable_if_t<std::is_same_v<Envelope3d, vda5050::Envelope3d>, Envelope3d> generate();

template <typename Error>
typename std::enable_if_t<std::is_same_v<Error, vda5050::Error>, Error> generate();

template <typename ErrorLevel>
typename std::enable_if_t<std::is_same_v<ErrorLevel, vda5050::ErrorLevel>, ErrorLevel> generate();

template <typename ErrorReference>
typename std::enable_if_t<std::is_same_v<ErrorReference, vda5050::ErrorReference>, ErrorReference>
generate();

template <typename EStop>
typename std::enable_if_t<std::is_same_v<EStop, vda5050::EStop>, EStop> generate();

template <typename HeaderVDA5050>
typename std::enable_if_t<std::is_same_v<HeaderVDA5050, vda5050::HeaderVDA5050>, HeaderVDA5050>
generate();

template <typename Info>
typename std::enable_if_t<std::is_same_v<Info, vda5050::Info>, Info> generate();

template <typename InfoLevel>
typename std::enable_if_t<std::is_same_v<InfoLevel, vda5050::InfoLevel>, InfoLevel> generate();

template <typename InfoReference>
typename std::enable_if_t<std::is_same_v<InfoReference, vda5050::InfoReference>, InfoReference>
generate();

template <typename InstantActions>
typename std::enable_if_t<std::is_same_v<InstantActions, vda5050::InstantActions>, InstantActions>
generate();

template <typename Json>
typename std::enable_if_t<std::is_same_v<Json, vda5050::json>, Json> generate();

template <typename Load>
typename std::enable_if_t<std::is_same_v<Load, vda5050::Load>, Load> generate();

template <typename LoadDimensions>
typename std::enable_if_t<std::is_same_v<LoadDimensions, vda5050::LoadDimensions>, LoadDimensions>
generate();

template <typename LoadSet>
typename std::enable_if_t<std::is_same_v<LoadSet, vda5050::LoadSet>, LoadSet> generate();

template <typename LoadSpecification>
typename std::enable_if_t<std::is_same_v<LoadSpecification, vda5050::LoadSpecification>,
                          LoadSpecification>
generate();

template <typename Map>
typename std::enable_if_t<std::is_same_v<Map, vda5050::Map>, Map> generate();

template <typename MapStatus>
typename std::enable_if_t<std::is_same_v<MapStatus, vda5050::MapStatus>, MapStatus> generate();

template <typename MaxArrayLens>
typename std::enable_if_t<std::is_same_v<MaxArrayLens, vda5050::MaxArrayLens>, MaxArrayLens>
generate();

template <typename MaxStringLens>
typename std::enable_if_t<std::is_same_v<MaxStringLens, vda5050::MaxStringLens>, MaxStringLens>
generate();

template <typename Network>
typename std::enable_if_t<std::is_same_v<Network, vda5050::Network>, Network> generate();

template <typename Node>
typename std::enable_if_t<std::is_same_v<Node, vda5050::Node>, Node> generate();

template <typename NodePosition>
typename std::enable_if_t<std::is_same_v<NodePosition, vda5050::NodePosition>, NodePosition>
generate();

template <typename NodeState>
typename std::enable_if_t<std::is_same_v<NodeState, vda5050::NodeState>, NodeState> generate();

template <typename OperatingMode>
typename std::enable_if_t<std::is_same_v<OperatingMode, vda5050::OperatingMode>, OperatingMode>
generate();

template <typename OptionalParameter>
typename std::enable_if_t<std::is_same_v<OptionalParameter, vda5050::OptionalParameter>,
                          OptionalParameter>
generate();

template <typename Order>
typename std::enable_if_t<std::is_same_v<Order, vda5050::Order>, Order> generate();

template <typename OrientationType>
typename std::enable_if_t<std::is_same_v<OrientationType, vda5050::OrientationType>,
                          OrientationType>
generate();

template <typename PhysicalParameters>
typename std::enable_if_t<std::is_same_v<PhysicalParameters, vda5050::PhysicalParameters>,
                          PhysicalParameters>
generate();

template <typename PolygonPoint>
typename std::enable_if_t<std::is_same_v<PolygonPoint, vda5050::PolygonPoint>, PolygonPoint>
generate();

template <typename Position>
typename std::enable_if_t<std::is_same_v<Position, vda5050::Position>, Position> generate();

template <typename ProtocolFeatures>
typename std::enable_if_t<std::is_same_v<ProtocolFeatures, vda5050::ProtocolFeatures>,
                          ProtocolFeatures>
generate();

template <typename ProtocolLimits>
typename std::enable_if_t<std::is_same_v<ProtocolLimits, vda5050::ProtocolLimits>, ProtocolLimits>
generate();

template <typename SafetyState>
typename std::enable_if_t<std::is_same_v<SafetyState, vda5050::SafetyState>, SafetyState>
generate();

template <typename State>
typename std::enable_if_t<std::is_same_v<State, vda5050::State>, State> generate();

template <typename Support>
typename std::enable_if_t<std::is_same_v<Support, vda5050::Support>, Support> generate();

template <typename Timing>
typename std::enable_if_t<std::is_same_v<Timing, vda5050::Timing>, Timing> generate();

template <typename Trajectory>
typename std::enable_if_t<std::is_same_v<Trajectory, vda5050::Trajectory>, Trajectory> generate();

template <typename TypeSpecification>
typename std::enable_if_t<std::is_same_v<TypeSpecification, vda5050::TypeSpecification>,
                          TypeSpecification>
generate();

template <typename ValueDataType>
typename std::enable_if_t<std::is_same_v<ValueDataType, vda5050::ValueDataType>, ValueDataType>
generate();

template <typename VehicleConfig>
typename std::enable_if_t<std::is_same_v<VehicleConfig, vda5050::VehicleConfig>, VehicleConfig>
generate();

template <typename Velocity>
typename std::enable_if_t<std::is_same_v<Velocity, vda5050::Velocity>, Velocity> generate();

template <typename VersionInfo>
typename std::enable_if_t<std::is_same_v<VersionInfo, vda5050::VersionInfo>, VersionInfo>
generate();

template <typename Visualization>
typename std::enable_if_t<std::is_same_v<Visualization, vda5050::Visualization>, Visualization>
generate();

template <typename WheelDefinition>
typename std::enable_if_t<std::is_same_v<WheelDefinition, vda5050::WheelDefinition>,
                          WheelDefinition>
generate();

template <typename WheelType>
typename std::enable_if_t<std::is_same_v<WheelType, vda5050::WheelType>, WheelType> generate();

}  // namespace generator

#endif  // TEST_INCLUDE_GENERATOR_VDA5050_H_

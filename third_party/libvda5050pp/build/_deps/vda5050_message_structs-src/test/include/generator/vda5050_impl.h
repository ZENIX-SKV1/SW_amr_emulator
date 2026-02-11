// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//

#ifndef TEST_INCLUDE_GENERATOR_VDA5050_IMPL_H_
#define TEST_INCLUDE_GENERATOR_VDA5050_IMPL_H_

#include "generator/vda5050.h"

namespace generator {

template <typename Action>
typename std::enable_if_t<std::is_same_v<Action, vda5050::Action>, Action> generate() {
  vda5050::Action gen;

  generate_to(gen.actionId);
  generate_to(gen.actionType);
  generate_to(gen.actionParameters);
  generate_to(gen.actionDescription);
  generate_to(gen.blockingType);

  return gen;
}

template <typename ActionParameter>
typename std::enable_if_t<std::is_same_v<ActionParameter, vda5050::ActionParameter>,
                          ActionParameter>
generate() {
  vda5050::ActionParameter gen;

  generate_to(gen.key);
  generate_to(gen.value);

  return gen;
}

template <typename ActionParameterFactsheet>
typename std::enable_if_t<
    std::is_same_v<ActionParameterFactsheet, vda5050::ActionParameterFactsheet>,
    ActionParameterFactsheet>
generate() {
  vda5050::ActionParameterFactsheet gen;

  generate_to(gen.key);
  generate_to(gen.valueDataType);
  generate_to(gen.description);
  generate_to(gen.isOptional);

  return gen;
}

template <typename ActionScope>
typename std::enable_if_t<std::is_same_v<ActionScope, vda5050::ActionScope>, ActionScope>
generate() {
  std::uniform_int_distribution dist(0, 2);

  return vda5050::ActionScope(dist(RNG::get()));
}

template <typename ActionState>
typename std::enable_if_t<std::is_same_v<ActionState, vda5050::ActionState>, ActionState>
generate() {
  vda5050::ActionState gen;

  generate_to(gen.actionId);
  generate_to(gen.actionDescription);
  generate_to(gen.actionStatus);
  generate_to(gen.actionType);
  generate_to(gen.resultDescription);

  return gen;
}

template <typename ActionStatus>
typename std::enable_if_t<std::is_same_v<ActionStatus, vda5050::ActionStatus>, ActionStatus>
generate() {
  std::uniform_int_distribution dist(0, 5);

  return vda5050::ActionStatus(dist(RNG::get()));
}

template <typename AgvAction>
typename std::enable_if_t<std::is_same_v<AgvAction, vda5050::AgvAction>, AgvAction> generate() {
  vda5050::AgvAction gen;

  generate_to(gen.actionType);
  generate_to(gen.actionDescription);
  generate_to(gen.actionScopes);
  generate_to(gen.actionParameters);
  generate_to(gen.resultDescription);
  generate_to(gen.blockingTypes);

  return gen;
}

template <typename AgvFactsheet>
typename std::enable_if_t<std::is_same_v<AgvFactsheet, vda5050::AgvFactsheet>, AgvFactsheet>
generate() {
  vda5050::AgvFactsheet gen;

  generate_to(gen.agvGeometry);
  generate_to(gen.protocolFeatures);
  generate_to(gen.header);
  generate_to(gen.loadSpecification);
  generate_to(gen.physicalParameters);
  generate_to(gen.protocolLimits);
  generate_to(gen.typeSpecification);
  generate_to(gen.vehicleConfig);

  return gen;
}

template <typename AgvGeometry>
typename std::enable_if_t<std::is_same_v<AgvGeometry, vda5050::AgvGeometry>, AgvGeometry>
generate() {
  vda5050::AgvGeometry gen;

  generate_to(gen.wheelDefinitions);
  generate_to(gen.envelopes2d);
  generate_to(gen.envelopes3d);

  return gen;
}

template <typename AGVPosition>
typename std::enable_if_t<std::is_same_v<AGVPosition, vda5050::AGVPosition>, AGVPosition>
generate() {
  vda5050::AGVPosition gen;

  generate_to(gen.deviationRange);
  generate_to(gen.localizationScore);
  generate_to(gen.mapDescription);
  generate_to(gen.mapId);
  generate_to(gen.positionInitialized);
  generate_to(gen.theta);
  generate_to(gen.x);
  generate_to(gen.y);

  return gen;
}

template <typename BatteryState>
typename std::enable_if_t<std::is_same_v<BatteryState, vda5050::BatteryState>, BatteryState>
generate() {
  vda5050::BatteryState gen;

  generate_to(gen.batteryCharge);
  generate_to(gen.batteryHealth);
  generate_to(gen.batteryVoltage);
  generate_to(gen.charging);
  generate_to(gen.reach);

  return gen;
}

template <typename BlockingType>
typename std::enable_if_t<std::is_same_v<BlockingType, vda5050::BlockingType>, BlockingType>
generate() {
  std::uniform_int_distribution dist(0, 2);

  return vda5050::BlockingType(dist(RNG::get()));
}

template <typename BoundingBoxReference>
typename std::enable_if_t<std::is_same_v<BoundingBoxReference, vda5050::BoundingBoxReference>,
                          BoundingBoxReference>
generate() {
  vda5050::BoundingBoxReference gen;

  generate_to(gen.theta);
  generate_to(gen.x);
  generate_to(gen.y);
  generate_to(gen.z);

  return gen;
}

template <typename Connection>
typename std::enable_if_t<std::is_same_v<Connection, vda5050::Connection>, Connection> generate() {
  vda5050::Connection gen;

  generate_to(gen.connectionState);
  generate_to(gen.header);

  return gen;
}

template <typename ConnectionState>
typename std::enable_if_t<std::is_same_v<ConnectionState, vda5050::ConnectionState>,
                          ConnectionState>
generate() {
  std::uniform_int_distribution dist(0, 2);

  return vda5050::ConnectionState(dist(RNG::get()));
}

template <typename Corridor>
typename std::enable_if_t<std::is_same_v<Corridor, vda5050::Corridor>, Corridor> generate() {
  vda5050::Corridor gen;

  generate_to(gen.leftWidth);
  generate_to(gen.rightWidth);
  generate_to(gen.corridorRefPoint);

  return gen;
}

template <typename CorridorRefPoint>
typename std::enable_if_t<std::is_same_v<CorridorRefPoint, vda5050::CorridorRefPoint>,
                          CorridorRefPoint>
generate() {
  std::uniform_int_distribution dist(0, 1);

  return vda5050::CorridorRefPoint(dist(RNG::get()));
}

template <typename ControlPoint>
typename std::enable_if_t<std::is_same_v<ControlPoint, vda5050::ControlPoint>, ControlPoint>
generate() {
  vda5050::ControlPoint gen;

  generate_to(gen.weight);
  generate_to(gen.x);
  generate_to(gen.y);

  return gen;
}

template <typename Edge>
typename std::enable_if_t<std::is_same_v<Edge, vda5050::Edge>, Edge> generate() {
  vda5050::Edge gen;

  generate_to(gen.actions);
  generate_to(gen.direction);
  generate_to(gen.edgeDescription);
  generate_to(gen.edgeId);
  generate_to(gen.endNodeId);
  generate_to(gen.length);
  generate_to(gen.maxHeight);
  generate_to(gen.maxRotationSpeed);
  generate_to(gen.maxSpeed);
  generate_to(gen.minHeight);
  generate_to(gen.orientation);
  generate_to(gen.orientationType);
  generate_to(gen.released);
  generate_to(gen.rotationAllowed);
  generate_to(gen.sequenceId);
  generate_to(gen.startNodeId);
  generate_to(gen.trajectory);

  return gen;
}

template <typename EdgeState>
typename std::enable_if_t<std::is_same_v<EdgeState, vda5050::EdgeState>, EdgeState> generate() {
  vda5050::EdgeState gen;

  generate_to(gen.edgeDescription);
  generate_to(gen.edgeId);
  generate_to(gen.released);
  generate_to(gen.sequenceId);
  generate_to(gen.trajectory);

  return gen;
}

template <typename Envelope2d>
typename std::enable_if_t<std::is_same_v<Envelope2d, vda5050::Envelope2d>, Envelope2d> generate() {
  vda5050::Envelope2d gen;

  generate_to(gen.description);
  generate_to(gen.polygonPoints);
  generate_to(gen.set);

  return gen;
}

template <typename Envelope3d>
typename std::enable_if_t<std::is_same_v<Envelope3d, vda5050::Envelope3d>, Envelope3d> generate() {
  vda5050::Envelope3d gen;

  generate_to(gen.data);
  generate_to(gen.description);
  generate_to(gen.format);
  generate_to(gen.set);
  generate_to(gen.url);

  return gen;
}

template <typename Error>
typename std::enable_if_t<std::is_same_v<Error, vda5050::Error>, Error> generate() {
  vda5050::Error gen;

  generate_to(gen.errorDescription);
  generate_to(gen.errorLevel);
  generate_to(gen.errorReferences);
  generate_to(gen.errorType);

  return gen;
}

template <typename ErrorLevel>
typename std::enable_if_t<std::is_same_v<ErrorLevel, vda5050::ErrorLevel>, ErrorLevel> generate() {
  std::uniform_int_distribution dist(0, 1);

  return vda5050::ErrorLevel(dist(RNG::get()));
}

template <typename ErrorReference>
typename std::enable_if_t<std::is_same_v<ErrorReference, vda5050::ErrorReference>, ErrorReference>
generate() {
  vda5050::ErrorReference gen;

  generate_to(gen.referenceKey);
  generate_to(gen.referenceValue);

  return gen;
}

template <typename EStop>
typename std::enable_if_t<std::is_same_v<EStop, vda5050::EStop>, EStop> generate() {
  std::uniform_int_distribution dist(0, 3);

  return vda5050::EStop(dist(RNG::get()));
}

template <typename HeaderVDA5050>
typename std::enable_if_t<std::is_same_v<HeaderVDA5050, vda5050::HeaderVDA5050>, HeaderVDA5050>
generate() {
  vda5050::HeaderVDA5050 gen;

  generate_to(gen.headerId);
  generate_to(gen.manufacturer);
  generate_to(gen.serialNumber);
  generate_to(gen.timestamp);
  generate_to(gen.version);

  return gen;
}

template <typename Info>
typename std::enable_if_t<std::is_same_v<Info, vda5050::Info>, Info> generate() {
  vda5050::Info gen;

  generate_to(gen.infoDescription);
  generate_to(gen.infoLevel);
  generate_to(gen.infoReferences);
  generate_to(gen.infoType);

  return gen;
}

template <typename InfoLevel>
typename std::enable_if_t<std::is_same_v<InfoLevel, vda5050::InfoLevel>, InfoLevel> generate() {
  std::uniform_int_distribution dist(0, 1);

  return vda5050::InfoLevel(dist(RNG::get()));
}

template <typename InfoReference>
typename std::enable_if_t<std::is_same_v<InfoReference, vda5050::InfoReference>, InfoReference>
generate() {
  vda5050::InfoReference gen;

  generate_to(gen.referenceKey);
  generate_to(gen.referenceValue);

  return gen;
}

template <typename InstantActions>
typename std::enable_if_t<std::is_same_v<InstantActions, vda5050::InstantActions>, InstantActions>
generate() {
  vda5050::InstantActions gen;

  generate_to(gen.actions);
  generate_to(gen.header);

  return gen;
}

template <typename Json>
typename std::enable_if_t<std::is_same_v<Json, vda5050::json>, Json> generate() {
  vda5050::json gen;

  gen["int"] = generate<int>();
  gen["string"] = generate<std::string>();
  gen["bool"] = generate<bool>();
  gen["array"] = generate<std::vector<int>>();

  // Arbitrary nesting
  static std::bernoulli_distribution dist(0.66);
  if (dist(RNG::get())) {
    gen["object"] = generate<vda5050::json>();
  }

  return gen;
}

template <typename Load>
typename std::enable_if_t<std::is_same_v<Load, vda5050::Load>, Load> generate() {
  vda5050::Load gen;

  generate_to(gen.boundingBoxReference);
  generate_to(gen.loadDimensions);
  generate_to(gen.loadId);
  generate_to(gen.loadPosition);
  generate_to(gen.loadType);
  generate_to(gen.weight);

  return gen;
}

template <typename LoadDimensions>
typename std::enable_if_t<std::is_same_v<LoadDimensions, vda5050::LoadDimensions>, LoadDimensions>
generate() {
  vda5050::LoadDimensions gen;

  generate_to(gen.height);
  generate_to(gen.length);
  generate_to(gen.width);

  return gen;
}

template <typename LoadSet>
typename std::enable_if_t<std::is_same_v<LoadSet, vda5050::LoadSet>, LoadSet> generate() {
  vda5050::LoadSet gen;

  generate_to(gen.agvAccelerationLimit);
  generate_to(gen.agvDecelerationLimit);
  generate_to(gen.agvSpeedLimit);
  generate_to(gen.boundingBoxReference);
  generate_to(gen.description);
  generate_to(gen.dropTime);
  generate_to(gen.loadDimensions);
  generate_to(gen.loadPositions);
  generate_to(gen.loadType);
  generate_to(gen.maxLoadhandlingDepth);
  generate_to(gen.maxLoadhandlingHeight);
  generate_to(gen.maxLoadhandlingTilt);
  generate_to(gen.maxWeight);
  generate_to(gen.minLoadhandlingDepth);
  generate_to(gen.minLoadhandlingHeight);
  generate_to(gen.minLoadhandlingTilt);
  generate_to(gen.pickTime);
  generate_to(gen.setName);

  return gen;
}

template <typename LoadSpecification>
typename std::enable_if_t<std::is_same_v<LoadSpecification, vda5050::LoadSpecification>,
                          LoadSpecification>
generate() {
  vda5050::LoadSpecification gen;

  generate_to(gen.loadPositions);
  generate_to(gen.loadSets);

  return gen;
}

template <typename Map>
typename std::enable_if_t<std::is_same_v<Map, vda5050::Map>, Map> generate() {
  vda5050::Map gen;

  generate_to(gen.mapId);
  generate_to(gen.mapVersion);
  generate_to(gen.mapDescription);
  generate_to(gen.mapStatus);

  return gen;
}

template <typename MapStatus>
typename std::enable_if_t<std::is_same_v<MapStatus, vda5050::MapStatus>, MapStatus> generate() {
  std::uniform_int_distribution dist(0, 1);

  return vda5050::MapStatus(dist(RNG::get()));
}

template <typename MaxArrayLens>
typename std::enable_if_t<std::is_same_v<MaxArrayLens, vda5050::MaxArrayLens>, MaxArrayLens>
generate() {
  vda5050::MaxArrayLens gen;

  generate_to(gen.actionActionsParameters);
  generate_to(gen.edgeActions);
  generate_to(gen.errorErrorReferences);
  generate_to(gen.informationsInfoReferences);
  generate_to(gen.instantActions);
  generate_to(gen.nodeActions);
  generate_to(gen.orderEdges);
  generate_to(gen.orderNodes);
  generate_to(gen.stateActionStates);
  generate_to(gen.stateEdgeStates);
  generate_to(gen.stateErrors);
  generate_to(gen.stateInformations);
  generate_to(gen.stateLoads);
  generate_to(gen.stateNodeStates);
  generate_to(gen.trajectoryControlPoints);
  generate_to(gen.trajectoryKnotVector);

  return gen;
}

template <typename MaxStringLens>
typename std::enable_if_t<std::is_same_v<MaxStringLens, vda5050::MaxStringLens>, MaxStringLens>
generate() {
  vda5050::MaxStringLens gen;

  generate_to(gen.enumLen);
  generate_to(gen.idLen);
  generate_to(gen.idNumericalOnly);
  generate_to(gen.loadIdLen);
  generate_to(gen.msgLen);
  generate_to(gen.topicElemLen);
  generate_to(gen.topicSerialLen);

  return gen;
}

template <typename Network>
typename std::enable_if_t<std::is_same_v<Network, vda5050::Network>, Network> generate() {
  vda5050::Network gen;

  generate_to(gen.dnsServers);
  generate_to(gen.ntpServers);
  generate_to(gen.localIpAddress);
  generate_to(gen.netmask);
  generate_to(gen.defaultGateway);

  return gen;
}

template <typename Node>
typename std::enable_if_t<std::is_same_v<Node, vda5050::Node>, Node> generate() {
  vda5050::Node gen;

  generate_to(gen.actions);
  generate_to(gen.nodeDescription);
  generate_to(gen.nodeId);
  generate_to(gen.nodePosition);
  generate_to(gen.released);
  generate_to(gen.sequenceId);

  return gen;
}

template <typename NodePosition>
typename std::enable_if_t<std::is_same_v<NodePosition, vda5050::NodePosition>, NodePosition>
generate() {
  vda5050::NodePosition gen;

  generate_to(gen.allowedDeviationTheta);
  generate_to(gen.allowedDeviationXY);
  generate_to(gen.mapDescription);
  generate_to(gen.mapId);
  generate_to(gen.theta);
  generate_to(gen.x);
  generate_to(gen.y);

  return gen;
}

template <typename NodeState>
typename std::enable_if_t<std::is_same_v<NodeState, vda5050::NodeState>, NodeState> generate() {
  vda5050::NodeState gen;

  generate_to(gen.nodeDescription);
  generate_to(gen.nodeId);
  generate_to(gen.nodePosition);
  generate_to(gen.released);
  generate_to(gen.sequenceId);

  return gen;
}

template <typename OperatingMode>
typename std::enable_if_t<std::is_same_v<OperatingMode, vda5050::OperatingMode>, OperatingMode>
generate() {
  std::uniform_int_distribution dist(0, 4);

  return vda5050::OperatingMode(dist(RNG::get()));
}

template <typename OptionalParameter>
typename std::enable_if_t<std::is_same_v<OptionalParameter, vda5050::OptionalParameter>,
                          OptionalParameter>
generate() {
  vda5050::OptionalParameter gen;

  generate_to(gen.description);
  generate_to(gen.parameter);
  generate_to(gen.support);

  return gen;
}

template <typename Order>
typename std::enable_if_t<std::is_same_v<Order, vda5050::Order>, Order> generate() {
  vda5050::Order gen;

  generate_to(gen.edges);
  generate_to(gen.header);
  generate_to(gen.nodes);
  generate_to(gen.orderId);
  generate_to(gen.orderUpdateId);
  generate_to(gen.zoneSetId);

  return gen;
}

template <typename OrientationType>
typename std::enable_if_t<std::is_same_v<OrientationType, vda5050::OrientationType>,
                          OrientationType>
generate() {
  std::uniform_int_distribution dist(0, 1);

  return vda5050::OrientationType(dist(RNG::get()));
}

template <typename PhysicalParameters>
typename std::enable_if_t<std::is_same_v<PhysicalParameters, vda5050::PhysicalParameters>,
                          PhysicalParameters>
generate() {
  vda5050::PhysicalParameters gen;

  generate_to(gen.accelerationMax);
  generate_to(gen.angularSpeedMax);
  generate_to(gen.angularSpeedMin);
  generate_to(gen.decelerationMax);
  generate_to(gen.heightMax);
  generate_to(gen.heightMin);
  generate_to(gen.length);
  generate_to(gen.speedMax);
  generate_to(gen.speedMin);
  generate_to(gen.width);

  return gen;
}

template <typename PolygonPoint>
typename std::enable_if_t<std::is_same_v<PolygonPoint, vda5050::PolygonPoint>, PolygonPoint>
generate() {
  vda5050::PolygonPoint gen;

  generate_to(gen.x);
  generate_to(gen.y);

  return gen;
}

template <typename Position>
typename std::enable_if_t<std::is_same_v<Position, vda5050::Position>, Position> generate() {
  vda5050::Position gen;

  generate_to(gen.theta);
  generate_to(gen.x);
  generate_to(gen.y);

  return gen;
}

template <typename ProtocolFeatures>
typename std::enable_if_t<std::is_same_v<ProtocolFeatures, vda5050::ProtocolFeatures>,
                          ProtocolFeatures>
generate() {
  vda5050::ProtocolFeatures gen;

  generate_to(gen.agvActions);
  generate_to(gen.optionalParameters);

  return gen;
}

template <typename ProtocolLimits>
typename std::enable_if_t<std::is_same_v<ProtocolLimits, vda5050::ProtocolLimits>, ProtocolLimits>
generate() {
  vda5050::ProtocolLimits gen;

  generate_to(gen.maxArrayLens);
  generate_to(gen.maxStringLens);
  generate_to(gen.timing);

  return gen;
}

template <typename SafetyState>
typename std::enable_if_t<std::is_same_v<SafetyState, vda5050::SafetyState>, SafetyState>
generate() {
  vda5050::SafetyState gen;

  generate_to(gen.eStop);
  generate_to(gen.fieldViolation);

  return gen;
}

template <typename State>
typename std::enable_if_t<std::is_same_v<State, vda5050::State>, State> generate() {
  vda5050::State gen;

  generate_to(gen.actionStates);
  generate_to(gen.agvPosition);
  generate_to(gen.batteryState);
  generate_to(gen.distanceSinceLastNode);
  generate_to(gen.driving);
  generate_to(gen.edgeStates);
  generate_to(gen.errors);
  generate_to(gen.header);
  generate_to(gen.information);
  generate_to(gen.lastNodeId);
  generate_to(gen.lastNodeSequenceId);
  generate_to(gen.loads);
  generate_to(gen.newBaseRequest);
  generate_to(gen.nodeStates);
  generate_to(gen.operatingMode);
  generate_to(gen.orderId);
  generate_to(gen.orderUpdateId);
  generate_to(gen.paused);
  generate_to(gen.safetyState);
  generate_to(gen.velocity);
  generate_to(gen.zoneSetId);

  return gen;
}

template <typename Support>
typename std::enable_if_t<std::is_same_v<Support, vda5050::Support>, Support> generate() {
  std::uniform_int_distribution dist(0, 1);

  return vda5050::Support(dist(RNG::get()));
}

template <typename Timing>
typename std::enable_if_t<std::is_same_v<Timing, vda5050::Timing>, Timing> generate() {
  vda5050::Timing gen;

  generate_to(gen.defaultStateInterval);
  generate_to(gen.minOrderInterval);
  generate_to(gen.minStateInterval);
  generate_to(gen.visualizationInterval);

  return gen;
}

template <typename Trajectory>
typename std::enable_if_t<std::is_same_v<Trajectory, vda5050::Trajectory>, Trajectory> generate() {
  vda5050::Trajectory gen;

  generate_to(gen.controlPoints);
  generate_to(gen.degree);
  generate_to(gen.knotVector);

  return gen;
}

template <typename TypeSpecification>
typename std::enable_if_t<std::is_same_v<TypeSpecification, vda5050::TypeSpecification>,
                          TypeSpecification>
generate() {
  vda5050::TypeSpecification gen;

  generate_to(gen.agvClass);
  generate_to(gen.agvKinematic);
  generate_to(gen.localizationTypes);
  generate_to(gen.maxLoadMass);
  generate_to(gen.navigationTypes);
  generate_to(gen.seriesDescription);
  generate_to(gen.seriesName);

  return gen;
}

template <typename ValueDataType>
typename std::enable_if_t<std::is_same_v<ValueDataType, vda5050::ValueDataType>, ValueDataType>
generate() {
  std::uniform_int_distribution dist(0, 6);

  return vda5050::ValueDataType(dist(RNG::get()));
}

template <typename VehicleConfig>
typename std::enable_if_t<std::is_same_v<VehicleConfig, vda5050::VehicleConfig>, VehicleConfig>
generate() {
  vda5050::VehicleConfig gen;

  generate_to(gen.versions);
  generate_to(gen.network);

  return gen;
}

template <typename Velocity>
typename std::enable_if_t<std::is_same_v<Velocity, vda5050::Velocity>, Velocity> generate() {
  vda5050::Velocity gen;

  generate_to(gen.omega);
  generate_to(gen.vx);
  generate_to(gen.vy);

  return gen;
}

template <typename VersionInfo>
typename std::enable_if_t<std::is_same_v<VersionInfo, vda5050::VersionInfo>, VersionInfo>
generate() {
  vda5050::VersionInfo gen;

  generate_to(gen.key);
  generate_to(gen.value);

  return gen;
}

template <typename Visualization>
typename std::enable_if_t<std::is_same_v<Visualization, vda5050::Visualization>, Visualization>
generate() {
  vda5050::Visualization gen;

  generate_to(gen.agvPosition);
  generate_to(gen.header);
  generate_to(gen.velocity);

  return gen;
}

template <typename WheelDefinition>
typename std::enable_if_t<std::is_same_v<WheelDefinition, vda5050::WheelDefinition>,
                          WheelDefinition>
generate() {
  vda5050::WheelDefinition gen;

  generate_to(gen.centerDisplacement);
  generate_to(gen.constraints);
  generate_to(gen.diameter);
  generate_to(gen.isActiveDriven);
  generate_to(gen.isActiveSteered);
  generate_to(gen.position);
  generate_to(gen.type);
  generate_to(gen.width);

  return gen;
}

template <typename WheelType>
typename std::enable_if_t<std::is_same_v<WheelType, vda5050::WheelType>, WheelType> generate() {
  std::uniform_int_distribution dist(0, 3);

  return vda5050::WheelType(dist(RNG::get()));
}

}  // namespace generator

#endif  // TEST_INCLUDE_GENERATOR_VDA5050_IMPL_H_

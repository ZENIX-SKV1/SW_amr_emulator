// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//
//

#include <catch2/catch_all.hpp>
#include <iostream>

#include "generator.h"

template <typename T> void testSerialization(size_t num = 100) {
  std::vector<T> vec(num);

  std::generate(vec.begin(), vec.end(), generator::generate<T>);

  SECTION("Serialization: " + std::string(typeid(T).name())) {
    for (const auto &x : vec) {
      auto ser = nlohmann::json(x).dump();
      T y = nlohmann::json::parse(ser);
      REQUIRE(x == y);
    }
  }
}

template <typename T> void testEquality(size_t num = 100) {
  std::vector<T> vec(num);

  std::generate(vec.begin(), vec.end(), generator::generate<T>);

  SECTION("Equality: " + std::string(typeid(T).name())) {
    for (const auto &x : vec) {
      auto y = x;
      REQUIRE(x == y);
      REQUIRE_FALSE(x != y);
    }
  }
}

TEST_CASE("vda5050_message_structs - serialization", "[vda5050_message_structs][serialization]") {
  generator::RNG::seed(Catch::rngSeed());
  testSerialization<vda5050::AGVPosition>();
  testSerialization<vda5050::Action>();
  testSerialization<vda5050::ActionParameter>();
  testSerialization<vda5050::ActionParameterFactsheet>();
  testSerialization<vda5050::ActionScope>();
  testSerialization<vda5050::ActionState>();
  testSerialization<vda5050::ActionStatus>();
  testSerialization<vda5050::AgvAction>();
  testSerialization<vda5050::AgvFactsheet>();
  testSerialization<vda5050::AgvGeometry>();
  testSerialization<vda5050::BatteryState>();
  testSerialization<vda5050::BlockingType>();
  testSerialization<vda5050::BoundingBoxReference>();
  testSerialization<vda5050::Connection>();
  testSerialization<vda5050::ConnectionState>();
  testSerialization<vda5050::ControlPoint>();
  testSerialization<vda5050::Corridor>();
  testSerialization<vda5050::CorridorRefPoint>();
  testSerialization<vda5050::EStop>();
  testSerialization<vda5050::Edge>();
  testSerialization<vda5050::EdgeState>();
  testSerialization<vda5050::Envelope2d>();
  testSerialization<vda5050::Envelope3d>();
  testSerialization<vda5050::Error>();
  testSerialization<vda5050::ErrorLevel>();
  testSerialization<vda5050::ErrorReference>();
  testSerialization<vda5050::HeaderVDA5050>();
  testSerialization<vda5050::Info>();
  testSerialization<vda5050::InfoLevel>();
  testSerialization<vda5050::InfoReference>();
  testSerialization<vda5050::InstantActions>();
  testSerialization<vda5050::Load>();
  testSerialization<vda5050::LoadDimensions>();
  testSerialization<vda5050::LoadSet>();
  testSerialization<vda5050::LoadSpecification>();
  testSerialization<vda5050::Map>();
  testSerialization<vda5050::MapStatus>();
  testSerialization<vda5050::MaxArrayLens>();
  testSerialization<vda5050::MaxStringLens>();
  testSerialization<vda5050::Network>();
  testSerialization<vda5050::Node>();
  testSerialization<vda5050::NodePosition>();
  testSerialization<vda5050::NodeState>();
  testSerialization<vda5050::OperatingMode>();
  testSerialization<vda5050::OptionalParameter>();
  testSerialization<vda5050::Order>();
  testSerialization<vda5050::OrientationType>();
  testSerialization<vda5050::PhysicalParameters>();
  testSerialization<vda5050::PolygonPoint>();
  testSerialization<vda5050::Position>();
  testSerialization<vda5050::ProtocolFeatures>();
  testSerialization<vda5050::ProtocolLimits>();
  testSerialization<vda5050::SafetyState>();
  testSerialization<vda5050::State>();
  testSerialization<vda5050::Support>();
  testSerialization<vda5050::Timing>();
  testSerialization<vda5050::Trajectory>();
  testSerialization<vda5050::TypeSpecification>();
  testSerialization<vda5050::ValueDataType>();
  testSerialization<vda5050::VehicleConfig>();
  testSerialization<vda5050::Velocity>();
  testSerialization<vda5050::VersionInfo>();
  testSerialization<vda5050::Visualization>();
  testSerialization<vda5050::WheelDefinition>();
  testSerialization<vda5050::WheelType>();
}

TEST_CASE("vda5050_message_structs - equality", "[vda5050_message_structs][equality]") {
  generator::RNG::seed(Catch::rngSeed());
  testEquality<vda5050::AGVPosition>();
  testEquality<vda5050::Action>();
  testEquality<vda5050::ActionParameter>();
  testEquality<vda5050::ActionParameterFactsheet>();
  testEquality<vda5050::ActionScope>();
  testEquality<vda5050::ActionState>();
  testEquality<vda5050::ActionStatus>();
  testEquality<vda5050::AgvAction>();
  testEquality<vda5050::AgvFactsheet>();
  testEquality<vda5050::AgvGeometry>();
  testEquality<vda5050::BatteryState>();
  testEquality<vda5050::BlockingType>();
  testEquality<vda5050::BoundingBoxReference>();
  testEquality<vda5050::Connection>();
  testEquality<vda5050::ConnectionState>();
  testEquality<vda5050::ControlPoint>();
  testEquality<vda5050::Corridor>();
  testEquality<vda5050::CorridorRefPoint>();
  testEquality<vda5050::EStop>();
  testEquality<vda5050::Edge>();
  testEquality<vda5050::EdgeState>();
  testEquality<vda5050::Envelope2d>();
  testEquality<vda5050::Envelope3d>();
  testEquality<vda5050::Error>();
  testEquality<vda5050::ErrorLevel>();
  testEquality<vda5050::ErrorReference>();
  testEquality<vda5050::HeaderVDA5050>();
  testEquality<vda5050::Info>();
  testEquality<vda5050::InfoLevel>();
  testEquality<vda5050::InfoReference>();
  testEquality<vda5050::InstantActions>();
  testEquality<vda5050::Load>();
  testEquality<vda5050::LoadDimensions>();
  testEquality<vda5050::LoadSet>();
  testEquality<vda5050::LoadSpecification>();
  testEquality<vda5050::Map>();
  testEquality<vda5050::MapStatus>();
  testEquality<vda5050::MaxArrayLens>();
  testEquality<vda5050::MaxStringLens>();
  testEquality<vda5050::Network>();
  testEquality<vda5050::Node>();
  testEquality<vda5050::NodePosition>();
  testEquality<vda5050::NodeState>();
  testEquality<vda5050::OperatingMode>();
  testEquality<vda5050::OptionalParameter>();
  testEquality<vda5050::Order>();
  testEquality<vda5050::OrientationType>();
  testEquality<vda5050::PhysicalParameters>();
  testEquality<vda5050::PolygonPoint>();
  testEquality<vda5050::Position>();
  testEquality<vda5050::ProtocolFeatures>();
  testEquality<vda5050::ProtocolLimits>();
  testEquality<vda5050::SafetyState>();
  testEquality<vda5050::State>();
  testEquality<vda5050::Support>();
  testEquality<vda5050::Timing>();
  testEquality<vda5050::Trajectory>();
  testEquality<vda5050::TypeSpecification>();
  testEquality<vda5050::ValueDataType>();
  testEquality<vda5050::VehicleConfig>();
  testEquality<vda5050::Velocity>();
  testEquality<vda5050::VersionInfo>();
  testEquality<vda5050::Visualization>();
  testEquality<vda5050::WheelDefinition>();
  testEquality<vda5050::WheelType>();
}

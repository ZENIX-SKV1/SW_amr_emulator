// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//

#include "vda5050++/core/factsheet/gather.h"

#include <catch2/catch_all.hpp>

#include "vda5050++/core/instance.h"

TEST_CASE("core::factsheet::gather", "[core][factsheet]") {
  vda5050pp::Config cfg;
  cfg.refGlobalConfig().useWhiteList();
  cfg.refGlobalConfig().bwListModule(vda5050pp::core::module_keys::k_factsheet_event_handler_key);
  cfg.refGlobalConfig().bwListModule(vda5050pp::core::module_keys::k_interpreter_event_handler_key);
  cfg.refGlobalConfig().setLogLevel(vda5050pp::config::LogLevel::k_debug);

  // Set arbitrary non-defaults
  cfg.refAgvDescription().agv_geometry.envelopes2d = std::vector<vda5050::Envelope2d>();
  cfg.refAgvDescription().agv_geometry.envelopes3d = std::vector<vda5050::Envelope3d>();
  cfg.refAgvDescription().agv_geometry.wheelDefinitions = std::vector<vda5050::WheelDefinition>();
  cfg.refAgvDescription().load_specification.loadPositions = {"a", "b", "c"};
  cfg.refAgvDescription().load_specification.loadSets = std::vector<vda5050::LoadSet>();
  cfg.refAgvDescription().physical_parameters.accelerationMax = 1.0;
  cfg.refAgvDescription().physical_parameters.heightMax = 100.0;
  cfg.refAgvDescription().type_specification.agvClass = "class";
  cfg.refAgvDescription().type_specification.seriesName = "series_name";
  cfg.refAgvDescription().user_versions = {
      vda5050::VersionInfo{
          "a",
          "1",
      },
  };
  cfg.refAgvDescription().network.defaultGateway = "gateway";
  cfg.refAgvDescription().network.dnsServers = {{"dns1", "dns2"}};

  vda5050pp::core::Instance::reset();
  auto instance = vda5050pp::core::Instance::init(cfg).lock();

  SECTION("The AGVGeometry from the cfg is used") {
    REQUIRE(cfg.getAgvDescription().agv_geometry == vda5050pp::core::factsheet::gatherGeometry());
  }

  SECTION("The LoadSpecification from the cfg is used") {
    REQUIRE(cfg.getAgvDescription().load_specification ==
            vda5050pp::core::factsheet::gatherLoadSpecification());
  }

  SECTION("The PhysicalParameters from the cfg is used") {
    REQUIRE(cfg.getAgvDescription().physical_parameters ==
            vda5050pp::core::factsheet::gatherPhysicalParameters());
  }

  SECTION("The ProtocolFeatures from the supported actions are used") {
    auto sub = instance->getActionEventManager().getScopedActionEventSubscriber();
    sub.subscribe([](std::shared_ptr<vda5050pp::events::ActionList> evt) {
      auto tkn = evt->acquireResultToken();
      auto actions = std::make_shared<std::list<vda5050::AgvAction>>();
      actions->push_back(vda5050::AgvAction{});
      actions->push_back(vda5050::AgvAction{});
      actions->push_back(vda5050::AgvAction{});
      tkn.setValue(actions);
    });

    auto features = vda5050pp::core::factsheet::gatherProtocolFeatures();
    REQUIRE(features.agvActions.size() == 8);  // 3 custom + 5 built-in
  }

  SECTION("The Protocol limits are returned without an exception") {
    REQUIRE_NOTHROW(vda5050pp::core::factsheet::gatherProtocolLimits());
  }

  SECTION("The TypeSpecification from the cfg is used") {
    REQUIRE(cfg.getAgvDescription().type_specification ==
            vda5050pp::core::factsheet::gatherTypeSpecification());
  }

  SECTION("The Vehicle config from the cfg is used") {
    REQUIRE(cfg.getAgvDescription().network ==
            vda5050pp::core::factsheet::gatherVehicleConfig().network);
    auto versions = vda5050pp::core::factsheet::gatherVehicleConfig().versions;
    auto it = std::find_if(versions->begin(), versions->end(), [](const vda5050::VersionInfo &vi) {
      return vi.key == "a" && vi.value == "1";
    });
    REQUIRE(it != versions->end());
  }
}

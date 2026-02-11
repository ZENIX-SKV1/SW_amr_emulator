// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//

#include "vda5050++/core/state/map_manager.h"

#include <catch2/catch_all.hpp>

static vda5050::State getDump(const vda5050pp::core::state::MapManager &map_manager) {
  vda5050::State state;
  map_manager.dumpTo(state);
  return state;
}

TEST_CASE("core::state::MapManager - behavior", "[core][state]") {
  vda5050pp::core::state::MapManager map_manager;

  vda5050::Map map11{
      "map1",
      "1",
      "Map 11",
      vda5050::MapStatus::DISABLED,
  };

  vda5050::Map map12{
      "map1",
      "2",
      "Map 12",
      vda5050::MapStatus::DISABLED,
  };

  vda5050::Map map21{
      "map2",
      "1",
      "Map 21",
      vda5050::MapStatus::DISABLED,
  };

  vda5050::Map map22{
      "map2",
      "2",
      "Map 22",
      vda5050::MapStatus::DISABLED,
  };

  WHEN("The map manager is empty") {
    THEN("Delete map does not throw") { REQUIRE_NOTHROW(map_manager.deleteMap("map1", "1.0.0")); }
    THEN("Get map returns nullopt") { REQUIRE(map_manager.getMap("map1") == std::nullopt); }
    THEN("Has map returns false") { REQUIRE_FALSE(map_manager.hasMap("map1", "1.0.0")); }
    THEN("Enable map does not throw") { REQUIRE_NOTHROW(map_manager.enableMap("map1", "1.0.0")); }
    THEN("The dumped state map field is unset") {
      auto dump = getDump(map_manager);
      REQUIRE(dump.maps.has_value());
      REQUIRE(dump.maps->empty());
    }
  }

  WHEN("Adding maps") {
    map_manager.addMap(map11);
    map_manager.addMap(map12);
    map_manager.addMap(map21);
    map_manager.addMap(map22);

    WHEN("No map is enabled") {
      THEN("Getting maps returns nullopt") {
        REQUIRE(map_manager.getMap("map1") == std::nullopt);
        REQUIRE(map_manager.getMap("map2") == std::nullopt);
      }
      THEN("Has map returns true") {
        REQUIRE(map_manager.hasMap("map1"));
        REQUIRE(map_manager.hasMap("map2"));
        REQUIRE(map_manager.hasMap("map1", "1"));
        REQUIRE(map_manager.hasMap("map2", "2"));
      }
    }
    WHEN("A version of a map is enabled") {
      map_manager.enableMap("map1", "2");
      map_manager.enableMap("map2", "1");

      THEN("Getting maps returns the correct map") {
        REQUIRE(map_manager.getMap("map1") != std::nullopt);
        REQUIRE(map_manager.getMap("map2") != std::nullopt);
        REQUIRE(map_manager.getMap("map1")->mapDescription == "Map 12");
        REQUIRE(map_manager.getMap("map2")->mapDescription == "Map 21");
      }
      WHEN("Deleting an enabled map") {
        map_manager.deleteMap("map1", "2");

        THEN("Getting maps returns the correct map") {
          REQUIRE(map_manager.getMap("map1") == std::nullopt);
          REQUIRE(map_manager.getMap("map2") != std::nullopt);
          REQUIRE(map_manager.getMap("map2")->mapDescription == "Map 21");
        }
      }
      THEN("Dumping the state contains the maps") {
        auto state = getDump(map_manager);
        REQUIRE(state.maps.has_value());
        REQUIRE(state.maps->size() == 4);
        REQUIRE(state.maps->at(0).mapDescription == "Map 11");
        REQUIRE(state.maps->at(1).mapDescription == "Map 12");
        REQUIRE(state.maps->at(2).mapDescription == "Map 21");
        REQUIRE(state.maps->at(3).mapDescription == "Map 22");
      }
    }
  }
}

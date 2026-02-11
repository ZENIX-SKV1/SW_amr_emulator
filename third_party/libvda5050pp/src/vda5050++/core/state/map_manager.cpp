// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//
#include "vda5050++/core/state/map_manager.h"

using namespace vda5050pp::core::state;

void MapManager::enableMap(std::string_view map_id, std::string_view map_version) {
  for (auto &map : this->maps_) {
    if (map.mapId == map_id) {
      if (map.mapVersion == map_version) {
        map.mapStatus = vda5050::MapStatus::ENABLED;
        return;
      } else {
        map.mapStatus = vda5050::MapStatus::DISABLED;
      }
    }
  }
}

void MapManager::deleteMap(std::string_view map_id, std::string_view map_version) {
  auto it = std::remove_if(this->maps_.begin(), this->maps_.end(), [&](const vda5050::Map &map) {
    return map.mapId == map_id && map.mapVersion == map_version;
  });

  this->maps_.erase(it, this->maps_.end());
}

void MapManager::addMap(const vda5050::Map &map) { this->maps_.push_back(map); }

bool MapManager::hasMap(std::string_view map_id) const {
  return std::any_of(this->maps_.begin(), this->maps_.end(),
                     [&](const vda5050::Map &map) { return map.mapId == map_id; });
}

bool MapManager::hasMap(std::string_view map_id, std::string_view map_version) const {
  return std::any_of(this->maps_.begin(), this->maps_.end(), [&](const vda5050::Map &map) {
    return map.mapId == map_id && map.mapVersion == map_version;
  });
}

std::optional<vda5050::Map> MapManager::getMap(std::string_view map_id) const {
  auto it = std::find_if(this->maps_.begin(), this->maps_.end(), [&](const vda5050::Map &map) {
    return map.mapId == map_id && map.mapStatus == vda5050::MapStatus::ENABLED;
  });

  if (it == this->maps_.end()) {
    return std::nullopt;
  }

  return *it;
}

void MapManager::dumpTo(vda5050::State &state) const { state.maps = this->maps_; }
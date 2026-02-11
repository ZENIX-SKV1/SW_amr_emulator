// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_2B_2B_CORE_STATE_MAP_MANAGER_H_
#define VDA5050_2B_2B_CORE_STATE_MAP_MANAGER_H_

#include <optional>
#include <string>
#include <vector>

#include "vda5050/Map.h"
#include "vda5050/MapStatus.h"
#include "vda5050/State.h"

namespace vda5050pp::core::state {

///
///\brief A manager for all maps.
///
/// This currently keeps track of all map objects for the vda5050 state.
/// It makes sure, that only one map per id is enabled at a time.
///
class MapManager {
private:
  std::vector<vda5050::Map> maps_;

public:
  ///
  ///\brief Enable a map. (Disable all other maps with the same id)
  ///
  ///\param map_id the map id
  ///\param map_version the map version
  ///
  void enableMap(std::string_view map_id, std::string_view map_version);

  ///
  ///\brief Delete a map.
  ///
  ///\param map_id the map id
  ///\param map_version the map version
  ///
  void deleteMap(std::string_view map_id, std::string_view map_version);

  ///
  ///\brief Add a map.
  ///
  ///\param map The map to add. It should be disabled, otherwise there may be two enabled maps.
  ///
  void addMap(const vda5050::Map &map);

  ///
  ///\brief Check if a map is available.
  ///
  ///\param map_id the map id to check
  ///\return is this map available?
  ///
  bool hasMap(std::string_view map_id) const;

  ///
  ///\brief Check if a map is available.
  ///
  ///\param map_id the map id to check
  ///\param map_version the map version
  ///\return is this map available?
  ///
  bool hasMap(std::string_view map_id, std::string_view map_version) const;

  ///
  ///\brief Get an enabled map by it's id.
  ///
  ///\param map_id The id of the map to get.
  ///\return std::optional<vda5050::Map> The map if it is enabled, otherwise std::nullopt.
  ///
  std::optional<vda5050::Map> getMap(std::string_view map_id) const;

  ///
  ///\brief Write all maps to the state.
  ///
  ///\param state the state to write to
  ///
  void dumpTo(vda5050::State &state) const;
};

}  // namespace vda5050pp::core::state

#endif  // VDA5050_2B_2B_CORE_STATE_MAP_MANAGER_H_

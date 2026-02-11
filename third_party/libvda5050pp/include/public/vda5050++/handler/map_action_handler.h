//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//

#ifndef PUBLIC_VDA5050_2B_2B_HANDLER_MAP_ACTION_HANDLER_H_
#define PUBLIC_VDA5050_2B_2B_HANDLER_MAP_ACTION_HANDLER_H_

#include <vda5050/Map.h>

#include <future>

#include "vda5050++/handler/simple_multi_action_handler.h"

namespace vda5050pp::handler {

///
///\brief A pre implemented BaseActionHandler for map related actions.
///
/// Covers:
/// - enableMap
/// - downloadMap
/// - deleteMap
///
/// NOTE: The library will call the member functions of this class with one dedicated thread.
/// The user has to make sure to return from the functions in order to receive the next function
/// call.
///
class MapActionHandler : public vda5050pp::handler::SimpleMultiActionHandler {
private:
  ///
  ///\brief This function is called to prepare a valid action.
  /// This will be mainly used to organize possible things in the user-code and set callbacks
  /// for the concerning action.
  ///
  ///\param action_state the ActionState (may not be used here)
  ///\param parameters the parsed parameters of the validate() call
  ///\return ActionCallbacks the callbacks used for the Action associated with action_state.
  ///
  vda5050pp::handler::ActionCallbacks prepare(
      std::shared_ptr<vda5050pp::handler::ActionState> action_state,
      std::shared_ptr<ParametersMap> parameters) noexcept(false) override;

protected:
  ///\brief Default reset implementation does nothing here.
  void reset() override;

public:
  MapActionHandler();
  ~MapActionHandler() override = default;

  ///
  ///\brief Add a map to the internal (VDA5050) state.
  ///
  ///\param map The map to add
  ///
  void stateAddMap(const vda5050::Map &map) const;

  ///
  ///\brief Enable a map in the internal (VDA5050) state.
  ///
  ///\param map_id the map id of the map to enable
  ///\param map_version the map version of the map to enable
  ///
  void stateEnableMap(std::string_view map_id, std::string_view map_version) const;

  ///
  ///\brief Delete a map from the internal (VDA5050) state.
  ///
  ///\param map_id the map id of the map to delete
  ///\param map_version the map version of the map to delete
  ///
  void stateDeleteMap(std::string_view map_id, std::string_view map_version) const;

  ///
  ///\brief Enable a map. This function is triggered by the MC via the enableMap action.
  ///
  /// The user has to make sure to call stateEnableMap to enable the map in the internal state.
  ///
  ///\param map_id the map id of the map to enable.
  ///\param map_version the map version of the map to enable
  ///\return std::list<vda5050::Error> a list of error. empty iff. success
  ///
  virtual std::list<vda5050::Error> enableMap(std::string_view map_id,
                                              std::string_view map_version) = 0;

  ///
  ///\brief Delete a map. This function is triggered by the MC via the deleteMap action.
  ///
  /// The user has to make sure to call stateDeleteMap to delete the map in the internal state.
  ///
  ///\param map_id the map id of the map to delete.
  ///\param map_version the map version of the map to delete
  ///\return std::list<vda5050::Error> a list of error. empty iff. success
  ///
  virtual std::list<vda5050::Error> deleteMap(std::string_view map_id,
                                              std::string_view map_version) = 0;

  ///
  ///\brief Download a new map. This function is triggered by the MC via the downloadMap action.
  ///
  /// The user has to make sure to call stateAddMap to add the map to the internal state.
  ///
  ///\param map_id the map id of the map to download.
  ///\param map_version the map version of the map to download
  ///\param map_download_link the download link of the map
  ///\param map_hash the hash of the map (optional)
  ///\return std::list<vda5050::Error> a list of error. empty iff. success
  ///
  virtual std::list<vda5050::Error> downloadMap(std::string_view map_id,
                                                std::string_view map_version,
                                                std::string_view map_download_link,
                                                std::optional<std::string_view> map_hash) = 0;
};

}  // namespace vda5050pp::handler

#endif  // PUBLIC_VDA5050_2B_2B_HANDLER_MAP_ACTION_HANDLER_H_

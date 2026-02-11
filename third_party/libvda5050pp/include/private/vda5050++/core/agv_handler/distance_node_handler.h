//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_2B_2B_CORE_AGV_HANDLER_DISTANCE_NODE_HANDLER_H_
#define VDA5050_2B_2B_CORE_AGV_HANDLER_DISTANCE_NODE_HANDLER_H_

#include <optional>

#include "vda5050++/core/module.h"
#include "vda5050++/core/navigation_status_manager.h"

namespace vda5050pp::core::agv_handler {

///
///\brief
///
class DistanceNodeHandler : public vda5050pp::core::Module {
  ///
  ///\brief Handles a NavigationStatusPosition event.
  ///
  ///
  ///
  ///\param evt the event data
  ///
  ///\throws VDA5050PPInvalidEventData when evt is empty
  ///
  void handleNavigationStatusPosition(
      std::shared_ptr<vda5050pp::events::NavigationStatusPosition> evt);

  ///
  ///\brief Handles a NavigationStatusNodeReached event.
  ///
  ///
  ///\param evt the event data
  ///
  ///\throws VDA5050PPInvalidEventData when evt is empty
  ///
  void handleNavigationStatusNodeReached(
      std::shared_ptr<vda5050pp::events::NavigationStatusNodeReached> evt);

  std::optional<vda5050pp::core::ScopedNavigationStatusSubscriber> navigation_status_subscriber_;
  struct vec_2_t {
    double x, y;
  };
  std::optional<vec_2_t> last_position_;
  double distance_sum_;

public:
  ///
  ///\brief Setup all subscribers and get the loaded config from the instance
  ///
  ///\param instance the current library instance
  ///
  void initialize(Instance &instance) override;

  ///
  ///\brief Unsuscribe all subscribers and reset the config
  ///
  ///\param instance the current library instance
  ///
  void deinitialize(Instance &instance) override;

  ///
  ///\brief Get the module's description (possibly deprecated)
  ///
  ///\return std::string_view
  ///
  std::string_view describe() const override;

  ///
  ///\brief Create a new DistanceNodeSubConfig instance
  ///
  ///\return std::shared_ptr<config::ModuleSubConfig> the created subconfig
  ///
  std::shared_ptr<config::ModuleSubConfig> generateSubConfig() const override;
};

}  // namespace vda5050pp::core::agv_handler

#endif  // VDA5050_2B_2B_CORE_AGV_HANDLER_NODE_REACHED_HANDLER_H_

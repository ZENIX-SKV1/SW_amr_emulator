//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_2B_2B_CORE_AGV_HANDLER_NAVIGATION_EVENT_HANDLER_H_
#define VDA5050_2B_2B_CORE_AGV_HANDLER_NAVIGATION_EVENT_HANDLER_H_

#include <memory>
#include <optional>

#include "vda5050++/core/module.h"
#include "vda5050++/core/navigation_event_manager.h"

namespace vda5050pp::core::agv_handler {

///
///\brief The NavigationEventHandler module is responsible for handling NavigationEvents
/// and forwarding them to the registered user NavigationHandler.
///
class NavigationEventHandler : public Module {
private:
  std::optional<vda5050pp::core::ScopedNavigationEventSubscriber> subscriber_;

  ///
  ///\brief Forward a HorizonUpdate event.
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPInvalidEventData when data is empty
  ///\throws any exception thrown by the user's NavigationHandler::horizonUpdated
  ///
  void handleHorizonUpdate(std::shared_ptr<vda5050pp::events::NavigationHorizonUpdate> data) const
      noexcept(false);

  ///
  ///\brief Forward a BaseIncreased event.
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPInvalidEventData when data is empty
  ///\throws any exception thrown by the user's NavigationHandler::baseIncreased
  ///
  void handleBaseIncreased(std::shared_ptr<vda5050pp::events::NavigationBaseIncreased> data) const
      noexcept(false);

  ///
  ///\brief Forward a NextNode event as call to NavigationHandler::navigateToNextNode
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPInvalidEventData when data is empty
  ///\throws any exception thrown by the user's NavigationHandler::navigateToNextNode
  ///
  void handleNextNode(std::shared_ptr<vda5050pp::events::NavigationNextNode> data) const
      noexcept(false);

  ///
  ///\brief Forward a UpcomingSegment event as call to NavigationHandler::upcomingSegment
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPInvalidEventData when data is empty
  ///\throws any exception thrown by the user's NavigationHandler::upcomingSegment
  ///
  void handleUpcomingSegment(
      std::shared_ptr<vda5050pp::events::NavigationUpcomingSegment> data) const noexcept(false);

  ///
  ///\brief Forward a NavigationControl event as a call to cancel/pause/resume functions
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPInvalidEventData when data is empty
  ///\throws VDA5050PPInvalidEventData when the control type is unknown
  ///\throws any exception thrown by the user's cancel/pause/resume functions
  ///
  void handleControl(std::shared_ptr<vda5050pp::events::NavigationControl> data) const
      noexcept(false);

  ///
  ///\brief Handle a NavigationReset event as a call to the user's NavigationHandler::reset
  ///
  ///\param data the event data
  ///\throws VDA5050PPInvalidEventData when data is empty
  ///\throws any exception thrown by the user's BaseNavigationHandler::reset
  ///
  void handleReset(std::shared_ptr<vda5050pp::events::NavigationReset> data) const noexcept(false);

public:
  ///
  ///\brief Setup all subscribers for the NavigationEventHandler
  ///
  ///\param instance the current instance of the library
  ///
  void initialize(vda5050pp::core::Instance &instance) override;

  ///
  ///\brief Unsubscribe all subscribers
  ///
  ///\param instance the current instance of the library
  ///
  void deinitialize(vda5050pp::core::Instance &instance) override;

  ///
  ///\brief Describe the module (possibly deprecated)
  ///
  ///\return std::string_view the module's description
  ///
  std::string_view describe() const override;
};

}  // namespace vda5050pp::core::agv_handler

#endif  // VDA5050_2B_2B_CORE_AGV_HANDLER_NAVIGATION_EVENT_HANDLER_H_

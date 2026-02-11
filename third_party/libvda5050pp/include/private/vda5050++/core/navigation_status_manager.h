//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//

#ifndef PRIVATE_VDA5050_2B_2B_CORE_NAVIGATION_STATUS_MANAGER_H_
#define PRIVATE_VDA5050_2B_2B_CORE_NAVIGATION_STATUS_MANAGER_H_

#include <eventpp/eventqueue.h>
#include <eventpp/utilities/scopedremover.h>

#include <functional>
#include <memory>
#include <thread>

#include "vda5050++/core/common/scoped_thread.h"
#include "vda5050++/core/events/event_manager_options.h"
#include "vda5050++/events/navigation_event.h"

namespace vda5050pp::core {

using NavigationStatusQueue =
    eventpp::EventQueue<vda5050pp::events::NavigationStatusType,
                        void(std::shared_ptr<vda5050pp::events::NavigationStatus>)>;

///
///\brief A RAII based subscriber for navigation status events.
///
class ScopedNavigationStatusSubscriber {
private:
  friend class NavigationStatusManager;
  eventpp::ScopedRemover<NavigationStatusQueue> remover_;

  ///
  ///\brief Construct a new scoped navigation status subscriber
  ///
  ///\param queue
  ///
  explicit ScopedNavigationStatusSubscriber(NavigationStatusQueue &queue);

public:
  ///
  ///\brief Subscribe to NavigationStatusPosition events
  ///
  ///\param callback the callback
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::NavigationStatusPosition>)>
                     &&callback) noexcept(true);

  ///
  ///\brief Subscribe to NavigationStatusVelocity events
  ///
  ///\param callback the callback
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::NavigationStatusVelocity>)>
                     &&callback) noexcept(true);
  ///
  ///\brief Subscribe to NavigationStatusNodeReached events
  ///
  ///\param callback the callback
  ///
  void subscribe(
      std::function<void(std::shared_ptr<vda5050pp::events::NavigationStatusNodeReached>)>
          &&callback) noexcept(true);

  ///
  ///\brief Subscribe to NavigationStatusDistanceSinceLastNode events
  ///
  ///\param callback the callback
  ///
  void subscribe(
      std::function<void(std::shared_ptr<vda5050pp::events::NavigationStatusDistanceSinceLastNode>)>
          &&callback) noexcept(true);

  ///
  ///\brief Subscribe to NavigationStatusDriving events
  ///
  ///\param callback the callback
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::NavigationStatusDriving>)>
                     &&callback) noexcept(true);

  ///
  ///\brief Subscribe to NavigationStatusControl events
  ///
  ///\param callback the callback
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::NavigationStatusControl>)>
                     &&callback) noexcept(true);
};

///
///\brief The NavigationStatusManager encapsulates the event queue and the thread that
/// processes the events.
///
class NavigationStatusManager {
private:
  NavigationStatusQueue navigation_status_queue_;

  const vda5050pp::core::events::EventManagerOptions &opts_;

  vda5050pp::core::common::ScopedThread<void()> thread_;

  ///
  ///\brief The event-loop, which processes the events and calls the callbacks.
  ///
  ///\param tkn the stop-token
  ///
  void threadTask(vda5050pp::core::common::StopToken tkn) noexcept(true);

public:
  ///
  ///\brief Construct a new NavigationStatusManager
  ///
  ///\param opts the event manager options
  ///
  explicit NavigationStatusManager(const vda5050pp::core::events::EventManagerOptions &opts);

  ///
  ///\brief Dispatch a NavigationStatus event
  ///
  ///\param data the event
  ///
  void dispatch(std::shared_ptr<vda5050pp::events::NavigationStatusPosition> data) noexcept(true);

  ///
  ///\brief Dispatch a NavigationStatus event
  ///
  ///\param data the event
  ///
  void dispatch(std::shared_ptr<vda5050pp::events::NavigationStatusVelocity> data) noexcept(true);

  ///
  ///\brief Dispatch a NavigationStatus event
  ///
  ///\param data the event
  ///
  void dispatch(std::shared_ptr<vda5050pp::events::NavigationStatusNodeReached> data) noexcept(
      true);

  ///
  ///\brief Dispatch a NavigationStatus event
  ///
  ///\param data the event
  ///
  void dispatch(std::shared_ptr<vda5050pp::events::NavigationStatusDistanceSinceLastNode>
                    data) noexcept(true);

  ///
  ///\brief Dispatch a NavigationStatus event
  ///
  ///\param data the event
  ///
  void dispatch(std::shared_ptr<vda5050pp::events::NavigationStatusDriving> data) noexcept(true);

  ///
  ///\brief Dispatch a NavigationStatus event
  ///
  ///\param data the event
  ///
  void dispatch(std::shared_ptr<vda5050pp::events::NavigationStatusControl> data) noexcept(true);

  ///
  ///\brief Get a new ScopedNavigationStatusSubscriber.
  ///
  ///\return ScopedNavigationStatusSubscriber
  ///
  ScopedNavigationStatusSubscriber getScopedNavigationStatusSubscriber() noexcept(true);

  /// @brief Stop and join the thread associated with this event manager
  void join() {
    if (thread_.joinable()) {
      thread_.stop();
      thread_.join();
    }
  }
};

}  // namespace vda5050pp::core

#endif  // PRIVATE_VDA5050_2B_2B_CORE_NAVIGATION_STATUS_MANAGER_H_

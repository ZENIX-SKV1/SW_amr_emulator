//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//

#ifndef PRIVATE_VDA5050_2B_2B_CORE_NAVIGATION_EVENT_MANAGER_H_
#define PRIVATE_VDA5050_2B_2B_CORE_NAVIGATION_EVENT_MANAGER_H_

#include <eventpp/eventqueue.h>
#include <eventpp/utilities/scopedremover.h>

#include <functional>
#include <memory>

#include "vda5050++/core/common/scoped_thread.h"
#include "vda5050++/core/events/event_manager_options.h"
#include "vda5050++/events/navigation_event.h"
#include "vda5050++/events/scoped_navigation_event_subscriber.h"

namespace vda5050pp::core {

using NavigationEventQueue =
    eventpp::EventQueue<vda5050pp::events::NavigationEventType,
                        void(std::shared_ptr<vda5050pp::events::NavigationEvent>)>;
///
///\brief A RAII based subscriber for navigation events. It's a specialization of the
/// public ScopedNavigationEventSubscriber, which is instantiated by the NavigationEventManager
/// and handed to the user.
///
class ScopedNavigationEventSubscriber : public vda5050pp::events::ScopedNavigationEventSubscriber {
private:
  friend class NavigationEventManager;
  eventpp::ScopedRemover<NavigationEventQueue> remover_;

  ///
  ///\brief Construct a new ScopedNavigationEventSubscriber object for a given queue.
  ///
  ///\param queue
  ///
  explicit ScopedNavigationEventSubscriber(NavigationEventQueue &queue);

public:
  ///
  ///\brief Subscribe to NavigationHorizonUpdate events.
  ///
  ///\param callback the callback
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::NavigationHorizonUpdate>)>
                     &&callback) noexcept(true) override;

  ///
  ///\brief Subscribe to NavigationBaseIncreased events.
  ///
  ///\param callback the callback
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::NavigationBaseIncreased>)>
                     &&callback) noexcept(true) override;

  ///
  ///\brief Subscribe to NavigationNextNode events.
  ///
  ///\param callback the callback
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::NavigationNextNode>)>
                     &&callback) noexcept(true) override;

  ///
  ///\brief Subscribe to NavigationUpcomingSegment events.
  ///
  ///\param callback the callback
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::NavigationUpcomingSegment>)>
                     &&callback) noexcept(true) override;

  ///
  ///\brief Subscribe to NavigationControl events.
  ///
  ///\param callback the callback
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::NavigationControl>)>
                     &&callback) noexcept(true) override;

  ///
  ///\brief Subscribe to NavigationReset events.
  ///
  ///\param callback the callback
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::NavigationReset>)>
                     &&callback) noexcept(true) override;
};

///
///\brief The NavigationEventManager encapsulates the event queue and the thread that
/// processes the events.
///
class NavigationEventManager {
private:
  NavigationEventQueue navigation_event_queue_;

  const vda5050pp::core::events::EventManagerOptions &opts_;

  vda5050pp::core::common::ScopedThread<void()> thread_;

  ///
  ///\brief The event-loop, which processes the events and calls the callbacks.
  ///
  ///\param tkn the stop-token
  ///
  void threadTask(vda5050pp::core::common::StopToken tkn) noexcept(true);

public:
  explicit NavigationEventManager(const vda5050pp::core::events::EventManagerOptions &opts);

  ///
  ///\brief Dispatch a Navigation event
  ///
  ///\param data the event
  ///\param synchronized_dispatch if true, the event will be processed synchronously
  ///
  void dispatch(std::shared_ptr<vda5050pp::events::NavigationHorizonUpdate> data,
                bool synchronized_dispatch = false) noexcept(true);

  ///
  ///\brief Dispatch a Navigation event
  ///
  ///\param data the event
  ///\param synchronized_dispatch if true, the event will be processed synchronously
  ///
  void dispatch(std::shared_ptr<vda5050pp::events::NavigationBaseIncreased> data,
                bool synchronized_dispatch = false) noexcept(true);

  ///
  ///\brief Dispatch a Navigation event
  ///
  ///\param data the event
  ///\param synchronized_dispatch if true, the event will be processed synchronously
  ///
  void dispatch(std::shared_ptr<vda5050pp::events::NavigationNextNode> data,
                bool synchronized_dispatch = false) noexcept(true);

  ///
  ///\brief Dispatch a Navigation event
  ///
  ///\param data the event
  ///\param synchronized_dispatch if true, the event will be processed synchronously
  ///
  void dispatch(std::shared_ptr<vda5050pp::events::NavigationUpcomingSegment> data,
                bool synchronized_dispatch = false) noexcept(true);

  ///
  ///\brief Dispatch a Navigation event
  ///
  ///\param data the event
  ///\param synchronized_dispatch if true, the event will be processed synchronously
  ///
  void dispatch(std::shared_ptr<vda5050pp::events::NavigationControl> data,
                bool synchronized_dispatch = false) noexcept(true);

  ///
  ///\brief Dispatch a Navigation event
  ///
  ///\param data the event
  ///\param synchronized_dispatch if true, the event will be processed synchronously
  ///
  void dispatch(std::shared_ptr<vda5050pp::events::NavigationReset> data,
                bool synchronized_dispatch = false) noexcept(true);

  ///
  ///\brief Get a new ScopedNavigationEventSubscriber.
  ///
  ///\return ScopedNavigationEventSubscriber
  ///
  ScopedNavigationEventSubscriber getScopedNavigationEventSubscriber() noexcept(true);

  /// @brief Stop and join the thread associated with this event manager
  void join() {
    if (thread_.joinable()) {
      thread_.stop();
      thread_.join();
    }
  }
};

}  // namespace vda5050pp::core

#endif  // PRIVATE_VDA5050_2B_2B_CORE_NAVIGATION_EVENT_MANAGER_H_

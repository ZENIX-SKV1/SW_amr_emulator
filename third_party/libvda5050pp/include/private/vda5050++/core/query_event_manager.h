//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//

#ifndef PRIVATE_VDA5050_2B_2B_CORE_QUERY_EVENT_MANAGER_H_
#define PRIVATE_VDA5050_2B_2B_CORE_QUERY_EVENT_MANAGER_H_

#include <eventpp/eventqueue.h>
#include <eventpp/utilities/scopedremover.h>

#include <functional>
#include <memory>
#include <thread>

#include "vda5050++/core/common/scoped_thread.h"
#include "vda5050++/core/events/event_manager_options.h"
#include "vda5050++/events/query_event.h"
#include "vda5050++/events/scoped_query_event_subscriber.h"

namespace vda5050pp::core {

using QueryEventQueue = eventpp::EventQueue<vda5050pp::events::QueryEventType,
                                            void(std::shared_ptr<vda5050pp::events::QueryEvent>)>;
///
///\brief A RAII based subscriber for query events. It's a specialization of the
/// public ScopedQueryEventSubscriber, which is instantiated by the QueryEventManager
/// and handed to the user.
///
class ScopedQueryEventSubscriber : public vda5050pp::events::ScopedQueryEventSubscriber {
private:
  friend class QueryEventManager;
  eventpp::ScopedRemover<QueryEventQueue> remover_;

  ///
  ///\brief Construct a new scoped query event subscriber for a given queue.
  ///
  ///\param queue The event managers queue
  ///
  explicit ScopedQueryEventSubscriber(QueryEventQueue &queue);

public:
  ///
  ///\brief Subscribe to QueryPauseable events.
  ///
  ///\param callback the callback to call
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::QueryPauseable>)>
                     &&callback) noexcept(true) override;

  ///
  ///\brief Subscribe to QueryResumable events.
  ///
  ///\param callback the callback to call
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::QueryResumable>)>
                     &&callback) noexcept(true) override;

  ///
  ///\brief Subscribe to QueryAcceptZoneSet events.
  ///
  ///\param callback the callback to call
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::QueryAcceptZoneSet>)>
                     &&callback) noexcept(true) override;

  ///
  ///\brief Subscribe to QueryAcceptZoneSet events.
  ///
  ///\param callback the callback to call
  ///
  void subscribe(
      std::function<void(std::shared_ptr<vda5050pp::events::QueryNodeTriviallyReachable>)>
          &&callback) noexcept(true) override;
};

///
///\brief The QueryEventManager encapsulates the event queue and the thread that
/// processes the events.
///
class QueryEventManager {
private:
  QueryEventQueue query_event_queue_;

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
  ///\brief Construct a new QueryEventManager
  ///
  ///\param opts the event manager options
  ///
  explicit QueryEventManager(const vda5050pp::core::events::EventManagerOptions &opts);

  ///
  ///\brief Dispatch a QueryPauseable event
  ///
  ///\param data the event
  ///\param synchronous if running async, force synchronous processing by setting this to true
  ///
  void dispatch(std::shared_ptr<vda5050pp::events::QueryPauseable> data,
                bool synchronous = false) noexcept(true);

  ///
  ///\brief Dispatch a QueryResumable event
  ///
  ///\param data the event
  ///\param synchronous if running async, force synchronous processing by setting this to true
  ///
  void dispatch(std::shared_ptr<vda5050pp::events::QueryResumable> data,
                bool synchronous = false) noexcept(true);

  ///
  ///\brief Dispatch a QueryAcceptZoneSet event
  ///
  ///\param data the event
  ///\param synchronous if running async, force synchronous processing by setting this to true
  ///
  void dispatch(std::shared_ptr<vda5050pp::events::QueryAcceptZoneSet> data,
                bool synchronous = false) noexcept(true);

  ///
  ///\brief Dispatch a QueryNodeTriviallyReachable event
  ///
  ///\param data the event
  ///\param synchronous if running async, force synchronous processing by setting this to true
  ///
  void dispatch(std::shared_ptr<vda5050pp::events::QueryNodeTriviallyReachable> data,
                bool synchronous = false) noexcept(true);

  ///
  ///\brief Get a new ScopedQueryEventSubscriber object for this event manager.
  ///
  ///\return ScopedQueryEventSubscriber the subscriber
  ///
  ScopedQueryEventSubscriber getScopedQueryEventSubscriber() noexcept(true);

  /// @brief Stop and join the thread associated with this event manager
  void join() {
    if (thread_.joinable()) {
      thread_.stop();
      thread_.join();
    }
  }
};

}  // namespace vda5050pp::core

#endif  // PRIVATE_VDA5050_2B_2B_CORE_QUERY_EVENT_MANAGER_H_

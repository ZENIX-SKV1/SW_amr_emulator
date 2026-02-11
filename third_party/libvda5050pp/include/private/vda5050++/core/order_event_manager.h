//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//

#ifndef PRIVATE_VDA5050_2B_2B_CORE_ORDER_EVENT_MANAGER_H_
#define PRIVATE_VDA5050_2B_2B_CORE_ORDER_EVENT_MANAGER_H_

#include <eventpp/eventqueue.h>
#include <eventpp/utilities/scopedremover.h>

#include <functional>
#include <memory>

#include "vda5050++/core/common/scoped_thread.h"
#include "vda5050++/core/events/event_manager_options.h"
#include "vda5050++/events/order_event.h"
#include "vda5050++/events/scoped_order_event_subscriber.h"

namespace vda5050pp::core {

using OrderEventQueue = eventpp::EventQueue<vda5050pp::events::OrderEventType,
                                            void(std::shared_ptr<vda5050pp::events::OrderEvent>)>;
///
///\brief A RAII based subscriber for (AGV-Interface) order events.
///
class ScopedOrderEventSubscriber : public vda5050pp::events::ScopedOrderEventSubscriber {
private:
  friend class OrderEventManager;
  eventpp::ScopedRemover<OrderEventQueue> remover_;

  ///
  ///\brief Construct a new ScopedOrderEventSubscriber for a given queue.
  ///
  ///\param queue the event queue
  ///
  explicit ScopedOrderEventSubscriber(OrderEventQueue &queue);

public:
  ~ScopedOrderEventSubscriber() override = default;
  ///
  ///\brief Subscribe to OrderBaseChanged events.
  ///
  ///\param callback the callback to call
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::OrderBaseChanged>)>
                     &&callback) noexcept(true) override;

  ///
  ///\brief Subscribe to OrderHorizonChanged events.
  ///
  ///\param callback the callback to call
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::OrderHorizonChanged>)>
                     &&callback) noexcept(true) override;

  ///
  ///\brief Subscribe to OrderActionStatesChanged events.
  ///
  ///\param callback the callback to call
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::OrderActionStatesChanged>)>
                     &&callback) noexcept(true) override;

  ///
  ///\brief Subscribe to OrderErrorsChanged events.
  ///
  ///\param callback the callback to call
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::OrderErrorsChanged>)>
                     &&callback) noexcept(true) override;

  ///
  ///\brief Subscribe to OrderLastNodeChanged events.
  ///
  ///\param callback the callback to call
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::OrderLastNodeChanged>)>
                     &&callback) noexcept(true) override;
};

///
///\brief The OrderEventManager encapsulates the event queue and the thread that
/// processes the events.
///
class OrderEventManager {
private:
  OrderEventQueue order_event_queue_;

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
  ///\brief Construct a new OrderEventManager
  ///
  ///\param opts the event manager options
  ///
  explicit OrderEventManager(const vda5050pp::core::events::EventManagerOptions &opts);

  ///
  ///\brief Dispatch a Order event. Order events contain runtime information of their types, such
  /// that only one dispatch function is required.
  ///
  ///\param data the event
  ///\param synchronous if running async, force synchronous processing by setting this to true
  ///
  void dispatch(std::shared_ptr<vda5050pp::events::OrderEvent> data,
                bool synchronous = false) noexcept(false);

  ///
  ///\brief Get a new ScopedStatusEventSubscriber for this event manager.
  ///
  ///\return ScopedStatusEventSubscriber the subscriber
  ///
  std::shared_ptr<ScopedOrderEventSubscriber> getScopedOrderEventSubscriber() noexcept(true);

  /// @brief Stop and join the thread associated with this event manager
  void join() {
    if (thread_.joinable()) {
      thread_.stop();
      thread_.join();
    }
  }
};

}  // namespace vda5050pp::core

#endif  // PRIVATE_VDA5050_2B_2B_CORE_ORDER_EVENT_MANAGER_H_

//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//

#ifndef PRIVATE_VDA5050_2B_2B_CORE_STATUS_EVENT_MANAGER_H_
#define PRIVATE_VDA5050_2B_2B_CORE_STATUS_EVENT_MANAGER_H_

#include <eventpp/eventqueue.h>
#include <eventpp/utilities/scopedremover.h>

#include <functional>
#include <memory>

#include "vda5050++/core/common/scoped_thread.h"
#include "vda5050++/core/events/event_manager_options.h"
#include "vda5050++/events/status_event.h"

namespace vda5050pp::core {

using StatusEventQueue = eventpp::EventQueue<vda5050pp::events::StatusEventType,
                                             void(std::shared_ptr<vda5050pp::events::StatusEvent>)>;
///
///\brief A RAII based subscriber for status events.
///
class ScopedStatusEventSubscriber {
private:
  friend class StatusEventManager;
  eventpp::ScopedRemover<StatusEventQueue> remover_;

  ///
  ///\brief Construct a new ScopedStatusEventSubscriber for a given queue.
  ///
  ///\param queue the event queue
  ///
  explicit ScopedStatusEventSubscriber(StatusEventQueue &queue);

public:
  ///
  ///\brief Subscribe to LoadAdd events.
  ///
  ///\param callback the callback to call
  ///
  void subscribe(
      std::function<void(std::shared_ptr<vda5050pp::events::LoadAdd>)> &&callback) noexcept(true);
  ///
  ///\brief Subscribe to LoadRemove events.
  ///
  ///\param callback the callback to call
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::LoadRemove>)>
                     &&callback) noexcept(true);

  ///
  ///\brief Subscribe to LoadsGet events.
  ///
  ///\param callback the callback to call
  ///
  void subscribe(
      std::function<void(std::shared_ptr<vda5050pp::events::LoadsGet>)> &&callback) noexcept(true);

  ///
  ///\brief Subscribe to LoadsAlter events.
  ///
  ///\param callback the callback to call
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::LoadsAlter>)>
                     &&callback) noexcept(true);

  ///
  ///\brief Subscribe to OperatingModeSet events.
  ///
  ///\param callback the callback to call
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::OperatingModeSet>)>
                     &&callback) noexcept(true);

  ///
  ///\brief Subscribe to OperatingModeGet events.
  ///
  ///\param callback the callback to call
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::OperatingModeGet>)>
                     &&callback) noexcept(true);

  ///
  ///\brief Subscribe to OperatingModeAlter events.
  ///
  ///\param callback the callback to call
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::OperatingModeAlter>)>
                     &&callback) noexcept(true);

  ///
  ///\brief Subscribe to BatteryStateSet events.
  ///
  ///\param callback the callback to call
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::BatteryStateSet>)>
                     &&callback) noexcept(true);

  ///
  ///\brief Subscribe to BatteryStateGet events.
  ///
  ///\param callback the callback to call
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::BatteryStateGet>)>
                     &&callback) noexcept(true);

  ///
  ///\brief Subscribe to BatteryStateAlter events.
  ///
  ///\param callback the callback to call
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::BatteryStateAlter>)>
                     &&callback) noexcept(true);

  ///
  ///\brief Subscribe to SafetyStateSet events.
  ///
  ///\param callback the callback to call
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::SafetyStateSet>)>
                     &&callback) noexcept(true);

  ///
  ///\brief Subscribe to SafetyStateGet events.
  ///
  ///\param callback the callback to call
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::SafetyStateGet>)>
                     &&callback) noexcept(true);

  ///
  ///\brief Subscribe to SafetyStateAlter events.
  ///
  ///\param callback the callback to call
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::SafetyStateAlter>)>
                     &&callback) noexcept(true);

  ///
  ///\brief Subscribe to RequestNewBase events.
  ///
  ///\param callback the callback to call
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::RequestNewBase>)>
                     &&callback) noexcept(true);

  ///
  ///\brief Subscribe to ErrorAdd events.
  ///
  ///\param callback the callback to call
  ///
  void subscribe(
      std::function<void(std::shared_ptr<vda5050pp::events::ErrorAdd>)> &&callback) noexcept(true);

  ///
  ///\brief Subscribe to ErrorsAlter events.
  ///
  ///\param callback the callback to call
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::ErrorsAlter>)>
                     &&callback) noexcept(true);

  ///
  ///\brief Subscribe to InfoAdd events.
  ///
  ///\param callback the callback to call
  ///
  void subscribe(
      std::function<void(std::shared_ptr<vda5050pp::events::InfoAdd>)> &&callback) noexcept(true);

  ///
  ///\brief Subscribe to InfosAlter events.
  ///
  ///\param callback the callback to call
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::InfosAlter>)>
                     &&callback) noexcept(true);

  ///
  ///\brief Subscribe to AddMap events.
  ///
  ///\param callback the callback to call
  ///
  void subscribe(
      std::function<void(std::shared_ptr<vda5050pp::events::AddMap>)> &&callback) noexcept(true);

  ///
  ///\brief Subscribe to DeleteMap events.
  ///
  ///\param callback the callback to call
  ///
  void subscribe(
      std::function<void(std::shared_ptr<vda5050pp::events::DeleteMap>)> &&callback) noexcept(true);

  ///
  ///\brief Subscribe to EnableMap events.
  ///
  ///\param callback the callback to call
  ///
  void subscribe(
      std::function<void(std::shared_ptr<vda5050pp::events::EnableMap>)> &&callback) noexcept(true);
};

///
///\brief The StatusEventManager encapsulates the event queue and the thread that
/// processes the events.
///
class StatusEventManager {
private:
  StatusEventQueue status_event_queue_;

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
  ///\brief Construct a new StatusEventManager
  ///
  ///\param opts the event manager options
  ///
  explicit StatusEventManager(const vda5050pp::core::events::EventManagerOptions &opts);

  ///
  ///\brief Dispatch a Status event. Status events contain runtime information of their types, such
  /// that only one dispatch function is required.
  ///
  ///\param data the event
  ///\param synchronous if running async, force synchronous processing by setting this to true
  ///
  void dispatch(std::shared_ptr<vda5050pp::events::StatusEvent> data,
                bool synchronous = false) noexcept(false);

  ///
  ///\brief Get a new ScopedStatusEventSubscriber for this event manager.
  ///
  ///\return ScopedStatusEventSubscriber the subscriber
  ///
  ScopedStatusEventSubscriber getScopedStatusEventSubscriber() noexcept(true);

  /// @brief Stop and join the thread associated with this event manager
  void join() {
    if (thread_.joinable()) {
      thread_.stop();
      thread_.join();
    }
  }
};

}  // namespace vda5050pp::core

#endif  // PRIVATE_VDA5050_2B_2B_CORE_STATUS_EVENT_MANAGER_H_

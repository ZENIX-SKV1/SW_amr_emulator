//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//

#ifndef PRIVATE_VDA5050_2B_2B_CORE_ACTION_EVENT_MANAGER_H_
#define PRIVATE_VDA5050_2B_2B_CORE_ACTION_EVENT_MANAGER_H_

#include <eventpp/eventqueue.h>
#include <eventpp/utilities/scopedremover.h>

#include <functional>
#include <memory>
#include <thread>

#include "vda5050++/core/common/scoped_thread.h"
#include "vda5050++/core/events/event_manager_options.h"
#include "vda5050++/events/action_event.h"
#include "vda5050++/events/scoped_action_event_subscriber.h"

namespace vda5050pp::core {

using ActionEventQueue = eventpp::EventQueue<vda5050pp::events::ActionEventType,
                                             void(std::shared_ptr<vda5050pp::events::ActionEvent>)>;
///
///\brief A RAII based subscriber for action events. It's a specialization of the
/// public ScopedActionEventSubscriber, which is instantiated by the ActionEventManager
/// and handed to the user.
///
class ScopedActionEventSubscriber : public vda5050pp::events::ScopedActionEventSubscriber {
private:
  friend class ActionEventManager;
  eventpp::ScopedRemover<ActionEventQueue> remover_;

  ///
  ///\brief Construct a new ScopedActionEventSubscriber object for a given queue.
  ///
  ///\param queue
  ///
  explicit ScopedActionEventSubscriber(ActionEventQueue &queue);

public:
  ///
  ///\brief Set a callback for ActionList events
  ///
  ///\param callback the callback
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::ActionList>)>
                     &&callback) noexcept(true) override;
  ///
  ///\brief Set a callback for ActionValidate events
  ///
  ///\param callback the callback
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::ActionValidate>)>
                     &&callback) noexcept(true) override;
  ///
  ///\brief Set a callback for ActionPrepare events
  ///
  ///\param callback the callback
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::ActionPrepare>)>
                     &&callback) noexcept(true) override;
  ///
  ///\brief Set a callback for ActionStat events
  ///
  ///\param callback the callback
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::ActionStart>)>
                     &&callback) noexcept(true) override;
  ///
  ///\brief Set a callback for ActionPause events
  ///
  ///\param callback the callback
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::ActionPause>)>
                     &&callback) noexcept(true) override;
  ///
  ///\brief Set a callback for ActionResume events
  ///
  ///\param callback the callback
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::ActionResume>)>
                     &&callback) noexcept(true) override;
  ///
  ///\brief Set a callback for ActionCancel events
  ///
  ///\param callback the callback
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::ActionCancel>)>
                     &&callback) noexcept(true) override;
  ///
  ///\brief Set a callback for ActionForget events
  ///
  ///\param callback the callback
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::ActionForget>)>
                     &&callback) noexcept(true) override;

  ///
  ///\brief Set a callback for ActionReset events
  ///
  ///\param callback the callback
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::ActionReset>)>
                     &&callback) noexcept(true) override;
};

///
///\brief The ActionEventManager encapsulates the event queue and the thread that
/// processes the events.
///
class ActionEventManager {
private:
  ActionEventQueue action_event_queue_;

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
  ///\brief Construct a new ActionEventManager
  ///
  ///\param opts event-manager options
  ///
  explicit ActionEventManager(const vda5050pp::core::events::EventManagerOptions &opts);

  ///
  ///\brief Dispatch an event to the event queue.
  ///
  ///\param data the event
  ///\param synchronous if running async, force synchronous processing by setting this to true
  ///
  void dispatch(std::shared_ptr<vda5050pp::events::ActionList> data,
                bool synchronous = false) noexcept(false);

  ///
  ///\brief Dispatch an event to the event queue.
  ///
  ///\param data the event
  ///\param synchronous if running async, force synchronous processing by setting this to true
  ///
  void dispatch(std::shared_ptr<vda5050pp::events::ActionValidate> data,
                bool synchronous = false) noexcept(false);

  ///
  ///\brief Dispatch an event to the event queue.
  ///
  ///\param data the event
  ///\param synchronous if running async, force synchronous processing by setting this to true
  ///
  void dispatch(std::shared_ptr<vda5050pp::events::ActionPrepare> data,
                bool synchronous = false) noexcept(false);

  ///
  ///\brief Dispatch an event to the event queue.
  ///
  ///\param data the event
  ///\param synchronous if running async, force synchronous processing by setting this to true
  ///
  void dispatch(std::shared_ptr<vda5050pp::events::ActionStart> data,
                bool synchronous = false) noexcept(false);

  ///
  ///\brief Dispatch an event to the event queue.
  ///
  ///\param data the event
  ///\param synchronous if running async, force synchronous processing by setting this to true
  ///
  void dispatch(std::shared_ptr<vda5050pp::events::ActionPause> data,
                bool synchronous = false) noexcept(false);

  ///
  ///\brief Dispatch an event to the event queue.
  ///
  ///\param data the event
  ///\param synchronous if running async, force synchronous processing by setting this to true
  ///
  void dispatch(std::shared_ptr<vda5050pp::events::ActionResume> data,
                bool synchronous = false) noexcept(false);

  ///
  ///\brief Dispatch an event to the event queue.
  ///
  ///\param data the event
  ///\param synchronous if running async, force synchronous processing by setting this to true
  ///
  void dispatch(std::shared_ptr<vda5050pp::events::ActionCancel> data,
                bool synchronous = false) noexcept(false);

  ///
  ///\brief Dispatch an event to the event queue.
  ///
  ///\param data the event
  ///\param synchronous if running async, force synchronous processing by setting this to true
  ///
  void dispatch(std::shared_ptr<vda5050pp::events::ActionForget> data,
                bool synchronous = false) noexcept(false);

  ///
  ///\brief Dispatch an event to the event queue.
  ///
  ///\param data the event
  ///\param synchronous if running async, force synchronous processing by setting this to true
  ///
  void dispatch(std::shared_ptr<vda5050pp::events::ActionReset> data,
                bool synchronous = false) noexcept(false);

  ///
  ///\brief Get a scoped action event subscriber for this event manager.
  ///
  ///\return ScopedActionEventSubscriber
  ///
  ScopedActionEventSubscriber getScopedActionEventSubscriber() noexcept(true);

  /// @brief Stop and join the thread associated with this event manager
  void join() {
    if (thread_.joinable()) {
      thread_.stop();
      thread_.join();
    }
  }
};

}  // namespace vda5050pp::core

#endif  // PRIVATE_VDA5050_2B_2B_CORE_ACTION_EVENT_MANAGER_H_

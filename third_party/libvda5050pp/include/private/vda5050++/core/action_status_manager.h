//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//

#ifndef PRIVATE_VDA5050_2B_2B_CORE_ACTION_STATUS_MANAGER_H_
#define PRIVATE_VDA5050_2B_2B_CORE_ACTION_STATUS_MANAGER_H_

#include <eventpp/eventqueue.h>
#include <eventpp/utilities/scopedremover.h>

#include <functional>
#include <memory>
#include <thread>

#include "vda5050++/core/common/scoped_thread.h"
#include "vda5050++/core/events/event_manager_options.h"
#include "vda5050++/events/action_event.h"

namespace vda5050pp::core {

using ActionStatusQueue =
    eventpp::EventQueue<vda5050pp::events::ActionStatusType,
                        void(std::shared_ptr<vda5050pp::events::ActionStatus>)>;

///
///\brief A RAII based subscriber for action events. It's not handed to the user,
/// since those events are not meant to be handled by the user.
///
class ScopedActionStatusSubscriber {
private:
  friend class ActionStatusManager;
  eventpp::ScopedRemover<ActionStatusQueue> remover_;

  ///
  ///\brief Construct a new ScopedActionStatusSubscriber object for a given queue.
  ///
  ///\param queue
  ///
  explicit ScopedActionStatusSubscriber(ActionStatusQueue &queue);

public:
  ///
  ///\brief Subscribe to ActionStatusWaiting events.
  ///
  ///\param callback the callback to call
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::ActionStatusWaiting>)>
                     &&callback) noexcept(true);

  ///
  ///\brief Subscribe to ActionStatusInitializing events.
  ///
  ///\param callback the callback to call
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::ActionStatusInitializing>)>
                     &&callback) noexcept(true);

  ///
  ///\brief Subscribe to ActionStatusRunning events.
  ///
  ///\param callback the callback to call
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::ActionStatusRunning>)>
                     &&callback) noexcept(true);

  ///
  ///\brief Subscribe to ActionStatusPaused events.
  ///
  ///\param callback the callback to call
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::ActionStatusPaused>)>
                     &&callback) noexcept(true);

  ///
  ///\brief Subscribe to ActionStatusFinished events.
  ///
  ///\param callback the callback to call
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::ActionStatusFinished>)>
                     &&callback) noexcept(true);

  ///
  ///\brief Subscribe to ActionStatusFailed events.
  ///
  ///\param callback the callback to call
  ///
  void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::ActionStatusFailed>)>
                     &&callback) noexcept(true);
};

///
///\brief The ActionStatusManager encapsulates the event queue and the thread that
/// processes the events.
///
class ActionStatusManager {
private:
  ActionStatusQueue action_status_queue_;

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
  ///\brief Construct a new ActionStatusManager object
  ///
  ///\param opts the event manager options
  ///
  explicit ActionStatusManager(const vda5050pp::core::events::EventManagerOptions &opts);

  ///
  ///\brief Dispatch an ActionStatus event
  ///
  ///\param data the event
  ///
  void dispatch(std::shared_ptr<vda5050pp::events::ActionStatusWaiting> data) noexcept(true);

  ///
  ///\brief Dispatch an ActionStatus event
  ///
  ///\param data the event
  ///
  void dispatch(std::shared_ptr<vda5050pp::events::ActionStatusInitializing> data) noexcept(true);

  ///
  ///\brief Dispatch an ActionStatus event
  ///
  ///\param data the event
  ///
  void dispatch(std::shared_ptr<vda5050pp::events::ActionStatusRunning> data) noexcept(true);

  ///
  ///\brief Dispatch an ActionStatus event
  ///
  ///\param data the event
  ///
  void dispatch(std::shared_ptr<vda5050pp::events::ActionStatusPaused> data) noexcept(true);

  ///
  ///\brief Dispatch an ActionStatus event
  ///
  ///\param data the event
  ///
  void dispatch(std::shared_ptr<vda5050pp::events::ActionStatusFinished> data) noexcept(true);

  ///
  ///\brief Dispatch an ActionStatus event
  ///
  ///\param data the event
  ///
  void dispatch(std::shared_ptr<vda5050pp::events::ActionStatusFailed> data) noexcept(true);

  ///
  ///\brief Get a scoped subscriber for this event manager.
  ///
  ///\return ScopedActionStatusSubscriber
  ///
  ScopedActionStatusSubscriber getScopedActionStatusSubscriber() noexcept(true);

  /// @brief Stop and join the thread associated with this event manager
  void join() {
    if (thread_.joinable()) {
      thread_.stop();
      thread_.join();
    }
  }
};

}  // namespace vda5050pp::core

#endif  // PRIVATE_VDA5050_2B_2B_CORE_ACTION_STATUS_MANAGER_H_

//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_2B_2B_CORE_STATE_STATE_UPDATE_TIMER_H_
#define VDA5050_2B_2B_CORE_STATE_STATE_UPDATE_TIMER_H_

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>

#include "vda5050++/core/common/interruptable_timer.h"
#include "vda5050++/core/common/scoped_thread.h"
#include "vda5050++/core/module.h"
#include "vda5050++/core/state/state_update_urgency.h"

namespace vda5050pp::core::state {

///
/// \brief The StateUpdateTimer Class has a thread, that periodically sends a new state.
/// Upon requests this period might be decreased.
///
class StateUpdateTimer : public vda5050pp::core::Module {
private:
  using TimePointT = std::chrono::system_clock::time_point;
  using DurationT = std::chrono::system_clock::duration;

  std::optional<
      vda5050pp::core::GenericEventManager<vda5050pp::core::events::StateEvent>::ScopedSubscriber>
      state_subscriber_;

  vda5050pp::core::common::InterruptableTimer timer_;     // Must be destroyed after thread_
  vda5050pp::core::common::ScopedThread<void()> thread_;  // Must be destroyed before timer_

  TimePointT last_sent_;
  std::optional<TimePointT> next_scheduled_update_;
  std::mutex next_scheduled_update_lock_;

  ///
  ///\brief The timer routine, which will wake up on the next_scheduled_update time point and
  /// performs the state update. The timer routime will also wake up to adjust the next scheduled
  /// update time point or exit if the stop_token is set.
  ///
  ///\param stop_token the stop token used to signal the routine to exit
  ///
  void timerRoutine(vda5050pp::core::common::StopToken stop_token);

  ///
  ///\brief Handle a RequestStateUpdate event, i.e. update the next scheduled update time point
  /// to satisfy the request
  ///
  ///\param data the event
  ///
  ///\throws VDA5050PPInvalidEventData if the event is empty
  ///
  void handleRequestStateUpdateEvent(
      std::shared_ptr<vda5050pp::core::events::RequestStateUpdateEvent> data) noexcept(false);

  ///
  ///\brief Do the actual state update, i.e. dump the state and dispatch a message event
  ///
  void doStateUpdate() const;

public:
  StateUpdateTimer();

  ///
  ///\brief Initialize the timer. This will start the timer thread and set up the subscriber.
  ///
  ///\param instance the current library instance
  ///
  void initialize(vda5050pp::core::Instance &instance) override;

  ///
  ///\brief Stop the timer and unsubscribe the subscriber.
  ///
  ///\param instance the current library instance
  ///
  void deinitialize(vda5050pp::core::Instance &instance) override;

  ///
  ///\brief Get a brief description of the module (possibly deprecated)
  ///
  ///\return std::string_view the description
  ///
  std::string_view describe() const override;

  ///
  ///\brief Generate the sub-config for the state update timer
  ///
  ///\return std::shared_ptr<vda5050pp::config::ModuleSubConfig> the sub-config
  ///
  std::shared_ptr<vda5050pp::config::ModuleSubConfig> generateSubConfig() const override;

  ///
  /// \brief Request an update. The next update time point might be set to be sooner.
  ///
  /// \param urgency the urgency of the request
  ///
  void requestUpdate(StateUpdateUrgency urgency) noexcept(true);
};

}  // namespace vda5050pp::core::state

#endif  // VDA5050_2B_2B_CORE_STATE_STATE_UPDATE_TIMER_H_

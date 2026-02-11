//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_2B_2B_CORE_STATE_STATE_UPDATE_URGENCY_H_
#define VDA5050_2B_2B_CORE_STATE_STATE_UPDATE_URGENCY_H_

#include <chrono>

namespace vda5050pp::core::state {

///
///\brief Instance of this class can be send to the StateUpdateTimer
/// request an update in the specified interval.
///
class StateUpdateUrgency {
private:
  std::chrono::system_clock::duration max_delay_ = std::chrono::seconds(0);

public:
  StateUpdateUrgency() = default;
  ///
  ///\brief Construct a new urgency with a given maximum delay until the next update
  ///
  ///\param max_delay the delay until the next update
  ///
  explicit StateUpdateUrgency(std::chrono::system_clock::duration max_delay);

  ///
  ///\brief Request an immediate update, waking up the timer thread and instantly dispatching the
  /// update.
  ///
  ///\return StateUpdateUrgency the immediate urgency
  ///
  static StateUpdateUrgency immediate();

  ///
  ///\brief Request an update with high urgency (100ms until next update)
  ///
  ///\return StateUpdateUrgency the urgency
  ///
  static StateUpdateUrgency high();

  ///
  ///\brief Request an update with medium urgency (1s until next update)
  ///
  ///\return StateUpdateUrgency the urgency
  ///
  static StateUpdateUrgency medium();

  ///
  ///\brief Request an update with medium urgency (5s until next update)
  ///
  ///\return StateUpdateUrgency the urgency
  ///
  static StateUpdateUrgency low();

  ///
  ///\brief Request an update with a custom urgency
  ///
  ///\param max_delay the maximum delay until the next update
  ///\return StateUpdateUrgency the urgency
  ///
  static StateUpdateUrgency custom(std::chrono::system_clock::duration max_delay);

  ///
  ///\brief Get the max delay of the urgency
  ///
  ///\return std::chrono::system_clock::duration the max delay
  ///
  std::chrono::system_clock::duration getMaxDelay() const;

  ///
  ///\brief Check if the update must be dispatched immediately
  ///
  ///\return true if the update must be dispatched immediately
  ///
  bool isImmediate() const;
};

}  // namespace vda5050pp::core::state

#endif  // VDA5050_2B_2B_CORE_STATE_STATE_UPDATE_URGENCY_H_

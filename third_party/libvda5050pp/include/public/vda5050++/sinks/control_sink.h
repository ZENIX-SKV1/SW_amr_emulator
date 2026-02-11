//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//

#ifndef PUBLIC_VDA5050_2B_2B_SINKS_CONTROL_SINK_H_
#define PUBLIC_VDA5050_2B_2B_SINKS_CONTROL_SINK_H_

#include <optional>
#include <string_view>

namespace vda5050pp::sinks {

///
///\brief The ControlSink can be used to control the librarie's scheduler.
/// Pausing/resuming/canceling the current order. Note that these actions won't be reflected by
/// actions in the state and may interfere with the MC's state. Use with care.
///
class ControlSink {
public:
  ///
  ///\brief Initiate a pause action.
  /// This will add a "internal_startPause_<id>" action to the current state, and behaves the same
  /// as receiving a "startPause" action from the MC.
  ///
  ///\param reason an optional reason for the pause action reflected in the action's description.
  ///\param blocking if true, the function will block until the order is paused. If false, it will
  /// return immediately.
  ///
  void initiatePause(std::optional<std::string_view> reason = std::nullopt,
                     bool blocking = false) const noexcept(false);

  ///
  ///\brief Initiate a resume action.
  /// This will add a "internal_stopPause_<id>" action to the current state, and behaves the same
  /// as receiving a "stopPause" action from the MC.
  ///
  ///\param reason an optional reason for the resume action reflected in the action's description.
  ///\param blocking if true, the function will block until the order is resumed. If false, it will
  /// return immediately.
  ///
  void initiateResume(std::optional<std::string_view> reason = std::nullopt,
                      bool blocking = false) const noexcept(false);

  ///
  ///\brief Initiate a cancel action.
  /// This will add a "internal_cancelOrder_<id>" action to the current state, and behaves the same
  /// as receiving a "cancelOrder" action from the MC.
  ///
  ///\param reason an optional reason for the cancel action reflected in the action's description.
  ///\param blocking if true, the function will block until the order is canceled. If false, it will
  /// return immediately.
  ///
  void initiateCancel(std::optional<std::string_view> reason = std::nullopt,
                      bool blocking = false) const noexcept(false);
};
}  // namespace vda5050pp::sinks

#endif

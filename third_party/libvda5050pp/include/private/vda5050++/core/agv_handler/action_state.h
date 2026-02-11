//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_2B_2B_CORE_AGV_HANDLER_ACTION_STATE_H_
#define VDA5050_2B_2B_CORE_AGV_HANDLER_ACTION_STATE_H_

#include <list>

#include "vda5050++/handler/action_state.h"

namespace vda5050pp::core::agv_handler {

///
///\brief This class represents the state of an action that is being handled by the
/// ActionEventHandler.
///
/// An instance is passed to the user for each running action.
/// Note, that this is a specialization of the Interface's ActionState, such,
/// that it cannot be instanciated by the user by accident.
///
class ActionState : public vda5050pp::handler::ActionState {
public:
  ///
  ///\brief Construct for an Action
  ///
  ///\param action the associated action
  ///
  explicit ActionState(std::shared_ptr<const vda5050::Action> action) noexcept(true);

  ~ActionState() override = default;

  ///
  ///\brief Set the associated action to RUNNING
  ///
  void setRunning() noexcept(false) override;

  ///
  ///\brief Set the associated action to PAUSED
  ///
  void setPaused() noexcept(false) override;

  ///
  ///\brief Set the associated action to FINISHED
  ///
  void setFinished() noexcept(false) override;

  ///
  ///\brief Set the associated action to FINISHED with an result_code
  ///
  ///\param result_code the result of the action
  ///
  void setFinished(std::string_view result_code) noexcept(false) override;

  ///
  ///\brief Set the associated action to FAILED
  ///
  void setFailed() noexcept(false) override;

  ///
  ///\brief Set the associated action to FAILED with a list of errors to add to the state
  ///
  ///\param errors a list of errors associated to the action's failure.
  ///
  void setFailed(const std::list<vda5050::Error> &errors) noexcept(false) override;
};

}  // namespace vda5050pp::core::agv_handler

#endif  // VDA5050_2B_2B_CORE_AGV_HANDLER_ACTION_STATE_H_

//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_2B_2B_CORE_VALIDATION_VALIDATION_EVENT_HANDLER_H_
#define VDA5050_2B_2B_CORE_VALIDATION_VALIDATION_EVENT_HANDLER_H_

#include <optional>

#include "vda5050++/core/events/validation_event.h"
#include "vda5050++/core/module.h"

namespace vda5050pp::core::validation {

///
///\brief The ValidationEventHandler module is responsible for handling validation events.
/// It runs all the necessary checks and aggregates the results.
///
class ValidationEventHandler : public vda5050pp::core::Module {
private:
  std::optional<vda5050pp::core::GenericEventManager<
      vda5050pp::core::events::ValidationEvent>::ScopedSubscriber>
      subscriber_;

  ///
  ///\brief Handle a ValidateOrderEvent.
  ///
  /// Checks:
  /// - checkOrderGraphConsistency
  /// - checkOrderId
  /// - checkOrderAppend
  /// - checkOrderActionIds
  /// - action validation for each action
  ///
  ///\param evt the event (result will be set)
  ///
  void handleValidateOrder(std::shared_ptr<vda5050pp::core::events::ValidateOrderEvent> evt) const;

  ///
  ///\brief Handle a ValidateInstantActionsEvent.
  ///
  /// Runs all action checks for each action in the event.
  /// Failed instant actions will be marked as failed in the state.actionStates array.
  ///
  ///\param evt the event (result will be set)
  ///
  void handleValidateInstantActions(
      std::shared_ptr<vda5050pp::core::events::ValidateInstantActionsEvent> evt) const;

public:
  ///
  ///\brief Setup all subscribers.
  ///
  ///\param instance the current library instance
  ///
  void initialize(vda5050pp::core::Instance &instance) override;

  ///
  ///\brief Unsubscribe all subscribers.
  ///
  ///\param instance the current library instance
  ///
  void deinitialize(vda5050pp::core::Instance &instance) override;

  ///
  ///\brief Get a brief description of the module. (possible deprecated)
  ///
  ///\return std::string_view the module' description
  ///
  std::string_view describe() const override;
};

}  // namespace vda5050pp::core::validation

#endif  // VDA5050_2B_2B_CORE_VALIDATION_VALIDATION_EVENT_HANDLER_H_

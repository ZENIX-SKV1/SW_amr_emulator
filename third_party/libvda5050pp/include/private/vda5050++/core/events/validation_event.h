//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_2B_2B_CORE_EVENTS_VALIDATION_EVENT_H_
#define VDA5050_2B_2B_CORE_EVENTS_VALIDATION_EVENT_H_

#include <vda5050/Error.h>
#include <vda5050/InstantActions.h>
#include <vda5050/Order.h>

#include <list>

#include "vda5050++/events/event_type.h"
#include "vda5050++/events/synchronized_event.h"

namespace vda5050pp::core::events {

///
///\brief The identifier type for ValidationEvents
///
///
enum class ValidationEventType {
  k_validate_order,
  k_validate_instant_actions,
};

///
///\brief ValidationEvents are dispatched, when a message shall be validated.
///
struct ValidationEvent : public vda5050pp::events::Event<ValidationEventType> {};

using ValidationResult = std::list<vda5050::Error>;

///
///\brief This event can be generated when a new order shall be validated.
/// It's synchonous, i.e. the sender can wait for a validation result of the order.
///
struct ValidateOrderEvent
    : public vda5050pp::events::EventId<ValidationEvent, ValidationEventType::k_validate_order>,
      public vda5050pp::events::SynchronizedEvent<ValidationResult> {
  ///\brief the order to validate
  std::shared_ptr<const vda5050::Order> order;
};

///
///\brief This event can be generated when a new instant actions message shall be validated.
/// It's synchonous, i.e. the sender can wait for a validation result of the instant actions.
///
struct ValidateInstantActionsEvent
    : public vda5050pp::events::EventId<ValidationEvent,
                                        ValidationEventType::k_validate_instant_actions>,
      public vda5050pp::events::SynchronizedEvent<ValidationResult> {
  ///\brief the instant actions to validate
  std::shared_ptr<const vda5050::InstantActions> instant_actions;
};

}  // namespace vda5050pp::core::events

#endif  // VDA5050_2B_2B_CORE_EVENTS_VALIDATION_EVENT_H_

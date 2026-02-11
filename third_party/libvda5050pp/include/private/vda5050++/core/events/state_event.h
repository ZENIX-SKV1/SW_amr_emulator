//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_2B_2B_CORE_EVENTS_STATE_EVENT_H_
#define VDA5050_2B_2B_CORE_EVENTS_STATE_EVENT_H_

#include "vda5050++/core/state/state_update_urgency.h"
#include "vda5050++/events/event_type.h"
#include "vda5050++/events/synchronized_event.h"

namespace vda5050pp::core::events {

///
///\brief The identifier type for StateEvents
///
enum class StateEventType {
  k_request_state_update,
  k_operating_mode_changed,
};

///
///\brief State events are received by the state module.
///
struct StateEvent : public vda5050pp::events::Event<StateEventType> {};

///
///\brief This event can be dispatched, when a new state update should be triggered.
///
struct RequestStateUpdateEvent
    : public vda5050pp::events::EventId<StateEvent, StateEventType::k_request_state_update>,
      vda5050pp::events::SynchronizedEvent<void> {
  ///\brief the urgency of the state update. It can be set to different priorities
  /// to indicate a time, in which the update shall be sent.
  /// This enables "batching" of multiple concurrent state updates.
  vda5050pp::core::state::StateUpdateUrgency urgency;
};

///
///\brief This event is dispatched when the operating mode *changed*
///
struct OperatingModeChangedEvent
    : public vda5050pp::events::EventId<StateEvent, StateEventType::k_operating_mode_changed> {
  ///
  ///\brief The new operating mode
  ///
  vda5050::OperatingMode operating_mode;
};

}  // namespace vda5050pp::core::events

#endif  // VDA5050_2B_2B_CORE_EVENTS_STATE_EVENT_H_

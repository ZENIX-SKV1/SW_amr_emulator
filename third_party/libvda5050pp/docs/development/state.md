## Subscribed Events

| Event                                               | Behavior                                                                                                                            |
| --------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------- |
| `vda5050pp::core::events::YieldGraphExtension`      | Update graph and increase `orderUpdatedId`                                                                                          |
| `vda5050pp::core::events::YieldGraphReplacement`    | Replace graph and update `orderId`, also resets `orderUpdateId`                                                                     |
| `vda5050pp::core::events::YieldNewAction`           | Add an `vda5050::ActionState` state for this action                                                                                 |
| `vda5050pp::core::events::YieldClearActions`        | Clear all `vda5050::ActionState`s                                                                                                   |
| `vda5050pp::core::events::OrderNewLastNodeId`       | Update the saved `lastNodeId`                                                                                                       |
| `vda5050pp::core::events::OrderActionStatusChanged` | Update a `vda5050::ActionState`                                                                                                     |
| `vda5050pp::core::events::OrderStatus`              | Remember it and set `vda5050::State::paused` accordingly                                                                            |
| `vda5050pp::core::events::OrderClearAfterCancel`    | Clear the graph, set all `WAITING` actions to `FAILED`                                                                              |
| `vda5050pp::core::events::OrderClearAfterReset`     | Clear the graph, set all `WAITING` actions to `FAILED`. (Currently this is the same as `ClearAfterCancel`, so it might get removed) |
| `vda5050pp::events::NavigationStatus`               | Update the status for the concrete event type                                                                                       |
| `vda5050pp::events::NavigationStatusNodeReached`    | Special case: Update the graph                                                                                                      |
| `vda5050pp::events::StatusEvent`                    | Store the data of the event                                                                                                         |

## Dispatched Events

| Event                                                    | Cause                                                                                                                                                        |
| -------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `vda5050pp::events::NavigationBaseIncrease`              | When the graph's base was updated (send added Nodes and Edges)                                                                                               |
| `vda5050pp::events::OrderBaseChanged`                    | When the graph's base was updated (send whole base)                                                                                                          |
| `vda5050pp::events::NavigationHorizonUpdate`             | When the graph's horizon was updated (send horizon)                                                                                                          |
| `vda5050pp::events::OrderHorizonChanged`                 | When the graph's horizon was updated (send horizon, currently the same as `NavigationHorizonUpdate`)                                                         |
| `vda5050pp::events::OrderLastNodeChanged`                | When the `lastNodeId` of the AGV changed                                                                                                                     |
| `vda5050pp::events::OrderActionStatesChanged`            | When an `vda5050::ActionState` has changed                                                                                                                   |
| `vda5050pp::events::RequestStateUpdateEvent`             | Data **changed** according to [VDA5050 6.10](https://github.com/VDA5050/VDA5050/blob/release/2.1.0/VDA5050_EN.md#610-topic-state-from-agv-to-master-control) |
| `vda5050pp::core::events::SendStateMessageEvent`         | The [StateUpdateTimer](#stateupdatetimer) has scheduled an state update                                                                                      |
| `vda5050pp::core::events::SendVisualizationMessageEvent` | The [VisualizationTimer](#stateupdatetimer) has scheduled an visualization message                                                                           |


## Overview

The `vda5050pp::core::state::StateEventHandler` module manages the events that are subscribed and dispatched. It essentially manages the data using:

- The [`vda5050pp::core::state::OrderManager`](/doxygen/html/classvda5050pp_1_1core_1_1state_1_1OrderManager.html) for order related data, i.e. action states and the graph
- The [`vda5050pp::core::state::StatusManager`](/doxygen/html/classvda5050pp_1_1core_1_1state_1_1StatusManager.html) for all auxiliary data like loads and battery states
- The [`vda5050pp::core::state::MapManager`](/doxygen/html/classvda5050pp_1_1core_1_1state_1_1MapManager.html) for map related data


## StateUpdateTimer

The [StateUpdateTimer](/doxygen/html/classvda5050pp_1_1core_1_1state_1_1StateUpdateTimer.html) keeps track of the periodic state update messages,
that have to be dispatched, as defined in VDA5050. The maximum time between two update messages is configured [here](../configuration.md#modulestateupdatetimer-subtable).

To achieve this a dedicated thread sleeps until the next time point where an update is required. Once it wakes up, a `vda5050pp::core::events::SendStateMessageEvent`
is generated from the contents of the [`vda5050pp::core::OrderManager`](/doxygen/html/classvda5050pp_1_1core_1_1state_1_1OrderManager.html)
and [`vda5050pp::core::StatusManager`](/doxygen/html/classvda5050pp_1_1core_1_1state_1_1StatusManager.html) by calling their `dumpTo()` functions.

When an earlier state update is required, the `vda5050pp::events::RequestStateUpdateEvent` is dispatched. It contains
and _urgency_ duration, that requires an state update within that duration. This allows the `StateUpdateTimer` to group 
multiple simultaneous updates into one. For example there are two requests: one with `100ms` and one with `120ms` will result
in an update in `100ms`.

Upon receiving the event the `StateUpdateTimer`'s thread is interrupted and is put to sleep again until the updated time-point.
If the _urgency_ is 0 seconds i.e. immediate, `vda5050pp::core::events::SendStateMessageEvent` is sent before setting the new wake up time-point.

## VisualizationTimer

The [VisualizationTimer](/doxygen/html/classvda5050pp_1_1core_1_1state_1_1VisualizationTimer.html) keeps track of the periodic visualization messages,
that can be dispatched, as defined in VDA5050. The time between two visualization messages is configured [here](../configuration.md#modulevisualizationtimer-subtable).

It is using a dedicated thread, like the `StateUpdateTimer`. However it's simpler, because no earlier wake-ups are required.
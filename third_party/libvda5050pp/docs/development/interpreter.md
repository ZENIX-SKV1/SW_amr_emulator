## Subscribed Events 

| Event                                                      | Behavior                                                                                                                                         |
| ---------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------ |
| `vda5050pp::core::events::ValidInstantActionMessageEvent`  | Dispatch `vda5050pp::core::events::YieldNewAction` events. Then either run the actions as [control](#control-actions) or scheduler actions.      |
| `vda5050pp::core::events::ValidOrderMessageEvent`          | Interpret the order and dispatch `Yield` events according to it.                                                                                 |
| `vda5050pp::events::ActionValidate`                        | If a [control](#control-actions) action is validated, check it and [fill](events.md#synchronized-events-obtaining-event-results) in the results. |
| `vda5050pp::core::events::FactsheetControlActionListEvent` | [Fill](events.md#synchronized-events-obtaining-event-results) the declarations of each control action.                                           |

## Dispatched Events


| Event                                            | Cause                                                               |
| ------------------------------------------------ | ------------------------------------------------------------------- |
| `vda5050pp::core::events::YieldNewAction`        | for each encountered action in any order or instant actions message |
| `vda5050pp::core::events::YieldActionGroup`      | for each encountered action group in any order                      |
| `vda5050pp::core::events::YieldNavigationStep`   | for each encountered navigation step in any order                   |
| `vda5050pp::core::events::YieldGraphReplacement` | once for each order received                                        |
| `vda5050pp::core::events::YieldGraphExtension`   | alternatively to the above if the order extends                     |
| `vda5050pp::core::events::InterpreterDone`       | interpretation of an order has finished                             |

## Control Actions

Control actions are instant actions that require special treatment and must be handled by the library.
Currently these actions are handled as control instant actions:

- `cancelOrder`:
- `startPause`
- `stopPause`
- `factsheetRequest`
- `stateRequest`

Since those actions require a high level view onto the components of the library in order to function,
multiple events have to be dispatched and received. For example, when canceling the current order:

1. the action must be set to running
2. an `vda5050pp::core::events::InterpreterOrderControl` event with a cancel request must be dispatched
3. wait until the scheduler dispatches an `vda5050pp::core::events::OrderStatus` event with an idle flag
4. the action must be set to finished

To avoid using a dedicated thread for the purpose of handling this flow of events,
[`EventControlBlock`](/doxygen/html/classvda5050pp_1_1core_1_1events_1_1EventControlBlock.html)s
were added.
An `EventControlBlock` sets up dedicated subscribers with it's virtual `enable()` function.
With a `teardown()` function the EventControlBlock can be disabled and cleaned-up from outside.

The `EventLatch<EventT>` block for example subscribes a specified `EventT` and runs a virtual `predicate()`
for each received event. Once it returns true, the virtual `done()` function is called. This allows
for chaining of different blocks. Together with other blocks like 
`EventControlChain`, `EventControlParallel` or the simple `FunctionBlock` complex event flows can be constructed.

The Interpreter constructs individual `EventControlBlock`s on the fly to handle each control instant action.


## Order Interpretation

The key idea in interpreting the order is, that atomic tasks are extracted, following
[VDA5050 6.12](https://github.com/VDA5050/VDA5050/blob/release/2.1.0/VDA5050_EN.md#612-action-blocking-types-and-sequence).
Atomic tasks are:
- A group of actions that can be executed in parallel (by blocking type)
- Navigation to a goal node via an edge
- Navigation of a Segment, i.e. a list of nodes and edges, that can be traversed without stopping.
  This means there are no `HARD` blocking actions on them.

The interpreter uses a set of iterators on objects of the valid `vda5050::Order` object.
Those iterators are tied together in the [`EventIter`](/doxygen/html/classvda5050pp_1_1core_1_1interpreter_1_1EventIter.html),
it essentially holds a `vda5050::Node`, a `vda5050::Edge` and an `vda5050::Action` iterator for the current node/edge
and action on the node/edge.

Using the `vda5050pp::core::interpreter::nextEvent()` function, the order can be interpreted step-by-step.
It the returns following events in order they shall be executed:

- `vda5050pp::core::events::YieldNewAction` for each encountered action
- `vda5050pp::core::events::YieldActionGroup` for each group of actions encountered, that can be executed simultaneously
- `vda5050pp::core::events::YieldNavigationStep` for each encountered navigation step (to a goal node via an edge).
  A `stop_at_goal` indicates, that the AGV has to stop once it reaches the goal. I.e. there is a blocking action at the node (or the edge leading away from the node).
- `vda5050pp::core::events::YieldGraphReplacement` or `vda5050pp::core::events::YieldGraphExtension` is dispatched in the end. They contain the graph of the order.
- `vda5050pp::core::events::InterpreterDone` as the last event for this order and tells the scheduler, that it can start processing the received events.

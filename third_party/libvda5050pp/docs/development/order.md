## Subscribed Events

| Event                                               | Behavior                                                                          |
| --------------------------------------------------- | --------------------------------------------------------------------------------- |
| `vda5050pp::core::events::YieldActionGroupEvent`    | Put the action group into the staging queue of the scheduler.                     |
| `vda5050pp::core::events::YieldNavigationStepEvent` | Put the navigation step into the staging queue of the scheduler.                  |
| `vda5050pp::core::events::YieldInstantActionGroup`  | Enqueue the actions into the interrupt queue of the scheduler.                    |
| `vda5050pp::core::events::YieldNewAction`           | Forward action as an `vda5050pp::events::ActionPrepare` event.                    |
| `vda5050pp::core::events::InterpreterDone`          | Commit the staging queue of the scheduler to the processing queue.                |
| `vda5050pp::core::events::InterpreterOrderControl`  | Pause/Resume/Cancel the scheduler.                                                |
| `vda5050pp::events::ActionStatusEvent`s             | Tell the scheduler, that the status of an (non-control) action has changed.       |
| `vda5050pp::events::NavigationStatusEvent`s         | Tell the scheduler, that the status of navigation has changed (node reached ect.) |
| `vda5050pp::events::OperatingModeChangedEvent`      | If the operating mode changed to `MANUAL`, reset the scheduler.                   |

## Dispatched Events

| Event                                               | Cause                                                                     |
| --------------------------------------------------- | ------------------------------------------------------------------------- |
| `vda5050pp::events::ActionEvent`                    | An action task changed it's state and the user has to handle it           |
| `vda5050pp::events::NavigationEvent`                | The navigation task changed it's state and the user has to handle it      |
| `vda5050pp::core::events::OrderActionStatusChanged` | The status of an action has changed                                       |
| `vda5050pp::core::events::OrderNewLastNodeId`       | A navigation task was finished                                            |
| `vda5050pp::core::events::OrderStatus`              | The scheduler has changed it's state                                      |
| `vda5050pp::core::events::OrderClearAfterCancel`    | The scheduler was canceled and wants the order to be cleared in the state |
| `vda5050pp::core::events::OrderClearAfterReset`     | The scheduler was reset and wants the order to be cleared in the state    |


## Scheduler

The [`vda5050pp::core::order::Scheduler`](/doxygen/html/classvda5050pp_1_1core_1_1order_1_1Scheduler.html) contains
the _main_ order processing logic of the libVDA5050++. It receives atomic steps from the
[Interpreter](interpreter.md) and executes them in order.

The scheduler itself does not listen to events, this is done by the
[`vda5050pp::core::order::OrderEventHandler`](/doxygen/html/classvda5050pp_1_1core_1_1order_1_1OrderEventHandler.html)
which forwards the calls to a scheduler object.


| State/Action | cancel                           | pause                         | resume                          | interrupt                                     | update                                | reset                  |
| ------------ | -------------------------------- | ----------------------------- | ------------------------------- | --------------------------------------------- | ------------------------------------- | ---------------------- |
| Idle         | Goto Canceling                   | Goto IdlePaused               | /                               | Goto Interrupting                             | Fetch next task and maybe goto Active | Goto Idle              |
| IdlePaused   | Clear queues                     | /                             | -                               | Goto Idle                                     | NOP                                   | Goto IdlePaused        |
| Active       | Cancel all tasks, goto Canceling | Pause all tasks, goto Pausing | /                               | [Interrupt](#interrupting), goto Interrupting | [Update Tasks](#update-tasks)         | Reset tasks, goto Idle |
| Canceling    | /                                | /                             | /                               | /                                             | [Update Tasks](#update-tasks)         | Reset tasks, goto Idle |
| Resuming     | /                                | /                             | /                               | /                                             | [Update Tasks](#update-tasks)         | Reset tasks, goto Idle |
| Pausing      | /                                | /                             | /                               | /                                             | [Update Tasks](#update-tasks)         | Reset tasks, goto Idle |
| Paused       | Cancel all tasks, goto Canceling | /                             | Resume all tasks, goto Resuming | /                                             | [Update Tasks](#update-tasks)         | Reset tasks, goto Idle |
| Failed       | /                                | /                             | /                               | /                                             | NOP                                   | Reset tasks, goto Idle |
| Interrupting | /                                | /                             | /                               | /                                             | [Update Tasks](#update-tasks)         | Reset tasks, goto Idle |

### Update Tasks

When updating tasks, they are re-/moved between the running active and paused container.
Running tasks will be in the running container, paused tasks in the paused container. All non-terminal tasks
(not finished/failed) are in the active mapping.

- During normal operation the tasks are updated and new ones are fetched. When all tasks are in a paused state,
  the scheduler will be in a paused state. When there are no longer active tasks, the scheduler will be in an idle state.
- During cancel tasks will be updated as usual. However fetching new tasks will only fetch navigation tasks (without actions)
  to allow a _graceful_ stop (since future tasks may already be known due to segments). When there are no longer active tasks, goto
  Idle.
- During pausing updating and pausing is analogous to during canceling. When all tasks are paused or no task is active, goto Idle.
- During resuming the tasks are updated as usual. No new tasks are fetched. When no task is paused, goto Active.
- During interrupting only tasks from the interrupt queue are fetched.


### Interrupting

The scheduler can be interrupted by instant actions. So when the `vda5050pp::core::events::YieldInstantActionGroup` event is received,
it is put ino the interrupt queue with `vda5050pp::core::Scheduler::enqueueInterruptActions()`. Upon calling this function, the scheduler
will cancel active action tasks and pause the active navigation task based on the blocking types of the interrupting actions.
Afterwards it tries to transition into the `Interrupting` state. Once in this state, only actions from the interrupt queue are fetched until
it is empty. When the queue is empty the scheduler will go back to the `Active` state and resume normal operation.

Should the scheduler be interrupted during it's `Interrupting` state, again all running action tasks will be canceled
and the interrupting actions take precedence.


## ActionTask

The Scheduler manages actions with [`ActionTask`](/doxygen/html/classvda5050pp_1_1core_1_1order_1_1ActionTask.html) objects.
Each ActionTask represents a unique action. The ActionTask is essentially a state
machine implementing a superset of [VDA5050 6.11](https://github.com/VDA5050/VDA5050/blob/release/2.1.0/VDA5050_EN.md#611-action-states).
The state machine additionally includes intermediate states like `Pausing` to indicate the AGV is currently between a `Running` and `Paused` action.
When a `vda5050pp::events::ActionStatusEvent` is received, the Scheduler calls the `transition` function of the associated `ActionTask`.
Then the `effect()` function of the internal state is called to dispatch a `vda5050pp::core::events::OrderActionStatusChanged` event.
Other states, like `Initializing` have the `effect()` to dispatch a `vda5050pp::events::ActionEvent` which tells the user to
start the action.

## NavigationTask

The Scheduler manages navigation steps with [`NavigationTask`](/doxygen/html/classvda5050pp_1_1core_1_1order_1_1NavigationTask.html) objects.
There is at most one navigation task at a time known by the Scheduler. It represents the navigation of the AGV
to the _next_ Node (i.e. goal node) via an edge (i.e. via-edge).
Next to this information the `NavigationTask` may contain a [Segment](interpreter.md#segments) to indicate that multiple
nodes may be traversed after this node (without stopping).

Like the `ActionTask` the `NavigationTask` is a state machine handling
the current status of the navigation. Transitioning a running `NavigationTask` to a finished `NavigationTask` requires a transition
with the matching `sequenceId` of the goal node. The transitions result in `vda5050pp::events::NavigationEvent` and
`vda5050pp::core::events::OrderNewLastNodeId` events. The first ones to notify the user about how to navigate, the second ones to
manifest the last node id in the state.

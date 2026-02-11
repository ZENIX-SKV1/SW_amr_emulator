In general the user is notified about tasks triggered by the [Scheduler](order.md#scheduler) via
events. However there are convenience wrappers for the user called [Handlers](../api.md#handler).

## ActionEventHandler 

The [`ActionEventHandler`](/doxygen/html/classvda5050pp_1_1core_1_1agv__handler_1_1ActionEventHandler.html)
module subscribes to all `vda5050pp::event::ActionEvent`s and tries to use the
registered handlers to respond to those events.

#### Subscribed Events

| Event                               | Behavior                                                                                                                                          |
| ----------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------- |
| `vda5050pp::events::ActionValidate` | Call `validate()` for the matching handler. If valid, store the parsed parameters, an [ActionState](#actionstate) and handler in an action store. |
| `vda5050pp::events::ActionList`     | Call `getActionDescription()` for each registered handler.                                                                                        |
| `vda5050pp::events::ActionPrepare`  | Use an registered handler and call `prepare()` to get the callbacks for the action.                                                               |
| `vda5050pp::events::ActionCancel`   | Use an registered handler and call the cancel callback.                                                                                           |
| `vda5050pp::events::ActionPause`    | Use an registered handler and call the pause callback.                                                                                            |
| `vda5050pp::events::ActionResume`   | Use an registered handler and call the resume callback.                                                                                           |
| `vda5050pp::events::ActionStart`    | Use an registered handler and call the start callback.                                                                                            |
| `vda5050pp::events::ActionForget`   | Remove the associated action store.                                                                                                               |
| `vda5050pp::events::ActionReset`    | Call reset on each registered handler and clear all action stores.                                                                                |

#### ActionState

The [`vda5050pp::handler::ActionState`](/doxygen/html/classvda5050pp_1_1handler_1_1ActionState.html) is an abstract class.
It's specialization [`vda5050pp::core::handler::ActionState`](/doxygen/html/classvda5050pp_1_1core_1_1agv__handler_1_1ActionState.html)
is constructed only by the library.  The user can set the state of a running action with objects of this class.
Calling the functions will directly dispatch corresponding `vda5050pp::events::ActionStatus` events.


## NavigationEventHandler

The [`NavigationEventHandler`](/doxygen/html/classvda5050pp_1_1core_1_1agv__handler_1_1NavigationEventHandler.html) module
subscribes to all `vda5050pp::event::NavigationEvent`s and uses the registered navigation handler to respond to
those events.

#### Subscribed Events

| Event                                          | Behavior                                                             |
| ---------------------------------------------- | -------------------------------------------------------------------- |
| `vda5050pp::events::NavigationHorizonUpdate`   | Call `horizonUpdated()` for the navigation handler.                  |
| `vda5050pp::events::NavigationBaseIncreased`   | Call `baseIncreased()` for the navigation handler.                   |
| `vda5050pp::events::NavigationNextNode`        | Call `navigateToNextNode()` for the navigation handler.              |
| `vda5050pp::events::NavigationUpcomingSegment` | Call `upcomingSegment()` for the navigation handler.                 |
| `vda5050pp::events::NavigationControl`         | Call `pause()`, `resume()` or `cancel()` for the navigation handler. |
| `vda5050pp::events::NavigationReset`           | Call `reset()` for the navigation handler.                           |


## QueryEventHandler

The [`QueryEventHandler`](/doxygen/html/classvda5050pp_1_1core_1_1agv__handler_1_1QueryEventHandler.html) module
subscribes to all `vda5050pp::event::QueryEvent`s and uses the registered query handler to respond to
those events.

#### Subscribed Events

| Event                                            | Behavior                                                                                                           |
| ------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------ |
| `vda5050pp::events::QueryPauseable`              | Call query handler `queryPauseable()` if overridden, use this result. Otherwise fall back to default.              |
| `vda5050pp::events::QueryResumable`              | Call query handler `queryResumable()` if overridden, use this result. Otherwise fall back to default.              |
| `vda5050pp::events::QueryAcceptZoneSet`          | Call query handler `queryAcceptZoneSet()` if overridden, use this result. Otherwise fall back to default.          |
| `vda5050pp::events::QueryNodeTriviallyReachable` | Call query handler `queryNodeTriviallyReachable()` if overridden, use this result. Otherwise fall back to default. |


## NodeReachedHandler

The [`NodeReachedHandler`](/doxygen/html/classvda5050pp_1_1core_1_1agv__handler_1_1NodeReachedHandler.html) module
listens to `vda5050pp::events::NavigationStatusPosition` events and checks if the position is on the currently pursued goal node.
The check is only performed if explicitly specified in the event.
If the check succeeds a `vda5050pp::events::NavigationStatusNodeReached` event is dispatched.

The AGV is assumed to be on the node according to the following table:

| NodePosition available | NodeTheta available | NodeDeviation available | AGVPosition.deviationRange available | Performed Check                                                                                                                |
| ---------------------- | ------------------- | ----------------------- | ------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------ |
| N                      | Y/N                 | Y/N                     | Y/N                                  | No check performed.                                                                                                            |
| Y                      | N                   | N                       | N                                    | AGVPosition inside of default node deviation.                                                                                  |
| Y                      | N                   | N                       | Y                                    | AGVPosition including deviation inside of default node deviation.                                                              |
| Y                      | N                   | Y                       | N                                    | AGVPosition inside of node deviation.                                                                                          |
| Y                      | N                   | Y                       | Y                                    | AGVPosition including deviation inside of node deviation.                                                                      |
| Y                      | Y                   | N                       | N                                    | AGVPosition inside of default node deviation and angle difference is smaller then default theta deviation.                     |
| Y                      | Y                   | N                       | Y                                    | AGVPosition including deviation inside of default node deviation and angle difference is smaller then default theta deviation. |
| Y                      | Y                   | Y                       | N                                    | AGVPosition inside of node deviation and angle difference is smaller then theta deviation.                                     |
| Y                      | Y                   | Y                       | Y                                    | AGVPosition including deviation inside of node deviation and angle difference is smaller then theta deviation.                 |


## DistanceNodeHandler

The [`DistanceNodeHandler`](/doxygen/html/classvda5050pp_1_1core_1_1agv__handler_1_1DistanceNodeHandler.html) module
listens to `vda5050pp::events::NavigationStatusPosition` events interpolates the distance driven
since the last reached node according to a chosen `interpolation_type` (currently NONE or LINEAR).
After each interpolation step, a new `vda5050pp::events::NavigationStatusDistanceSinceLastNode` event is
dispatched (unless the interpolation type is NONE).
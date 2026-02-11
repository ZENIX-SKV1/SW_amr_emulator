## Subscribed Events

| Event                                                  | Behavior                                                                                                                                            |
| ------------------------------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------- |
| `vda5050pp::core::events::ValidateOrderEvent`          | Run all [checks for orders](#order-checks) and [fill](events#synchronized-events--obtaining-event-results) in the results.                          |
| `vda5050pp::core::events::ValidateInstantActionsEvent` | Run all [checks for instant actions](#instant-action-checks-checks) and [fill](events#synchronized-events--obtaining-event-results) in the results. |

## Dispatched Events

| Event                                            | Cause                                                                                                                 |
| ------------------------------------------------ | --------------------------------------------------------------------------------------------------------------------- |
| `vda5050pp::events::ActionValidate`              | An instant action or an action on a node/edge has to be validated.                                                    |
| `vda5050pp::events::QueryAcceptZoneSet`          | An order contains a `zoneSetId` and must be accepted (by the user)                                                    |
| `vda5050pp::events::QueryNodeTriviallyReachable` | When an order has `orderUpdateId == 0` i.e. first node may mismatch, check if the first node is reachable by the AGV. |

## Order Checks

Upon validating an order the following things are checked:

- Header is correct (manufacturer / sn)
- Header version is compatible
- OrderID matching (see [VDA5050 6.6.4.3](https://github.com/VDA5050/VDA5050/blob/release/2.1.0/VDA5050_EN.md#6643-vehicle-gets-a-new-order-with-the-same-orderid-but-a-lower-orderupdateid-than-the-current-orderupdateid))
- Order is not empty
- Order's `sequenceId`s are well formed
- Order's horizon is clearly separated from the base
- An order either replaces the current one if the scheduler is idle or appends to the current one
- All `actionId`s are unique
- All actions are valid

The errors of all checks are gathered in one list. If the list is empty the order is assumed to be correct.
Otherwise the returned errors will be set by the [MessageEventHandler](messages.md).

## Instant Action Checks

Upon validating an order the following things are checked:

- Header is correct (manufacturer / sn)
- Header version is compatible
- All actions are valid

The errors of all checks are gathered in one list. If the list is empty the order is assumed to be correct.
Otherwise the returned errors will be set by the [MessageEventHandler](messages.md).
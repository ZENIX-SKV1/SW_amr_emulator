## Subscribed Events (MessageEventHandler)

| Event                                                       | Behavior                                                                                                           |
| ----------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------ |
| `vda5050pp::core::events::ReceiveOrderMessageEvent`         | Dispatch and collect validation events, then dispatch a `vda5050pp::core::events::ValidOrderMessageEvent`          |
| `vda5050pp::core::events::ReceiveInstantActionMEssageEvent` | Dispatch and collect validation events, then dispatch a `vda5050pp::core::events::ValidInstantActionsMessageEvent` |

## Dispatched Events (MessageEventHandler)

| Event                                                  | Cause                                                                                                         |
| ------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------- |
| `vda5050pp::core::events::ValidateOrderEvent`          | A new order was received and a validation is pending. The result will determine if errors are set.            |
| `vda5050pp::core::events::ValidateInstantActionsEvent` | New instant actions were received and the validation is pending. The result will determine if errors are set. |
| `vda5050pp::events::ErrorAdd`                          | Errors occurred during validation.                                                                            |
| `vda5050pp::events::ValidOrderMessageEvent`            | An received order had no errors after validation.                                                             |
| `vda5050pp::events::ValidInstantActionsMessageEvent`   | Received instant actions had no errors after validation.                                                      |

## Subscribed Events (MQTT Module)

| Event                                                    | Behavior                                                                                                             |
| -------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------- |
| `vda5050pp::core::events::SendStateMessageEvent`         | Fill in the header of the message and send it on the `<interface>/<version>/<manufacturer>/<sn>/state` topic         |
| `vda5050pp::core::events::SendVisualizationMessageEvent` | Fill in the header of the message and send it on the `<interface>/<version>/<manufacturer>/<sn>/visualization` topic |
| `vda5050pp::core::events::SendFactsheetMessageEvent`     | Fill in the header of the message and send it on the `<interface>/<version>/<manufacturer>/<sn>/factsheet` topic     |

## Dispatched Events (MQTT Module)

| Event                                                        | Cause                                                                                                                      |
| ------------------------------------------------------------ | -------------------------------------------------------------------------------------------------------------------------- |
| `vda5050pp::core::events::ReceiveOrderMessageEvent`          | A message on the `<interface>/<version>/<manufacturer>/<sn>/state` topic was received (and syntactically correct)          |
| `vda5050pp::core::events::ReceiveInstantActionsMessageEvent` | A message on the `<interface>/<version>/<manufacturer>/<sn>/instantActions` topic was received (and syntactically correct) |
| `vda5050pp::core::events::MessageErrorEvent`                 | MQTT delivery error or json deserialization error.                                                                         |
| `vda5050pp::core::events::ConnectionChanged`                 | The connection to the MQTT broker changed.                                                                                 |
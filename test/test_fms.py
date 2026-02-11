import paho.mqtt.client as mqtt
import json
import time

# 예시 ORDER 메시지(표준 VDA5050 2.0 Order 구조에 맞게 조정 필요)
order = {
    "header": {
        "version": "2.0.0",
        "manufacturer": "FMS_TEST",
        "serialNumber": "FMS001",
        "timestamp": "2024-07-20T15:00:00.123Z",
        "messageId": "order_0001"
    },
    "orderId": "ORDER_1",
    "nodes": [
        { "nodeId": "A", "sequenceId": 1, "nodePosition": { "x": 0.0, "y": 0.0 } },
        { "nodeId": "B", "sequenceId": 2, "nodePosition": { "x": 2.0, "y": 0.0 } },
        { "nodeId": "C", "sequenceId": 3, "nodePosition": { "x": 4.0, "y": 0.0 } },
        { "nodeId": "D", "sequenceId": 4, "nodePosition": { "x": 6.0, "y": 1.0 } },
        { "nodeId": "E", "sequenceId": 5, "nodePosition": { "x": 6.0, "y": 3.0 } }
    ],
    "edges": [
        { "edgeId": "E1", "sequenceId": 1, "startNodeId": "A", "endNodeId": "B" },
        { "edgeId": "E2", "sequenceId": 2, "startNodeId": "B", "endNodeId": "C" },
        { "edgeId": "E3", "sequenceId": 3, "startNodeId": "C", "endNodeId": "D" },
        { "edgeId": "E4", "sequenceId": 4, "startNodeId": "D", "endNodeId": "E" }
    ]
}

broker = "localhost"
topic = "/agv/001/order"

client = mqtt.Client()
client.connect(broker, 1883, 60)
client.loop_start()
while True:
    client.publish(topic, json.dumps(order))
    print("Published VDA5050 order message")
    time.sleep(10)

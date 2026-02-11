#include <mqtt/async_client.h>
#include <iostream>
#include <string>
#include <chrono>
#include <thread>

const std::string BROKER       = "tcp://localhost:1883";
const std::string CLIENT_ID    = "cpp_fms_test_pub";
const std::string TOPIC        = "/agv/001/order";

// 실제 구동 전 VDA5050 메시지 스펙에 맞는 JSON 구조를 반드시 확인하세요.
const std::string ORDER_PAYLOAD = R"(
{
  "header": {
    "version": "2.0.0",
    "manufacturer": "FMS_CPP",
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
)";

int main() {
    mqtt::async_client client(BROKER, CLIENT_ID);

    mqtt::connect_options conn_opts;
    try {
        std::cout << "Connecting to MQTT broker..." << std::endl;
        client.connect(conn_opts)->wait();
        std::cout << "Connected!" << std::endl;

        mqtt::message_ptr msg = mqtt::make_message(TOPIC, ORDER_PAYLOAD);
        msg->set_qos(1);

        std::cout << "Publishing VDA5050 order..." << std::endl;
        client.publish(msg)->wait();

        std::cout << "Published!" << std::endl;
        // 1초 후 종료
        std::this_thread::sleep_for(std::chrono::seconds(1));
        client.disconnect()->wait();
    }
    catch (const mqtt::exception& exc) {
        std::cerr << exc.what() << std::endl;
        return 1;
    }
    return 0;
}
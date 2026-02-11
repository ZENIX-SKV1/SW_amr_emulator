#include <iostream>
#include <string>
#include <mqtt/async_client.h>

const std::string SERVER_ADDRESS("tcp://localhost:1883");
const std::string CLIENT_ID("fms_client");
const std::string ORDER_TOPIC("vda5050/agvs/amr_0/order");
const std::string STATE_TOPIC("vda5050/agvs/amr_0/state");
const std::string VISUALIZATION_TOPIC("vda5050/agvs/amr_0/visualization");

const int QOS = 1;

// MQTT 콜백 핸들러 클래스 정의
class Callback : public virtual mqtt::callback
{
public:
    void message_arrived(mqtt::const_message_ptr msg) override
    {
        std::cout << "[MQTT] Message arrived on topic: " << msg->get_topic() << std::endl;
        std::cout << "[MQTT] Payload:\n" << msg->to_string() << std::endl << std::endl;
    }

    void connection_lost(const std::string& cause) override
    {
        std::cout << "[MQTT] Connection lost";
        if (!cause.empty())
            std::cout << ", cause: " << cause;
        std::cout << std::endl;
    }
};

int main()
{
    mqtt::async_client client(SERVER_ADDRESS, CLIENT_ID);
    mqtt::connect_options connOpts;
    connOpts.set_clean_session(true);

    Callback cb;
    client.set_callback(cb);

    try {
        std::cout << "Connecting to the MQTT server..." << std::endl;
        mqtt::token_ptr conntok = client.connect(connOpts);
        conntok->wait();
        std::cout << "Connected!" << std::endl;

        // 1. State 토픽 구독
        // std::cout << "Subscribing to state topic: " << STATE_TOPIC << std::endl;
        // client.subscribe(STATE_TOPIC, QOS)->wait();

        // // 2. Visualization 토픽 구독
        // std::cout << "Subscribing to Visualization topic: " << VISUALIZATION_TOPIC << std::endl;       
        // client.subscribe(VISUALIZATION_TOPIC, QOS)->wait();
        client.subscribe("vda5050/agvs/amr_0/#", QOS)->wait();
        // 3. Order 메시지 발행
        std::string payload = R"({
              "headerId": "header_test",
              "timestamp": 1650000000,
              "orderId": "order_test_1",
              "nodes": [
                {"nodeId": "N1", "sequenceId": 0, "nodePosition": {"x":0.0, "y":0.0, "theta":0.0}},
                {"nodeId": "N2", "sequenceId": 1, "nodePosition": {"x":3.0, "y":0.0, "theta":1.0}},
                {"nodeId": "N3", "sequenceId": 2, "nodePosition": {"x":3.0, "y":3.0, "theta":1.0}},
                {"nodeId": "N4", "sequenceId": 3, "nodePosition": {"x":0.0, "y":3.0, "theta":1.0}},
                {"nodeId": "N5", "sequenceId": 4, "nodePosition": {"x":0.0, "y":0.0, "theta":1.0}}
              ],
              "edges": [
                {"edgeId": "E1", "sequenceId": 0, "startNodeId": "N1", "endNodeId": "N2"},
                {"edgeId": "E2", "sequenceId": 1, "startNodeId": "N2", "endNodeId": "N3"},
                {"edgeId": "E3", "sequenceId": 2, "startNodeId": "N3", "endNodeId": "N4"},
                {"edgeId": "E4", "sequenceId": 3, "startNodeId": "N4", "endNodeId": "N5"}
              ]
            })";

        auto msg = mqtt::make_message(ORDER_TOPIC, payload);
        msg->set_qos(QOS);

        std::cout << "Publishing message to topic: " << ORDER_TOPIC << std::endl;
        client.publish(msg)->wait_for(std::chrono::seconds(10));
        std::cout << "Message published!" << std::endl;

        // 4. 상태(state) 메시지 수신 대기 (예: 10초간)
        std::cout << "Waiting for state messages for 10 seconds..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(10));

        // 5. 구독 해제 및 연결 종료
        // client.unsubscribe(VISUALIZATION_TOPIC)->wait();
        // client.unsubscribe(STATE_TOPIC)->wait();
        client.unsubscribe("vda5050/agvs/amr_0/#")->wait();
        client.disconnect()->wait();

        std::cout << "Disconnected." << std::endl;
    }
    catch (const mqtt::exception& exc)
    {
        std::cerr << "Error: " << exc.what() << std::endl;
        return 1;
    }

    return 0;
}
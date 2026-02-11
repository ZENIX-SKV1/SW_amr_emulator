// #include <iostream>
// #include <string>
// #include <chrono>
// #include <thread>
// #include <ctime>
// #include <iomanip> 
// #include <sstream> 
// #include <vector>

// #include <mqtt/client.h> 
// #include <json.hpp> 

// // MQTT 브로커 정보
// const std::string BROKER_ADDRESS = "tcp://127.0.0.1:1883"; // 동일 PC의 MQTT 브로커
// const std::string CLIENT_ID = "test_fms_client"; // FMS 클라이언트 ID

// // VDA 5050 Order 메시지 생성 함수
// std::string createVda5050OrderMessage(int orderId, const std::string& agvId, const std::string& startNode, const std::string& targetNode) {
//     nlohmann::json order_json;
//     order_json["headerId"] = "test-order-header-" + std::to_string(orderId);
    
//     // 현재 UTC 시간 포맷팅
//     auto now = std::chrono::system_clock::now();
//     auto tt = std::chrono::system_clock::to_time_t(now);
//     std::stringstream ss;
//     ss << std::put_time(std::gmtime(&tt), "%Y-%m-%dT%H:%M:%S.000Z");
//     order_json["timestamp"] = ss.str();

//     order_json["version"] = "2.0.0";
//     order_json["manufacturer"] = "TestCompany";
//     order_json["serialNumber"] = agvId;
//     order_json["agvId"] = agvId;
//     order_json["orderId"] = "order-" + std::to_string(orderId);
//     order_json["orderType"] = "NORMAL";

//     nlohmann::json node_path;
//     // 경로 예시: 시작 노드 -> 목표 노드
//     // 실제 VDA5050에서는 노드 위치(x,y,theta) 정보도 포함되어야 함
//     node_path.push_back({{"nodeId", startNode}, {"sequenceId", 0}, {"nodePosition", {{"x", 0.0}, {"y", 0.0}, {"theta", 0.0}}}});
//     node_path.push_back({{"nodeId", targetNode}, {"sequenceId", 1}, {"nodePosition", {{"x", 5.0}, {"y", 0.0}, {"theta", 0.0}}}});

//     order_json["nodes"] = node_path;
//     order_json["edges"] = nlohmann::json::array(); // 간단한 예시에서는 엣지 생략

//     return order_json.dump(2); // 예쁘게 포맷팅하여 반환
// }

// // MQTT 콜백 핸들러 클래스
// class Callback : public virtual mqtt::callback,
//                  public virtual mqtt::iaction_listener
// {
//     void connection_lost(const std::string& cause) override {
//         std::cout << "\nConnection lost" << std::endl;
//         if (!cause.empty())
//             std::cout << "\tcause: " << cause << std::endl;
//     }

//     void delivery_complete(mqtt::delivery_token_ptr tok) override {
//         // 메시지 발행 완료 시 호출됨 (여기서는 Order 발행)
//         // std::cout << "Delivery complete for token: " << (tok ? tok->get_message_id() : -1) << std::endl;
//     }

//     void message_arrived(mqtt::const_message_ptr msg) override {
//         // 메시지 수신 시 호출됨 (여기서는 State 수신)
//         std::cout << "\n-------------------------------------------------" << std::endl;
//         std::cout << "Message arrived on topic: '" << msg->get_topic() << "'" << std::endl;
//         std::cout << "Payload: " << msg->to_string() << std::endl;
//         std::cout << "-------------------------------------------------" << std::endl;

//         // VDA 5050 State 메시지 파싱 및 주요 정보 출력 (선택 사항)
//         try {
//             nlohmann::json state_json = nlohmann::json::parse(msg->to_string());
//             std::cout << "  [Parsed State] AGV ID: " << state_json["agvId"].get<std::string>() << std::endl;
//             if (state_json.count("batteryState") && state_json["batteryState"].count("batteryCharge")) {
//                 std::cout << "  [Parsed State] Battery Charge: " << state_json["batteryState"]["batteryCharge"].get<double>() << std::endl;
//             }
//             if (state_json.count("drivingStatus")) {
//                 std::cout << "  [Parsed State] Driving Status: " << state_json["drivingStatus"].get<std::string>() << std::endl;
//             }
//             if (state_json.count("info") && state_json["info"].is_array()) {
//                 for (const auto& info_item : state_json["info"]) {
//                     if (info_item.count("infoType") && info_item["infoType"] == "current_node") {
//                         std::cout << "  [Parsed State] Current Info: " << info_item["infoDescription"].get<std::string>() << std::endl;
//                     }
//                 }
//             }

//         } catch (const nlohmann::json::parse_error& e) {
//             std::cerr << "  [ERROR] Failed to parse state JSON: " << e.what() << std::endl;
//         } catch (const std::exception& e) {
//             std::cerr << "  [ERROR] Error processing state message: " << e.what() << std::endl;
//         }
//     }

//     void on_success(const mqtt::token& tok) override {
//         // 구독 성공 시 호출됨
//         std::cout << "Subscription success for token: " << tok.get_message_id() << std::endl;
//     }

//     void on_failure(const mqtt::token& tok) override {
//         // 구독 실패 시 호출됨
//         std::cout << "Subscription failed for token: " << tok.get_message_id() << std::endl;
//     }
// };


// int main(int argc, char* argv[]) {
//     mqtt::client client(BROKER_ADDRESS, CLIENT_ID);
//     Callback cb;
//     client.set_callback(cb); // 콜백 핸들러 설정

//     mqtt::connect_options connOpts;
//     connOpts.set_keep_alive_interval(20);
//     connOpts.set_clean_session(true); // 클린 세션으로 접속 (재연결 시 구독 초기화)

//     try {
//         std::cout << "Connecting to the MQTT broker at " << BROKER_ADDRESS << "..." << std::endl;
//         client.connect(connOpts);
//         std::cout << "Connected." << std::endl;

//         int amr_count = 1; // 테스트할 AMR 개수 (AmrManager config와 일치시켜야 함)
//         std::string target_agv_id_prefix = "amr_"; 

//         // 모든 AMR의 상태 토픽 구독
//         std::vector<std::string> state_topics_to_subscribe;
//         for (int i = 0; i < amr_count; ++i) {
//             std::string agv_id = target_agv_id_prefix + std::to_string(i);
//             state_topics_to_subscribe.push_back("vda5050/agv/" + agv_id + "/state");
//         }

//         if (!state_topics_to_subscribe.empty()) {
//             std::cout << "Subscribing to state topics..." << std::endl;
//             client.subscribe(state_topics_to_subscribe, std::vector<int>(state_topics_to_subscribe.size(), 1)); // QoS 1로 구독
//             std::cout << "Subscribed to " << state_topics_to_subscribe.size() << " topics." << std::endl;
//         }

//         int current_order_id = 1;
        
//         while (true) {
//             std::cout << "\nEnter command (s: send order, q: quit): ";
//             std::string command;
//             std::getline(std::cin, command);

//             if (command == "q") {
//                 break;
//             } else if (command == "s") {
//                 std::cout << "Enter target AMR index (0 to " << (amr_count - 1) << "): ";
//                 std::string index_str;
//                 std::getline(std::cin, index_str);
//                 int target_idx;
//                 try {
//                     target_idx = std::stoi(index_str);
//                     if (target_idx < 0 || target_idx >= amr_count) {
//                         std::cerr << "Invalid AMR index." << std::endl;
//                         continue;
//                     }
//                 } catch (const std::exception& e) {
//                     std::cerr << "Invalid input: " << e.what() << std::endl;
//                     continue;
//                 }

//                 std::string target_agv_id = target_agv_id_prefix + std::to_string(target_idx);
//                 std::string order_topic = "vda5050/agv/" + target_agv_id + "/order";

//                 // VDA 5050 Order 메시지 생성
//                 // 노드 위치 정보를 추가하여 AMR이 이동을 시뮬레이션할 수 있도록 함
//                 std::string order_msg_payload = createVda5050OrderMessage(
//                     current_order_id++, 
//                     target_agv_id,
//                     "ChargingStation", // 예시 시작 노드
//                     "DeliveryPointA"   // 예시 목표 노드
//                 );

//                 std::cout << "\nSending VDA 5050 Order to topic: " << order_topic << "\n";
//                 std::cout << order_msg_payload << std::endl;

//                 // MQTT 메시지 발행 (QoS 1, retain false)
//                 mqtt::message_ptr pubmsg = mqtt::make_message(order_topic, order_msg_payload);
//                 pubmsg->set_qos(1); // At least once delivery
//                 client.publish(pubmsg);
//                 std::cout << "Order published." << std::endl;

//             } else {
//                 std::cout << "Invalid command." << std::endl;
//             }
//         }

//     } catch (const mqtt::exception& exc) {
//         std::cerr << "MQTT Error: " << exc.what() << std::endl;
//         return 1;
//     } catch (const std::exception& exc) {
//         std::cerr << "Error: " << exc.what() << std::endl;
//         return 1;
//     }

//     try {
//         std::cout << "Disconnecting from the MQTT broker..." << std::endl;
//         client.disconnect();
//         std::cout << "Disconnected." << std::endl;
//     } catch (const mqtt::exception& exc) {
//         std::cerr << "Error disconnecting: " << exc.what() << std::endl;
//         return 1;
//     }

//     return 0;
// }


#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <vector>

#include <mqtt/client.h>
#include <json.hpp>

// MQTT 브로커 정보
const std::string BROKER_ADDRESS = "tcp://127.0.0.1:1883"; // 동일 PC의 MQTT 브로커
const std::string CLIENT_ID = "test_fms_client"; // FMS 클라이언트 ID

// VDA 5050 Order 메시지 생성 함수 (노드 5개, 엣지 4개 포함)
std::string createVda5050OrderMessage(int orderId, const std::string& agvId) {
    nlohmann::json order_json;

    order_json["headerId"] = "test-order-header-" + std::to_string(orderId);

    // 현재 UTC 시간 포맷팅
    auto now = std::chrono::system_clock::now();
    auto tt = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::gmtime(&tt), "%Y-%m-%dT%H:%M:%S.000Z");
    order_json["timestamp"] = ss.str();

    order_json["version"] = "2.0.0";
    order_json["manufacturer"] = "TestCompany";
    order_json["serialNumber"] = agvId;
    order_json["agvId"] = agvId;
    order_json["orderId"] = "order-" + std::to_string(orderId);
    order_json["orderType"] = "NORMAL";

    // 노드 5개 생성
    nlohmann::json nodes = nlohmann::json::array();
    nodes.push_back({{"nodeId", "Node1"}, {"sequenceId", 0}, {"nodePosition", {{"x", 0.0}, {"y", 0.0}, {"theta", 0.0}}}});
    nodes.push_back({{"nodeId", "Node2"}, {"sequenceId", 1}, {"nodePosition", {{"x", 1.0}, {"y", 0.0}, {"theta", 0.0}}}});
    nodes.push_back({{"nodeId", "Node3"}, {"sequenceId", 2}, {"nodePosition", {{"x", 2.0}, {"y", 0.0}, {"theta", 0.0}}}});
    nodes.push_back({{"nodeId", "Node4"}, {"sequenceId", 3}, {"nodePosition", {{"x", 3.0}, {"y", 0.0}, {"theta", 0.0}}}});
    nodes.push_back({{"nodeId", "Node5"}, {"sequenceId", 4}, {"nodePosition", {{"x", 4.0}, {"y", 0.0}, {"theta", 0.0}}}});
    order_json["nodes"] = nodes;

    // 엣지 4개 생성 (startNodeId, endNodeId 포함)
    nlohmann::json edges = nlohmann::json::array();
    edges.push_back({{"edgeId", "Edge1"}, {"sequenceId", 0}, {"startNodeId", "Node1"}, {"endNodeId", "Node2"}});
    edges.push_back({{"edgeId", "Edge2"}, {"sequenceId", 1}, {"startNodeId", "Node2"}, {"endNodeId", "Node3"}});
    edges.push_back({{"edgeId", "Edge3"}, {"sequenceId", 2}, {"startNodeId", "Node3"}, {"endNodeId", "Node4"}});
    edges.push_back({{"edgeId", "Edge4"}, {"sequenceId", 3}, {"startNodeId", "Node4"}, {"endNodeId", "Node5"}});
    order_json["edges"] = edges;

    return order_json.dump(2); // JSON 예쁘게 포맷팅
}

// MQTT 콜백 핸들러 클래스
class Callback : public virtual mqtt::callback,
                 public virtual mqtt::iaction_listener {
public:
    // 연결 끊김 처리
    void connection_lost(const std::string& cause) override {
        std::cout << "\nConnection lost";
        if (!cause.empty())
            std::cout << ", cause: " << cause;
        std::cout << std::endl;
    }

    // 메시지 배달 완료 처리 (주로 publish 완료)
    void delivery_complete(mqtt::delivery_token_ptr tok) override {
        // 필요시 로그 출력 가능
        // std::cout << "Delivery complete for token: " << (tok ? tok->get_message_id() : -1) << std::endl;
    }

    // 메시지 수신 처리 (state 수신)
    void message_arrived(mqtt::const_message_ptr msg) override {
        std::cout << "\n-------------------------------------------------" << std::endl;
        std::cout << "Message arrived on topic: '" << msg->get_topic() << "'" << std::endl;
        std::cout << "Payload: " << msg->to_string() << std::endl;
        std::cout << "-------------------------------------------------" << std::endl;

        // JSON 파싱 및 AGV 상태 주요 정보 출력 예제
        try {
            nlohmann::json state_json = nlohmann::json::parse(msg->to_string());
            if (state_json.contains("agvId")) {
                std::cout << "  [Parsed State] AGV ID: " << state_json["agvId"].get<std::string>() << std::endl;
            }
            if (state_json.contains("batteryState") && state_json["batteryState"].contains("batteryCharge")) {
                std::cout << "  [Parsed State] Battery Charge: " << state_json["batteryState"]["batteryCharge"].get<double>() << std::endl;
            }
            if (state_json.contains("drivingStatus")) {
                std::cout << "  [Parsed State] Driving Status: " << state_json["drivingStatus"].get<std::string>() << std::endl;
            }
            if (state_json.contains("info") && state_json["info"].is_array()) {
                for (const auto& info_item : state_json["info"]) {
                    if (info_item.contains("infoType") && info_item["infoType"] == "current_node") {
                        std::cout << "  [Parsed State] Current Info: " << info_item["infoDescription"].get<std::string>() << std::endl;
                    }
                }
            }
        } catch (const nlohmann::json::parse_error& e) {
            std::cerr << "  [ERROR] Failed to parse state JSON: " << e.what() << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "  [ERROR] Error processing state message: " << e.what() << std::endl;
        }
    }

    void on_success(const mqtt::token& tok) override {
        std::cout << "Subscription success for token: " << tok.get_message_id() << std::endl;
    }

    void on_failure(const mqtt::token& tok) override {
        std::cout << "Subscription failed for token: " << tok.get_message_id() << std::endl;
    }
};

int main(int argc, char* argv[]) {
    mqtt::client client(BROKER_ADDRESS, CLIENT_ID);
    Callback cb;
    client.set_callback(cb);

    mqtt::connect_options connOpts;
    connOpts.set_keep_alive_interval(20);
    connOpts.set_clean_session(true);

    try {
        std::cout << "Connecting to MQTT broker at " << BROKER_ADDRESS << "..." << std::endl;
        client.connect(connOpts);
        std::cout << "Connected." << std::endl;

        const int amr_count = 1; // 테스트할 AMR 수
        const std::string agv_id_prefix = "amr_";

        // AMR 상태 토픽 구독 (예: vda5050/agv/amr_0/state)
        std::vector<std::string> state_topics;
        for (int i = 0; i < amr_count; ++i) {
            state_topics.push_back("vda5050/agv/" + agv_id_prefix + std::to_string(i) + "/state");
        }
        if (!state_topics.empty()) {
            std::cout << "Subscribing to AMR state topics..." << std::endl;
            client.subscribe(state_topics, std::vector<int>(state_topics.size(), 1)); // QoS 1
            std::cout << "Subscribed to " << state_topics.size() << " state topics." << std::endl;
        }

        int current_order_id = 1;

        while (true) {
            std::cout << "\nEnter command (s: send order, q: quit): ";
            std::string command;
            std::getline(std::cin, command);

            if (command == "q") {
                break;
            } else if (command == "s") {
                std::cout << "Enter target AMR index (0 to " << (amr_count - 1) << "): ";
                std::string idx_str;
                std::getline(std::cin, idx_str);

                int target_idx = 0;
                try {
                    target_idx = std::stoi(idx_str);
                    if (target_idx < 0 || target_idx >= amr_count) {
                        std::cerr << "Invalid AMR index." << std::endl;
                        continue;
                    }
                } catch (const std::exception& e) {
                    std::cerr << "Invalid input: " << e.what() << std::endl;
                    continue;
                }

                std::string target_agv_id = agv_id_prefix + std::to_string(target_idx);
                std::string order_topic = "vda5050/agv/" + target_agv_id + "/order";

                // 노드 5개, 엣지 4개 포함 Order 메시지 생성
                std::string order_msg = createVda5050OrderMessage(current_order_id++, target_agv_id);

                std::cout << "\nPublishing VDA 5050 Order to topic: " << order_topic << std::endl;
                std::cout << order_msg << std::endl;

                mqtt::message_ptr pubmsg = mqtt::make_message(order_topic, order_msg);
                pubmsg->set_qos(1); // QoS 1
                client.publish(pubmsg);

                std::cout << "Order published." << std::endl;
            } else {
                std::cout << "Invalid command." << std::endl;
            }
        }
    } catch (const mqtt::exception& e) {
        std::cerr << "MQTT Error: " << e.what() << std::endl;
        return 1;
    }

    try {
        std::cout << "Disconnecting from MQTT broker..." << std::endl;
        client.disconnect();
        std::cout << "Disconnected." << std::endl;
    } catch (const mqtt::exception& e) {
        std::cerr << "Disconnect error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}


#include <uwebsockets/App.h>
#include <iostream>
#include <nlohmann/json.hpp>
#include <thread>
#include <chrono>

using json = nlohmann::json;

// AMR에 보낼 주문 메시지
json get_order_message() {
    return {
        {"headerId", 1},
        {"orderId", "order-001"},
        {"nodes", {
            {{"nodeId", "start"}, {"sequenceId", 0}},
            {{"nodeId", "goal"}, {"sequenceId", 1}}
        }},
        {"edges", {
            {
                {"edgeId", "edge-01"},
                {"sequenceId", 0},
                {"startNodeId", "start"},
                {"endNodeId", "goal"},
                {"trajectory", {
                    {"controlPoints", json::array({ 
                        {{"x", 0.0}, {"y", 0.0}, {"weight", 1.0}},
                        {{"x", 1.0}, {"y", 1.0}, {"weight", 1.0}}
                    })}
                }}
            }
        }}
    };
}

int main() {
    uWS::App().ws<false>("/*", {
        .open = [](auto* ws) {
            std::cout << "🚀 Connection opened\n";

            // 주문 전송
            auto order = get_order_message();
            std::string order_json = order.dump();
            ws->send(order_json, uWS::OpCode::TEXT);

            std::cout << "📤 Sent order:\n" << order_json << "\n";
        },
        .message = [](auto* ws, std::string_view message, uWS::OpCode) {
            std::cout << "📥 Received message:\n" << message << "\n";

            // 여기에 state, actionState 등 수신 데이터 파싱 가능
        },
        .close = [](auto*, int, std::string_view) {
            std::cout << "❌ Connection closed\n";
        }
    }).listen(8080, [](auto* listen_socket) {
        if (listen_socket) {
            std::cout << "✅ FMS WebSocket Server listening on port 8080\n";
        }
    }).run();

    return 0;
}

// #include <iostream>
// #include <chrono>
// #include <thread>
// #include <memory> // For std::shared_ptr

// // vda5050pp 라이브러리의 주요 헤더 파일
// #include <vda5050++/handle.h>
// #include <vda5050++/config.h>
// #include <vda5050++/config/types.h> // vda5050pp::model::* 타입 정의를 위해 추가

// int main() {
//     vda5050pp::Config config;
//     std::string fms_id = "fms_client_1";

//     // FMS 클라이언트의 MQTT 설정 (에뮬레이터와 동일한 브로커 사용)
//     config.refMqttPubConfig().refOptions().server = "tcp://localhost:1883";
//     config.refMqttPubConfig().refOptions().client_id = fms_id;
//     config.refMqttPubConfig().refOptions().username = "testuser"; // 에뮬레이터 설정에 따라 변경
//     config.refMqttPubConfig().refOptions().password = "testpassword"; // 에뮬레이터 설정에 따라 변경
//     // FMS 클라이언트가 상태 메시지를 수신하려면 구독 설정도 필요하지만,
//     // 현재 handle.h에는 상태 수신을 위한 직접적인 핸들러 등록 함수가 없으므로
//     // 이 설정은 주문 발행에만 중점을 둡니다.
//     config.refMqttSubConfig().refOptions().server = "tcp://localhost:1883"; // MQTT SubConfig도 필요할 수 있음 (내부적으로 구독)
//     config.refMqttSubConfig().refOptions().client_id = fms_id + "_sub";
//     config.refMqttSubConfig().refOptions().username = "testuser";
//     config.refMqttSubConfig().refOptions().password = "testpassword";


//     vda5050pp::Handle fms_handle;
//     fms_handle.initialize(config);

//     // AGV ID (에뮬레이터의 amr_params.yaml에 설정된 AGV ID와 일치해야 함. 예: amr_0)
//     std::string target_agv_id = "amr_0"; 

//     std::cout << "[FMS Client] Sending VDA5050 Order to " << target_agv_id << std::endl;

//     // VDA5050 Order 메시지 생성
//     vda5050pp::model::Order order; 
//     order.orderId = "order_123";
//     order.orderUpdateId = 0;
//     order.agvId = target_agv_id;
//     order.timestamp = vda5050pp::util::createCurrentTimestamp(); // vda5050pp::util::createCurrentTimestamp() 사용
//     order.version = "2.0.0"; // VDA5050 버전

//     // 노드 정의
//     vda5050pp::model::Node node1; 
//     node1.nodeId = "node_start";
//     node1.nodePosition = vda5050pp::model::AgvPosition(); 
//     node1.nodePosition->x = 0.0;
//     node1.nodePosition->y = 0.0;
//     node1.nodePosition->theta = 0.0;

//     vda5050pp::model::Node node2;
//     node2.nodeId = "node_intermediate";
//     node2.nodePosition = vda5050pp::model::AgvPosition();
//     node2.nodePosition->x = 5.0;
//     node2.nodePosition->y = 0.0;
//     node2.nodePosition->theta = 0.0;

//     vda5050pp::model::Node node3;
//     node3.nodeId = "node_end";
//     node3.nodePosition = vda5050pp::model::AgvPosition();
//     node3.nodePosition->x = 5.0;
//     node3.nodePosition->y = 5.0;
//     node3.nodePosition->theta = 90.0;

//     order.nodes.push_back(node1);
//     order.nodes.push_back(node2);
//     order.nodes.push_back(node3);

//     // 엣지 정의 (노드 연결)
//     vda5050pp::model::Edge edge1; 
//     edge1.edgeId = "edge_start_to_intermediate";
//     edge1.startNodeId = "node_start";
//     edge1.endNodeId = "node_intermediate";
//     edge1.length = 5.0;

//     order.edges.push_back(edge1);

//     vda5050pp::model::Edge edge2;
//     edge2.edgeId = "edge_intermediate_to_end";
//     edge2.startNodeId = "node_intermediate";
//     edge2.endNodeId = "node_end";
//     edge2.length = 5.0;

//     order.edges.push_back(edge2);

//     // FMS_Handle을 통해 Order 메시지 publish
//     // handle.h에는 직접적인 publishOrder가 보이지 않지만,
//     // amr_emulator의 Vda5050Protocol 구현에서 handle을 통해 publishState를 호출하는 것으로 미루어 보아
//     // publishOrder 또한 Handle 클래스에 존재하거나, libvda5050pp의 FMS 측 API를 통해 제공될 것으로 예상합니다.
//     try {
//         fms_handle.publishOrder(order); // 이 부분은 libvda5050pp의 실제 publishOrder 함수명에 따라 달라질 수 있습니다.
//         std::cout << "[FMS Client] Order message published successfully." << std::endl;
//     } catch (const std::exception& e) {
//         std::cerr << "[FMS Client] Error publishing order: " << e.what() << std::endl;
//     }

//     // 에뮬레이터로부터 상태 메시지를 받을 수 있도록 일정 시간 대기 (이 코드에서는 상태 수신 로직이 없지만, 에뮬레이터 처리 시간 대기)
//     std::cout << "[FMS Client] Waiting for 10 seconds (for emulator to process order and publish state)..." << std::endl;
//     std::this_thread::sleep_for(std::chrono::seconds(10));

//     fms_handle.shutdown();
//     std::cout << "[FMS Client] FMS client shut down." << std::endl;

//     return 0;
// }
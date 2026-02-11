#include "custom_tcp_protocol.h"
#include <nlohmann/json.hpp>
#include <thread>
#include <chrono>

// CustomTcpProtocol::CustomTcpProtocol() : amr_(nullptr) 
// {

// }

// void CustomTcpProtocol::setAmr(IAmr* amr) 
// {
//     amr_ = amr;
// }

// void CustomTcpProtocol::handleMessage(const std::string& msg, IAmr* amr_ptr) 
// {
//     std::cout << "[CustomTCP] Received message for AMR " << amr_ptr->getState() << ": " << msg << std::endl;
//     try {
//         auto json_msg = nlohmann::json::parse(msg);
//         std::string command = json_msg["command"].get<std::string>();
//         std::string agv_id = json_msg["agvId"].get<std::string>();

//         // Check if the command is for this AMR
//         if (amr_ptr->getState().find("AMR" + std::to_string(amr_ptr->getCurIdx())) == std::string::npos &&
//             agv_id.find("amr_" + std::to_string(amr_ptr->getCurIdx())) == std::string::npos) {
//             std::cerr << "[CustomTCP] Message not for this AMR: " << agv_id << ", this AMR is: " << amr_ptr->getState() << std::endl;
//             return;
//         }

//         if (command == "move") {
//             // Simulate movement
//             std::string target_node = json_msg["target_node"].get<std::string>();
//             std::cout << "[CustomTCP] AMR " << amr_ptr->getState() << " moving to " << target_node << std::endl;
//             // Simplified: just update the current node after a delay
//             std::thread([amr_ptr, target_node]() {
//                 std::this_thread::sleep_for(std::chrono::seconds(3));
//                 // In a real scenario, you'd update AMR's internal state to reflect movement
//                 // For this example, we'll just print completion.
//                 amr_ptr->setOrder({{target_node, 0.0, 0.0}}, {});
//                 std::cout << "[CustomTCP] AMR " << amr_ptr->getState() << " arrived at " << target_node << std::endl;
//             }).detach();
//         } else if (command == "charge") {
//             std::cout << "[CustomTCP] AMR " << amr_ptr->getState() << " is charging..." << std::endl;
//             std::thread([amr_ptr]() {
//                 std::this_thread::sleep_for(std::chrono::seconds(5));
//                 std::cout << "[CustomTCP] AMR " << amr_ptr->getState() << " finished charging." << std::endl;
//             }).detach();
//         } else {
//             std::cerr << "[CustomTCP] Unknown command: " << command << std::endl;
//         }
//     } catch (const nlohmann::json::parse_error& e) {
//         std::cerr << "[CustomTCP] JSON parse error: " << e.what() << std::endl;
//     } catch (const std::exception& e) {
//         std::cerr << "[CustomTCP] Error processing message: " << e.what() << std::endl;
//     }
// }

// std::string CustomTcpProtocol::makeStateMessage(IAmr* amr) {
//     if (!amr) return "";
//     return amr->getState();
// }

// void CustomTcpProtocol::start() {
//     // Custom TCP protocol doesn't have a separate "start" logic like MQTT connection.
//     // Its functionality is mainly driven by handleMessage via TcpServer.
//     std::cout << "[CustomTCP] Protocol started (no specific background tasks)." << std::endl;
// }